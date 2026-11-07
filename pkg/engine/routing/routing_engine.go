// Package routing provides routing algorithms and engines for finding fastest path in road networks.
package routing

import (
	"iter"
	"log"
	"runtime"
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/maypok86/otter/v2"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

// CRPRoutingEngine represents CRP routing engine.
type CRPRoutingEngine[W util.Numeric] struct {
	graph        *da.Graph
	overlayGraph *da.OverlayGraph
	rn           *da.RoadNetworkDataContainer
	metrics      *met.Metric[W]
	lm           *landmark.Landmark[W]
	logger       *zap.Logger
	puCache      *otter.Cache[uint64, []da.Index]

	fHeapPool sync.Pool
	bHeapPool sync.Pool

	uniHeapPool sync.Pool

	puHeapPool sync.Pool

	puovHeapPool sync.Pool
	mu           *sync.RWMutex

	// parameter yang di init pakai read yml config
	gammaLookup         ParamLookup[float64]
	alphaLookup         ParamLookup[float64]
	epsilonLookup       ParamLookup[float64]
	upperBoundLookup    ParamLookup[float64]
	maxCandidatesLookup ParamLookup[int]

	motorwayPenalty float64

	defaultGamma, defaultAlpha, defaultEpsilon, defaultUpperbound float64
	defaultMaxCandidatesToUnpack                                  int

	landmarkFile     string
	numberOfVertices int

	unpackerWorkers                     int
	unpackerForAlternativeRoutesWorkers int
}

// NewCRPRoutingEngine creates and returns CRP routing engine.
func NewCRPRoutingEngine[W util.Numeric](graph *da.Graph,
	overlayGraph *da.OverlayGraph, metrics *met.Metric[W],
	logger *zap.Logger, puCache *otter.Cache[uint64, []da.Index],
	landmarkFile string, rn *da.RoadNetworkDataContainer,
) *CRPRoutingEngine[W] {
	var err error

	lm := landmark.NewLandmark[W]()
	if landmarkFile != "" {
		lm, err = landmark.ReadLandmark[W](landmarkFile)
		if err != nil {
			log.Fatalf("NewCRPRoutingEngine: failed to read precomputed landmark distances: %v", err)
		}
	}

	crp := &CRPRoutingEngine[W]{
		graph:            graph,
		metrics:          metrics,
		overlayGraph:     overlayGraph,
		logger:           logger,
		puCache:          puCache,
		lm:               lm,
		rn:               rn,
		landmarkFile:     landmarkFile,
		mu:               &sync.RWMutex{},
		numberOfVertices: graph.NumberOfVertices(),
	}
	crp.BuildQueryHeapPool()
	crp.initParameter()
	return crp
}

// GetGraph returns road network graph.
func (crp *CRPRoutingEngine[W]) GetGraph() *da.Graph {
	return crp.graph
}

// GetOverlayGraph returns overlay graph.
func (crp *CRPRoutingEngine[W]) GetOverlayGraph() *da.OverlayGraph {
	return crp.overlayGraph
}

// GetMetrics returns metrics.
func (crp *CRPRoutingEngine[W]) GetMetrics() *met.Metric[W] {
	crp.mu.RLock()
	defer crp.mu.RUnlock()
	return crp.metrics
}

// GetLandmark returns landmark.
func (crp *CRPRoutingEngine[W]) GetLandmark() *landmark.Landmark[W] {
	crp.mu.RLock()
	defer crp.mu.RUnlock()
	return crp.lm
}

// GetRoadNetworkContainer returns road network container.
func (crp *CRPRoutingEngine[W]) GetRoadNetworkContainer() *da.RoadNetworkDataContainer {
	return crp.rn
}

// BuildQueryHeapPool builds query heap pool.
func (crp *CRPRoutingEngine[W]) BuildQueryHeapPool() {
	maxVerticesInCell := crp.graph.GetMaxVerticesInCell()
	numOverlayVertices := crp.overlayGraph.NumberOfOverlayVertices()

	// todo: ini kayake bisa dioptimize dengan gak reinitialize slice dari overlay vertices index di TwoLevelStorage?
	// kaya di implementasi crp by wagner ini: https://github.com/michaelwegner/CRP/blob/master/algorithm/CRPQuery.cpp
	// di implementasi crp by wagner, id dari explored graph vertices di offset biar jadi range [0, 2*maxNumVerticesInCell)
	// sedangkan overlay vertices id nya di offset biar jadi range [2*maxNumVerticesInCell, 2*maxNumVerticesInCell + numOverlayVertices)
	// graph vertices index nya bisa di simpan di array nya TwoLevelStorage (https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp)
	// overlay vertices index nya bisa di simpan di hashmap nya TwoLevelStorage (https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp)
	// tujuan utamanya adalah biar gak initialize slice QueryHeap.verticesIndex buat simpan overlay vertices index yang jumlah nya bisa ratusan ribu atau jutaan. lihat TwoLevelStorage.Clear() di index_storage.go
	// atau https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp
	// di jateng_jabar osm file, number of overlay/boundary vertices sekitar 790k. maybe this initialize ~800k elements dari slice bisa makan 1-2ms?
	// harus cari cara yang bikin code querynya nya masih enak dilihat
	// DONE :)

	// crp query heap pool

	crpMaxSearchSize := da.Index(2*maxVerticesInCell + da.Index(numOverlayVertices))

	approxMaxExploredSize := da.CRPMaxExploredSize
	crp.fHeapPool = sync.Pool{
		New: func() any {
			qh := da.NewQueryHeap[da.QueryKey, W](crpMaxSearchSize, approxMaxExploredSize, da.ArrayCRPStorageMode)
			qh.SetIsCRPQuery()
			return qh
		},
	}

	crp.bHeapPool = sync.Pool{
		New: func() any {
			qh := da.NewQueryHeap[da.QueryKey, W](crpMaxSearchSize, approxMaxExploredSize, da.ArrayCRPStorageMode)
			qh.SetIsCRPQuery()
			return qh
		},
	}

	crp.uniHeapPool = sync.Pool{
		New: func() any {
			qh := da.NewQueryHeap[da.QueryKey, W](crpMaxSearchSize, approxMaxExploredSize, da.ArrayStorageUniMode)
			qh.SetIsCRPQuery()
			return qh
		},
	}

	// path unpacking heap pool
	crp.puovHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](da.Index(numOverlayVertices), approxMaxExploredSize, da.ArrayCRPStorageMode)
		},
	}

	crp.puHeapPool = sync.Pool{
		New: func() any {
			pq := da.NewQueryHeap[da.Index, W](da.Index(maxVerticesInCell), approxMaxExploredSize, da.ArrayStorageMode)
			pq.SetLevelOneCellQuery()
			return pq
		},
	}
}

func (crp *CRPRoutingEngine[W]) initParameter() {
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	numCPU := runtime.NumCPU()
	crp.unpackerWorkers = numCPU / 6
	crp.unpackerForAlternativeRoutesWorkers = numCPU / 6
	roadnetwork := crp.graph.IsRoadNetworkGraph()
	if roadnetwork {
		altConfig := viper.GetStringMap("alternatives")
		var (
			gammaMap, alphaMap, epsilonMap, upperBoundMap map[float64]float64
			maxCandidatesToUnpackMap                      map[float64]int
		)
		gammaMap, crp.defaultGamma = util.ToFloat64Map(altConfig["gamma"])
		alphaMap, crp.defaultAlpha = util.ToFloat64Map(altConfig["alpha"])
		epsilonMap, crp.defaultEpsilon = util.ToFloat64Map(altConfig["epsilon"])
		upperBoundMap, crp.defaultUpperbound = util.ToFloat64Map(altConfig["upper_bound"])
		crp.validateUpperbound(upperBoundMap, crp.defaultUpperbound)
		maxCandidatesToUnpackMap, crp.defaultMaxCandidatesToUnpack = util.ToFloat64IntMap(altConfig["max_candidates_to_unpack"])
		mp := altConfig["motorway_penalty"]
		if val, ok := mp.(float64); ok {
			crp.motorwayPenalty = val
		} else {
			crp.logger.Fatal("motorway_penalty must be float64")
		}

		crp.gammaLookup = newParamLookup(gammaMap, crp.defaultGamma)
		crp.alphaLookup = newParamLookup(alphaMap, crp.defaultAlpha)
		crp.epsilonLookup = newParamLookup(epsilonMap, crp.defaultEpsilon)
		crp.upperBoundLookup = newParamLookup(upperBoundMap, crp.defaultUpperbound)
		crp.maxCandidatesLookup = newParamLookup(maxCandidatesToUnpackMap, crp.defaultMaxCandidatesToUnpack)
	}
}

func (crp *CRPRoutingEngine[W]) validateUpperbound(upperBoundMap map[float64]float64, defaultUpperbound float64) {
	if defaultUpperbound > 1.75 {
		crp.logger.Fatal("bidirectional search upperbound must less than or equal 1.75")
	}
	for _, val := range upperBoundMap {
		if val > 1.75 {
			crp.logger.Fatal("bidirectional search upperbound must less than or equal 1.75")
		}
	}
}

// Close handles close caches.
func (crp *CRPRoutingEngine[W]) Close() {
	crp.puCache.InvalidateAll()
	crp.puCache.StopAllGoroutines()
}

// ShortestPathSearch compute the point-to-point fastest path.
func (crp *CRPRoutingEngine[W]) ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, string, []da.Index, bool) {
	crpQuery := NewCRPUniALTQuery(crp)
	if reroute {
		crpQuery.SetReroute()
	}
	s := sp.GetVId()
	t := tp.GetVId()
	cost, segmentPath, found := crpQuery.ShortestPathSearch(s, t)
	routePolyline, totalDist, totalCost := crp.GetSegmentPath(segmentPath, sp, tp, float64(cost))
	return util.WeightToSeconds(totalCost), totalDist, routePolyline, segmentPath, found
}

// ShortestPathWithAlternativesSearch finds path with alternatives search.
func (crp *CRPRoutingEngine[W]) ShortestPathWithAlternativesSearch(sp, tp da.PhantomNode, k int, reroute bool, _ da.Index) ([]float64, []float64, []string, [][]da.Index, bool) {
	crpQuery := NewCRPQuery(crp) // better via-vertices
	s := sp.GetVId()
	t := tp.GetVId()
	ars := NewAlternativeRouteSearch(crp)
	param := ars.ParameterByRequest(s, t)
	crpQuery.SetUpperBound(param.upperBound)

	crpQuery.forAlternatives = true
	if reroute {
		crpQuery.reroute = true
	}

	cost, optPath, found := crpQuery.ShortestPathSearch(s, t)
	if !found {
		return EmptyCost, EmptyCost, EmptyCoordsRoutes, EmptyIndexSetRoutes, false
	}

	optRoutePolyline, optDist, optCost := crp.GetSegmentPath(optPath, sp, tp, float64(cost))
	optCost = util.WeightToSeconds(optCost)
	alts, _ := ars.ComputeAlternativeRoutes(s, t, k, reroute, optCost, optPath, crpQuery, param)

	n := len(alts) + 1
	var (
		routeCosts     = make([]float64, n)
		routeDists     = make([]float64, n)
		routePolylines = make([]string, n)
		routePaths     = make([][]da.Index, n)
	)
	routeCosts[0] = optCost
	routeDists[0] = optDist
	routePolylines[0] = optRoutePolyline
	routePaths[0] = optPath

	for i, alt := range alts {
		j := i + 1
		altPolyline, altDist, altCost := crp.GetSegmentPath(alt.GetSegmentPath(), sp, tp, alt.GetTravelTime())
		routeCosts[j] = altCost
		routeDists[j] = altDist
		routePolylines[j] = altPolyline
		routePaths[j] = alt.GetSegmentPath()
	}
	return routeCosts, routeDists, routePolylines, routePaths, true
}

// GetSegmentPath returns  full coordinates path of the fastest path.
func (crp *CRPRoutingEngine[W]) GetSegmentPath(segmentIdPath []da.Index, sp, tp da.PhantomNode, cost float64) (string, float64, float64) {

	dist := 0.0
	n := len(segmentIdPath)

	cCount := 0
	met := crp.GetMetrics()
	for i := 1; i < n-1; i++ { // skip road segments s & t. (kita append path nya di AppendPhantomNode)
		segId := segmentIdPath[i]
		dist += met.GetSegmentLength(segId)
		cCount += int(crp.rn.GetSegmentGeometryLength(segId))
	}
	cCount = cCount + sp.ForwardGeometrySize() + tp.ReverseGeometrySize() + 2

	totalCost, totalDistance := crp.appendPhantomNodesToPath(sp, tp, cost, dist)
	routeCoords := crp.forPathCoordinates(segmentIdPath, sp, tp)
	routePolyline := da.GooglePolylineFromCoords(routeCoords, cCount)
	return routePolyline, totalDistance, totalCost
}

func (crp *CRPRoutingEngine[W]) forPathCoordinates(segmentIdPath []da.Index, sp, tp da.PhantomNode) iter.Seq[da.Coordinate] {
	sameSegment := isSameSourceDestinationSegment(sp, tp)
	return func(yield func(da.Coordinate) bool) {
		crp.rn.YieldFullRouteCoordinates(segmentIdPath, sp, tp, sameSegment, yield)
	}
}

func (crp *CRPRoutingEngine[W]) appendPhantomNodesToPath(sp, tp da.PhantomNode, cost float64, dist float64) (float64, float64) {

	cost -= sp.ForwardCost()     // by default pakai edge-based graph, hasil router added duration/cost dari road segment s
	dist += sp.ForwardDistance() // kita gak tambahin distance dari projected s ke head dari road segment s di GetSegmentPath()

	cost += tp.ReverseCost()     // by default pakai edge-based graph, hasil router gak add duration/cost dari road segment t
	dist += tp.ReverseDistance() // kita gak tambahin distance dari projected t ke head dari road segment t di GetSegmentPath()

	return cost, dist
}

func isSameSourceDestinationSegment(sp, tp da.PhantomNode) bool {
	return sp.GetVId() == tp.GetVId()
}

// GetV returns head vertex v of edge (u,v) (or reversed edge (u,v)).
func (crp *CRPRoutingEngine[W]) GetV(eId da.Index, out bool) da.Index {
	if out {
		return crp.graph.GetHead(eId)
	}
	return crp.graph.GetTail(eId)
}

// GetOV returns boundary/overlay vertex at specified id.
func (crp *CRPRoutingEngine[W]) GetOV(ovOffset, wOffset, numExit, i da.Index, out bool) (da.Index, da.Index) {
	if out {
		v := crp.overlayGraph.GetCellExitVertex(ovOffset + i)
		return v, wOffset + i
	}
	v := crp.overlayGraph.GetCellEntryVertex(ovOffset + i)
	return v, wOffset + i*numExit
}

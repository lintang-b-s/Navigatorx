// Package routing provides routing algorithms and engines for finding fastest path in road networks.
package routing

import (
	"bufio"
	"fmt"
	"runtime"
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/maypok86/otter/v2"
	"go.uber.org/zap"
)

type CRPRoutingEngine[W util.RoutingNumber] struct {
	graph               *da.Graph
	overlayGraph        *da.OverlayGraph
	metrics             *met.Metric[W]
	lm                  *landmark.Landmark[W]
	logger              *zap.Logger
	puCache             *otter.Cache[da.PUCacheKey, []da.Index]
	verticesLookupTable *customizer.LookupTable[uint64]

	fHeapPool sync.Pool
	bHeapPool sync.Pool

	pufOverlayHeapPool sync.Pool
	pufBaseHeapPool    sync.Pool

	coordsPool         sync.Pool
	bidirSearchPool    sync.Pool
	altBidirSearchPool sync.Pool
	pathUnpackerPool   sync.Pool
	landmarkFile       string

	// wirhout turn cost
	fHeapNoTurnCostPool sync.Pool
	bHeapNoTurnCostPool sync.Pool

	pufBaseNoTurnCostHeapPool  sync.Pool
	pathUnpackerNoTurnCostPool sync.Pool

	unpackerWorkers                     int
	unpackerForAlternativeRoutesWorkers int
}

func NewCRPRoutingEngine[W util.RoutingNumber](graph *da.Graph,
	overlayGraph *da.OverlayGraph, metrics *met.Metric[W],
	logger *zap.Logger, puCache *otter.Cache[da.PUCacheKey, []da.Index],
	landmarkFile string, readBuf *bufio.Reader,
) *CRPRoutingEngine[W] {
	var err error

	lm := landmark.NewLandmark[W]()
	if landmarkFile != "" {
		lm, err = landmark.ReadLandmark[W](landmarkFile, readBuf)
		if err != nil {
			panic(fmt.Errorf("NewCRPRoutingEngine: failed to read precomputed landmark distances: %v", err))
		}
	}

	vertexOsmIds := graph.GetVertexOsmIds()
	verticesLookupTable := customizer.NewLookupTable(vertexOsmIds, func(a, b uint64) bool {
		return a < b
	})

	crp := &CRPRoutingEngine[W]{
		graph:               graph,
		metrics:             metrics,
		overlayGraph:        overlayGraph,
		logger:              logger,
		puCache:             puCache,
		lm:                  lm,
		landmarkFile:        landmarkFile,
		verticesLookupTable: verticesLookupTable,
	}
	crp.BuildQueryHeapPool()
	crp.initParameter()
	return crp
}

func (crp *CRPRoutingEngine[W]) GetGraph() *da.Graph {
	return crp.graph
}

func (crp *CRPRoutingEngine[W]) GetOverlayGraph() *da.OverlayGraph {
	return crp.overlayGraph
}

func (crp *CRPRoutingEngine[W]) GetMetrics() *met.Metric[W] {
	return crp.metrics
}

func (crp *CRPRoutingEngine[W]) GetCostFunction() *costfunction.TimeFunction[W] {
	return crp.metrics.GetCostFunction()
}

func (crp *CRPRoutingEngine[W]) BuildQueryHeapPool() {
	maxEdgesInCell := crp.graph.GetMaxEdgesInCell()
	numberOfOverlayVertices := crp.overlayGraph.NumberOfOverlayVertices()
	maxSearchSize := uint32(maxEdgesInCell*2) + uint32(numberOfOverlayVertices)
	// crp query heap pool
	crp.fHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey, W](maxSearchSize, uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}
	crp.bHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey, W](maxSearchSize, uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	crp.fHeapNoTurnCostPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKeyNoTurnCost, W](maxSearchSize, uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	crp.bHeapNoTurnCostPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKeyNoTurnCost, W](maxSearchSize, uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	// path unpacking heap pool
	crp.pufOverlayHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](da.OVERLAY_CELL_INFO_SIZE, uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	crp.pufBaseHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxEdgesInCell)*2, uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	crp.pufBaseNoTurnCostHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](uint32(maxEdgesInCell)*2, uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	crp.coordsPool = sync.Pool{
		New: func() any {
			cs := da.NewCoordinatesWithCap(0)
			return cs
		},
	}

	crp.bidirSearchPool = sync.Pool{
		New: func() any {
			return newCRPBidirectionalSearchAlloc[W](crp)
		},
	}

	crp.altBidirSearchPool = sync.Pool{
		New: func() any {
			return newCRPALTBidirectionalSearchAlloc[W](crp)
		},
	}

	crp.pathUnpackerPool = sync.Pool{
		New: func() any {
			pu := newPathUnpackerALTAlloc[W](crp)
			return pu
		},
	}

	crp.pathUnpackerNoTurnCostPool = sync.Pool{
		New: func() any {
			pu := newPathUnpackerALTNoTurnCostAlloc[W](crp)
			return pu
		},
	}
}

func (crp *CRPRoutingEngine[W]) initParameter() {
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	numCPU := runtime.NumCPU()
	crp.unpackerWorkers = numCPU / 6
	crp.unpackerForAlternativeRoutesWorkers = numCPU / 6
}

func (crp *CRPRoutingEngine[W]) Close() {
	crp.puCache.InvalidateAll()
	crp.puCache.StopAllGoroutines()
}

// GetWeight. get weight of outEdge/inEdge
func (crp *CRPRoutingEngine[W]) getWeight(eId da.Index, outEdge bool) W {
	if !outEdge {
		eId = crp.graph.GetExitIdOfInEdge(eId)
	}
	return crp.metrics.GetWeight(eId)
}

func (crp *CRPRoutingEngine[W]) GetWeightSeconds(eId da.Index, outEdge bool) float64 {
	return crp.metrics.GetCostFunction().WeightToSeconds(crp.getWeight(eId, outEdge))
}

func (crp *CRPRoutingEngine[W]) getWeightFromLength(eId da.Index, outEdge bool, eLength uint32) W {
	if !outEdge {
		eId = crp.graph.GetExitIdOfInEdge(eId)
	}
	return crp.metrics.GetWeightFromLength(eId, eLength)
}

// GetWeightFromLength. get weight (traveltime /duration) of a road semgent given road segment length
func (crp *CRPRoutingEngine[W]) GetWeightFromLength(eId da.Index, outEdge bool, eLength float64) float64 {
	length := crp.metrics.GetCostFunction().DistanceFromMeters(eLength)

	return crp.metrics.GetCostFunction().WeightToSeconds(crp.getWeightFromLength(eId, outEdge, length))
}

func (crp *CRPRoutingEngine[W]) getSegmentLength(eId da.Index, outEdge bool) uint32 {
	if !outEdge {
		eId = crp.graph.GetExitIdOfInEdge(eId)
	}
	return crp.metrics.GetSegmentLength(eId)
}

// GetSegmentLength. get road segment (edge) length in meters
func (crp *CRPRoutingEngine[W]) GetSegmentLength(eId da.Index, outEdge bool) float64 {
	return crp.metrics.GetCostFunction().DistanceToMeters(crp.getSegmentLength(eId, outEdge))
}

// GetWeight. get speed of outEdge in m/s
func (crp *CRPRoutingEngine[W]) GetSegmentSpeed(eId da.Index, outEdge bool) float64 {
	if outEdge {
		return crp.metrics.GetCostFunction().SpeedToMetersPerSecond(crp.metrics.GetSegmentSpeed(eId))
	}

	eExitId := crp.graph.GetExitIdOfInEdge(eId)
	return crp.metrics.GetCostFunction().SpeedToMetersPerSecond(crp.metrics.GetSegmentSpeed(eExitId))
}

func (crp *CRPRoutingEngine[W]) IsDummyOutEdge(eId da.Index) bool {
	return crp.graph.IsDummyOutEdge(eId)
}

func (crp *CRPRoutingEngine[W]) IsDummyInEdge(eId da.Index) bool {
	return crp.graph.IsDummyInEdge(eId)
}

// PutCoordsToPool returns a *Coordinates to the engine's coords pool so the
// underlying slice can be reused on the next GetEdgePath / GetCoords call.
func (crp *CRPRoutingEngine[W]) PutCoordsToPool(coords *da.Coordinates) {
	coords.Reset()
	crp.coordsPool.Put(coords)
}

// GetCoordsFromPool fetches a *Coordinates from the engine's coords pool,
// resetting its length to 0 while preserving capacity.
func (crp *CRPRoutingEngine[W]) GetCoordsFromPool() *da.Coordinates {
	c := crp.coordsPool.Get().(*da.Coordinates)
	c.Reset()
	return c
}

func (crp *CRPRoutingEngine[W]) ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, *da.Coordinates, []da.Index, bool) {
	crpQuery := NewCRPALTBidirectionalSearch(crp, 1.0)
	if reroute {
		crpQuery.SetReroute()
	}
	weight, distance, path, edgePath, found := crpQuery.ShortestPathSearch(sp, tp)
	return crp.metrics.GetCostFunction().WeightToSeconds(weight), distance, path, edgePath, found
}

// EmptyCoords is a package-level sentinel *Coordinates used to represent a
// "not found" path without heap-allocating a new Coordinates value.
var EmptyCoords = da.NewCoordinatesWithCap(0)

// EmptyIndexSet is a package-level sentinel []da.Index used to represent a
// "not found" edge-id path without heap-allocating a new slice.
var EmptyIndexSet = []da.Index{}

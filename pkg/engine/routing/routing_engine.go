// Package routing provides routing algorithms and engines for finding fastest path in road networks.
package routing

import (
	"fmt"
	"runtime"
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/maypok86/otter/v2"
	"go.uber.org/zap"
)

type CRPRoutingEngine[W util.RoutingNumber] struct {
	graph        *da.Graph
	overlayGraph *da.OverlayGraph
	rn           *da.RoadNetworkDataContainer
	metrics      *met.Metric[W]
	lm           *landmark.Landmark[W]
	logger       *zap.Logger
	puCache      *otter.Cache[da.PUCacheKey, []da.Index]

	fHeapPool  sync.Pool
	bHeapPool  sync.Pool
	puHeapPool sync.Pool

	puovHeapPool sync.Pool
	mu           *sync.RWMutex

	landmarkFile string

	unpackerWorkers                     int
	unpackerForAlternativeRoutesWorkers int
}

func NewCRPRoutingEngine[W util.RoutingNumber](graph *da.Graph,
	overlayGraph *da.OverlayGraph, metrics *met.Metric[W],
	logger *zap.Logger, puCache *otter.Cache[da.PUCacheKey, []da.Index],
	landmarkFile string, rn *da.RoadNetworkDataContainer,
) *CRPRoutingEngine[W] {
	var err error

	lm := landmark.NewLandmark[W]()
	if landmarkFile != "" {
		lm, err = landmark.ReadLandmark[W](landmarkFile)
		if err != nil {
			panic(fmt.Errorf("NewCRPRoutingEngine: failed to read precomputed landmark distances: %v", err))
		}
	}

	crp := &CRPRoutingEngine[W]{
		graph:        graph,
		metrics:      metrics,
		overlayGraph: overlayGraph,
		logger:       logger,
		puCache:      puCache,
		lm:           lm,
		rn:           rn,
		landmarkFile: landmarkFile,
		mu:           &sync.RWMutex{},
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
	crp.mu.RLock()
	defer crp.mu.RUnlock()
	return crp.metrics
}

func (crp *CRPRoutingEngine[W]) GetLandmark() *landmark.Landmark[W] {
	crp.mu.RLock()
	defer crp.mu.RUnlock()
	return crp.lm
}

func (crp *CRPRoutingEngine[W]) GetRoadNetworkContainer() *da.RoadNetworkDataContainer {
	return crp.rn
}

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
	crp.fHeapPool = sync.Pool{
		New: func() any {
			qh := da.NewQueryHeap[da.QueryKey, W](crpMaxSearchSize, da.ARRAY_STORAGE)
			qh.SetIsCRPQuery()
			return qh
		},
	}

	crp.bHeapPool = sync.Pool{
		New: func() any {
			qh := da.NewQueryHeap[da.QueryKey, W](crpMaxSearchSize, da.ARRAY_STORAGE)
			qh.SetIsCRPQuery()
			return qh
		},
	}

	// path unpacking heap pool
	crp.puovHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](da.Index(numOverlayVertices), da.ARRAY_STORAGE)
		},
	}

	crp.puHeapPool = sync.Pool{
		New: func() any {
			pq := da.NewQueryHeap[da.Index, W](da.Index(maxVerticesInCell), da.ARRAY_STORAGE)
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
}

func (crp *CRPRoutingEngine[W]) Close() {
	crp.puCache.InvalidateAll()
	crp.puCache.StopAllGoroutines()
}

func (crp *CRPRoutingEngine[W]) ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, *da.Coordinates, []da.Index, bool) {
	crpQuery := NewCRPALTQuery(crp)
	if reroute {
		crpQuery.SetReroute()
	}
	s := sp.GetVId()
	t := tp.GetVId()
	weight, segmentPath, found := crpQuery.ShortestPathSearch(s, t)
	segmentIdPath, dist := crp.GetSegmentPath(segmentPath)
	return util.WeightToSeconds(weight), dist, segmentIdPath, segmentPath, found
}

var EmptyCoords = da.NewCoordinatesWithCap(0)
var EmptyIndexSet = []da.Index{}

func (crp *CRPRoutingEngine[W]) GetSegmentPath(segmentIdPath []da.Index) (*da.Coordinates, float64) {

	totalDistance := 0.0

	path := &da.Coordinates{}

	met := crp.GetMetrics()
	for i := 1; i < len(segmentIdPath)-1; i++ { // skip road segments s & t. (kita append path nya di AppendPhantomNode)
		segId := segmentIdPath[i]
		path.Append(crp.rn.GetSegmentGeometry(segId))
		totalDistance += met.GetSegmentLength(segId)
	}

	return path, totalDistance
}

func (crp *CRPRoutingEngine[W]) GetV(offset, i da.Index, out bool) (da.Index, da.Index) {
	eId := offset + i
	if out {
		return crp.graph.GetHead(eId), eId
	}
	return crp.graph.GetTail(eId), eId
}

func (crp *CRPRoutingEngine[W]) GetOV(ovOffset, wOffset, numExit, i da.Index, out bool) (da.Index, da.Index) {
	if out {
		v := crp.overlayGraph.GetCellExitVertex(ovOffset + i)
		return v, wOffset + i
	}
	v := crp.overlayGraph.GetCellEntryVertex(ovOffset + i)
	return v, wOffset + i*numExit
}

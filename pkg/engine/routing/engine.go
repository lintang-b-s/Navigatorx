package routing

import (
	"runtime"
	"sync"

	"github.com/dgraph-io/ristretto/v2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type CRPRoutingEngine struct {
	graph              *da.Graph
	overlayGraph       *da.OverlayGraph
	metrics            *met.Metric
	lm                 *landmark.Landmark
	logger             *zap.Logger
	puCache            *ristretto.Cache[[]byte, []da.Index]
	fHeapPool          sync.Pool
	bHeapPool          sync.Pool
	pufOverlayHeapPool sync.Pool
	pubOverlayHeapPool sync.Pool
	pufBaseHeapPool    sync.Pool
	pubBaseHeapPool    sync.Pool
	packedPathPool     sync.Pool
	unpackedPathPool   sync.Pool
	pathCoordsPool     sync.Pool
	customizer         Customizer
	costFunction       CostFunction

	unpackerWorkers                     int
	unpackerForAlternativeRoutesWorkers int
}

func NewCRPRoutingEngine(graph *da.Graph,
	overlayGraph *da.OverlayGraph, metrics *met.Metric,
	logger *zap.Logger, puCache *ristretto.Cache[[]byte, []da.Index],
	customizer Customizer, costFunction CostFunction, lm *landmark.Landmark) *CRPRoutingEngine {
	e := &CRPRoutingEngine{
		graph:        graph,
		metrics:      metrics,
		overlayGraph: overlayGraph,
		logger:       logger,
		puCache:      puCache,
		customizer:   customizer,
		costFunction: costFunction,
		lm:           lm,
	}
	e.BuildQueryHeapPool()
	e.initParameter()
	return e
}

func (crp *CRPRoutingEngine) GetGraph() *da.Graph {
	return crp.graph
}

func (crp *CRPRoutingEngine) GetOverlayGraph() *da.OverlayGraph {
	return crp.overlayGraph
}

func (crp *CRPRoutingEngine) GetMetrics() *met.Metric {
	return crp.metrics
}

func (crp *CRPRoutingEngine) BuildQueryHeapPool() {
	maxEdgesInCell := crp.graph.GetMaxEdgesInCell()
	numberOfOverlayVertices := crp.overlayGraph.NumberOfOverlayVertices()
	maxSearchSize := int(maxEdgesInCell*2) + numberOfOverlayVertices
	// crp query heap pool
	crp.fHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, int(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}
	crp.bHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, int(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	// path unpacking heap pool
	crp.pufOverlayHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index](da.OVERLAY_CELL_INFO_SIZE, int(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	crp.pubOverlayHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index](da.OVERLAY_CELL_INFO_SIZE, int(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	crp.pufBaseHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	crp.pubBaseHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	crp.packedPathPool = sync.Pool{
		New: func() any {
			s := make([]da.VertexEdgePair, 0, PACKED_PATH_SIZE)
			return s
		},
	}

	crp.unpackedPathPool = sync.Pool{
		New: func() any {
			s := make([]da.Index, 0, UNPACKED_PATH_SIZE)
			return s
		},
	}

	crp.pathCoordsPool = sync.Pool{
		New: func() any {
			pathCoords := make([]da.Coordinate, 0, UNPACKED_PATH_SIZE)
			return pathCoords
		},
	}
}

func (crp *CRPRoutingEngine) initParameter() {
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	numCpu := runtime.NumCPU()
	crp.unpackerWorkers = numCpu / 6
	crp.unpackerForAlternativeRoutesWorkers = numCpu / 6
}

func (crp *CRPRoutingEngine) GetMaxSpeed(e *da.OutEdge) float64 {
	return crp.metrics.GetMaxSpeed(e)
}

func (crp *CRPRoutingEngine) Close() {
	crp.puCache.Close()
}

func (crp *CRPRoutingEngine) DoneQuery(pathCoords []da.Coordinate) {
	pathCoords = pathCoords[:0] // reset length, tapi capacity tetep sama
	crp.pathCoordsPool.Put(pathCoords)
}

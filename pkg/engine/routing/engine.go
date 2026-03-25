package routing

import (
	"sync"

	lru "github.com/hashicorp/golang-lru/v2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type CRPRoutingEngine struct {
	graph              *da.Graph
	overlayGraph       *da.OverlayGraph
	metrics            *met.Metric
	logger             *zap.Logger
	puCache            *lru.Cache[PUCacheKey, []da.Index]
	fHeapPool          sync.Pool
	bHeapPool          sync.Pool
	pufOverlayHeapPool sync.Pool
	pubOverlayHeapPool sync.Pool
	pufBaseHeapPool    sync.Pool
	pubBaseHeapPool    sync.Pool
	packedPathPool     sync.Pool
	edgePathPool       sync.Pool
	pathPool           sync.Pool
	unpackedPathPool   sync.Pool
	customizer         Customizer
	costFunction       CostFunction
}

func NewCRPRoutingEngine(graph *da.Graph,
	overlayGraph *da.OverlayGraph, metrics *met.Metric,
	logger *zap.Logger, puCache *lru.Cache[PUCacheKey, []da.Index],
	customizer Customizer, costFunction CostFunction) *CRPRoutingEngine {
	e := &CRPRoutingEngine{
		graph:        graph,
		metrics:      metrics,
		overlayGraph: overlayGraph,
		logger:       logger,
		puCache:      puCache,
		customizer:   customizer,
		costFunction: costFunction,
	}
	e.BuildQueryHeapPool()
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

	crp.fHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell), da.TWO_LEVEL_STORAGE, true)
		},
	}
	crp.bHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell), da.TWO_LEVEL_STORAGE, true)
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
			s := make([]da.VertexEdgePair, 0, 1024)
			return s
		},
	}

	crp.edgePathPool = sync.Pool{
		New: func() any {
			edgePath := make([]da.OutEdge, 0, 1024)
			return edgePath
		},
	}

	crp.pathPool = sync.Pool{
		New: func() any {
			path := make([]da.Coordinate, 0, 1024)
			return path
		},
	}

	crp.unpackedPathPool = sync.Pool{
		New: func() any {
			s := make([]da.Index, 0, 1024)
			return s
		},
	}
}

func (crp *CRPRoutingEngine) GetMaxSpeed(e *da.OutEdge) float64 {
	return crp.metrics.GetMaxSpeed(e)
}

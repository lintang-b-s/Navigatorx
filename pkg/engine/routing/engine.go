package routing

import (
	"sync"

	lru "github.com/hashicorp/golang-lru/v2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type CRPRoutingEngine struct {
	graph        *da.Graph
	overlayGraph *da.OverlayGraph
	metrics      *met.Metric
	logger       *zap.Logger
	puCache      *lru.Cache[PUCacheKey, []da.Index]
	fBufPool     sync.Pool
	bBufPool     sync.Pool
	customizer   Customizer
	costFunction CostFunction
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
	e.BuildBufferPool()
	return e
}

func (crp *CRPRoutingEngine) GetGraph() *da.Graph {
	return crp.graph
}

func (crp *CRPRoutingEngine) GetOverlayGraph() *da.OverlayGraph {
	return crp.overlayGraph
}

func (crp *CRPRoutingEngine) BuildBufferPool() {
	maxEdgesInCell := crp.graph.GetMaxEdgesInCell()

	crp.fBufPool = sync.Pool{
		New: func() any {
			return NewTwoLevelStorage[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell))
		},
	}
	crp.bBufPool = sync.Pool{
		New: func() any {
			return NewTwoLevelStorage[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell))
		},
	}
}

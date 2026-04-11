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
	pufBaseHeapPool    sync.Pool
	packedPathPool     sync.Pool
	unpackedPathPool   sync.Pool
	stallingEntryPool  sync.Pool
	stallingExitPool   sync.Pool

	customizer   Customizer
	costFunction CostFunction

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
	maxSearchSize := uint32(maxEdgesInCell*2) + uint32(numberOfOverlayVertices)
	// crp query heap pool
	crp.fHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}
	crp.bHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	// path unpacking heap pool
	crp.pufOverlayHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index](da.OVERLAY_CELL_INFO_SIZE, uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	crp.pufBaseHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](uint32(maxEdgesInCell)*2, uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
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

	crp.stallingEntryPool = sync.Pool{
		New: func() any {
			stallingEntry := make([]float64, maxEdgesInCell*2)
			return stallingEntry
		},
	}

	crp.stallingExitPool = sync.Pool{
		New: func() any {
			stallingExit := make([]float64, maxEdgesInCell*2)
			return stallingExit
		},
	}

}

func (crp *CRPRoutingEngine) initParameter() {
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	numCpu := runtime.NumCPU()
	crp.unpackerWorkers = numCpu / 6
	crp.unpackerForAlternativeRoutesWorkers = numCpu / 6
}

func (crp *CRPRoutingEngine) GetMaxSpeed(e da.OutEdge) float64 {
	return crp.metrics.GetMaxSpeed(&e)
}

func (crp *CRPRoutingEngine) Close() {
	crp.puCache.Close()
}

// GetWeight. get weight of outEdge/inEdge
func (crp *CRPRoutingEngine) GetWeight(eId da.Index, outEdge bool) float64 {
	if outEdge {
		eDefaultWeight, eLength, eHighwayType := crp.graph.GetOutEdgeTripleWeight(eId)
		return crp.metrics.GetWeight(eHighwayType, eDefaultWeight, eLength)
	}
	eDefaultWeight, eLength, eHighwayType := crp.graph.GetInEdgeTripleWeight(eId)
	return crp.metrics.GetWeight(eHighwayType, eDefaultWeight, eLength)
}

func (crp *CRPRoutingEngine) GetWeightFromLength(eId da.Index, eLength float64, outEdge bool) float64 {
	if outEdge {
		eDefaultWeight, _, eHighwayType := crp.graph.GetOutEdgeTripleWeight(eId)
		return crp.metrics.GetWeight(eHighwayType, eDefaultWeight, eLength)
	}
	eDefaultWeight, _, eHighwayType := crp.graph.GetInEdgeTripleWeight(eId)
	return crp.metrics.GetWeight(eHighwayType, eDefaultWeight, eLength)
}

func (crp *CRPRoutingEngine) IsDummyOutEdge(eId da.Index) bool {
	return crp.graph.IsDummyOutEdge(eId)
}

func (crp *CRPRoutingEngine) IsDummyInEdge(eId da.Index) bool {
	return crp.graph.IsDummyInEdge(eId)
}

// Package routing provides routing algorithms and engines for finding fastest path in road networks.
package routing

import (
	"bufio"
	"fmt"
	"runtime"
	"sync"

	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type CRPRoutingEngine struct {
	graph               *da.Graph
	overlayGraph        *da.OverlayGraph
	metrics             *met.Metric
	lm                  *landmark.Landmark
	logger              *zap.Logger
	puCache             *ristretto.Cache[[]byte, []da.Index]
	verticesLookupTable *customizer.LookupTable[uint64]

	fHeapPool          sync.Pool
	bHeapPool          sync.Pool
	pufOverlayHeapPool sync.Pool
	pufBaseHeapPool    sync.Pool
	packedPathPool     sync.Pool
	unpackedPathPool   sync.Pool
	stallingEntryPool  sync.Pool
	stallingExitPool   sync.Pool
	landmarkFile       string

	unpackerWorkers                     int
	unpackerForAlternativeRoutesWorkers int
}

func NewCRPRoutingEngine(graph *da.Graph,
	overlayGraph *da.OverlayGraph, metrics *met.Metric,
	logger *zap.Logger, puCache *ristretto.Cache[[]byte, []da.Index],
	landmarkFile string, readBuf *bufio.Reader) *CRPRoutingEngine {
	var err error

	lm := landmark.NewLandmark()
	if landmarkFile != "" {
		lm, err = landmark.ReadLandmark(landmarkFile, readBuf)
		if err != nil {
			panic(fmt.Errorf("NewCRPRoutingEngine: failed to read precomputed landmark distances: %v", err))
		}
	}

	vertexOsmIds := graph.GetVertexOsmIds()
	verticesLookupTable := customizer.NewLookupTable(vertexOsmIds, func(a, b uint64) bool {
		return a < b
	})

	crp := &CRPRoutingEngine{
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

func (crp *CRPRoutingEngine) GetGraph() *da.Graph {
	return crp.graph
}

func (crp *CRPRoutingEngine) GetOverlayGraph() *da.OverlayGraph {
	return crp.overlayGraph
}

func (crp *CRPRoutingEngine) GetMetrics() *met.Metric {
	return crp.metrics
}

func (crp *CRPRoutingEngine) GetCostFunction() *costfunction.TimeFunction {
	return crp.metrics.GetCostFunction()
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
			return &s
		},
	}

	crp.unpackedPathPool = sync.Pool{
		New: func() any {
			s := make([]da.Index, 0, UNPACKED_PATH_SIZE)
			return &s
		},
	}

	crp.stallingEntryPool = sync.Pool{
		New: func() any {
			stallingEntry := make([]float64, maxEdgesInCell*2)
			return &stallingEntry
		},
	}

	crp.stallingExitPool = sync.Pool{
		New: func() any {
			stallingExit := make([]float64, maxEdgesInCell*2)
			return &stallingExit
		},
	}

}

func (crp *CRPRoutingEngine) initParameter() {
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	numCPU := runtime.NumCPU()
	crp.unpackerWorkers = numCPU / 6
	crp.unpackerForAlternativeRoutesWorkers = numCPU / 6
}

func (crp *CRPRoutingEngine) Close() {
	crp.puCache.Close()
}

// GetWeight. get weight of outEdge/inEdge
func (crp *CRPRoutingEngine) GetWeight(eId da.Index, outEdge bool) float64 {
	if !outEdge {
		eId = crp.graph.GetExitIdOfInEdge(eId)
	}
	return crp.metrics.GetWeight(eId)
}

func (crp *CRPRoutingEngine) GetWeightFromLength(eId da.Index, outEdge bool, eLength float64) float64 {
	if !outEdge {
		eId = crp.graph.GetExitIdOfInEdge(eId)
	}
	return crp.metrics.GetWeightFromLength(eId, eLength)
}

func (crp *CRPRoutingEngine) GetSegmentLength(eId da.Index, outEdge bool) float64 {
	if !outEdge {
		eId = crp.graph.GetExitIdOfInEdge(eId)
	}
	return crp.metrics.GetSegmentLength(eId)
}

// GetWeight. get speed of outEdge
func (crp *CRPRoutingEngine) GetSegmentSpeed(eId da.Index, outEdge bool) float64 {
	if outEdge {
		return crp.metrics.GetSegmentSpeed(eId)
	}

	eExitId := crp.graph.GetExitIdOfInEdge(eId)
	return crp.metrics.GetSegmentSpeed(eExitId)
}

func (crp *CRPRoutingEngine) IsDummyOutEdge(eId da.Index) bool {
	return crp.graph.IsDummyOutEdge(eId)
}

func (crp *CRPRoutingEngine) IsDummyInEdge(eId da.Index) bool {
	return crp.graph.IsDummyInEdge(eId)
}

func (crp *CRPRoutingEngine) ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, *da.Coordinates, []da.Index, bool) {
	crpQuery := NewCRPALTBidirectionalSearch(crp, 1.0)
	if reroute {
		crpQuery.SetReroute()
	}
	return crpQuery.ShortestPathSearch(sp, tp)
}

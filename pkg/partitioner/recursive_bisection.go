package partitioner

import (
	"container/list"
	"math"
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"go.uber.org/zap"
)

type RecursiveBisection struct {
	originalGraph   *datastructure.Graph
	maximumCellSize int
	finalPartition  []int // map from vertex id to partition id
	partitionCount  int
	logger          *zap.Logger
	mu              sync.Mutex
	k               float64
	unitCapacity    bool
}

func NewRecursiveBisection(graph *datastructure.Graph, maximumCellSize int, logger *zap.Logger, k int, unitCapacity bool,
) *RecursiveBisection {
	finalPartitions := make([]int, graph.NumberOfVertices())
	for i := range finalPartitions {
		finalPartitions[i] = INVALID_PARTITION_ID
	}
	return &RecursiveBisection{
		originalGraph:   graph,
		maximumCellSize: maximumCellSize,
		finalPartition:  finalPartitions,
		partitionCount:  0,
		logger:          logger,
		k:               float64(k),
		unitCapacity:    unitCapacity,
	}
}

/*
[On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf

time complexity:
let U = rb.maximumCellSize
T(N) = N^2 * \frac{N * (N-1)}{2} *c + T(N-1)
base case: N-k = U

T(N) \in O(N^4(N-U))
*/
func (rb *RecursiveBisection) Partition(initialNodeIds []datastructure.Index) {
	pg := rb.buildInitialPartitionGraph(initialNodeIds)
	queue := list.New()
	queue.PushBack(pg)
	tooSmall := func(partitionSize int) bool {
		return partitionSize < rb.maximumCellSize
	}
	if tooSmall(pg.NumberOfVertices()) {
		rb.assignFinalPartition(pg)
		return
	}

	

	type bisectionRes struct {
		partOne, partTwo *da.PartitionGraph
	}

	NewBisectionRes := func(partOne, partTwo *da.PartitionGraph) bisectionRes {
		return bisectionRes{partOne: partOne, partTwo: partTwo}
	}

	n := len(initialNodeIds)
	worstCaseNumOfOps := n - rb.maximumCellSize + 1 // m=number of vertices in current partition graph. worst case ketika hasil partisi setiap computeIflow = (m-1, 1).....
	wpInertialFlow := concurrent.NewWorkerPool[*da.PartitionGraph, bisectionRes](BISECTION_WORKERS, worstCaseNumOfOps)
	computeIflow := func(pg *da.PartitionGraph) bisectionRes {
		iflow := NewInertialFlow(pg)
		cut := iflow.computeInertialFlowDinic(SOURCE_SINK_RATE)
		partOne, partTwo := rb.applyBisection(cut, pg)
		return NewBisectionRes(partOne, partTwo)
	}

	wpInertialFlow.Start(computeIflow)
	wpInertialFlow.Wait()

	for queue.Len() > 0 {
		curPartitionGraph := queue.Remove(queue.Front()).(*datastructure.PartitionGraph)

		wpInertialFlow.AddJob(curPartitionGraph)

		res := <-wpInertialFlow.CollectResults()
		partOne := res.partOne
		partTwo := res.partTwo
		if !tooSmall(partOne.NumberOfVertices()) {
			queue.PushBack(partOne)
		} else {
			rb.assignFinalPartition(partOne)
		}
		if !tooSmall(partTwo.NumberOfVertices()) {
			queue.PushBack(partTwo)
		} else {
			rb.assignFinalPartition(partTwo)
		}
	}

	wpInertialFlow.Close()
}

func (rb *RecursiveBisection) applyBisection(cut *MinCut, graph *datastructure.PartitionGraph) (*datastructure.PartitionGraph, *datastructure.PartitionGraph) {
	var (
		partitionOne = datastructure.NewPartitionGraph(graph.NumberOfVertices() - cut.GetNumNodesInPartitionTwo())
		partitionTwo = datastructure.NewPartitionGraph(cut.GetNumNodesInPartitionTwo())
	)

	// remap id
	partOneId := datastructure.Index(0)
	partTwoId := datastructure.Index(0)

	n := graph.NumberOfVertices()
	partOneMap := make([]datastructure.Index, n)
	partTwoMap := make([]datastructure.Index, n)
	origGraphVertIdMap := make(map[datastructure.Index]datastructure.Index, n*2) // map from original graph vertex id to partition graph vertex id
	graph.ForEachVertices(func(v datastructure.PartitionVertex) {
		if v.GetOriginalVertexID() == datastructure.Index(ARTIFICIAL_SOURCE_ID) ||
			v.GetOriginalVertexID() == datastructure.Index(ARTIFICIAL_SINK_ID) {
			// skip artificial source and sink
			return
		}
		origGraphVertIdMap[v.GetOriginalVertexID()] = v.GetID()

		lat, lon := v.GetVertexCoordinate()
		if cut.GetFlag(v.GetID()) {
			newVertex := datastructure.NewPartitionVertex(partOneId, v.GetOriginalVertexID(),
				lat, lon)
			partitionOne.AddVertex(newVertex)
			partOneMap[v.GetID()] = partOneId
			partOneId++
		} else {
			newVertex := datastructure.NewPartitionVertex(partTwoId, v.GetOriginalVertexID(),
				lat, lon)
			partitionTwo.AddVertex(newVertex)
			partTwoMap[v.GetID()] = partTwoId
			partTwoId++
		}
	})

	for _, uVertex := range graph.GetVertices() { // O(V+E)
		rb.originalGraph.ForOutEdgesOfVertex(uVertex.GetOriginalVertexID(), func(e *datastructure.OutEdge, exitPoint datastructure.Index) {
			v, ok := origGraphVertIdMap[e.GetHead()]
			if !ok {
				return
			}
			u := uVertex.GetID()
			eWeight := int64(1)
			if rb.unitCapacity {
				eWeight = 1
			} else if da.Eq(e.GetWeight(), 0) {
				eWeight = 1
			} else {
				eWeight = int64(e.GetWeight() * math.Pow(10, rb.k))
			}

			if cut.GetFlag(u) && cut.GetFlag(v) {
				uPartOne := partOneMap[u]
				vPartOne := partOneMap[v]
				partitionOne.AddEdge(uPartOne, vPartOne, eWeight, false)
			} else if !cut.GetFlag(u) && !cut.GetFlag(v) {
				uPartTwo := partTwoMap[u]
				vPartTwo := partTwoMap[v]
				partitionTwo.AddEdge(uPartTwo, vPartTwo, eWeight, false)
			}
		})
	}

	return partitionOne, partitionTwo
}

func (rb *RecursiveBisection) assignFinalPartition(graph *datastructure.PartitionGraph) {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	for i := 0; i < graph.NumberOfVertices(); i++ {
		v := graph.GetVertex(datastructure.Index(i))
		originalId := v.GetOriginalVertexID()
		rb.finalPartition[originalId] = rb.partitionCount
	}
	rb.partitionCount++
}

func (rb *RecursiveBisection) buildInitialPartitionGraph(initialVerticeIds []datastructure.Index) *datastructure.PartitionGraph {
	pg := datastructure.NewPartitionGraph(len(initialVerticeIds))
	// initialVerticeIds = stil original vertex id

	initialVerticeIdSet := makeNodeSet(initialVerticeIds)

	newVid := datastructure.Index(0)
	newMapVid := make(map[datastructure.Index]datastructure.Index, len(initialVerticeIds))
	for _, vId := range initialVerticeIds {
		lat, lon := rb.originalGraph.GetVertexCoordinates(vId)
		vertex := datastructure.NewPartitionVertex(newVid, vId, lat, lon)
		newMapVid[vId] = newVid
		pg.AddVertex(vertex)
		newVid++
	}

	for _, vId := range initialVerticeIds {
		rb.originalGraph.ForOutEdgesOfVertex(vId, func(e *datastructure.OutEdge, exitPoint datastructure.Index) {
			if _, adjVertexInSet := initialVerticeIdSet[exitPoint]; !adjVertexInSet {
				// skip arc that have endpoint outside current cell
				return
			}

			eWeight := int64(1)
			if rb.unitCapacity {
				eWeight = 1
			} else if da.Eq(e.GetWeight(), 0) {
				eWeight = 1
			} else {
				eWeight = int64(e.GetWeight() * math.Pow(10, rb.k))
			}
			pg.AddEdge(newMapVid[vId], newMapVid[e.GetHead()], eWeight, false)
		})
	}

	return pg
}

func (rb *RecursiveBisection) GetFinalPartition() []int {
	return rb.finalPartition
}

func makeNodeSet(nodeIds []datastructure.Index) map[datastructure.Index]struct{} {
	set := make(map[datastructure.Index]struct{}, len(nodeIds)*2)
	for _, nodeId := range nodeIds {
		set[nodeId] = struct{}{}
	}

	return set
}

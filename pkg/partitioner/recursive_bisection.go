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
	originalGraph          *datastructure.Graph
	maximumCellSize        int
	finalPartition         []int // map from vertex id to partition id
	partitionCount         int
	logger                 *zap.Logger
	mu                     sync.Mutex
	k                      float64
	prePartitionWithSCC    bool
	unitCapacity           bool
	inertialFlowIterations int
}

func NewRecursiveBisection(graph *datastructure.Graph, maximumCellSize int, logger *zap.Logger, k int, unitCapacity, prePartitionWithSCC bool,
	inertialFlowIterations int,
) *RecursiveBisection {
	finalPartitions := make([]int, graph.NumberOfVertices())
	for i := range finalPartitions {
		finalPartitions[i] = INVALID_PARTITION_ID
	}
	return &RecursiveBisection{
		originalGraph:          graph,
		maximumCellSize:        maximumCellSize,
		finalPartition:         finalPartitions,
		partitionCount:         0,
		logger:                 logger,
		k:                      float64(k),
		unitCapacity:           unitCapacity,
		prePartitionWithSCC:    prePartitionWithSCC,
		inertialFlowIterations: inertialFlowIterations,
	}
}

/*
[On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf
(i) sort the vertices by longitude (or latitude, or some linear
combination) and (ii) compute the maximum flow from the first k nodes
(forming the source) to the last k nodes (forming the sink).  Return the
corresponding minimum cut as an edge separator (or recurse until the
resulting subgraphs are sufficiently small).

time complexity:
let U = rb.maximumCellSize. computeInertialFlowDinic is just run dinic algorithm for multiple times. dinic time complexity O(n^2 * m).
worst case ketika the graph adlh complete graph, m=n(n-1)/2 -> dinic O(n^2 * n(n-1)/2)
worst case dari inertial flow ketika dinic return minimum cut dengan partisi S, T, dengan |S|=1  dan |T|= n-1  (atau |T|=1 dan |S|=n-1).
T(n) = n^2 * \frac{n * (n-1)}{2} *c + T(n-1)
base case: n-k = U
T(n) in O(n^4(n-U))
*/
func (rb *RecursiveBisection) Partition(initialNodeIds []datastructure.Index) {

	initialPg := rb.buildInitialPartitionGraph(initialNodeIds)

	tooSmall := func(partitionSize int) bool {
		return partitionSize < rb.maximumCellSize
	}

	if tooSmall(initialPg.NumberOfVertices()) {
		rb.assignFinalPartition(initialPg)
		return
	}

	type bisectionRes struct {
		partOne, partTwo *da.PartitionGraph
	}

	NewBisectionRes := func(partOne, partTwo *da.PartitionGraph) bisectionRes {
		return bisectionRes{partOne: partOne, partTwo: partTwo}
	}

	partitionComponent := func(comp *da.PartitionGraph) bool {
		n := comp.NumberOfVertices()

		worstCaseNumOfOps := n - rb.maximumCellSize + 1
		wpInertialFlowComp := concurrent.NewWorkerPool[*da.PartitionGraph, bisectionRes](BISECTION_WORKERS, worstCaseNumOfOps)

		computeIflow := func(pg *da.PartitionGraph) bisectionRes {
			iflow := NewInertialFlow(pg, rb.inertialFlowIterations)
			cut := iflow.computeInertialFlowDinic(SOURCE_SINK_RATE) // O(n^2 * m)
			partOne, partTwo := rb.applyBisection(cut, pg)
			return NewBisectionRes(partOne, partTwo)
		}

		wpInertialFlowComp.Start(computeIflow)
		wpInertialFlowComp.Wait()

		queue := list.New()
		queue.PushBack(comp)

		for queue.Len() > 0 {
			curPartitionGraph := queue.Remove(queue.Front()).(*datastructure.PartitionGraph)

			wpInertialFlowComp.AddJob(curPartitionGraph)

			res := <-wpInertialFlowComp.CollectResults()
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

		wpInertialFlowComp.Close()

		return true
	}
	var (
		components []*da.PartitionGraph
	)

	if rb.prePartitionWithSCC {
		components = prePartitionWithSCC(initialPg, rb.maximumCellSize) // O(n+m)
	} else {
		components = append(components, initialPg)
	}

	wpInertialFlow := concurrent.NewWorkerPool[*da.PartitionGraph, bool](BISECTION_WORKERS, len(components))

	for _, component := range components {
		if tooSmall(component.NumberOfVertices()) {
			rb.assignFinalPartition(component)
			continue
		}

		wpInertialFlow.AddJob(component)
	}

	wpInertialFlow.Close()
	wpInertialFlow.Start(partitionComponent)
	wpInertialFlow.WaitDirect()
}

// applyBisection. bisect st-cut jadi partisi S dan T yang saling disjoint
func (rb *RecursiveBisection) applyBisection(cut *MinCut, pg *datastructure.PartitionGraph) (*datastructure.PartitionGraph, *datastructure.PartitionGraph) {
	var (
		partitionOne = datastructure.NewPartitionGraph(pg.NumberOfVertices() - cut.GetNumNodesInPartitionTwo())
		partitionTwo = datastructure.NewPartitionGraph(cut.GetNumNodesInPartitionTwo())
	)

	// remap id untuk partisi S dan T
	partOneId := datastructure.Index(0)
	partTwoId := datastructure.Index(0)

	n := pg.NumberOfVertices()
	partOneNewVIdMap := make([]datastructure.Index, n)
	partTwoNewVIdMapMap := make([]datastructure.Index, n)
	origVIdToCurPartVIdMap := make(map[datastructure.Index]datastructure.Index, n*2) // map from original pg vertex id to partition pg vertex id
	pg.ForEachVertices(func(v datastructure.PartitionVertex) {
		if v.GetOriginalVertexID() == datastructure.Index(ARTIFICIAL_SOURCE_ID) ||
			v.GetOriginalVertexID() == datastructure.Index(ARTIFICIAL_SINK_ID) {
			// skip artificial source and sink
			return
		}
		origVIdToCurPartVIdMap[v.GetOriginalVertexID()] = v.GetID()

		lat, lon := v.GetVertexCoordinate()
		if cut.GetFlag(v.GetID()) {
			// v in partisi S
			newVertex := datastructure.NewPartitionVertex(partOneId, v.GetOriginalVertexID(),
				lat, lon)
			partitionOne.AddVertex(newVertex)
			partOneNewVIdMap[v.GetID()] = partOneId
			partOneId++
		} else {
			// v in partisi T
			newVertex := datastructure.NewPartitionVertex(partTwoId, v.GetOriginalVertexID(),
				lat, lon)
			partitionTwo.AddVertex(newVertex)
			partTwoNewVIdMapMap[v.GetID()] = partTwoId
			partTwoId++
		}
	})

	for _, uVertex := range pg.GetVertices() { // O(V+E)
		uOriVId := uVertex.GetOriginalVertexID()
		rb.originalGraph.ForOutEdgesOfVertex(uOriVId, func(e *datastructure.OutEdge, exitPoint datastructure.Index) {
			v, ok := origVIdToCurPartVIdMap[e.GetHead()] // get vertex id di current partition graph pg
			if !ok {
				// v not in current partition Graph
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
				// v in partisi S
				uId := partOneNewVIdMap[u]
				vId := partOneNewVIdMap[v]
				partitionOne.AddEdge(uId, vId, eWeight, false)
			} else if !cut.GetFlag(u) && !cut.GetFlag(v) {
				// v in partisi T
				uId := partTwoNewVIdMapMap[u]
				vId := partTwoNewVIdMapMap[v]
				partitionTwo.AddEdge(uId, vId, eWeight, false)
			}
		})
	}

	return partitionOne, partitionTwo
}

// assignFinalPartition. assign id partisi dari setiap vertices in partitionGraph
func (rb *RecursiveBisection) assignFinalPartition(partitionGraph *datastructure.PartitionGraph) {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	for i := 0; i < partitionGraph.NumberOfVertices(); i++ {
		v := partitionGraph.GetVertex(datastructure.Index(i))
		originalId := v.GetOriginalVertexID()
		rb.finalPartition[originalId] = rb.partitionCount
	}
	rb.partitionCount++
}

/*
buildInitialPartitionGraph. build partitionGraph

initialVerticeIds = nodeIds dari original graph

return partitionGraph
partitionGraph punya vertices sama dengan vertices di initialVerticeIds, tapi dengan id baru
edges dari partitionGraph cuma include edges yang tail dan head dari edge satu partisi atau in initialVerticeIds
*/
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
			if _, adjVertexInSet := initialVerticeIdSet[e.GetHead()]; !adjVertexInSet {
				// skip arc that its head outside current cell
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

			newV := newMapVid[vId]
			newHead := newMapVid[e.GetHead()]
			pg.AddEdge(newV, newHead, eWeight, false)
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

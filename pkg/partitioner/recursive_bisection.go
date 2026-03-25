package partitioner

import (
	"math"
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type RecursiveBisection struct {
	originalGraph          *datastructure.Graph
	maximumCellSize        int
	finalPartition         []int // map from vertex id to partition id
	partitionCount         int
	numVerticesAssigned    int
	logger                 *zap.Logger
	mu                     sync.Mutex
	k                      float64
	prePartitionWithSCC    bool
	unitCapacity           bool
	inertialFlowIterations int
	directed               bool
}

func NewRecursiveBisection(graph *datastructure.Graph, maximumCellSize int, logger *zap.Logger, k int, unitCapacity, prePartitionWithSCC bool,
	inertialFlowIterations int, directed bool,
) *RecursiveBisection {

	n := graph.NumberOfVertices()
	finalPartitions := make([]int, n)
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
		directed:               directed,
	}
}

/*
[On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf
partisi road networks graph dengan cara:
(1) sort vertices by linear kombinaasi latitude & longitude
(2) compute max flow/min st-cut dari first k nodes (sources) to last k nodes(sinks) dari sortest vertices
(3) return minimum st-cut sebagai edge separator (atau recurse sampai size dari resulting subgraphs < maximumCellSize)

time complexity:
let U = rb.maximumCellSize. computeInertialFlowDinic is just run dinic algorithm for multiple times. dinic time complexity O(n^2 * m).
worst case ketika the graph adlh complete graph, m=n(n-1)/2 -> dinic O(n^2 * n(n-1)/2)
worst case dari inertial flow ketika dinic return minimum cut dengan partisi S, T, dengan |S|=1  dan |T|= n-1  (atau |T|=1 dan |S|=n-1).
T(n) = n^2 * \frac{n * (n-1)}{2} *c + T(n-1)
base case: n-k = U
T(n) = O(n^5)
*/
func (rb *RecursiveBisection) Partition(initialVerticeIds []datastructure.Index) {

	initialPg := rb.buildInitialPartitionGraph(initialVerticeIds) // O(n+m), n = len(initialVerticeIds), m = number of edges that its tail vertex in initialVerticeIds

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

	var (
		components []*da.PartitionGraph
	)

	if rb.prePartitionWithSCC {
		components = prePartitionWithSCC(initialPg, rb.maximumCellSize) // O(n+m)
	} else {
		components = append(components, initialPg)
	}

	n := len(initialVerticeIds)

	worstCaseNumOfOps := n - rb.maximumCellSize + 1
	wpInertialFlowComp := concurrent.NewWorkerPool[*da.PartitionGraph, bisectionRes](BISECTION_WORKERS, worstCaseNumOfOps)

	computeIflow := func(pg *da.PartitionGraph) bisectionRes {
		iflow := NewInertialFlow(pg, rb.inertialFlowIterations, rb.directed)
		cut := iflow.computeInertialFlowDinic(SOURCE_SINK_RATE) // O(n^2 * m), n,m = number of vertices & edges in current partition graph pg
		partOne, partTwo := rb.applyBisection(cut, pg)          // O(n+m)
		return NewBisectionRes(partOne, partTwo)
	}

	wpInertialFlowComp.Start(computeIflow)
	wpInertialFlowComp.Wait()

	numJobs := 0
	for _, component := range components {
		if tooSmall(component.NumberOfVertices()) {
			rb.assignFinalPartition(component)
			continue
		}
		wpInertialFlowComp.AddJob(component)
		numJobs++
	}

	if numJobs == 0 {
		wpInertialFlowComp.Close()
	}

	for res := range wpInertialFlowComp.CollectResults() {
		// only stop when wp.jobQueue closed && wp.jobQueue empty -> wp.results closed -> this loop terminate
		partOne := res.partOne
		partTwo := res.partTwo
		if !tooSmall(partOne.NumberOfVertices()) {
			wpInertialFlowComp.AddJob(partOne)
		} else {
			rb.assignFinalPartition(partOne) // O(p), p = number of vertitices in partition one (partOne)
		}
		if !tooSmall(partTwo.NumberOfVertices()) {
			wpInertialFlowComp.AddJob(partTwo)
		} else {
			rb.assignFinalPartition(partTwo) // O(q), q = number of vertitices in partition two (partTwo)
		}

		if rb.allVerticesAssignedToCells(n) { // O(1)
			// close wp.jobQueue, kita bisa close wp.jobQueue disini karena
			// semua vertices in initialPg sudah di assign ke rb.finalPartition
			// atau dengan kata lain setiap cells dari partisi dari initialPg berukuran kurang dari rb.maximumCellSize
			wpInertialFlowComp.Close()
		}
	}

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
	origVIdToPgVIdMap := make(map[datastructure.Index]datastructure.Index, n*2) // map from original vertex id to partition pg vertex id

	pg.ForEachVertices(func(v datastructure.PartitionVertex) { // O(n), n=number of vertices in pg
		if v.GetOriginalVertexID() == datastructure.Index(ARTIFICIAL_SOURCE_ID) ||
			v.GetOriginalVertexID() == datastructure.Index(ARTIFICIAL_SINK_ID) {
			// skip artificial source and sink
			return
		}
		origVIdToPgVIdMap[v.GetOriginalVertexID()] = v.GetID()

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

	for _, uVertex := range pg.GetVertices() { // O(n+m), m = number of edges in pgs
		uOriVId := uVertex.GetOriginalVertexID()

		rb.originalGraph.ForOutEdgesOfVertex(uOriVId, func(e *datastructure.OutEdge, exitPoint datastructure.Index) {
			v, ok := origVIdToPgVIdMap[e.GetHead()] // get vertex id di current partition graph pg
			if !ok {
				// v not in current partition Graph
				return
			}
			u := uVertex.GetID()
			eWeight := int64(1)
			if rb.unitCapacity {
				eWeight = 1
			} else if util.Eq(e.GetWeight(), 0) {
				eWeight = 1
			} else {
				eWeight = int64(e.GetWeight() * math.Pow(10, rb.k))
			}

			if cut.GetFlag(u) && cut.GetFlag(v) {
				// v in partisi S
				uId := partOneNewVIdMap[u]
				vId := partOneNewVIdMap[v]
				partitionOne.AddEdge(uId, vId, eWeight, rb.directed)
			} else if !cut.GetFlag(u) && !cut.GetFlag(v) {
				// v in partisi T
				uId := partTwoNewVIdMapMap[u]
				vId := partTwoNewVIdMapMap[v]
				partitionTwo.AddEdge(uId, vId, eWeight, rb.directed)
			}
		})
	}

	return partitionOne, partitionTwo
}

// assignFinalPartition. assign id partisi dari setiap vertices in partitionGraph
func (rb *RecursiveBisection) assignFinalPartition(partitionGraph *datastructure.PartitionGraph) {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	for i := 0; i < partitionGraph.NumberOfVertices(); i++ { // O(n), n=number of vertices in partitionGraph
		v := partitionGraph.GetVertex(datastructure.Index(i))
		originalVId := v.GetOriginalVertexID()
		rb.finalPartition[originalVId] = rb.partitionCount
		rb.numVerticesAssigned++
	}
	rb.partitionCount++
}

// allVerticesAssignedToCells. cek jika semua vertices in comp sudah di assign ke rb.finalPartition
// atau dengan kata lain setiap cells dari partisi dari comp berukuran kurang dari rb.maximumCellSize
// index dari rb.finalPartition adalah vertex Id dari original road network graph (bukan comp)
func (rb *RecursiveBisection) allVerticesAssignedToCells(n int) bool {

	if rb.numVerticesAssigned == n {
		return true
	}

	return false
}

/*
buildInitialPartitionGraph. build partitionGraph

initialVerticeIds = nodeIds dari original graph

return partitionGraph
partitionGraph punya vertices sama dengan vertices di initialVerticeIds, tapi dengan id baru
edges dari partitionGraph cuma include edges yang tail dan head dari edge satu partisi atau in initialVerticeIds
*/
func (rb *RecursiveBisection) buildInitialPartitionGraph(initialVerticeIds []datastructure.Index) *datastructure.PartitionGraph {
	n := len(initialVerticeIds)
	pg := datastructure.NewPartitionGraph(n)
	// initialVerticeIds = stil original vertex id

	initialVerticeIdSet := makeNodeSet(initialVerticeIds) // O(n), n= len(initialVerticeIds)

	newVid := datastructure.Index(0)
	newMapVid := make(map[datastructure.Index]datastructure.Index, len(initialVerticeIds))
	for _, vId := range initialVerticeIds { // O(n)
		lat, lon := rb.originalGraph.GetVertexCoordinates(vId)
		vertex := datastructure.NewPartitionVertex(newVid, vId, lat, lon)
		newMapVid[vId] = newVid
		pg.AddVertex(vertex)
		newVid++
	}

	for _, vId := range initialVerticeIds { // O(n+m), m = number of edges that its tail vertex in initialVerticeIds
		rb.originalGraph.ForOutEdgesOfVertex(vId, func(e *datastructure.OutEdge, exitPoint datastructure.Index) {
			if _, headInSet := initialVerticeIdSet[e.GetHead()]; !headInSet {
				// skip arc that its head outside current cell
				return
			}

			eWeight := int64(1)
			if rb.unitCapacity {
				eWeight = 1
			} else if util.Eq(e.GetWeight(), 0) {
				eWeight = 1
			} else {
				eWeight = int64(e.GetWeight() * math.Pow(10, rb.k))
			}

			newV := newMapVid[vId]
			newHead := newMapVid[e.GetHead()]
			pg.AddEdge(newV, newHead, eWeight, rb.directed)
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

package partitioner

import (
	"context"
	"math"
	"sync"

	"github.com/bytedance/gopkg/util/gopool"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type RecursiveBisection struct {
	originalGraph          *da.Graph
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

func NewRecursiveBisection(graph *da.Graph, maximumCellSize int, logger *zap.Logger, k int, unitCapacity, prePartitionWithSCC bool,
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
func (rb *RecursiveBisection) Partition(initialVerticeIds []da.Index) {

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

	wg := sync.WaitGroup{}

	worstCaseNumOfOps := n - rb.maximumCellSize + 1
	iflowInChan := make(chan *da.PartitionGraph, worstCaseNumOfOps)
	iflowOutChan := make(chan bisectionRes, worstCaseNumOfOps)

	computeIflow := func() {
		for pg := range iflowInChan {
			iflow := NewInertialFlow(pg, rb.inertialFlowIterations, rb.directed)
			cut := iflow.computeInertialFlowDinic(SOURCE_SINK_RATE) // O(n^2 * m), n,m = number of vertices & edges in current partition graph pg
			partOne, partTwo := rb.applyBisection(cut, pg)          // O(n+m)
			iflowOutChan <- NewBisectionRes(partOne, partTwo)
		}
	}

	for i := 0; i < BISECTION_WORKERS; i++ {
		gopool.CtxGo(context.Background(), computeIflow)
	}

	numJobs := 0
	for _, component := range components {
		if tooSmall(component.NumberOfVertices()) {
			rb.assignFinalPartition(component)
			continue
		}
		iflowInChan <- component
		numJobs++
	}
	wg.Add(1)

	if numJobs == 0 {

		close(iflowInChan)
		wg.Done()
	}

	go func() {
		wg.Wait()
		close(iflowOutChan)
	}()

	for res := range iflowOutChan {
		// only stop when wp.jobQueue closed && wp.jobQueue empty -> wp.results closed -> this loop terminate
		partOne := res.partOne
		partTwo := res.partTwo

		if !tooSmall(partOne.NumberOfVertices()) {
			iflowInChan <- partOne

		} else {
			rb.assignFinalPartition(partOne) // O(p), p = number of vertitices in partition one (partOne)
		}
		if !tooSmall(partTwo.NumberOfVertices()) {
			iflowInChan <- partTwo
		} else {
			rb.assignFinalPartition(partTwo) // O(q), q = number of vertitices in partition two (partTwo)
		}

		if rb.allVerticesAssignedToCells(n) { // O(1)
			// close wp.jobQueue, kita bisa close wp.jobQueue disini karena
			// semua vertices in initialPg sudah di assign ke rb.finalPartition
			// atau dengan kata lain setiap cells dari partisi dari initialPg berukuran kurang dari rb.maximumCellSize
			close(iflowInChan)
			wg.Done()
		}
	}

}

// applyBisection. bisect st-cut jadi partisi S dan T yang saling disjoint
func (rb *RecursiveBisection) applyBisection(cut *MinCut, pg *da.PartitionGraph) (*da.PartitionGraph, *da.PartitionGraph) {
	var (
		partitionOne = da.NewPartitionGraph(pg.NumberOfVertices() - cut.GetNumNodesInPartitionTwo())
		partitionTwo = da.NewPartitionGraph(cut.GetNumNodesInPartitionTwo())
	)

	// remap id untuk partisi S dan T
	partOneId := da.Index(0)
	partTwoId := da.Index(0)

	n := pg.NumberOfVertices()
	partOneNewVIdMap := make([]da.Index, n)
	partTwoNewVIdMapMap := make([]da.Index, n)
	origVIdToPgVIdMap := make(map[da.Index]da.Index, n*2) // map from original vertex id to partition pg vertex id

	pg.ForEachVertices(func(v da.PartitionVertex) { // O(n), n=number of vertices in pg
		if v.GetOriginalVertexID() == da.Index(ARTIFICIAL_SOURCE_ID) ||
			v.GetOriginalVertexID() == da.Index(ARTIFICIAL_SINK_ID) {
			// skip artificial source and sink
			return
		}
		origVIdToPgVIdMap[v.GetOriginalVertexID()] = v.GetID()

		lat, lon := v.GetVertexCoordinate()
		if cut.GetFlag(v.GetID()) {
			// v in partisi S
			newVertex := da.NewPartitionVertex(partOneId, v.GetOriginalVertexID(),
				lat, lon)
			partitionOne.AddVertex(newVertex)
			partOneNewVIdMap[v.GetID()] = partOneId
			partOneId++
		} else {
			// v in partisi T
			newVertex := da.NewPartitionVertex(partTwoId, v.GetOriginalVertexID(),
				lat, lon)
			partitionTwo.AddVertex(newVertex)
			partTwoNewVIdMapMap[v.GetID()] = partTwoId
			partTwoId++
		}
	})

	for _, uVertex := range pg.GetVertices() { // O(n+m), m = number of edges in pgs
		uOriVId := uVertex.GetOriginalVertexID()

		rb.originalGraph.ForOutEdgesOfVertex(uOriVId, func(head, exitPoint da.Index, weight float64) {
			v, ok := origVIdToPgVIdMap[head] // get vertex id di current partition graph pg
			if !ok {
				// v not in current partition Graph
				return
			}
			u := uVertex.GetID()
			eWeight := int64(1)
			if rb.unitCapacity {
				eWeight = 1
			} else if util.Eq(weight, 0) {
				eWeight = 1
			} else {
				eWeight = int64(weight * math.Pow(10, rb.k))
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
func (rb *RecursiveBisection) assignFinalPartition(partitionGraph *da.PartitionGraph) {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	for i := 0; i < partitionGraph.NumberOfVertices(); i++ { // O(n), n=number of vertices in partitionGraph
		v := partitionGraph.GetVertex(da.Index(i))
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
func (rb *RecursiveBisection) buildInitialPartitionGraph(initialVerticeIds []da.Index) *da.PartitionGraph {
	n := len(initialVerticeIds)
	pg := da.NewPartitionGraph(n)
	// initialVerticeIds = stil original vertex id

	initialVerticeIdSet := makeNodeSet(initialVerticeIds) // O(n), n= len(initialVerticeIds)

	newVid := da.Index(0)
	newMapVid := make(map[da.Index]da.Index, len(initialVerticeIds))
	for _, vId := range initialVerticeIds { // O(n)
		lat, lon := rb.originalGraph.GetVertexCoordinates(vId)
		vertex := da.NewPartitionVertex(newVid, vId, lat, lon)
		newMapVid[vId] = newVid
		pg.AddVertex(vertex)
		newVid++
	}

	for _, vId := range initialVerticeIds { // O(n+m), m = number of edges that its tail vertex in initialVerticeIds
		rb.originalGraph.ForOutEdgesOfVertex(vId, func(head, exitPoint da.Index, weight float64) {
			if _, headInSet := initialVerticeIdSet[head]; !headInSet {
				// skip arc that its head outside current cell
				return
			}

			eWeight := int64(1)
			if rb.unitCapacity {
				eWeight = 1
			} else if util.Eq(weight, 0) {
				eWeight = 1
			} else {
				eWeight = int64(weight * math.Pow(10, rb.k))
			}

			newV := newMapVid[vId]
			newHead := newMapVid[head]
			pg.AddEdge(newV, newHead, eWeight, rb.directed)
		})
	}

	return pg
}

func (rb *RecursiveBisection) GetFinalPartition() []int {
	return rb.finalPartition
}

func makeNodeSet(nodeIds []da.Index) map[da.Index]struct{} {
	set := make(map[da.Index]struct{}, len(nodeIds)*2)
	for _, nodeId := range nodeIds {
		set[nodeId] = struct{}{}
	}

	return set
}

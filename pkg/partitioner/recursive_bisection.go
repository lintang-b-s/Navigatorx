package partitioner

import (
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
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
	prePartitionWithSCC    bool
	inertialFlowIterations int
	directed               bool
}

func NewRecursiveBisection(graph *da.Graph, maximumCellSize int, logger *zap.Logger, prePartitionWithSCC bool,
	inertialFlowIterations int, directed bool,
) *RecursiveBisection {

	n := graph.NumberOfVertices()
	finalPartitions := make([]int, n)
	for i := range finalPartitions {
		finalPartitions[i] = INVALID_PARTITION_ID
	}

	return &RecursiveBisection{
		originalGraph:   graph,
		maximumCellSize: maximumCellSize,
		finalPartition:  finalPartitions,
		partitionCount:  0,
		logger:          logger,

		prePartitionWithSCC:    prePartitionWithSCC,
		inertialFlowIterations: inertialFlowIterations,
		directed:               directed,
	}
}

/*
ref1: [On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf
partisi road networks graph dengan cara: (b = parameter balance)
(1) sort vertices by linear kombinaasi latitude & longitude
(2) compute max flow/st-mincut dari first k=n*b nodes (sources) to last k=n*b nodes(sinks) dari sortest vertices
(3) return st-mincut sebagai edge separator (atau recurse sampai size dari resulting subgraphs < maximumCellSize U).

time complexity:
let U = rb.maximumCellSize. computeInertialFlowDinic is just run dinic algorithm for multiple times.

ref2: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf

time complexity dinic algorithm on unit capacity graph::
see lemama 4.2 ref2, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})
karena di implementasi inertial flow ini kita selalu pakai unit capacity..
let T_d(n)=worst case time complexity dinic algorithm on unit capacity graph pada graph n vertices dan m edges = O(min{m * sqrt(m), m * n^(2/3)})
b=SOURCE_SINK_RATE atau parameter balance b dari algoritma inertial flow ref1. 0<b<=1/2
worst case ketika hasil st b-balanced mincut selalu |S|=b*n, |T|=(1-b)*n

T(n)=T(n*(1-b)) + T(b*n) +T_d(n)
	<= 2T(n*(1-b))+T_d(n)
	<= 2^2 T(n(1-b)^2) + 2T_d(n(1-b)) + T_d(n)
	<= 2^k * T_d(n) + 2^k*T(n(1-b)^k)


base case T(U)=O(1)
k=log_{1/(1-b)} (n/U)

T(n)=O(2^{log_{1/(1-b)} n} * T_d(n))

*/ //
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

	iflowInChan := make(chan *da.PartitionGraph, InertialFlowChanSize)
	iflowOutChan := make(chan bisectionRes, InertialFlowChanSize)

	computeIflow := func() {
		for pg := range iflowInChan {
			iflow := NewInertialFlow(pg, rb.inertialFlowIterations, rb.directed)
			cut := iflow.computeInertialFlowDinic(SOURCE_SINK_RATE) // O(n^2 * m), n,m = number of vertices & edges in current partition graph pg
			partOne, partTwo := rb.applyBisection(cut, pg)          // O(n+m)
			iflowOutChan <- NewBisectionRes(partOne, partTwo)
		}
	}

	for i := 0; i < BISECTION_WORKERS; i++ {
		go computeIflow()
	}

	queue := make([]*da.PartitionGraph, 0, len(components))
	numJobs := 0
	for _, component := range components {
		if tooSmall(component.NumberOfVertices()) {
			rb.assignFinalPartition(component)
			continue
		}
		queue = append(queue, component)
		numJobs++
	}

	if numJobs == 0 {
		close(iflowInChan)
	}

	numUncompletedJob := 0

	for len(queue) > 0 || numUncompletedJob > 0 {
		var (
			job     *da.PartitionGraph
			jobChan chan *da.PartitionGraph
		)

		if len(queue) > 0 {
			job = queue[0]
			queue = queue[1:]
			jobChan = iflowInChan
		}

		select {
		case jobChan <- job:
			numUncompletedJob++
		case res := <-iflowOutChan:
			// only stop when wp.jobQueue closed && wp.jobQueue empty -> wp.results closed -> this loop terminate
			partOne := res.partOne
			partTwo := res.partTwo

			if !tooSmall(partOne.NumberOfVertices()) {
				queue = append(queue, partOne)
			} else {
				rb.assignFinalPartition(partOne) // O(p), p = number of vertices in partition one (partOne)
			}
			if !tooSmall(partTwo.NumberOfVertices()) {
				queue = append(queue, partTwo)
			} else {
				rb.assignFinalPartition(partTwo) // O(q), q = number of vertices in partition two (partTwo)
			}

			numUncompletedJob--
		}
	}

	close(iflowInChan)
	close(iflowOutChan)
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

		rb.originalGraph.ForOutEdgesOfVertex(uOriVId, func(head, exitPoint da.Index) {
			v, ok := origVIdToPgVIdMap[head] // get vertex id di current partition graph pg
			if !ok {
				// v not in current partition Graph
				return
			}
			u := uVertex.GetID()
			eWeight := int64(1)

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
	if partitionGraph.NumberOfVertices() == 0 {
		return
	}
	for i := 0; i < partitionGraph.NumberOfVertices(); i++ { // O(n), n=number of vertices in partitionGraph
		v := partitionGraph.GetVertex(da.Index(i))
		originalVId := v.GetOriginalVertexID()
		rb.finalPartition[originalVId] = rb.partitionCount
		rb.numVerticesAssigned++
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
		rb.originalGraph.ForOutEdgesOfVertex(vId, func(head, exitPoint da.Index) {
			if _, headInSet := initialVerticeIdSet[head]; !headInSet {
				// skip arc that its head outside current cell
				return
			}

			eWeight := int64(1)

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

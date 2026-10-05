package partitioner

import (
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type RecursiveBisection struct {
	g                      *da.Graph
	maximumCellSize        int
	finalPartition         []int // map from vertex id to partition id
	partitionCount         int
	numVerticesAssigned    int
	logger                 *zap.Logger
	mu                     sync.Mutex
	inertialFlowIterations int
	directed               bool
	progress               *util.Progress
}

func (rb *RecursiveBisection) setProgress(progress *util.Progress) {
	rb.progress = progress
}

func NewRecursiveBisection(graph *da.Graph, maximumCellSize int, logger *zap.Logger,
	inertialFlowIterations int, directed bool,
) *RecursiveBisection {

	n := graph.NumberOfVertices()
	finalPartitions := make([]int, n)
	for i := range finalPartitions {
		finalPartitions[i] = INVALID_PARTITION_ID
	}

	return &RecursiveBisection{
		g:                      graph,
		maximumCellSize:        maximumCellSize,
		finalPartition:         finalPartitions,
		partitionCount:         0,
		logger:                 logger,
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
see lemma 4.2 ref2, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})
karena di implementasi inertial flow ini kita selalu pakai unit capacity..
in a typical road network, average degree of any vertex ~ 2. so, m = Theta(n)
let T_d(n)=worst case time complexity dinic algorithm on unit capacity graph pada road network graph n vertices dan m edges = O(min{n * sqrt(n), n * n^(2/3)})=O(n^{3/2})
b=SOURCE_SINK_RATE atau parameter balance b dari algoritma inertial flow ref1. 0<b<=1/2
worst case ketika hasil st b-balanced mincut selalu |S|=b*n, |T|=(1-b)*n

O(n*sqrt(n)*log_{1/(1-b)}n)

*/ // nolint: gofmt
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

	iflowInChan := make(chan *da.PartitionGraph, InertialFlowChanSize)
	iflowOutChan := make(chan bisectionRes, InertialFlowChanSize)

	computeIflow := func() {
		for pg := range iflowInChan {
			iflow := NewInertialFlow(pg, rb.inertialFlowIterations, rb.directed)
			cut := iflow.computeInertialFlowDinic(SOURCE_SINK_RATE) // O(min{m * sqrt(m), m * n^(2/3)})) dinic on unit capacity graph, n,m = number of vertices & edges in current partition graph pg
			partOne, partTwo := rb.applyBisection(cut, pg)          // O(n+m)
			iflowOutChan <- NewBisectionRes(partOne, partTwo)
		}
	}

	for i := 0; i < BISECTION_WORKERS; i++ {
		go computeIflow()
	}

	queue := make([]*da.PartitionGraph, 0, 10)
	numJobs := 0
	if tooSmall(initialPg.NumberOfVertices()) {
		rb.assignFinalPartition(initialPg)
		return
	}
	queue = append(queue, initialPg)
	numJobs++

	if numJobs == 0 {
		close(iflowInChan)
	}

	numUncompletedJob := 0

	for len(queue) > 0 || numUncompletedJob > 0 {
		var (
			pg     *da.PartitionGraph
			inChan chan *da.PartitionGraph
		)
		// https://go.dev/talks/2013/advconc.slide#30
		// https://go.dev/talks/2013/advconc.slide#31
		if len(queue) > 0 {
			pg = queue[0]
			inChan = iflowInChan
		}

		select {
		case inChan <- pg: // enable send only when queue is non-empty (https://go.dev/talks/2013/advconc.slide#30)
			numUncompletedJob++
			queue = queue[1:]
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
	povId := da.Index(0)
	ptvId := da.Index(0)

	n := pg.NumberOfVertices()
	partOneNewVIdMap := make([]da.Index, n)
	partTwoNewVIdMapMap := make([]da.Index, n)
	origVIdToPgVIdMap := make(map[da.Index]da.Index, n*2) // map from original vertex id to partition pg vertex id

	pg.ForEachVertices(func(v da.PartitionVertex) { // O(n), n=number of vertices in pg
		vId := v.GetOriginalVertexID()
		if vId == da.Index(ARTIFICIAL_SOURCE_ID) ||
			vId == da.Index(ARTIFICIAL_SINK_ID) {
			// skip artificial source and sink
			return
		}
		origVIdToPgVIdMap[vId] = v.GetID()

		lat, lon := v.GetVertexCoordinate()
		if cut.GetFlag(v.GetID()) {
			// v in partisi S
			newVertex := da.NewPartitionVertex(povId, vId,
				lat, lon)
			partitionOne.AddVertex(newVertex)
			pg.InitAdjListDeg(povId, rb.g.GetOutDegree(vId))
			partOneNewVIdMap[v.GetID()] = povId
			povId++
		} else {
			// v in partisi T
			newVertex := da.NewPartitionVertex(ptvId, vId,
				lat, lon)
			partitionTwo.AddVertex(newVertex)
			pg.InitAdjListDeg(ptvId, rb.g.GetOutDegree(vId))
			partTwoNewVIdMapMap[v.GetID()] = ptvId
			ptvId++
		}
	})

	for _, uVertex := range pg.GetVertices() { // O(m), m = number of edges in pgs
		uOriVId := uVertex.GetOriginalVertexID()

		rb.g.ForOutEdgesOf(uOriVId, func(eId, head, _ da.Index) {
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
	rb.progress.Add(partitionGraph.NumberOfVertices())
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

	nvId := da.Index(0)
	newMapVid := make(map[da.Index]da.Index, len(initialVerticeIds))
	for _, vId := range initialVerticeIds { // O(n)
		lat, lon := rb.g.GetVertexCoordinates(vId)
		vertex := da.NewPartitionVertex(nvId, vId, lat, lon)
		newMapVid[vId] = nvId
		pg.AddVertex(vertex)
		pg.InitAdjListDeg(nvId, rb.g.GetOutDegree(vId))
		nvId++
	}

	for _, vId := range initialVerticeIds { // O(m), m = number of edges that its tail vertex in initialVerticeIds
		rb.g.ForOutEdgesOf(vId, func(eId da.Index, head da.Index, _ da.Index) {
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

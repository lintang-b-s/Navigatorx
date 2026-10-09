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
	showProgress           bool
	progress               *util.Progress
}

func NewRecursiveBisection(graph *da.Graph, maximumCellSize int, logger *zap.Logger,
	inertialFlowIterations int, showProgress bool,
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
		showProgress:           showProgress,
	}
}

/*
ref1: [On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf
partisi road networks graph dengan cara: (b = parameter balance)
(1) sort vertices by linear kombinaasi latitude & longitude
(2) compute max flow/minimum st-cut dari first k=n*b nodes (sources) to last k=n*b nodes(sinks) dari sorted vertices
(3) return minimum st-cut sebagai edge separator (atau recurse sampai size dari resulting subgraphs < maximumCellSize U).

inspired by: https://github.com/Project-OSRM/osrm-backend/issues/3205 and https://github.com/Project-OSRM/osrm-backend/issues/3586
https://github.com/Project-OSRM/osrm-backend/blob/master/src/partitioner/recursive_bisection.cpp

time complexity:
ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf

time complexity dinic algorithm on unit capacity graph::
see lemma 4.2 ref1, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})
karena di implementasi inertial flow ini kita selalu pakai unit capacity..
in a typical road network, average out degree of any vertex ~ 2.43 (see DIMACS USA road network graph: https://www.diag.uniroma1.it/challenge9/download.shtml). so, m = Theta(n)
let T_d(n)=worst case time complexity dinic algorithm on unit capacity graph pada road network graph n vertices dan m edges = O(min{n * sqrt(n), n * n^(2/3)})=O(n^{3/2})
b=SOURCE_SINK_RATE atau parameter balance b dari algoritma inertial flow ref1. 0<b<=1/2
worst case ketika hasil st b-balanced mincut selalu |S|=b*n, |T|=(1-b)*n

O(n * sqrt(n) * log_{1/(1-b)} n)

*/ // nolint: gofmt
func (rb *RecursiveBisection) Partition(mlpCellVIds []da.Index) {
	mlpcg := rb.buildMLPCellGraph(mlpCellVIds)

	tooSmall := func(partitionSize int) bool {
		return partitionSize < rb.maximumCellSize
	}

	if tooSmall(mlpcg.NumberOfCellVertices()) {
		rb.assignFinalPartition(mlpcg)
		return
	}

	type bisectionRes struct {
		cellOne, cellTwo *da.CellGraph
	}

	NewBisectionRes := func(cellOne, cellTwo *da.CellGraph) bisectionRes {
		return bisectionRes{cellOne, cellTwo}
	}

	iflowInChan := make(chan *da.CellGraph, InertialFlowChanSize)
	iflowOutChan := make(chan bisectionRes, InertialFlowChanSize)

	computeIflow := func() {
		for cg := range iflowInChan {
			iflow := NewInertialFlow(cg, rb.inertialFlowIterations)
			cut := iflow.computeInertialFlowDinic(SOURCE_SINK_RATE) // O(n*sqrt(n)) dinic on unit capacity graph, n = number of vertices in current recursive bisection cell cg
			cellOne, cellTwo := rb.applyBisection(cut, cg)          // O(n)
			iflowOutChan <- NewBisectionRes(cellOne, cellTwo)
		}
	}

	for i := 0; i < BISECTION_WORKERS; i++ {
		go computeIflow()
	}

	queue := make([]*da.CellGraph, 0, 10)
	numJobs := 0
	queue = append(queue, mlpcg)
	numJobs++

	if numJobs == 0 {
		close(iflowInChan)
	}

	numUncompletedJob := 0

	for len(queue) > 0 || numUncompletedJob > 0 {
		var (
			cg     *da.CellGraph // recursive bisection cell graph
			inChan chan *da.CellGraph
		)
		// https://go.dev/talks/2013/advconc.slide#30
		// https://go.dev/talks/2013/advconc.slide#31
		if len(queue) > 0 {
			cg = queue[0]
			inChan = iflowInChan
		}

		select {
		case inChan <- cg: // enable send only when queue is non-empty (https://go.dev/talks/2013/advconc.slide#30)
			numUncompletedJob++
			queue = queue[1:]
		case res := <-iflowOutChan:
			// only stop when wp.jobQueue closed && wp.jobQueue empty -> wp.results closed -> this loop terminate
			cOne := res.cellOne
			cTwo := res.cellTwo

			if !tooSmall(cOne.NumberOfCellVertices()) {
				queue = append(queue, cOne)
			} else {
				rb.assignFinalPartition(cOne) // O(p), p = number of vertices in cell one (cOne)
			}
			if !tooSmall(cTwo.NumberOfCellVertices()) {
				queue = append(queue, cTwo)
			} else {
				rb.assignFinalPartition(cTwo) // O(q), q = number of vertices in cell two (cTwo)
			}

			numUncompletedJob--
		}
	}

	close(iflowInChan)
	close(iflowOutChan)

}

// applyBisection. bisect st-cut jadi cell S dan T yang saling disjoint
func (rb *RecursiveBisection) applyBisection(cut *MinCut, cg *da.CellGraph) (*da.CellGraph, *da.CellGraph) {
	one, two := cg.Partition(func(vId da.Index) bool {
		return cut.GetFlag(vId)
	})
	return one, two
}

// assignFinalPartition. assign id cell dari setiap vertices in cellGraph
func (rb *RecursiveBisection) assignFinalPartition(cellGraph *da.CellGraph) {
	rb.mu.Lock()
	defer rb.mu.Unlock()
	if cellGraph.NumberOfCellVertices() == 0 {
		return
	}

	cellGraph.ForEachCellVertices(func(_, gvId da.Index, _ da.Coordinate) {
		rb.finalPartition[gvId] = rb.partitionCount
		rb.numVerticesAssigned++
	})

	if rb.showProgress {
		rb.progress.Add(cellGraph.NumberOfCellVertices())
	}
	rb.partitionCount++
}

// buildMLPCellGraph. build initial MLP cell cellGraph.
// mlpCellVIds is the MultiLevelPartition (MLP) Cell vertices ids.
func (rb *RecursiveBisection) buildMLPCellGraph(mlpCellVIds []da.Index) *da.CellGraph {
	m := da.Index(len(mlpCellVIds))
	n := rb.g.NumberOfVertices()
	gv := make([]da.Index, n)
	for i, v := range mlpCellVIds {
		gv[v] = da.Index(i)
	}

	cg := da.NewCellGraph(rb.g, mlpCellVIds, mlpCellVIds, gv, 0, m)
	return cg
}

func (rb *RecursiveBisection) GetFinalPartition() []int {
	return rb.finalPartition
}

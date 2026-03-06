package partitioner

import (
	"math"
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type minCutJob struct {
	line []float64
}

func newMinCutJob(line []float64) minCutJob {
	return minCutJob{line: line}
}

func (mj minCutJob) getLine() []float64 {
	return mj.line
}

type inertialFlow struct {
	graph      *datastructure.PartitionGraph
	iterations int
}

func NewInertialFlow(graph *datastructure.PartitionGraph, iterations int) *inertialFlow {
	return &inertialFlow{graph: graph, iterations: iterations}
}

func (inf *inertialFlow) getPartitionGraph() *datastructure.PartitionGraph {
	return inf.graph
}

/*
computeInertialFlowDinic.
[On Balanced Separators in Road Networks, Schild, et al.] https://aschild.github.io/papers/roadseparator.pdf

(i) sort the vertices by longitude (or latitude, or some linear
combination) and (ii) compute the maximum flow from the first k nodes
(forming the source) to the last k nodes (forming the sink).

return st-mincut dengan partisi S, T yang saling disjoint.
time complexity:
karena cuma call algoritma dinic berkali kali sejumlah iterations, let k = number of iterations+2
O(k*n^2*m).
*/
func (inf *inertialFlow) computeInertialFlowDinic(sourceSinkRate float64) *MinCut {
	var (
		best                    *MinCut = &MinCut{}
		bestNumberOfMinCutEdges         = math.MaxInt
	)

	n := inf.graph.NumberOfVertices()
	iterations := inf.iterations
	if n >= 100000 {
		iterations = INERTIAL_FLOW_ITERATION_LARGE_GRAPH
	}

	wpInertialFlow := concurrent.NewWorkerPool[minCutJob, *MinCut](INERTIAL_FLOW_WORKERS, iterations+2)

	balanceDelta := func(numPartTwoNodes int, numberOfVertices int) int {
		diff := numberOfVertices/2 - numPartTwoNodes
		if diff < 0 {
			diff = -diff
		}
		return diff
	}

	for i := 0; i < iterations; i++ {
		slope := -1 + float64(i)*float64(2/iterations) // direction vectors (-1,0), ....., (0, 1)
		wpInertialFlow.AddJob(newMinCutJob([]float64{slope, (1 - math.Abs(slope))}))
	}

	wpInertialFlow.AddJob(newMinCutJob([]float64{1, 1}))
	wpInertialFlow.AddJob(newMinCutJob([]float64{-1, 1}))

	computeMinCut := func(input minCutJob) *MinCut {
		dn := NewDinicMaxFlow(inf.getPartitionGraph(), false, true)
		var (
			sources []datastructure.Index
			sinks   []datastructure.Index
		)

		sources, sinks = dn.sortVerticesByOrthoProjection(input.getLine(), sourceSinkRate)

		s, t := dn.createArtificialSourceSink(sources, sinks)
		return dn.ComputeMaxflowMinCut(s, t) //  O(n^2 * m), n,m=number of vertices & edges dari da.PartitionGraph
	}

	wpInertialFlow.Close()
	wpInertialFlow.Start(computeMinCut)
	wpInertialFlow.Wait()

	numberOfVertices := inf.graph.NumberOfVertices()
	for minCut := range wpInertialFlow.CollectResults() {

		if minCut.GetNumOfCutEdges() < bestNumberOfMinCutEdges ||
			(bestNumberOfMinCutEdges == minCut.GetNumOfCutEdges() &&
				balanceDelta(minCut.GetNumNodesInPartitionTwo(),
					numberOfVertices) < balanceDelta(best.GetNumNodesInPartitionTwo(),
					numberOfVertices)) {
			best = minCut
			bestNumberOfMinCutEdges = minCut.GetNumOfCutEdges()
		}
	}

	return best
}

func (dn *DinicMaxFlow) sortVerticesByOrthoProjection(l []float64, ratio float64) ([]datastructure.Index, []datastructure.Index) {

	vertices := dn.graph.GetVertices()

	type item struct {
		idx        int
		dotProduct float64
	}
	n := len(vertices)

	items := make([]item, n)
	for i, v := range vertices {
		lat, lon := v.GetVertexCoordinate()
		proj := dot(lon, lat, l[0], l[1])
		items[i] = item{idx: i, dotProduct: proj}
	}

	sort.Slice(items, func(i, j int) bool {
		return items[i].dotProduct < items[j].dotProduct
	})

	endpointsLength := int(float64(n) * ratio)
	sourceNodes := make([]datastructure.Index, 0, endpointsLength)
	sinkNodes := make([]datastructure.Index, 0, endpointsLength)

	for i := 0; i < endpointsLength; i++ {
		sourceNodes = append(sourceNodes, vertices[items[i].idx].GetID())
		sinkNodes = append(sinkNodes, vertices[items[n-1-i].idx].GetID())
	}
	return sourceNodes, sinkNodes
}

func dot(x1, y1, x2, y2 float64) float64 {
	return x1*x2 + y1*y2
}

func (dn *DinicMaxFlow) createArtificialSourceSink(sourceNodes, sinkNodes []datastructure.Index) (datastructure.Index, datastructure.Index) {
	artificialSource := datastructure.Index(dn.graph.NumberOfVertices())
	artificialSink := datastructure.Index(dn.graph.NumberOfVertices() + 1)

	dn.AddArtificialVertex(datastructure.NewPartitionVertex(artificialSource, datastructure.Index(ARTIFICIAL_SOURCE_ID), 0.0, 0.0))
	dn.AddArtificialVertex(datastructure.NewPartitionVertex(artificialSink, datastructure.Index(ARTIFICIAL_SINK_ID), 0.0, 0.0))

	for _, s := range sourceNodes {
		dn.AddArtificialEdge(artificialSource, s, pkg.INF_WEIGHT_INT, false)
	}

	for _, t := range sinkNodes {
		dn.AddSinks(t)
		dn.AddArtificialEdge(t, artificialSink, pkg.INF_WEIGHT_INT, false)
	}
	return artificialSource, artificialSink
}

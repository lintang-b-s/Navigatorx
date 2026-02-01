package partitioner

import (
	"math"
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type minCutJob struct {
	slope    float64
	diagonal bool
	line     []float64
}

func newMinCutJob(slope float64, diagonal bool, line []float64) minCutJob {
	return minCutJob{slope: slope, diagonal: diagonal, line: line}
}
func (mj minCutJob) GetSlope() float64 {
	return mj.slope
}

func (mj minCutJob) isDiagonal() bool {
	return mj.diagonal
}

func (mj minCutJob) getLine() []float64 {
	return mj.line
}

type inertialFlow struct {
	graph *datastructure.PartitionGraph
}

func NewInertialFlow(graph *datastructure.PartitionGraph) *inertialFlow {
	return &inertialFlow{graph: graph}
}

func (inf *inertialFlow) getPartitionGraph() *datastructure.PartitionGraph {
	return inf.graph
}

func (inf *inertialFlow) computeInertialFlowDinic(sourceSinkRate float64) *MinCut {
	var (
		best                    *MinCut = &MinCut{}
		bestNumberOfMinCutEdges         = math.MaxInt
	)

	wpInertialFlow := concurrent.NewWorkerPool[minCutJob, *MinCut](5, 20)

	balanceDelta := func(numPartTwoNodes int, numberOfVertices int) int {
		diff := numberOfVertices/2 - numPartTwoNodes
		if diff < 0 {
			diff = -diff
		}
		return diff
	}

	for i := 0; i < INERTIAL_FLOW_ITERATION; i++ {
		slope := -1 + float64(i)*(2.0/INERTIAL_FLOW_ITERATION)
		wpInertialFlow.AddJob(newMinCutJob(slope, false, []float64{}))
	}

	wpInertialFlow.AddJob(newMinCutJob(0, true, []float64{1, 0}))
	wpInertialFlow.AddJob(newMinCutJob(0, true, []float64{0, 1}))
	wpInertialFlow.AddJob(newMinCutJob(0, true, []float64{1, 1}))
	wpInertialFlow.AddJob(newMinCutJob(0, true, []float64{1, -1}))
	wpInertialFlow.AddJob(newMinCutJob(0, true, []float64{-1, 1}))

	computeMinCut := func(input minCutJob) *MinCut {
		slope := input.GetSlope()
		dn := NewDinicMaxFlow(inf.getPartitionGraph().Clone(), false)
		var (
			sources []datastructure.Index
			sinks   []datastructure.Index
		)
		if !input.isDiagonal() {
			sources, sinks = dn.sortVerticesByLineProjection(slope, sourceSinkRate)
		} else {
			sources, sinks = dn.sortVerticesByLineDiagonalProjection(input.getLine(), sourceSinkRate)
		}

		s, t := dn.createArtificialSourceSink(sources, sinks)
		return dn.ComputeMaxflowMinCut(s, t)
	}

	wpInertialFlow.Close()
	wpInertialFlow.Start(computeMinCut)
	wpInertialFlow.Wait()

	numberOfVertices := inf.graph.NumberOfVertices()
	for minCut := range wpInertialFlow.CollectResults() {
		if minCut.GetMinCut() < bestNumberOfMinCutEdges ||
			(best.GetMinCut() == minCut.GetMinCut() &&
				balanceDelta(minCut.GetNumNodesInPartitionTwo(),
					numberOfVertices) < balanceDelta(best.GetNumNodesInPartitionTwo(),
					numberOfVertices)) {
			best = minCut
			bestNumberOfMinCutEdges = minCut.GetMinCut()
		}
	}

	return best
}

func (dn *DinicMaxFlow) sortVerticesByLineProjection(slope, ratio float64) ([]datastructure.Index, []datastructure.Index) {

	vertices := dn.graph.GetVertices()

	type item struct {
		idx        int
		projection float64
	}
	n := len(vertices)

	items := make([]item, n)
	for i, v := range vertices {
		lat, lon := v.GetVertexCoordinate()
		proj := slope*lon + (1.0-math.Abs(slope))*lat
		items[i] = item{idx: i, projection: proj}
	}

	sort.Slice(items, func(i, j int) bool {
		return items[i].projection < items[j].projection
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

func (dn *DinicMaxFlow) sortVerticesByLineDiagonalProjection(line []float64, ratio float64) ([]datastructure.Index, []datastructure.Index) {

	vertices := dn.graph.GetVertices()

	type item struct {
		idx        int
		projection float64
	}
	n := len(vertices)

	items := make([]item, n)
	for i, v := range vertices {
		lat, lon := v.GetVertexCoordinate()
		proj := line[0]*lon + line[1]*lat
		items[i] = item{idx: i, projection: proj}
	}

	sort.Slice(items, func(i, j int) bool {
		return items[i].projection < items[j].projection
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

func (ek *DinicMaxFlow) createArtificialSourceSink(sourceNodes, sinkNodes []datastructure.Index) (datastructure.Index, datastructure.Index) {
	artificialSource := datastructure.Index(ek.graph.NumberOfVertices())
	artificialSink := datastructure.Index(ek.graph.NumberOfVertices() + 1)

	ek.graph.AddVertex(datastructure.NewPartitionVertex(artificialSource, datastructure.Index(ARTIFICIAL_SOURCE_ID), 0.0, 0.0))
	ek.graph.AddVertex(datastructure.NewPartitionVertex(artificialSink, datastructure.Index(ARTIFICIAL_SINK_ID), 0.0, 0.0))

	for _, s := range sourceNodes {
		ek.graph.AddInfEdge(artificialSource, s)
	}

	for _, t := range sinkNodes {
		ek.graph.AddInfEdge(t, artificialSink)
	}
	return artificialSource, artificialSink
}

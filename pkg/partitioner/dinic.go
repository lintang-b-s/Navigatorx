package partitioner

import (
	"container/list"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type DinicMaxFlow struct {
	graph *datastructure.PartitionGraph
	debug bool
}

func NewDinicMaxFlow(graph *datastructure.PartitionGraph, debug bool) *DinicMaxFlow {
	return &DinicMaxFlow{graph: graph, debug: debug}
}

func (dmf *DinicMaxFlow) bfsLevelGraph(
	source, target datastructure.Index) bool {

	dmf.graph.ForEachVertices(func(v datastructure.PartitionVertex) {
		dmf.graph.SetVertexLevel(v.GetID(), INVALID_LEVEL)
	})

	levelQueue := list.New()
	levelQueue.PushBack(source)
	dmf.graph.SetVertexLevel(source, 0)

	for levelQueue.Len() > 0 {
		u := levelQueue.Front().Value.(datastructure.Index)
		levelQueue.Remove(levelQueue.Front())

		uLevel := dmf.graph.GetVertexLevel(u)
		level := uLevel + 1
		if u == target {
			break
		}

		dmf.graph.ForEachVertexEdges(u, func(edge *datastructure.MaxFlowEdge) {
			v := edge.GetTo()

			residual := edge.GetCapacity() - edge.GetFlow()
			if residual > 0 && dmf.graph.GetVertexLevel(v) == INVALID_LEVEL {
				dmf.graph.SetVertexLevel(v, level)
				levelQueue.PushBack(v)
			}
		})
	}
	return dmf.graph.GetVertexLevel(target) != INVALID_LEVEL
}

func (dmf *DinicMaxFlow) dfsAugmentPath(u datastructure.Index, t datastructure.Index, f int) int {
	// termination

	if u == t || f == 0 {
		return f
	}

	for ; dmf.graph.GetLastEdgeIndex(u) < dmf.graph.GetVertexEdgesSize(u); dmf.graph.IncrementLastEdgeIndex(u) {
		j := dmf.graph.GetLastEdgeIndex(u)
		edge := dmf.graph.GetEdgeOfVertex(u, j)
		v := edge.GetTo()
		residual := edge.GetCapacity() - edge.GetFlow()
		if dmf.graph.GetVertexLevel(v) != dmf.graph.GetVertexLevel(u)+1 {
			continue
		}

		if pushed := dmf.dfsAugmentPath(v, t, util.MinInt(residual, f)); pushed > 0 {
			edge.AddFlow(pushed)
			revEdge := dmf.graph.GetReversedEdgeOfVertex(u, j)
			revEdge.AddFlow(-pushed)
			return pushed
		}
	}

	return 0.0
}

func (dmf *DinicMaxFlow) resetCurrentEdges() {
	for i := 0; i < dmf.graph.NumberOfVertices(); i++ {
		dmf.graph.SetLastEdgeIndex(datastructure.Index(i), 0)
	}
}

/*
time complexity: O(N^2 * M), N,M=number of vertices & edges dari datastructure.PartitionGraph
*/
func (dmf *DinicMaxFlow) ComputeMaxflowMinCut(s datastructure.Index, t datastructure.Index) *MinCut {
	var (
		minCut = NewMinCut(dmf.graph.NumberOfVertices() - 2) // exclude artificial source and sink
	)
	maxFlow := 0

	for dmf.bfsLevelGraph(s, t) {
		dmf.resetCurrentEdges()

		for {
			flow := dmf.dfsAugmentPath(s, t, math.MaxInt)
			if flow == 0 {
				break
			}
			maxFlow += flow
		}

	}
	dmf.makeMinCutFlags(minCut, maxFlow)
	return minCut //  or maxflow

}

func (dmf *DinicMaxFlow) makeMinCutFlags(minCut *MinCut, maxflow int) {
	for u := datastructure.Index(0); u < datastructure.Index(dmf.graph.NumberOfVertices()-2); u++ {
		if dmf.graph.GetVertexLevel(u) != INVALID_LEVEL {
			minCut.SetFlag(u, true)
		} else {
			minCut.incrementNumNodesInPartitionTwo()
		}
	}
	minCut.setMinCut(maxflow)
}

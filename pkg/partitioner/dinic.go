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
			target := edge.GetTo()

			residual := edge.GetCapacity() - edge.GetFlow()
			if residual > 0 && dmf.graph.GetVertexLevel(target) == INVALID_LEVEL {
				dmf.graph.SetVertexLevel(target, level)
				levelQueue.PushBack(target)
			}
		})
	}
	return dmf.graph.GetVertexLevel(target) != INVALID_LEVEL
}

func (dmf *DinicMaxFlow) dfsAugmentPath(nodeId datastructure.Index, t datastructure.Index, maxFlow int) int {
	// termination

	if nodeId == t || maxFlow == 0 {
		return maxFlow
	}

	for ; dmf.graph.GetLastEdgeIndex(nodeId) < dmf.graph.GetVertexEdgesSize(nodeId); dmf.graph.IncrementLastEdgeIndex(nodeId) {
		j := dmf.graph.GetLastEdgeIndex(nodeId)
		edge := dmf.graph.GetEdgeOfVertex(nodeId, j)
		v := edge.GetTo()
		residual := edge.GetCapacity() - edge.GetFlow()
		if dmf.graph.GetVertexLevel(v) != dmf.graph.GetVertexLevel(nodeId)+1 {
			continue
		}

		if flow := dmf.dfsAugmentPath(v, t, util.MinInt(residual, maxFlow)); flow > 0 {
			edge.AddFlow(flow)
			revEdge := dmf.graph.GetReversedEdgeOfVertex(nodeId, j)
			revEdge.AddFlow(-flow)
			return flow
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
Dinitz, Y. (2006) “Dinitz’ Algorithm: The Original Version and Even’s Version,” in O.
Goldreich, A.L. Rosenberg, and A.L. Selman (eds.) Theoretical Computer Science:
Essays in Memory of Shimon Even. Berlin, Heidelberg: Springer, pp. 218–240.
Available at: https://doi.org/10.1007/11685654_10.

time complexity: O(N^2 * M), N,M=number of vertices & edges dari datastructure.PartitionGraph 
*/
func (dmf *DinicMaxFlow) computeMinCutSuperSourceSink(s datastructure.Index, t datastructure.Index, sources, sinks []datastructure.Index) *MinCut {
	var (
		minCut = NewMinCut(dmf.graph.NumberOfVertices() - 2) // exclude artificial source and sink
	)
	maxFlow := 0
	for {

		dmf.resetCurrentEdges()
		if dmf.bfsLevelGraph(s, t) {
			for {
				flow := dmf.dfsAugmentPath(s, t, math.MaxInt)
				if flow == 0 {
					break
				}
				maxFlow += flow
			}
		} else {

			dmf.makeMinCutFlags(minCut, maxFlow)

			return minCut
		}
	}
}

func (dmf *DinicMaxFlow) makeMinCutFlags(minCut *MinCut, numberOfMinCutEdges int) {
	for u := datastructure.Index(0); u < datastructure.Index(dmf.graph.NumberOfVertices()-2); u++ {
		if dmf.graph.GetVertexLevel(u) != INVALID_LEVEL {
			minCut.SetFlag(u, true)
		} else {
			minCut.incrementNumNodesInPartitionTwo()
		}
	}
	minCut.setNumberofMinCutEdges(numberOfMinCutEdges)
}

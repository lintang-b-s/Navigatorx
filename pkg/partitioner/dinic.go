package partitioner

import (
	"container/list"
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type DinicMaxFlow struct {
	graph            *da.PartitionGraph
	edgeFlows        []int64
	level            []int
	last             []int
	superAdjList     map[da.Index][]int     // edges from super-source to sources
	superEdgeList    map[int]da.MaxFlowEdge // edges from sinks to sinks
	debug            bool
	multSourcesSinks bool
	sinks            map[da.Index]struct{}
}

func NewDinicMaxFlow(graph *da.PartitionGraph, debug, multSourcesSinks bool) *DinicMaxFlow {
	n := graph.NumberOfVertices()

	dc := &DinicMaxFlow{graph: graph, debug: debug, last: make([]int, n), level: make([]int, n), superAdjList: make(map[da.Index][]int, 2),
		multSourcesSinks: multSourcesSinks, sinks: make(map[da.Index]struct{}), superEdgeList: make(map[int]da.MaxFlowEdge)}
	copy(dc.last, graph.GetLast())
	copy(dc.level, graph.GetLevel())

	dc.edgeFlows = make([]int64, graph.GetNumberOfEdges())
	graph.ForEdges(func(i int, e da.MaxFlowEdge) {
		dc.edgeFlows[i] = e.GetFlow()
	})

	return dc
}

func (dmf *DinicMaxFlow) AddFlow(u da.Index, idx int, f int64) {
	edgeIndex := dmf.graph.GetEdgeId(u, idx)
	dmf.edgeFlows[edgeIndex] += f
}

func (dmf *DinicMaxFlow) AddFlowToReversedEdge(u da.Index, idx int, f int64) {
	edgeIndex := dmf.graph.GetReversedEdgeId(u, idx)
	dmf.edgeFlows[edgeIndex] += f
}

func (dmf *DinicMaxFlow) AddFlowToArtificalEdge(u da.Index, idx int, f int64) {
	edgeIndex := dmf.superAdjList[u][idx]
	dmf.edgeFlows[edgeIndex] += f
}

func (dmf *DinicMaxFlow) AddFlowToReversedArtificialEdge(u da.Index, idx int, f int64) {
	edgeIndex := dmf.superAdjList[u][idx] ^ 1
	dmf.edgeFlows[edgeIndex] += f
}

func (dmf *DinicMaxFlow) GetEdgeFlow(id da.Index) int64 {
	return dmf.edgeFlows[id]
}

func (dmf *DinicMaxFlow) SetVertexLevel(u da.Index, level int) {
	dmf.level[u] = level
}
func (dmf *DinicMaxFlow) GetVertexLevel(u da.Index) int {
	return dmf.level[u]
}

func (dmf *DinicMaxFlow) GetLastEdgeIndex(u da.Index) int {
	return dmf.last[u]
}

func (dmf *DinicMaxFlow) SetLastEdgeIndex(u da.Index, idx int) {
	dmf.last[u] = idx
}

func (dmf *DinicMaxFlow) IncrementLastEdgeIndex(u da.Index) {
	dmf.last[u]++
}

func (dmf *DinicMaxFlow) AddArtificialVertex(v da.PartitionVertex) {

	if len(dmf.level) < int(v.GetID())+1 {
		dmf.level = append(dmf.level, 0)
	}
	if len(dmf.last) < int(v.GetID())+1 {
		dmf.last = append(dmf.last, 0)
	}
}

func (dmf *DinicMaxFlow) AddSinks(u da.Index) {
	dmf.sinks[u] = struct{}{}
}

func (dmf *DinicMaxFlow) AddArtificialEdge(u, v da.Index, w int64, directed bool) {
	if u == v {
		return
	}

	newEId := len(dmf.edgeFlows)
	edge := da.NewMaxFlowEdge(newEId, u, v, w)
	dmf.superEdgeList[newEId] = edge
	dmf.edgeFlows = append(dmf.edgeFlows, 0)
	dmf.superAdjList[u] = append(dmf.superAdjList[u], newEId)

	newRevEId := len(dmf.edgeFlows)
	var reverseEdge da.MaxFlowEdge = da.MaxFlowEdge{}
	if directed {
		reverseEdge = da.NewMaxFlowEdge(newRevEId, v, u, 0)
	} else {
		reverseEdge = da.NewMaxFlowEdge(newRevEId, v, u, w)
	}

	dmf.superEdgeList[newRevEId] = reverseEdge
	dmf.edgeFlows = append(dmf.edgeFlows, 0)
	dmf.superAdjList[v] = append(dmf.superAdjList[v], newRevEId)
}

func (dmf *DinicMaxFlow) ForEachEdges(u, s da.Index, handle func(e da.MaxFlowEdge)) {
	if u == s && dmf.multSourcesSinks {
		for _, edgeIdx := range dmf.superAdjList[u] {
			handle(dmf.superEdgeList[edgeIdx])
		}
	} else if dmf.InSinks(u) && dmf.multSourcesSinks {
		dmf.graph.ForEachVertexEdges(u, handle)
		for _, edgeIdx := range dmf.superAdjList[u] {
			handle(dmf.superEdgeList[edgeIdx])
		}
	} else {
		dmf.graph.ForEachVertexEdges(u, handle)
	}
}

func (dmf *DinicMaxFlow) GetEdgeOfSuperVertex(u da.Index, idx int) da.MaxFlowEdge {
	edgeIndex := dmf.superAdjList[u][idx]
	return dmf.superEdgeList[edgeIndex]
}

func (dmf *DinicMaxFlow) InSinks(u da.Index) bool {
	_, ok := dmf.sinks[u]
	return ok
}
func (dmf *DinicMaxFlow) GetSuperVertexEdgesSize(u da.Index) int {
	return len(dmf.superAdjList[u])
}

func (dmf *DinicMaxFlow) bfsLevelGraph(
	source, target da.Index) bool {
	dmf.SetVertexLevel(target, INVALID_LEVEL)

	dmf.graph.ForEachVertices(func(v da.PartitionVertex) {
		dmf.SetVertexLevel(v.GetID(), INVALID_LEVEL)
	})

	levelQueue := list.New()
	levelQueue.PushBack(source)
	dmf.SetVertexLevel(source, 0)

	for levelQueue.Len() > 0 {
		u := levelQueue.Front().Value.(da.Index)
		levelQueue.Remove(levelQueue.Front())

		uLevel := dmf.GetVertexLevel(u)
		level := uLevel + 1
		if u == target {
			break
		}

		dmf.ForEachEdges(u, source, func(edge da.MaxFlowEdge) {
			v := edge.GetTo()

			eid := da.Index(edge.GetID())
			residual := edge.GetCapacity() - dmf.GetEdgeFlow(eid)
			if residual > 0 && dmf.GetVertexLevel(v) == INVALID_LEVEL {
				dmf.SetVertexLevel(v, level)
				levelQueue.PushBack(v)
			}
		})
	}
	return dmf.GetVertexLevel(target) != INVALID_LEVEL
}

func (dmf *DinicMaxFlow) dfsAugmentPath(u da.Index, s, t da.Index, f int64) int64 {
	// termination

	if u == t || f == 0 {
		return f
	}

	numOfEdges := 0
	superEdgesOffset := 0
	if u == s && dmf.multSourcesSinks {
		numOfEdges = dmf.GetSuperVertexEdgesSize(u)
	} else if dmf.InSinks(u) && dmf.multSourcesSinks {
		numOfEdges = dmf.graph.GetVertexEdgesSize(u) + dmf.GetSuperVertexEdgesSize(u)
		superEdgesOffset = dmf.graph.GetVertexEdgesSize(u)
	} else {
		numOfEdges = dmf.graph.GetVertexEdgesSize(u)
	}

	for ; dmf.GetLastEdgeIndex(u) < numOfEdges; dmf.IncrementLastEdgeIndex(u) {
		j := dmf.GetLastEdgeIndex(u)
		var edge da.MaxFlowEdge
		artificial := false
		if u == s && dmf.multSourcesSinks {
			// u sama dengan artificial source / super-source
			edge = dmf.GetEdgeOfSuperVertex(u, j)
			artificial = true
		} else if dmf.InSinks(u) && dmf.multSourcesSinks {
			// u \in sinks
			if j >= superEdgesOffset {
				j -= superEdgesOffset
				edge = dmf.GetEdgeOfSuperVertex(u, j)
			} else {
				edge = dmf.graph.GetEdgeOfVertex(u, j)
			}
			artificial = true
		} else {
			edge = dmf.graph.GetEdgeOfVertex(u, j)
		}

		v := edge.GetTo()
		eId := edge.GetID()
		residual := edge.GetCapacity() - dmf.GetEdgeFlow(da.Index(eId))
		if dmf.GetVertexLevel(v) != dmf.GetVertexLevel(u)+1 {
			continue
		}

		if pushed := dmf.dfsAugmentPath(v, s, t, util.MinInt64(residual, f)); pushed > 0 {
			if !artificial {
				dmf.AddFlow(u, j, pushed)
				dmf.AddFlowToReversedEdge(u, j, -pushed)
			} else {
				dmf.AddFlowToArtificalEdge(u, j, pushed)
				dmf.AddFlowToReversedArtificialEdge(u, j, -pushed)
			}

			return pushed
		}
	}

	return 0.0
}

func (dmf *DinicMaxFlow) resetCurrentEdges() {
	for i := 0; i < len(dmf.last); i++ {
		dmf.SetLastEdgeIndex(da.Index(i), 0)
	}
}

/*
time complexity: O(N^2 * M), N,M=number of vertices & edges dari da.PartitionGraph
*/
func (dmf *DinicMaxFlow) ComputeMaxflowMinCut(s da.Index, t da.Index) *MinCut {
	var (
		minCut = NewMinCut(dmf.graph.NumberOfVertices()) // exclude artificial source and sink. kita cuma tambahin super source sinks di slice superEdgeList, superAdjList, dmf.level, dmf.last
	)
	maxFlow := int64(0)

	for dmf.bfsLevelGraph(s, t) {
		dmf.resetCurrentEdges()

		for {
			flow := dmf.dfsAugmentPath(s, s, t, math.MaxInt)
			if flow == 0 {
				break
			}
			maxFlow += flow
		}
	}
	dmf.makeMinCutFlags(minCut, maxFlow)
	return minCut //  or maxflow
}

func (dmf *DinicMaxFlow) makeMinCutFlags(minCut *MinCut, maxflow int64) {

	cutEdges := 0

	artificalSourceId := da.Index(len(dmf.last) - 2)
	artificalSinkId := da.Index(len(dmf.last) - 1)

	for u := da.Index(0); u < da.Index(dmf.graph.NumberOfVertices()); u++ {
		if dmf.GetVertexLevel(u) != INVALID_LEVEL {
			minCut.SetFlag(u, true)
		} else {
			minCut.incrementNumNodesInPartitionTwo()
		}
	}

	for u := da.Index(0); u < da.Index(dmf.graph.NumberOfVertices()); u++ {
		for j := 0; j < dmf.graph.GetVertexEdgesSize(u); j++ {
			edge := dmf.graph.GetEdgeOfVertex(u, j)
			v := edge.GetTo()

			if u == artificalSourceId || v == artificalSinkId {
				continue
			}

			if minCut.GetFlag(u) && !minCut.GetFlag(v) {
				cutEdges++
			}
		}
	}

	minCut.setMaxFlow(maxflow)
	minCut.setNumOfCutEdges(cutEdges)
}

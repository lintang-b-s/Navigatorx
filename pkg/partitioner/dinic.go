package partitioner

import (
	"container/list"
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type DinicMaxFlow struct {
	graph              *da.PartitionGraph
	edgeFlows          []int64
	level              []int
	last               []int
	artificialAdjList  map[da.Index][]int     // edges from artificial-source to sources , edges from sinks to artificial-sink
	artificialEdgeList map[int]da.MaxFlowEdge // edge lists  artificial-source to sources, edge lists from sinks to artificial-sink
	debug              bool
	multSourcesSinks   bool
	sinks              map[da.Index]struct{}
}

func NewDinicMaxFlow(graph *da.PartitionGraph, debug, multSourcesSinks bool) *DinicMaxFlow {
	n := graph.NumberOfVertices()

	dc := &DinicMaxFlow{graph: graph, debug: debug, last: make([]int, n), level: make([]int, n), artificialAdjList: make(map[da.Index][]int, 2),
		multSourcesSinks: multSourcesSinks, sinks: make(map[da.Index]struct{}), artificialEdgeList: make(map[int]da.MaxFlowEdge, 10)}
	copy(dc.last, graph.GetLast())
	copy(dc.level, graph.GetLevel())

	dc.edgeFlows = make([]int64, graph.GetNumberOfEdges())
	graph.ForEdges(func(i int, e da.MaxFlowEdge) {
		dc.edgeFlows[i] = 0
	})

	return dc
}

func (dmf *DinicMaxFlow) AddFlow(u da.Index, idx int, f int64, artificial bool) {
	if !artificial {
		edgeIndex := dmf.graph.GetEdgeId(u, idx)
		dmf.edgeFlows[edgeIndex] += f
	} else {
		edgeIndex := dmf.artificialAdjList[u][idx]
		dmf.edgeFlows[edgeIndex] += f
	}
}

func (dmf *DinicMaxFlow) AddFlowToReversedEdge(u da.Index, idx int, f int64, artificial bool) {
	if !artificial {
		edgeIndex := dmf.graph.GetReversedEdgeId(u, idx)
		dmf.edgeFlows[edgeIndex] += f
	} else {
		edgeIndex := dmf.artificialAdjList[u][idx] ^ 1
		dmf.edgeFlows[edgeIndex] += f
	}
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
	dmf.artificialEdgeList[newEId] = edge
	dmf.edgeFlows = append(dmf.edgeFlows, 0)
	dmf.artificialAdjList[u] = append(dmf.artificialAdjList[u], newEId)

	newRevEId := len(dmf.edgeFlows)
	var reverseEdge da.MaxFlowEdge = da.MaxFlowEdge{}
	if directed {
		reverseEdge = da.NewMaxFlowEdge(newRevEId, v, u, 0)
	} else {
		reverseEdge = da.NewMaxFlowEdge(newRevEId, v, u, w)
	}

	dmf.artificialEdgeList[newRevEId] = reverseEdge
	dmf.edgeFlows = append(dmf.edgeFlows, 0)
	dmf.artificialAdjList[v] = append(dmf.artificialAdjList[v], newRevEId)
}

func (dmf *DinicMaxFlow) ForEachEdges(u, s da.Index, handle func(e da.MaxFlowEdge)) {
	if u == s && dmf.multSourcesSinks {
		for _, edgeIdx := range dmf.artificialAdjList[u] {
			handle(dmf.artificialEdgeList[edgeIdx])
		}
	} else if dmf.InSinks(u) && dmf.multSourcesSinks {
		dmf.graph.ForEachVertexEdges(u, handle)
		for _, edgeIdx := range dmf.artificialAdjList[u] {
			handle(dmf.artificialEdgeList[edgeIdx])
		}
	} else {
		dmf.graph.ForEachVertexEdges(u, handle)
	}
}

func (dmf *DinicMaxFlow) GetEdgeOfArtificialVertex(u da.Index, idx int) da.MaxFlowEdge {
	edgeIndex := dmf.artificialAdjList[u][idx]
	return dmf.artificialEdgeList[edgeIndex]
}

func (dmf *DinicMaxFlow) InSinks(u da.Index) bool {
	_, ok := dmf.sinks[u]
	return ok
}
func (dmf *DinicMaxFlow) GetArtificialVertexEdgesSize(u da.Index) int {
	return len(dmf.artificialAdjList[u])
}

func (dmf *DinicMaxFlow) GetEdgeOfVertex(u, s da.Index, j, superEdgesOffset int) (da.MaxFlowEdge, int, bool) {
	artificial := false
	var edge da.MaxFlowEdge
	if u == s && dmf.multSourcesSinks {
		// u sama dengan artificial source / super-source
		edge = dmf.GetEdgeOfArtificialVertex(u, j)
		artificial = true
	} else if dmf.InSinks(u) && dmf.multSourcesSinks {
		// u in sinks
		if j >= superEdgesOffset {
			j -= superEdgesOffset
			edge = dmf.GetEdgeOfArtificialVertex(u, j)
		} else {
			edge = dmf.graph.GetEdgeOfVertex(u, j)
		}
		artificial = true
	} else {
		edge = dmf.graph.GetEdgeOfVertex(u, j)
	}

	return edge, j, artificial
}

func (dmf *DinicMaxFlow) GetNodeDegree(u, s da.Index) (int, int) {
	numOfEdges := 0
	superEdgesOffset := 0
	if u == s && dmf.multSourcesSinks {
		numOfEdges = dmf.GetArtificialVertexEdgesSize(u)
	} else if dmf.InSinks(u) && dmf.multSourcesSinks {
		numOfEdges = dmf.graph.GetVertexEdgesSize(u) + dmf.GetArtificialVertexEdgesSize(u)
		superEdgesOffset = dmf.graph.GetVertexEdgesSize(u)
	} else {
		numOfEdges = dmf.graph.GetVertexEdgesSize(u)
	}
	return numOfEdges, superEdgesOffset
}

func (dmf *DinicMaxFlow) resetCurrentEdges() {
	for i := 0; i < len(dmf.last); i++ {
		dmf.SetLastEdgeIndex(da.Index(i), 0)
	}
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
	// ref1: https://cp-algorithms.com/graph/dinic.html
	// for general capacity graph:
	// note that this dfs only visit vertices that lie on shortest path from s to t in the level graph  (levels/spdist of each vertices in the shortest path from s to t secara berurutan +1 )
	// & only traverse admissible edges (edge (u,v) l(v)=l(u)+1) with residual capacity > 0 in the level graph
	// level of t or shortest path distance from s to t using unit distance is at most n-1 or in O(n)
	// let k=number of pointer dmf.last advances in this dfs execution, n=number of vertices in this partition graph
	// time complexity of dfsAugmentPath() is O(k+n)

	if u == t || f == 0 { // termination
		return f
	}

	numOfEdges, superEdgesOffset := dmf.GetNodeDegree(u, s)

	for ; dmf.GetLastEdgeIndex(u) < numOfEdges; dmf.IncrementLastEdgeIndex(u) {
		var (
			j          int
			edge       da.MaxFlowEdge
			artificial bool
		)

		j = dmf.GetLastEdgeIndex(u)
		edge, j, artificial = dmf.GetEdgeOfVertex(u, s, j, superEdgesOffset)

		v := edge.GetTo()
		eId := edge.GetID()
		residual := edge.GetCapacity() - dmf.GetEdgeFlow(da.Index(eId))
		if dmf.GetVertexLevel(v) != dmf.GetVertexLevel(u)+1 {
			continue
		}

		if pushed := dmf.dfsAugmentPath(v, s, t, util.MinInt64(residual, f)); pushed > 0 {
			dmf.AddFlow(u, j, pushed, artificial)
			dmf.AddFlowToReversedEdge(u, j, -pushed, artificial)

			return pushed
		}
	}

	return 0.0
}

func (dmf *DinicMaxFlow) blockingFlow(s, t da.Index) int64 {
	blockingFlowVal := int64(0)
	for {
		// ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf
		// for general capacity graph:
		// lemma 4.1 in ref1: each augmentating path (using dfsAugmentPath()) saturates (flow of the edge equal to its capacity) at least one edge
		// there is at most m edges in graph
		// so in this blocking flow loop, num of iterations is in O(m)
		// sum over all iterations of this blocking flow loop, time complexity of blocking flow:
		// sum_{i=1}^{m} O(k+n) = O(nm)
		flow := dmf.dfsAugmentPath(s, s, t, math.MaxInt) // O(k+n), with k=number of pointer dmf.last advances in this dfs execution
		if flow == 0 {
			break
		}
		blockingFlowVal += flow
	}
	return blockingFlowVal
}

/*
time complexity: O(n^2 * m), n,m=number of vertices & edges dari da.PartitionGraph
*/
func (dmf *DinicMaxFlow) ComputeMaxflowMinCut(s da.Index, t da.Index) *MinCut {
	var (
		minCut = NewMinCut(dmf.graph.NumberOfVertices()) // exclude artificial source and sink. kita cuma tambahin super source sinks di slice superEdgeList, superAdjList, dmf.level, dmf.last
	)
	maxFlow := int64(0)

	for dmf.bfsLevelGraph(s, t) {
		// ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf
		// for general capacity graph:
		// by lemma 3.1 in ref1, for each iteration of this loop, shortest path distance from s to t (or level) using unit distance (bfs) is increased by at least 1.
		// shortest path ditance from s to any vertices using unit distance (bfs) is at most n-1.
		// thus, num of iterations of  this loop is at in O(n)
		// time complexity of dinic algorithm: O(n^2*m)
		dmf.resetCurrentEdges()
		blockingFlowVal := dmf.blockingFlow(s, t) // O(nm)
		maxFlow += blockingFlowVal
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
			// https://www.cs.princeton.edu/courses/archive/fall14/cos226/lectures/64MaxFlow.pdf
			// partisi S = semua vertices connected to s by an undirected path with no full forward edges (full fe = its residual capacity = 0 ) or empty backward edges (empty be = its residual capacity = 0 )
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

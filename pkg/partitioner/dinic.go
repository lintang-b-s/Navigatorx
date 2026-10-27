package partitioner

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type FlowNumber interface { // for the inertial flow partitioner use int8, because we operate dinic algorithm on unit capacity graph.
	~int8 | ~int32 | ~int64
}

type DinicMaxFlow[W FlowNumber] struct {
	heads   []da.Index
	flows   []W
	caps    []W
	adjList [][]da.Index

	level      []da.Index
	levelQueue []da.Index
	last       []da.Index
	n          da.Index // number of vertices excluding artificial source and artificial sink.
	t          da.Index
}

// NewDinicMaxFlow create new DinicMaxFlow instance.
// cg is Recursive Bisection Cell Graph.
// adapted from dinic code implementation cp4 by steven halim: https://github.com/stevenhalim/cpbook-code/blob/master/ch8/maxflow.cpp
// currently, only tested for directed network graph.
func NewDinicMaxFlow[W FlowNumber](n int, multiSources, multiSinks bool) *DinicMaxFlow[W] {

	m := n
	if multiSources {
		m++ // additional artificial source
	}
	if multiSinks {
		m++ // additional artificial sink
	}
	dc := &DinicMaxFlow[W]{
		n:          da.Index(n),
		last:       make([]da.Index, m),
		level:      make([]da.Index, m),
		adjList:    make([][]da.Index, m),
		levelQueue: make([]da.Index, 0, m),
	}

	return dc
}

func (dmf *DinicMaxFlow[W]) AddFlow(u da.Index, i da.Index, f W) {
	id := dmf.adjList[u][i]
	dmf.flows[id] += f
	dmf.flows[id^1] -= f
}

func (dmf *DinicMaxFlow[W]) AddEdge(u, v da.Index, w W, directed bool) {
	if u == v {
		return
	}
	dmf.heads = append(dmf.heads, v)
	dmf.flows = append(dmf.flows, 0)
	dmf.caps = append(dmf.caps, w)
	dmf.adjList[u] = append(dmf.adjList[u], da.Index(len(dmf.flows)-1))
	wr := W(0)
	if !directed {
		wr = w
	}
	dmf.heads = append(dmf.heads, u)
	dmf.flows = append(dmf.flows, 0)
	dmf.caps = append(dmf.caps, wr)
	dmf.adjList[v] = append(dmf.adjList[v], da.Index(len(dmf.flows)-1))
}

func (dmf *DinicMaxFlow[W]) resetCurrentEdges() {
	for i := 0; i < len(dmf.last); i++ {
		dmf.last[i] = 0
	}
}

func (dmf *DinicMaxFlow[W]) bfsLevelGraph(
	source, target da.Index) bool {
	dmf.level[target] = INVALID_LEVEL

	m := da.Index(len(dmf.adjList))

	for vId := da.Index(0); vId < m; vId++ {
		dmf.level[vId] = INVALID_LEVEL
	}

	levelQueue := dmf.levelQueue[:0]
	levelQueue = append(levelQueue, source)
	dmf.level[source] = 0

	for len(levelQueue) > 0 {
		u := levelQueue[0]
		levelQueue = levelQueue[1:]

		uLevel := dmf.level[u]
		level := uLevel + 1
		if u == target {
			break
		}

		for _, eId := range dmf.adjList[u] {
			residual := dmf.caps[eId] - dmf.flows[eId]
			v := dmf.heads[eId]
			if residual > 0 && dmf.level[v] == INVALID_LEVEL {
				dmf.level[v] = level
				levelQueue = append(levelQueue, v)
			}
		}
	}

	reachable := dmf.level[target] != INVALID_LEVEL
	return reachable
}

func (dmf *DinicMaxFlow[W]) dfsAugmentPath(u da.Index, s, t da.Index, f int64) int64 {
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

	m := da.Index(len(dmf.adjList[u]))

	for ; dmf.last[u] < m; dmf.last[u]++ {

		j := dmf.last[u]
		e := dmf.adjList[u][j]
		v := dmf.heads[e]
		eCap := dmf.caps[e]
		eFlow := dmf.flows[e]

		residual := int64(eCap - eFlow)
		if dmf.level[v] != dmf.level[u]+1 {
			continue
		}

		if pushed := dmf.dfsAugmentPath(v, s, t, min(residual, f)); pushed > 0 {
			dmf.AddFlow(u, j, W(pushed))
			return pushed
		}
	}

	return 0.0
}

func (dmf *DinicMaxFlow[W]) blockingFlow(s, t da.Index) int64 {
	blockingFlowVal := int64(0)
	dmf.t = t
	for {
		// ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf
		// for general capacity graph:
		// lemma 4.1 in ref1: each augmentating path (using dfsAugmentPath()) saturates (flow of the edge equal to its capacity) at least one edge
		// there is at most m edges in graph
		// so in this blocking flow loop, num of iterations is in O(m)
		// sum over all iterations of this blocking flow loop, time complexity of blocking flow:
		// sum_{i=1}^{m} O(k+n) = O(nm)

		// for unit capacity graph:
		// time complexity of blocking flow unit capacity graph: O(m) (see lemma 4.2 ref1)
		flow := dmf.dfsAugmentPath(s, s, t, math.MaxInt64) // O(k+n), with k=number of pointer dmf.last advances in this dfs execution
		if flow == 0 {
			break
		}
		blockingFlowVal += flow
	}
	return blockingFlowVal
}

/*
ComputeMaxflowMinCut. compute max flow/min st-cut
ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf

time complexity:
general capacity graph:
see lemma 4.1 ref1, O(n^2 * m), n,m=number of vertices & edges dari da.PartitionGraph

for unit capacity graph:
see lemma 4.2 ref1, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})

inspired from dinic code implementation cp4 by steven halim: https://github.com/stevenhalim/cpbook-code/blob/master/ch8/maxflow.cpp
*/
func (dmf *DinicMaxFlow[W]) ComputeMaxflowMinCut(s da.Index, t da.Index) *MinCut {

	var (
		minCut = NewMinCut(1000) // exclude artificial source and sink. kita cuma tambahin super source sinks di slice superEdgeList, superAdjList, dmf.level, dmf.last
	)
	maxFlow := int64(0)

	for dmf.bfsLevelGraph(s, t) {
		// ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf
		// for general capacity graph:
		// by lemma 3.1 in ref1, for each iteration of this loop, shortest path distance from s to t (or level) using unit distance (bfs) is increased by at least 1.
		// shortest path ditance from s to any vertices using unit distance (bfs) is at most n-1.
		// thus, num of iterations of  this loop is O(n)
		// time complexity of dinic algorithm: O(n^2*m)
		// dmf.checkInvariant()
		// for unit capacity graph:
		// see lemma 4.2 ref1, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})
		dmf.resetCurrentEdges()
		blockingFlowVal := dmf.blockingFlow(s, t) // O(nm)
		maxFlow += blockingFlowVal
	}
	dmf.makeMinCutFlags(minCut, int64(maxFlow))
	return minCut //  or maxflow
}

func (dmf *DinicMaxFlow[W]) makeMinCutFlags(minCut *MinCut, maxflow int64) {
	n := da.Index(dmf.n)
	for u := da.Index(0); u < n; u++ {
		if dmf.level[u] != INVALID_LEVEL {
			// https://www.cs.princeton.edu/courses/archive/fall14/cos226/lectures/64MaxFlow.pdf
			// cell S = semua vertices connected to s by an undirected path with no full forward edges (full fe = its residual capacity = 0 ) or empty backward edges (empty be = its residual capacity = 0 )
			minCut.SetFlag(u, true)
		} else {
			minCut.nodesCountInCellT++
		}
	}

	minCut.maxflow = maxflow
}

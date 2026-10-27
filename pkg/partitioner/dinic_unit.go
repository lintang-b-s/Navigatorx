package partitioner

import (
	"github.com/bits-and-blooms/bitset"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

// DinicUnitMaxFlow is dinic algorithm for directed unit capacity graph, from multi-sources to multi-sinks. (more optimized than general capacity dinic.go).
// this directed unit capacity graph has neither parallel edges nor anti-parallel edges. (removed in partition_cell_graph.go).
type DinicUnitMaxFlow struct {
	g     *da.CellGraph
	flows []map[da.Index]struct{}

	level      []da.Index
	levelQueue []da.Index
	last       []da.Index
	sources    []da.Index
	isSink     *bitset.BitSet
	n          da.Index // number of vertices excluding super-source and super-sink.
}

// NewDinicUnitMaxFlow create new NewDinicUnitMaxFlow instance.
// cg is Recursive Bisection Cell Graph.
func NewDinicUnitMaxFlow(n int, g *da.CellGraph) *DinicUnitMaxFlow {

	flows := make([]map[da.Index]struct{}, n)
	for i := 0; i < n; i++ {
		flows[i] = make(map[da.Index]struct{}, 2)
	}

	dc := &DinicUnitMaxFlow{
		n:          da.Index(n),
		last:       make([]da.Index, n),
		level:      make([]da.Index, n),
		levelQueue: make([]da.Index, 0, n),
		flows:      flows,
		sources:    make([]da.Index, 0),
		isSink:     bitset.New(uint(n)),
		g:          g,
	}

	return dc
}

func (dmf *DinicUnitMaxFlow) resetCurrentEdges() {
	for i := 0; i < len(dmf.last); i++ {
		dmf.last[i] = 0
	}
}

func (dmf *DinicUnitMaxFlow) bfsLevelGraph() bool {
	for i := range dmf.level {
		dmf.level[i] = INVALID_LEVEL
	}

	queue := dmf.levelQueue[:0]
	for _, ss := range dmf.sources {
		if dmf.level[ss] == INVALID_LEVEL {
			// directly bfs from multi-sources instead of super-source
			dmf.level[ss] = 0
			queue = append(queue, ss)
		}
	}

	reachedSink := false
	for head := 0; head < len(queue); head++ {
		u := queue[head]
		if dmf.isSink.Test(uint(u)) {
			// reached sink u
			// we dont need to traverse artificial edge (u,t), where t is the super-sink
			// because artificial edge (u,t) have INF capacity,
			// if we reach one of the sink nodes, then the super-sink is reachable bcs of that INF capacity...
			reachedSink = true
			continue
		}

		deg := dmf.degree(u)
		for j := da.Index(0); j < deg; j++ {
			v, rev := dmf.edge(u, j)
			if dmf.level[v] == INVALID_LEVEL && dmf.residual(u, v, rev) {
				nextLevel := dmf.level[u] + 1
				dmf.level[v] = nextLevel
				queue = append(queue, v)
			}
		}
	}

	dmf.levelQueue = queue
	return reachedSink
}

func (dmf *DinicUnitMaxFlow) dfsAugmentPath(u da.Index) bool {

	if dmf.isSink.Test(uint(u)) { // termination
		// reached sink u
		// we dont need to traverse artificial edge (u,t), where t is the super-sink
		// because artificial edge (u,t) have INF capacity,
		// if we reach one of the sink nodes, then the super-sink is reachable bcs of that INF capacity...
		// also in this DinicUnitMaxFlow, we operate on unit capacity directed graph..
		// so the residual capacity of every augmenting path s-t is 1.
		return true
	}

	deg := dmf.degree(u)

	for ; dmf.last[u] < deg; dmf.last[u]++ {
		j := dmf.last[u]
		v, rev := dmf.edge(u, j)
		if dmf.level[v] != dmf.level[u]+1 {
			continue
		}
		residualExists := dmf.residual(u, v, rev)
		if !residualExists {
			continue
		}

		if pushed := dmf.dfsAugmentPath(v); pushed {
			dmf.addFlow(u, v)
			dmf.last[u]++
			return true
		}
	}

	return false
}

func (dmf *DinicUnitMaxFlow) blockingFlow() int32 {
	blockingFlowVal := int32(0)
	for _, ss := range dmf.sources {
		// ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf
		// for unit capacity graph:
		// time complexity of blocking flow unit capacity graph: O(m) (see lemma 4.2 ref1)
		for dmf.dfsAugmentPath(ss) {
			// compute augmenting path from this source ss in the level graph, until all augmenting paths from this source ss in the level graph blocked.
			blockingFlowVal++
		}
	}
	return blockingFlowVal
}

/*
ComputeMaxflowMinCut. compute max flow/min st-cut for unit-capacity graph.
ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf

time complexity:
for unit capacity graph:
see lemma 4.2 ref1, dinic unit capacity graph worst case: O(min{m * sqrt(m), m * n^(2/3)})
*/
func (dmf *DinicUnitMaxFlow) ComputeMaxflowMinCut() *MinCut {
	var (
		minCut = NewMinCut() // exclude artificial source and sink. kita cuma tambahin super source sinks di slice superEdgeList, superAdjList, dmf.level, dmf.last
	)
	maxFlow := int32(0)

	for dmf.bfsLevelGraph() {
		// ref1: https://kyng.inf.ethz.ch/courses/AGAO20/lectures/lecture11_maxflow-contd.pdf
		// for unit capacity graph (see Theorem 3.3 ref1):
		// dinic algorithm terminates in O(min{sqrt(m), n^(2/3)}) iterations
		dmf.resetCurrentEdges()
		blockingFlowVal := dmf.blockingFlow() // O(m) Lemma 4.2 ref1
		maxFlow += blockingFlowVal
	}
	dmf.makeMinCutFlags(minCut, int32(maxFlow))
	return minCut //  or maxflow
}

func (dmf *DinicUnitMaxFlow) residual(u, v da.Index, rev bool) bool {
	if !rev {
		// edge (u,v) has 1-unit residual capacity if its doesnt have 1-unit flow
		_, hasFlow := dmf.flows[u][v]
		return !hasFlow
	}

	// reversed edge (u,v) has 1-unit residual capacity iff edge (v,u) has 1-unit flow
	_, revHasFlow := dmf.flows[v][u]
	return revHasFlow
}

func (dmf *DinicUnitMaxFlow) addFlow(u, v da.Index) {

	// todo: instead of using hash table buat unit-capacity 1 flows, kita bisa langsung store edges beserta unit-flow dan reversed edge unit flow nya dalam struct kaya  https://dennisosrm.github.io/toolbox-rs/
	_, revFlowExists := dmf.flows[v][u]
	if !revFlowExists {
		// add 1-unit flow to edge (u,v) iff there is no 1-unit flow in reversed edge (v,u)
		dmf.flows[u][v] = struct{}{}
	} else {
		// cancel the reverse edge (v,u) 1-unit flow iff there is 1-unit flow in reversed edge (v,u)
		delete(dmf.flows[v], u)
	}
}

func (dmf *DinicUnitMaxFlow) makeMinCutFlags(minCut *MinCut, maxflow int32) {

	n := da.Index(dmf.g.NumberOfCellVertices())
	for u := da.Index(0); u < n; u++ {
		if dmf.level[u] != INVALID_LEVEL {
			// https://www.cs.princeton.edu/courses/archive/fall14/cos226/lectures/64MaxFlow.pdf
			// cell S = semua vertices connected to s by an undirected path with no full forward edges (full fe = its residual capacity = 0 ) or empty backward edges (empty be = its residual capacity = 0 )
			minCut.SetFlag(u, true)
		} else {
			minCut.numNodesInPartitionTwo++
		}
	}

	minCut.maxflow = int64(maxflow)
	minCut.cutEdgesCount = int(maxflow) // max-flow min-cut theorem
}

func (dmf *DinicUnitMaxFlow) degree(u da.Index) da.Index {
	return dmf.g.OutDegree(u) + dmf.g.InDegree(u)
}

// return i-th edge (or reversed edge) of u and reversed or not.
func (dmf *DinicUnitMaxFlow) edge(u, i da.Index) (da.Index, bool) {
	oDeg := dmf.g.OutDegree(u)
	if i < oDeg {
		return dmf.g.OutEdge(u, i), false
	}
	return dmf.g.InEdge(u, i-oDeg), true
}

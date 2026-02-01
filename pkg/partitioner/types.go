package partitioner

import "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

type MinCut struct {
	flags                  []bool // true if the vertex is reachable from source in residual graph, or partition one, else partition two
	numNodesInPartitionTwo int    // number of nodes in source partition (partition 2)
	minCut                 int
}

func NewMinCut(numberOfVertices int) *MinCut {
	return &MinCut{
		flags: make([]bool, numberOfVertices),
	}
}

func (mc *MinCut) SetFlag(u datastructure.Index, flag bool) {
	mc.flags[u] = flag
}

func (mc *MinCut) GetFlag(u datastructure.Index) bool {
	return mc.flags[u]
}

func (mc *MinCut) GetNumNodesInPartitionTwo() int {
	return mc.numNodesInPartitionTwo
}

func (mc *MinCut) incrementNumNodesInPartitionTwo() {
	mc.numNodesInPartitionTwo++
}

func (mc *MinCut) GetMinCut() int {
	return mc.minCut
}

func (mc *MinCut) setMinCut(maxflow int) {
	mc.minCut = maxflow
}

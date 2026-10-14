package partitioner

import (
	"github.com/bits-and-blooms/bitset"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type MinCut struct {
	flags                  *bitset.BitSet // true if the vertex is reachable from source in residual graph, or partition one, else partition two
	numNodesInPartitionTwo int            // number of nodes in source partition (partition 2)
	maxflow                int64
	numOfCutEdges          int
}

func NewMinCut(numberOfVertices int) *MinCut {
	return &MinCut{
		flags: bitset.New(uint(numberOfVertices)),
	}
}

func (mc *MinCut) SetFlag(u da.Index, flag bool) {
	mc.flags.Set(uint(u))
}

func (mc *MinCut) GetFlag(u da.Index) bool {
	return mc.flags.Test(uint(u))
}

func (mc *MinCut) GetNumNodesInPartitionTwo() int {
	return mc.numNodesInPartitionTwo
}

func (mc *MinCut) incrementNumNodesInPartitionTwo() {
	mc.numNodesInPartitionTwo++
}

func (mc *MinCut) GetMaxFlow() int {
	return int(mc.maxflow)
}

func (mc *MinCut) setMaxFlow(maxflow int64) {
	mc.maxflow = maxflow
}

func (mc *MinCut) GetNumOfCutEdges() int {
	return mc.numOfCutEdges
}

func (mc *MinCut) setNumOfCutEdges(numOfCut int) {
	mc.numOfCutEdges = numOfCut
}

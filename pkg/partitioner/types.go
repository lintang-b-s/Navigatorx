package partitioner

import (
	"github.com/bits-and-blooms/bitset"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type MinCut struct {
	flags             *bitset.BitSet // true if the vertex is reachable from source in residual graph, or cell S, else cell T
	nodesCountInCellT int            // number of nodes in cell T
	maxflow           int64
	cutEdgesCount     int
}

func NewMinCut(n int) *MinCut {
	return &MinCut{
		flags: bitset.New(uint(n)),
	}
}

func (mc *MinCut) SetFlag(u da.Index, flag bool) {
	mc.flags.Set(uint(u))
}

func (mc *MinCut) GetFlag(u da.Index) bool {
	return mc.flags.Test(uint(u))
}

func (mc *MinCut) GetNumNodesInCellT() int {
	return mc.nodesCountInCellT
}

func (mc *MinCut) GetMaxFlow() int {
	return int(mc.maxflow)
}

func (mc *MinCut) CutEdgesCount() int {
	return mc.cutEdgesCount
}

func (mc *MinCut) GetcutEdgesCount() int {
	return mc.CutEdgesCount()
}

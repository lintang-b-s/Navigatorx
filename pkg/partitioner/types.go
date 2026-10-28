package partitioner

import (
	"github.com/bits-and-blooms/bitset"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

// MinCut is a struct that represent minimum st-cut result
type MinCut struct {
	flags             *bitset.BitSet // true if the vertex is reachable from source in residual graph, or cell S, else cell T
	nodesCountInCellT int            // number of nodes in cell T
	maxflow           int64
	cutEdgesCount     int
}

// NewMinCut construct new *MinCut
func NewMinCut(n int) *MinCut {
	return &MinCut{
		flags: bitset.New(uint(n)),
	}
}

// SetFlag set flag of vertex u
func (mc *MinCut) SetFlag(u da.Index, flag bool) {
	mc.flags.Set(uint(u))
}

// GetFlag returns true if vertex u inside cell S.
func (mc *MinCut) GetFlag(u da.Index) bool {
	return mc.flags.Test(uint(u))
}

// GetNumNodesInCellT returns number of vertices inside cell T, from minimum st-cut result.
func (mc *MinCut) GetNumNodesInCellT() int {
	return mc.nodesCountInCellT
}

// GetMaxFlow return maximum flow value
func (mc *MinCut) GetMaxFlow() int {
	return int(mc.maxflow)
}

// CutEdgesCount returns st cut edges count
func (mc *MinCut) CutEdgesCount() int {
	return mc.cutEdgesCount
}

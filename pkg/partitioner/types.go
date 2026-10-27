package partitioner

import (
	"github.com/RoaringBitmap/roaring/v2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type MinCut struct {
	flags             *roaring.Bitmap // true if the vertex is reachable from source in residual graph, or cell one, else cell two
	nodesCountInCellT int             // number of nodes in cell T
	maxflow           int64
	cutEdgesCount     int
}

func NewMinCut() *MinCut {
	return &MinCut{
		flags: roaring.New(),
	}
}

func (mc *MinCut) SetFlag(u da.Index, flag bool) {
	mc.flags.Add(uint32(u))
}

func (mc *MinCut) GetFlag(u da.Index) bool {
	return mc.flags.Contains(uint32(u))
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

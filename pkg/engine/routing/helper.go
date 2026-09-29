package routing

import (
	"slices"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

// PathExists. cek apakah ada path (tanpa costs) dari u ke v.
func (crp *CRPRoutingEngine[W]) PathExists(u, v da.Index) bool {
	return crp.graph.PathExists(u, v)
}

// removeConsecutiveDuplicates removes consecutive duplicate edge IDs from arr in-place.
func removeConsecutiveDuplicates(arr []da.Index) []da.Index {
	if len(arr) < 2 {
		return arr
	}
	return slices.Compact(arr)
}

func (bs *CRPQuery[W]) GetForwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.fpq
}

func (bs *CRPQuery[W]) GetBackwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.bpq
}

func (bs *CRPQuery[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPQuery[W]) GetTCellNumber() da.Pv {
	return bs.tCellNumber
}

func (bs *CRPQuery[W]) GetNumExploredNodes() int {
	return bs.numExploredVertices
}

func (bs *CRPALTQuery[W]) GetForwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.fpq
}

func (bs *CRPALTQuery[W]) GetBackwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.bpq
}

func (bs *CRPALTQuery[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPALTQuery[W]) GetNumExploredVertices() int {
	return bs.numExploredVertices
}

func (bs *CRPALTQuery[W]) GetNumExploredOverlayVertices() int {
	return bs.numExploredOverlayVertices
}

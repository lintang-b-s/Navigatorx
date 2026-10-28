package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

// PathExists cek apakah ada path (tanpa costs) dari u ke v.
func (crp *CRPRoutingEngine[W]) PathExists(u, v da.Index) bool {
	return crp.graph.PathExists(u, v)
}

// GetForwardPQ returns forward search priority queue.
func (bs *CRPQuery[W]) GetForwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.fpq
}

// GetBackwardPQ returns backward search priority queue.
func (bs *CRPQuery[W]) GetBackwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.bpq
}

// GetSCellNumber returns level-1 s cell number.
func (bs *CRPQuery[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

// GetTCellNumber returns  level-1 t cell number.
func (bs *CRPQuery[W]) GetTCellNumber() da.Pv {
	return bs.tCellNumber
}

// GetNumExploredNodes returns number of explored vertices.
func (bs *CRPQuery[W]) GetNumExploredNodes() int {
	return bs.numExploredVertices
}

// GetForwardPQ returns forward search priority queue.
func (bs *CRPALTQuery[W]) GetForwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.fpq
}

// GetBackwardPQ returns backward search priority queue.
func (bs *CRPALTQuery[W]) GetBackwardPQ() *da.QueryHeap[da.QueryKey, W] {
	return bs.bpq
}

// GetSCellNumber returns level-1 s cell number.
func (bs *CRPALTQuery[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

// GetNumExploredVertices returns number of explored vertices.
func (bs *CRPALTQuery[W]) GetNumExploredVertices() int {
	return bs.numExploredVertices
}

// GetNumExploredOverlayVertices returns number of explored overlay vertices.
func (bs *CRPALTQuery[W]) GetNumExploredOverlayVertices() int {
	return bs.numExploredOverlayVertices
}

package routing

import (
	"slices"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// PathExists. cek apakah ada path (tanpa costs) dari u ke v.
func (crp *CRPRoutingEngine[W]) PathExists(u, v da.Index) bool {
	return crp.graph.PathExists(u, v)
}

type target struct {
	tId  da.Index
	atId da.Index
}

func newTarget(t, atId da.Index) target {
	return target{t, atId}
}

func (t target) gettId() da.Index {
	return t.tId
}

func (t target) getatId() da.Index {
	return t.atId
}

// removeConsecutiveDuplicates removes consecutive duplicate edge IDs from arr in-place.
func removeConsecutiveDuplicates(arr []da.Index) []da.Index {
	if len(arr) < 2 {
		return arr
	}
	return slices.Compact(arr)
}

func (bs *CRPQueryTurnCost[W]) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.forwardPq
}

func (bs *CRPQueryTurnCost[W]) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.backwardPq
}

func (bs *CRPQueryTurnCost[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPQueryTurnCost[W]) GetTCellNumber() da.Pv {
	return bs.tCellNumber
}

func (bs *CRPQueryTurnCost[W]) GetNumExploredNodes() int {
	return bs.numExploredVertices
}

func (bs *CRPALTQueryTurnCost[W]) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	// karena queryHeap diambil dari sync.Pool & a pointer,
	// bisa ada dipakai query lain buat write ke map & sekaligus dipakai alternative routes finder buat read map nya
	// udah coba load test endpoint alternative routes, dapet error concurrent map read & write
	// solusi awal kita bikin clone query heap sebelum put ke sync.Pool
	// kayake ada solusi lain
	return bs.forwardPq
}

func (bs *CRPALTQueryTurnCost[W]) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.backwardPq
}

func (bs *CRPALTQueryTurnCost[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPALTQueryTurnCost[W]) GetNumExploredVertices() int {
	return bs.numExploredVertices
}

func (bs *CRPALTQueryTurnCost[W]) GetNumExploredOverlayVertices() int {
	return bs.numExploredOverlayVertices
}

func initInfWeight[W util.RoutingNumber](dist []W) {
	for i := range dist {
		dist[i] = util.Infinity[W]()
	}
}

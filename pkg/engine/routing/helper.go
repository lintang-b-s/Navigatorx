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

func (bs *CRPBidirectionalSearch[W]) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.forwardPq
}

func (bs *CRPBidirectionalSearch[W]) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.backwardPq
}

func (bs *CRPBidirectionalSearch[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPBidirectionalSearch[W]) GetTCellNumber() da.Pv {
	return bs.tCellNumber
}

func (bs *CRPBidirectionalSearch[W]) GetNumScannedNodes() int {
	return bs.numScannedVertices
}

func (bs *CRPALTBidirectionalSearch[W]) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	// karena queryHeap diambil dari sync.Pool & a pointer,
	// bisa ada dipakai query lain buat write ke map & sekaligus dipakai alternative routes finder buat read map nya
	// udah coba load test endpoint alternative routes, dapet error concurrent map read & write
	// solusi awal kita bikin clone query heap sebelum put ke sync.Pool
	// kayake ada solusi lain
	return bs.forwardPq
}

func (bs *CRPALTBidirectionalSearch[W]) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.backwardPq
}

func (bs *CRPALTBidirectionalSearch[W]) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPALTBidirectionalSearch[W]) GetNumScannedVertices() int {
	return bs.numScannedVertices
}

func (bs *CRPALTBidirectionalSearch[W]) GetNumScannedOverlayVertices() int {
	return bs.numScannedOverlayVertices
}

func (bs *BidirectionalDijkstra[W]) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.forwardPq
}

func (bs *BidirectionalDijkstra[W]) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey, W] {
	return bs.backwardPq
}

func (bs *BidirectionalDijkstra[W]) GetSCellNumber() da.Pv {
	return 0
}

func initInfWeight[W util.RoutingNumber](dist []W) {
	for i := range dist {
		dist[i] = util.Infinity[W]()
	}
}

package routing

import (
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

// PathExists. cek apakah ada path (tanpa costs) dari u ke v.
func (crp *CRPRoutingEngine) PathExists(u, v datastructure.Index) bool {
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

func removeDuplicates(arr []da.Index) []da.Index {

	n := len(arr)

	arrIndices := make([]int, n)
	for i := 0; i < n; i++ {
		arrIndices[i] = i
	}

	// sort arrIndices by arr[arrIndices[i]] increasing
	sort.Slice(arrIndices, func(i, j int) bool {
		return arr[arrIndices[i]] < arr[arrIndices[j]]
	})

	isDuplicate := make([]bool, n)

	for i := 1; i < n; i++ {
		if arr[arrIndices[i]] == arr[arrIndices[i-1]] {
			isDuplicate[arrIndices[i]] = true
		}
	}

	l := 0

	for r := 0; r < n; r++ {
		v := arr[r]
		if !isDuplicate[r] {
			arr[l] = v
			l++
		}
	}

	return arr[:l]
}

func (bs *CRPBidirectionalSearch) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey] {
	return bs.forwardPq
}

func (bs *CRPBidirectionalSearch) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey] {
	return bs.backwardPq
}

func (bs *CRPBidirectionalSearch) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPBidirectionalSearch) GetTCellNumber() da.Pv {
	return bs.tCellNumber
}

func (bs *CRPBidirectionalSearch) GetNumScannedNodes() int {
	return bs.numScannedVertices
}

func (bs *CRPALTBidirectionalSearch) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey] {
	// karena queryHeap diambil dari sync.Pool & a pointer,
	// bisa ada dipakai query lain buat write ke map & sekaligus dipakai alternative routes finder buat read map nya
	// udah coba load test endpoint alternative routes, dapet error concurrent map read & write
	// solusi awal kita bikin clone query heap sebelum put ke sync.Pool
	// kayake ada solusi lain
	return bs.forwardPq
}

func (bs *CRPALTBidirectionalSearch) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey] {
	return bs.backwardPq
}

func (bs *CRPALTBidirectionalSearch) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPALTBidirectionalSearch) GetNumScannedVertices() int {
	return bs.numScannedVertices
}

func (bs *CRPALTBidirectionalSearch) GetNumScannedOverlayVertices() int {
	return bs.numScannedOverlayVertices
}

func (bs *BidirectionalDijkstra) GetForwardPQ() *da.QueryHeap[da.CRPQueryKey] {
	return bs.forwardPq
}

func (bs *BidirectionalDijkstra) GetBackwardPQ() *da.QueryHeap[da.CRPQueryKey] {
	return bs.backwardPq
}

func (bs *BidirectionalDijkstra) GetSCellNumber() da.Pv {
	return 0
}

func initInfWeight(dist []float64) {
	for i := range dist {
		dist[i] = pkg.INF_WEIGHT
	}
}

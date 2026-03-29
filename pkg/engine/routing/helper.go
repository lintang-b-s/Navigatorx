package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func (crp *CRPRoutingEngine) GetHaversineDistanceFromUtoV(u, v datastructure.Index) float64 {
	return crp.graph.GetHaversineDistanceFromUtoV(u, v)
}

func (crp *CRPRoutingEngine) GetVertexCoordinatesFromOutEdge(u datastructure.Index) (float64, float64) {
	return crp.graph.GetVertexCoordinatesFromOutEdge(u)
}

func (crp *CRPRoutingEngine) GetVertexCoordinatesFromInEdge(u datastructure.Index) (float64, float64) {

	return crp.graph.GetVertexCoordinatesFromInEdge(u)
}

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
	set := make(map[da.Index]struct{}, len(arr))
	j := 0

	for i := 0; i < len(arr); i++ {
		v := arr[i]
		if _, ok := set[v]; !ok {
			set[v] = struct{}{}
			arr[j] = v
			j++
		}
	}

	return arr[:j]
}

func removeDuplicatesVias(arr []*da.ViaVertex) []*da.ViaVertex {
	set := make(map[da.Index]struct{}, len(arr))
	newarr := make([]*da.ViaVertex, 0, len(arr))

	for _, v := range arr {
		if _, ok := set[v.GetVId()]; !ok {
			set[v.GetVId()] = struct{}{}
			newarr = append(newarr, v)
		}
	}
	return newarr
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

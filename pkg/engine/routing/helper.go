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

func (crp *CRPRoutingEngine) VerticeUToVConnected(u, v datastructure.Index) bool {
	return crp.graph.VerticeUToVConnected(u, v)
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

func removeDuplicates(arr []da.OutEdge) []da.OutEdge {
	set := make(map[da.Index]struct{}, len(arr))
	newarr := make([]da.OutEdge, 0, len(arr))

	for _, v := range arr {
		if _, ok := set[v.GetEdgeId()]; !ok {
			set[v.GetEdgeId()] = struct{}{}
			newarr = append(newarr, v)
		}
	}
	return newarr
}

func removeDuplicatesVias(arr []da.ViaVertex) []da.ViaVertex {
	set := make(map[da.Index]struct{}, len(arr))
	newarr := make([]da.ViaVertex, 0, len(arr))

	for _, v := range arr {
		if _, ok := set[v.GetVId()]; !ok {
			set[v.GetVId()] = struct{}{}
			newarr = append(newarr, v)
		}
	}
	return newarr
}

func (bs *CRPBidirectionalSearch) GetViaVertices() []da.ViaVertex {
	return bs.viaVertices
}

func (bs *CRPBidirectionalSearch) GetForwardInfo() *da.QueryHeap[da.CRPQueryKey] {
	return bs.forwardPq
}

func (bs *CRPBidirectionalSearch) GetBackwardInfo() *da.QueryHeap[da.CRPQueryKey] {
	return bs.backwardPq
}

func (bs *CRPBidirectionalSearch) GetSCellNumber() da.Pv {
	return bs.sCellNumber
}

func (bs *CRPBidirectionalSearch) GetNumScannedNodes() int {
	return bs.numScannedVertices
}

func (bs *CRPALTBidirectionalSearch) GetViaVertices() []da.ViaVertex {
	return bs.viaVertices
}

func (bs *CRPALTBidirectionalSearch) GetForwardInfo() *da.QueryHeap[da.CRPQueryKey] {
	return bs.forwardPq
}

func (bs *CRPALTBidirectionalSearch) GetBackwardInfo() *da.QueryHeap[da.CRPQueryKey] {
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

func initInfWeight(dist []float64) {
	for i := range dist {
		dist[i] = pkg.INF_WEIGHT
	}
}

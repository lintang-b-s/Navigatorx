package routing

import (
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

func (crp *CRPRoutingEngine) VerticeUandVAreConnected(u, v datastructure.Index) bool {
	return crp.graph.VerticeUandVAreConnected(u, v)
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

func removeDuplicates[T comparable](arr []T) []T {
	set := make(map[T]struct{})
	newarr := make([]T, 0, len(arr))

	for _, v := range arr {
		if _, ok := set[v]; !ok {
			set[v] = struct{}{}
			newarr = append(newarr, v)
		}
	}
	return newarr
}

func (bs *CRPBidirectionalSearch) GetViaVertices() []da.ViaVertex {
	return bs.viaVertices
}

func (bs *CRPBidirectionalSearch) GetForwardInfo() map[da.Index]VertexInfo {
	return bs.forwardInfo
}

func (bs *CRPBidirectionalSearch) GetBackwardInfo() map[da.Index]VertexInfo {
	return bs.backwardInfo
}

func (bs *CRPBidirectionalSearch) GetNumSettledNodes() int {
	return bs.numSettledNodes
}

func removeDuplicatesEdges(unpackedEdgePath []da.OutEdge) []da.OutEdge {
	set := make(map[da.Index]struct{})
	newarr := make([]da.OutEdge, 0, len(unpackedEdgePath))

	for _, v := range unpackedEdgePath {
		if _, ok := set[v.GetEdgeId()]; !ok {
			set[v.GetEdgeId()] = struct{}{}
			newarr = append(newarr, v)
		}
	}
	return newarr
}

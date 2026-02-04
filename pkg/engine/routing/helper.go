package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
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
	sccOfU := crp.graph.GetSCCOfAVertex(u)
	sccOfV := crp.graph.GetSCCOfAVertex(v)
	if sccOfU == sccOfV {
		return true
	}

	return crp.graph.CondensationGraphOrigintoDestinationConnected(u, v)
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

package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Customizer interface {
}

type CostFunction interface {
	GetWeight(eId da.Index) float64
	GetWeightFromLength(eId da.Index, eLength float64) float64
	GetSegmentLength(eId da.Index) float64
	GetTurnCost(turnTableId da.Index) float64
}

type Router interface {
	ShortestPathSearch(asId, atId da.Index) (float64, float64, []da.Coordinate,
		[]da.OutEdge, bool)
	GetViaVertices() []da.ViaVertex
	GetForwardInfo() da.VertexInfo
	GetBackwardInfo() da.VertexInfo
	GetSCellNumber() da.Pv
	GetNumScannedNodes() int
}

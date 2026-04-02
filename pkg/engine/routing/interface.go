package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Customizer interface {
}

type CostFunction interface {
	GetWeight(eHWType pkg.OsmHighwayType, eWeight float64, eLength float64) float64
	GetTurnCost(turnType pkg.TurnType) float64
}

type Router interface {
	ShortestPathSearch(asId, atId datastructure.Index) (float64, float64, []datastructure.Coordinate,
		[]datastructure.OutEdge, bool)
	GetViaVertices() []da.ViaVertex
	GetForwardInfo() da.VertexInfo
	GetBackwardInfo() da.VertexInfo
	GetSCellNumber() da.Pv
	GetNumScannedNodes() int
}

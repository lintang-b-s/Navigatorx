package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Customizer interface {
}

type CostFunction interface {
	GetWeight(e costfunction.EdgeAttributes) float64
	GetTurnCost(turnType pkg.TurnType) float64
}

type Router interface {
	ShortestPathSearch(asId, atId datastructure.Index) (float64, float64, []datastructure.Coordinate,
		[]datastructure.OutEdge, bool)
}

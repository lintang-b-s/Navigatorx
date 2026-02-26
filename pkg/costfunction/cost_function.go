package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type EdgeAttributes interface {
	GetWeight() float64
	GetEdgeSpeed() float64
	GetLength() float64
	GetEdgeId() datastructure.Index
	GetOriginalEdgeId() datastructure.Index
	GetHighwayType() pkg.OsmHighwayType
}

type CostFunction interface {
	GetWeight(e EdgeAttributes) float64
	GetTurnCost(turnType pkg.TurnType) float64
}

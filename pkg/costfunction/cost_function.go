package costfunction

import "github.com/lintang-b-s/navigatorx-crp/pkg"

type EdgeAttributes interface {
	GetWeight() float64
	GetEdgeSpeed() float64
	GetLength() float64
}

type CostFunction interface {
	GetWeight(e EdgeAttributes) float64
	GetTurnCost(turnType pkg.TurnType) float64
}

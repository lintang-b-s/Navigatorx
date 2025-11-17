package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Customizer interface {
	UpdateDirtyCells(costFunction costfunction.CostFunction, dirtyCells []datastructure.DirtyCell,
		penaltyEdgeCost map[datastructure.PenaltiedEdge]float64,maxLevel int) map[datastructure.Index]float64
}

type CostFunction interface {
	GetWeight(e costfunction.EdgeAttributes) float64
	GetTurnCost(turnType pkg.TurnType) float64
}

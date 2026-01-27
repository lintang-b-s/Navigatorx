package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Customizer interface {
	UpdateDirtyCells(costFunction costfunction.CostFunction, dirtyCells []datastructure.DirtyCell,
		penaltyEdgeCost map[datastructure.PenaltiedEdge]float64, maxLevel int) map[datastructure.Index]float64
}

type CostFunction interface {
	GetWeight(e costfunction.EdgeAttributes) float64
	GetTurnCost(turnType pkg.TurnType) float64
	GetWeightPWL(e costfunction.EdgeAttributes) *datastructure.PWL
	GetWeightAtTime(e costfunction.EdgeAttributes, time float64) float64
}

type Router interface {
	ShortestPathSearch(asId, atId datastructure.Index) (float64, float64, []datastructure.Coordinate,
		[]datastructure.OutEdge, bool)
}

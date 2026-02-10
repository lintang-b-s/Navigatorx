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
}

type CostFunction interface {
	GetWeight(e EdgeAttributes) float64
	GetWeightPWL(e EdgeAttributes) *datastructure.PWL
	GetTurnCost(turnType pkg.TurnType) float64
	GetWeightAtTime(e EdgeAttributes, time float64) float64
}

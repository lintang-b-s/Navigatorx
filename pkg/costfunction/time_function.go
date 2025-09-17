package costfunction

import (
	"math"

	"github.com/lintang-b-s/navigatorx-crp/pkg"
)

type TimeFunction struct {
}

func NewTimeCostFunction() *TimeFunction {
	return &TimeFunction{}
}

func (tf *TimeFunction) GetWeight(e EdgeAttributes) float64 {
	speed := e.GetEdgeSpeed()
	return e.GetLength() / speed
}

func (tf *TimeFunction) GetTurnCost(turnType pkg.TurnType) float64 {
	switch turnType {
	case pkg.U_TURN:
		return math.MaxFloat64
	case pkg.NO_ENTRY:
		return math.MaxFloat64
	default:
		return 0
	}

}

package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type TimeFunction struct {
}

func NewTimeCostFunction() *TimeFunction {
	return &TimeFunction{}
}

const (
	defaultSpeed = 20.0 // km/h
)

func (tf *TimeFunction) GetWeight(e EdgeAttributes) float64 {
	speed := e.GetEdgeSpeed()
	if speed == 0 {
		return e.GetLength() / (defaultSpeed * 1000 / 60)
	}
	return e.GetLength() / speed
}

func (tf *TimeFunction) GetTurnCost(turnType pkg.TurnType) float64 {
	switch turnType {
	case pkg.U_TURN:
		return pkg.INF_WEIGHT
	case pkg.NO_ENTRY:
		return pkg.INF_WEIGHT
	default:
		return 0
	}

}

func (tf *TimeFunction) GetWeightPWL(e EdgeAttributes) *datastructure.PWL {

	return nil
}

func (tf *TimeFunction) GetWeightAtTime(e EdgeAttributes, time float64) float64 {
	return 0
}

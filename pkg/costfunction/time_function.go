package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/spf13/viper"
)

type TimeFunction struct {
	maxspeeds []float64 // map from pkg.OsmHighwayType to maxspeed in km/h
	turnCosts []float64 // map from pkg.TurnType to turn cost in seconds
}

func NewTimeCostFunction() *TimeFunction {
	mapMaxSpeeds := viper.GetStringMap("maxspeeds")
	maxspeeds := make([]float64, 18)
	for roadType, speed := range mapMaxSpeeds {
		highwayType := pkg.GetHighwayType(roadType)
		switch v := speed.(type) {
		case int:
			maxspeeds[highwayType] = float64(v)
		case float64:
			maxspeeds[highwayType] = v
		default:
			panic("unsupported type")
		}
	}

	mapTurnCosts := viper.GetStringMap("turn_costs")
	turnCosts := make([]float64, 6)
	for turnTypeStr, cost := range mapTurnCosts {
		var turnType pkg.TurnType
		switch turnTypeStr {
		case "left_turn":
			turnType = pkg.LEFT_TURN
		case "right_turn":
			turnType = pkg.RIGHT_TURN
		case "straight_on":
			turnType = pkg.STRAIGHT_ON
		case "u_turn":
			turnType = pkg.U_TURN
		case "no_entry":
			turnType = pkg.NO_ENTRY
		case "none":
			turnType = pkg.NONE
		default:
			panic("unsupported turn type")
		}

		switch v := cost.(type) {
		case int:
			turnCosts[turnType] = float64(v)
		case float64:
			turnCosts[turnType] = v
		default:
			panic("unsupported type")
		}
	}
	return &TimeFunction{maxspeeds: maxspeeds}
}

func NewTimeCostFunctionEmpty() *TimeFunction {
	return &TimeFunction{}
}

const (
	defaultSpeed = 20.0 // km/h
)

func (tf *TimeFunction) GetWeight(e EdgeAttributes) float64 {

	maxspeed := defaultSpeed
	if tf.maxspeeds != nil {
		hwType := e.GetHighwayType()
		maxspeed = tf.maxspeeds[hwType]
	} else {
		return e.GetWeight()
	}

	// convert km/h to meter/minute
	maxspeed = maxspeed * 1000 / 60
	return e.GetLength() / maxspeed
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

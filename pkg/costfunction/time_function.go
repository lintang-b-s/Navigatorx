package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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

	if pkg.WITH_TURN_COSTS {
		mapTurnCosts := viper.GetStringMap("turncosts")
		turnCosts := make([]float64, 6)
		for turnTypeStr, cost := range mapTurnCosts {

			turnType := getTurnType(turnTypeStr)
			switch v := cost.(type) {
			case int:
				turnCosts[turnType] = float64(v)
			case float64:
				turnCosts[turnType] = v
			default:
				panic("unsupported type")
			}
		}
		turnCosts[pkg.NONE] = 0
		turnCosts[pkg.NO_ENTRY] = pkg.INF_WEIGHT
		return &TimeFunction{maxspeeds: maxspeeds, turnCosts: turnCosts}
	}

	return &TimeFunction{maxspeeds: maxspeeds}
}

func NewTimeCostFunctionEmpty() *TimeFunction {
	return &TimeFunction{}
}

const (
	defaultSpeed = 30.0 // km/h
)

func (tf *TimeFunction) GetWeight(eHighwayType pkg.OsmHighwayType, eDefaultWeight, eLength float64) float64 {

	maxspeed := defaultSpeed
	if eHighwayType == pkg.INVALID_HIGHWAY {
		return pkg.INF_WEIGHT // dummy edge
	}

	if tf.maxspeeds != nil {
		hwType := eHighwayType
		maxspeed = tf.maxspeeds[hwType]
		if maxspeed == 0 {
			maxspeed = defaultSpeed
		}
	} else {
		return eDefaultWeight
	}

	// convert km/h to meter/minute
	maxspeed = util.KMHToMMin(maxspeed)
	return eLength / maxspeed
}

func (tf *TimeFunction) GetMaxSpeed(e EdgeAttributes) float64 {

	maxspeed := defaultSpeed
	if tf.maxspeeds != nil {
		hwType := e.GetHighwayType()
		maxspeed = tf.maxspeeds[hwType]
		if maxspeed == 0 {
			maxspeed = defaultSpeed
		}
	} else {
		return e.GetWeight()
	}
	maxspeed = util.KMHToMMin(maxspeed)

	return maxspeed
}

func (tf *TimeFunction) GetTurnCost(turnType pkg.TurnType) float64 {
	if tf.turnCosts == nil {
		return 0
	}
	return tf.turnCosts[turnType]
}

func getTurnType(turnTypeStr string) pkg.TurnType {
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
	return turnType
}

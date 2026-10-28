// Package extractor provides functionality for parsing OpenStreetMap data and building road network graphs.
package extractor

import (
	"github.com/spf13/viper"
)

// NodeType represents node type.
type NodeType uint8

const (
	// EndNode ini node yang merupakan endpoint dari osm way dan bukan JUNCTION_NODE
	EndNode NodeType = iota
	// BetweenNode ini node yang berada di antaara endpoint dari osm way
	// dan bukan merupakan JUNCTION_NODE
	BetweenNode
	// JunctionNode ini junction node dari osm way
	// jadi ada setidaknya 2 way yang punya node yang sama
	// nah node yang sama ini ditandain JUNCTION_NODE
	JunctionNode
)

const (
	// StreetName is the street name value.
	StreetName = "STREET_NAME"
	// StreetRef is the street ref value.
	StreetRef = "STREET_REF" // https://wiki.openstreetmap.org/wiki/Exit_Info
	// DESTINATION is the destination value.
	DESTINATION = "DESTINATION"

	// DestinationRef is the destination ref value.
	DestinationRef = "DESTINATION_REF"

	// JUNCTION is the junction value.
	JUNCTION = "JUNCTION"

	// RoadClass is the road class value.
	RoadClass = "ROAD_CLASS"

	// RoadClassLink is the road class link value.
	RoadClassLink = "ROAD_CLASS_LINK"

	// LANES is the lanes value.
	LANES = "LANES"

	// DrivingLane is the driving lane value.
	DrivingLane = "DRIVING_LANE"

	// TrafficLight is the traffic light value.
	TrafficLight = "TRAFFIC_LIGHT"

	strNoEntry    = "no_entry"
	strNone       = "none"
	strRoundabout = "roundabout"
	strYes        = "yes"
	strGate       = "gate"
)

// TurnRestriction represents turn restriction.
type TurnRestriction uint8

const (
	// NoLeftTurn is the no left turn value.
	// NoRightTurn is the no right turn value.
	NoLeftTurn TurnRestriction = iota
	// NoRightTurn is the no right turn value.
	NoRightTurn

	// NoStraightOn is the no straight on value.
	NoStraightOn

	// NoUTurn is the no uturn value.
	NoUTurn

	// OnlyLeftTurn is the only left turn value.
	OnlyLeftTurn

	// OnlyRightTurn is the only right turn value.
	OnlyRightTurn

	// OnlyStraightOn is the only straight on value.
	OnlyStraightOn

	// NoEntry is the no entry value.
	NoEntry

	// NoExit is the no exit value.
	NoExit

	// INVALID is the invalid value.
	INVALID

	// NONE is the none value.
	NONE
)

func (tt TurnRestriction) String() string {
	switch tt {
	case NoLeftTurn:
		return "no_left_turn"
	case NoRightTurn:
		return "no_right_turn"
	case NoStraightOn:
		return "no_straight_on"
	case NoUTurn:
		return "no_u_turn"
	case OnlyLeftTurn:
		return "only_left_turn"
	case OnlyRightTurn:
		return "only_right_turn"
	case OnlyStraightOn:
		return "only_straight_on"
	case NoEntry:
		return strNoEntry
	case NoExit:
		return "no_exit"
	case INVALID:
		return "invalid"
	default:
		return strNone
	}
}

// IsNotAllowedToTurnType reports whether not allowed to turn type.
func IsNotAllowedToTurnType(tt TurnRestriction) bool {
	switch tt {
	case NoLeftTurn, NoRightTurn, NoStraightOn, NoUTurn, NoEntry:
		return true
	default:
		return false
	}
}

func parseTurnRestriction(s string) TurnRestriction {
	switch s {
	case "no_left_turn":
		return NoLeftTurn
	case "no_right_turn":
		return NoRightTurn
	case "no_straight_on":
		return NoStraightOn
	case "no_u_turn":
		return NoUTurn
	case "only_left_turn":
		return OnlyLeftTurn
	case "only_right_turn":
		return OnlyRightTurn
	case "only_straight_on":
		return OnlyStraightOn
	case strNoEntry:
		return NoEntry
	case "no_exit":
		return NoExit
	case "invalid":
		return INVALID
	default:

		return NONE
	}
}

// BarrierTypeT represents barrier type t.
type BarrierTypeT uint8

const (

	// BOLLARD is the bollard value.
	BOLLARD BarrierTypeT = iota
	// SwingGate is the swing gate value.
	SwingGate

	// JerseyBarrier is the jersey barrier value.
	JerseyBarrier

	// LiftGate is the lift gate value.
	LiftGate

	// BLOCK is the block value.
	BLOCK

	// GATE is the gate value.
	GATE

	// REMOVABLE is the removable value.
	REMOVABLE

	// NoBarrier is the no barrier value.
	NoBarrier

	// InvalidBarrier is the invalid barrier value.
	InvalidBarrier
)

func getBarrierType(barrierStr string) BarrierTypeT {
	switch barrierStr {
	case "bollard":
		return BOLLARD
	case "swing_gate":
		return SwingGate
	case "jersey_barrier":
		return JerseyBarrier
	case "lift_gate":
		return LiftGate
	case "block":
		return BLOCK
	case strGate:
		return GATE
	case "removable":
		return REMOVABLE
	case "":
		return NoBarrier
	default:
		return InvalidBarrier
	}
}

func initializeHighwayWhitelist() map[string]struct{} {
	hwlist := viper.GetStringSlice("highway_whitelist")
	hwlistSet := make(map[string]struct{}, len(hwlist))
	for _, hw := range hwlist {
		hwlistSet[hw] = struct{}{}
	}
	return hwlistSet
}

var (
	roundaboutSubName = []string{strRoundabout, "bundaran"}
)

const (
	outputEvery = 5
)

package osmparser

import (
	"github.com/spf13/viper"
)

type NodeType uint8

const (
	// ini node yang merupakan endpoint dari osm way dan bukan JUNCTION_NODE
	END_NODE NodeType = iota
	// ini node yang berada di antaara endpoint dari osm way
	// dan bukan merupakan JUNCTION_NODE
	BETWEEN_NODE
	// ini junction node dari osm way
	// jadi ada setidaknya 2 way yang punya node yang sama
	// nah node yang sama ini ditandain JUNCTION_NODE
	JUNCTION_NODE
)

const (
	STREET_NAME     = "STREET_NAME"
	STREET_REF      = "STREET_REF" // https://wiki.openstreetmap.org/wiki/Exit_Info
	DESTINATION     = "DESTINATION"
	DESTINATION_REF = "DESTINATION_REF"
	JUNCTION        = "JUNCTION"
	ROAD_CLASS      = "ROAD_CLASS"
	ROAD_CLASS_LINK = "ROAD_CLASS_LINK"
	LANES           = "LANES"
	DRIVING_LANE    = "DRIVING_LANE"
	TRAFFIC_LIGHT   = "TRAFFIC_LIGHT"
)

type TurnRestriction uint8

const (
	NO_LEFT_TURN TurnRestriction = iota
	NO_RIGHT_TURN
	NO_STRAIGHT_ON
	NO_U_TURN
	ONLY_LEFT_TURN
	ONLY_RIGHT_TURN
	ONLY_STRAIGHT_ON
	NO_ENTRY
	NO_EXIT
	INVALID
	NONE
)

func parseTurnRestriction(s string) TurnRestriction {
	switch s {
	case "no_left_turn":
		return NO_LEFT_TURN
	case "no_right_turn":
		return NO_RIGHT_TURN
	case "no_straight_on":
		return NO_STRAIGHT_ON
	case "no_u_turn":
		return NO_U_TURN
	case "only_left_turn":
		return ONLY_LEFT_TURN
	case "only_right_turn":
		return ONLY_RIGHT_TURN
	case "only_straight_on":
		return ONLY_STRAIGHT_ON
	case "no_entry":
		return NO_ENTRY
	case "no_exit":
		return NO_EXIT
	case "invalid":
		return INVALID
	default:

		return NONE
	}
}

type BarrierTypeT uint8

const (
	BOLLARD BarrierTypeT = iota
	SWING_GATE
	JERSEY_BARRIER
	LIFT_GATE
	BLOCK
	GATE
	REMOVABLE
	NO_BARRIER
	INVALID_BARRIER
)

func getBarrierType(barrierStr string) BarrierTypeT {
	switch barrierStr {
	case "bollard":
		return BOLLARD
	case "swing_gate":
		return SWING_GATE
	case "jersey_barrier":
		return JERSEY_BARRIER
	case "lift_gate":
		return LIFT_GATE
	case "block":
		return BLOCK
	case "gate":
		return GATE
	case "removable":
		return REMOVABLE
	case "":
		return NO_BARRIER
	default:
		return INVALID_BARRIER
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

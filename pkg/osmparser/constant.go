package osmparser

type NodeType int

const (
	END_NODE NodeType = iota
	BETWEEN_NODE
	JUNCTION_NODE
)

const (
	STREET_NAME     = "STREET_NAME"
	STREET_REF      = "STREET_REF"
	WAY_DISTANCE    = "WAY_DISTANCE"
	JUNCTION        = "JUNCTION"
	MAXSPEED        = "MAXSPEED"
	ROAD_CLASS      = "ROAD_CLASS"
	ROAD_CLASS_LINK = "ROAD_CLASS_LINK"
	LANES           = "LANES"
	TRAFFIC_LIGHT   = "TRAFFIC_LIGHT"
)

type TurnRestriction int

const (
	NO_LEFT_TURN TurnRestriction = iota
	NO_RIGHT_TURN
	NO_STRAIGHT_ON
	NO_U_TURN
	ONLY_LEFT_TURN
	ONLY_RIGHT_TURN
	ONLY_STRAIGHT_ON
	NO_ENTRY
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
	case "invalid":
		return INVALID
	default:

		return NONE
	}
}

const (
	NUM_TURN_TYPES = 6
)

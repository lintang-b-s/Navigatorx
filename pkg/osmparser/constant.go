package osmparser

type NodeType int

const (
	END_NODE NodeType = iota
	BETWEEN_NODE
	JUNCTION_NODE
)

const (
	NERF_MAXSPEED_OSM_REALISTIC = 0.7
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

var (
	skipHighway = map[string]struct{}{
		"footway":                struct{}{},
		"construction":           struct{}{},
		"cycleway":               struct{}{},
		"path":                   struct{}{},
		"pedestrian":             struct{}{},
		"busway":                 struct{}{},
		"steps":                  struct{}{},
		"bridleway":              struct{}{},
		"corridor":               struct{}{},
		"street_lamp":            struct{}{},
		"bus_stop":               struct{}{},
		"crossing":               struct{}{},
		"cyclist_waiting_aid":    struct{}{},
		"elevator":               struct{}{},
		"emergency_bay":          struct{}{},
		"emergency_access_point": struct{}{},
		"give_way":               struct{}{},
		"phone":                  struct{}{},
		"ladder":                 struct{}{},
		"milestone":              struct{}{},
		"passing_place":          struct{}{},
		"platform":               struct{}{},
		"speed_camera":           struct{}{},
		"track":                  struct{}{},
		"bus_guideway":           struct{}{},
		"speed_display":          struct{}{},
		"stop":                   struct{}{},
		"toll_gantry":            struct{}{},
		"traffic_mirror":         struct{}{},
		"traffic_signals":        struct{}{},
		"trailhead":              struct{}{},
	}

	// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
	acceptedHighway = map[string]struct{}{
		"motorway":         struct{}{},
		"motorway_link":    struct{}{},
		"trunk":            struct{}{},
		"trunk_link":       struct{}{},
		"primary":          struct{}{},
		"primary_link":     struct{}{},
		"secondary":        struct{}{},
		"secondary_link":   struct{}{},
		"residential":      struct{}{},
		"residential_link": struct{}{},
		"service":          struct{}{},
		"tertiary":         struct{}{},
		"tertiary_link":    struct{}{},
		"road":             struct{}{},
		"track":            struct{}{},
		"unclassified":     struct{}{},
		"undefined":        struct{}{},
		"unknown":          struct{}{},
		"living_street":    struct{}{},
		"private":          struct{}{},
		"motorroad":        struct{}{},
	}

	//https://wiki.openstreetmap.org/wiki/Key:barrier
	// for splitting street segment to 2 disconnected graph edge
	// if the access tag of the barrier node is != "no" , we dont split the segment
	// for example, at the barrier at the entrance to FMIPA UGM, where entry is only allowed after 16.00 WIB or before 8.00 wib. (https://www.openstreetmap.org/node/8837559088#map=19/-7.767125/110.375436&layers=N)

	acceptedBarrierType = map[string]struct{}{
		"bollard":    struct{}{},
		"swing_gate": struct{}{},

		"jersey_barrier": struct{}{},
		"lift_gate":      struct{}{},
		"block":          struct{}{},
		"gate":           struct{}{},
	}
)

func roadTypeSpeed(roadType string) float64 {
	switch roadType {
	default:
		return roadTypeMaxSpeedOsm(roadType) * NERF_MAXSPEED_OSM_REALISTIC
	}
}

func roadTypeMaxSpeedOsm(roadType string) float64 {
	switch roadType {
	case "motorway":
		return 100
	case "trunk":
		return 60
	case "primary":
		return 50
	case "secondary":
		return 40
	case "tertiary":
		return 40
	case "unclassified":
		return 40
	case "residential":
		return 30
	case "service":
		return 20
	case "motorway_link":
		return 70
	case "trunk_link":
		return 60
	case "primary_link":
		return 50
	case "secondary_link":
		return 40
	case "tertiary_link":
		return 40
	case "living_street":
		return 5
	case "road":
		return 20
	case "track":
		return 15
	case "motorroad":
		return 90
	default:
		return 30
	}
}

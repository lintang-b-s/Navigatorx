package pkg

// enum of turn_type
type TurnType uint8

const (
	LEFT_TURN TurnType = iota
	RIGHT_TURN
	STRAIGHT_ON
	U_TURN
	NO_ENTRY
	NONE
)

const (
	INF_WEIGHT     float64 = 1e15
	INF_WEIGHT_INT         = 1e15

	TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND  = 0.0
	TRAFFIC_LIGHT_PENALTY_SP_SECOND         = 0.0
	ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD = 90.0
	NERF_MAXSPEED_OSM                       = 0.9
	EPSILON_IMAI_IRI_APPROX_PWL             = 0.01 // 1.0%
	EPSILON_IMAI_IRI_APPROX_PWL_SHORTCUTS   = 0.01 // 1.0%
)

const (
	DEBUG = false
)

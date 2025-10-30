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
	INF_WEIGHT                             = 1e9
	TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND = 30
)

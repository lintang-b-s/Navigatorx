package online

const (
	DISTANCE_RESET_THRESHOLD = 120 // 120 meter
	INVALID_LAT              = 91
	INVALID_LON              = 181

	MAX_SEARCH_RADIUS_INITIAL = 0.1  // 100m
	MAX_SEARCH_RADIUS         = 0.08 // 80m

	SEARCH_RADIUS_MULTIPLIER = 1.1
	beta                     = 10.0
)

type TIPE_MHT int

const (
	MHT_TIPE_ONE = iota
	MHT_TIPE_TWO
)

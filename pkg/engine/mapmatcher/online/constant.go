package online

const (
	DISTANCE_RESET_THRESHOLD = 80 // 80 meter
	INVALID_LAT              = 91
	INVALID_LON              = 181

	MAX_SEARCH_RADIUS        = 0.5 // 500m
	SEARCH_RADIUS_MULTIPLIER = 2
)

func convertKilometerToMeter(x float64) float64 {
	return x * 1000
}

func convertMeterToKilometer(x float64) float64 {
	return x / 1000
}

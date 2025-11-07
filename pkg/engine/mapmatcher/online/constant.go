package online

const (
	DISTANCE_RESET_THRESHOLD = 80 // 80 meter
	INVALID_LAT              = 91
	INVALID_LON              = 181
	INVALID_EDGE_ID          = 10e8
)

func convertKilometerToMeter(x float64) float64 {
	return x * 1000
}

func convertMeterToKilometer(x float64) float64 {
	return x / 1000
}

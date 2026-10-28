package util

import (
	"math"
)

// SpeedFromKilometerPerHour converts CSV speed input to centimeters per centisecond.
func SpeedFromKilometerPerHour(value float64) uint32 {
	speed := math.Round(value / 3.6)
	return uint32(speed)
}

// WeightToSeconds converts an internal weight value to seconds.
func WeightToSeconds[W Numeric](value W) float64 {
	return float64(value) / CentiScale
}

// WeightFromSeconds converts seconds to an internal weight value of type W.
func WeightFromSeconds[W Numeric](value float64) W {
	return W(RoundCentiseconds(value))
}

// DistanceToMeters converts an internal distance value (in centimeters) to meters.
func DistanceToMeters(value uint32) float64 {
	return float64(value) / CentiScale
}

// DistanceFromMeters converts meters to an internal distance value in centimeters.
func DistanceFromMeters(value float64) uint32 {
	return uint32(math.Round(value * CentiScale))
}

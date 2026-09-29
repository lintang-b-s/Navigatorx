package util

import (
	"math"
)

// SpeedFromKilometerPerHour converts CSV speed input to centimeters per centisecond.
func SpeedFromKilometerPerHour(value float64) uint32 {
	speed := math.Round(value / 3.6)
	return uint32(speed)
}

func WeightToSeconds[W RoutingNumber](value W) float64 {
	return float64(value) / CentiScale
}

func WeightFromSeconds[W RoutingNumber](value float64) W {
	return W(RoundCentiseconds(value))
}

func DistanceToMeters(value uint32) float64 {
	return float64(value) / CentiScale
}
func DistanceFromMeters(value float64) uint32 {
	return uint32(math.Round(value * CentiScale))
}

func SpeedToMetersPerSecond(value uint32) float64 {
	return float64(value)
}

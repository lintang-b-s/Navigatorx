package util

import (
	"cmp"
	"math"
)

type RoutingNumber interface {
	cmp.Ordered
	~int32 | ~float64 | ~int64
}

const (
	CentiScale                = 100
	INF_WEIGHT_FIXED  int32   = 600_000_000 // 600 million centi seconds ~ 69 hari.
	INF_WEIGHT_FLOAT  float64 = 1e15
	INF_WEIGHT_INT64  int64   = math.MaxInt64
	TurnCostForbidden uint16  = math.MaxUint16
)

var (
	USE_INT32 = false
	USE_INT64 = false
)

func Infinity[T RoutingNumber]() T {
	switch USE_INT32 {
	case true:
		return T(INF_WEIGHT_FIXED)
	case false:
		if USE_INT64 {
			return any(INF_WEIGHT_INT64).(T)
		}
		return any(INF_WEIGHT_FLOAT).(T)
	}
	return any(INF_WEIGHT_FIXED).(T)
}

func RoundCentiseconds(seconds float64) int32 {
	return roundFixedPoint(seconds)
}

func RoundCentimeters(meters float64) int32 {
	return roundFixedPoint(meters)
}

func RoundCentimetersPerSecond(metersPerSecond float64) int32 {
	return roundFixedPoint(metersPerSecond)
}

func roundFixedPoint(value float64) int32 {
	scaled := math.Round(value * CentiScale)
	return int32(scaled)
}

func SecondsFromCentiseconds(value int32) float64 {
	return float64(value) / CentiScale
}

func MetersFromCentimeters(value uint32) float64 {
	return float64(value) / CentiScale
}

func MetersPerSecondFromCentimeters(value int32) float64 {
	return float64(value) / CentiScale
}

func QuantizeTurnCost(seconds float64, forbidden bool) uint16 {
	if forbidden {
		return TurnCostForbidden
	}
	return uint16(math.Round(seconds * CentiScale)) // return in centiseconds
}

const (
	EPS float64 = 1e-7
)

// equal operator
func Eq[T RoutingNumber](a, b T) bool {
	if USE_INT32 || USE_INT64 {
		return a == b
	}
	return math.Abs(float64(a-b)) <= EPS
}

// equal operator
func EqEps[T RoutingNumber](a, b, eps T) bool {
	if USE_INT32 || USE_INT64 {
		return a == b
	}
	return math.Abs(float64(a-b)) <= float64(eps)
}

// less than operator. a<b
func Lt[T RoutingNumber](a, b T) bool {
	if USE_INT32 || USE_INT64 {
		return a < b
	}
	return float64(a)+EPS < float64(b)
}

// greater than or equal to operator
func Ge[T RoutingNumber](a, b T) bool {
	return Le(b, a)
}

func Gt[T RoutingNumber](a, b T) bool {
	return Lt(b, a)
}

// less than or equal operator
func Le[T RoutingNumber](a, b T) bool {
	if USE_INT32 || USE_INT64 {
		return a <= b
	}
	return float64(a) <= float64(b)+EPS
}

// MaxFloat. maximum operator
func MaxFloat[T RoutingNumber](a, b T) T {
	if Gt(a, b) {
		return a
	}
	return b
}

func MinFloat[T RoutingNumber](a, b T) T {
	if Lt(a, b) {
		return a
	}
	return b
}

func ClampMin(a, b int) int {
	if a < b {
		return b
	}
	return a
}

func ActivateMode[W RoutingNumber]() {
	var zero W
	switch any(zero).(type) {
	case float64:
		USE_INT32 = false
		USE_INT64 = false
	case int32:
		USE_INT32 = true
	case int64:
		USE_INT32 = false
		USE_INT64 = true
	}
}

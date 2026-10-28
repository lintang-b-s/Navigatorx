package util

import (
	"cmp"
	"math"
	"runtime"

	"github.com/bytedance/gopkg/util/gopool"
)

// Numeric defines numeric types allowed for weights and distances.
type Numeric interface {
	cmp.Ordered
	~int32 | ~float64 | ~int64
}

// Weight scaling and infinity constants.
const (
	CentiScale                = 100
	InfWeightFixed    int32   = 600_000_000 // 600 million centi seconds ~ 69 hari.
	InfWeightFloat    float64 = 1e15
	InfWeightInt64    int64   = 1e17
	TurnCostForbidden uint16  = math.MaxUint16
)

// Global flags indicating the active weight numeric representation.
var (
	// USE_INT32 indicates whether 32-bit integer weights are active.
	UseInt32 = false // biar gak interface boxing wkwkwk. biar tetep cepet routing (int32) nya, meski tipe weight nya generic.
	// USE_INT64 indicates whether 64-bit integer weights are active.
	UseInt64 = false
)

// Infinity returns the representation of infinity for numeric type T.
func Infinity[T Numeric]() T {
	switch UseInt32 {
	case true:
		return T(InfWeightFixed)
	case false:
		if UseInt64 {
			return any(InfWeightInt64).(T)
		}
		return any(InfWeightFloat).(T)
	}
	return T(InfWeightFixed)
}

// RoundCentiseconds converts seconds to centiseconds as a rounded uint32.
func RoundCentiseconds(seconds float64) uint32 {
	return roundFixedPoint(seconds)
}

// RoundCentimeters converts meters to centimeters as a rounded uint32.
func RoundCentimeters(meters float64) uint32 {
	return roundFixedPoint(meters)
}

// RoundCentimetersPerSecond converts meters per second to centimeters per second as a rounded uint32.
func RoundCentimetersPerSecond(metersPerSecond float64) uint32 {
	return roundFixedPoint(metersPerSecond)
}

func roundFixedPoint(value float64) uint32 {
	scaled := math.Round(value * CentiScale)
	return uint32(scaled)
}

// SecondsFromCentiseconds converts centiseconds to seconds as float64.
func SecondsFromCentiseconds(value int32) float64 {
	return float64(value) / CentiScale
}

// MetersFromCentimeters converts centimeters to meters as float64.
func MetersFromCentimeters(value uint32) float64 {
	return float64(value) / CentiScale
}

// MetersPerSecondFromCentimeters converts centimeters per second to meters per second as float64.
func MetersPerSecondFromCentimeters(value int32) float64 {
	return float64(value) / CentiScale
}

// TurnCostCentiSeconds computes turn penalty in centiseconds, returning TurnCostForbidden if forbidden.
func TurnCostCentiSeconds(seconds float64, forbidden bool) uint16 {
	if forbidden {
		return TurnCostForbidden
	}
	return uint16(math.Round(seconds * CentiScale)) // return in centiseconds
}

// EPS represents the small epsilon value used for floating-point comparisons.
const (
	EPS float64 = 1e-7
)

// Eq reports whether a is equal to b.
func Eq[T Numeric](a, b T) bool {
	if UseInt32 || UseInt64 {
		return a == b
	}
	return math.Abs(float64(a-b)) <= EPS
}

// EqEps reports whether a is equal to b within tolerance eps.
func EqEps[T Numeric](a, b, eps T) bool {
	if UseInt32 || UseInt64 {
		return a == b
	}
	return math.Abs(float64(a-b)) <= float64(eps)
}

// Lt reports whether a is strictly less than b.
func Lt[T Numeric](a, b T) bool {
	if UseInt32 || UseInt64 {
		return a < b
	}
	return float64(a)+EPS < float64(b)
}

// Ge reports whether a is greater than or equal to b.
func Ge[T Numeric](a, b T) bool {
	return Le(b, a)
}

// Gt reports whether a is strictly greater than b.
func Gt[T Numeric](a, b T) bool {
	return Lt(b, a)
}

// Le reports whether a is less than or equal to b.
func Le[T Numeric](a, b T) bool {
	if UseInt32 || UseInt64 {
		return a <= b
	}
	return float64(a) <= float64(b)+EPS
}

// ActivateMode configures the numeric evaluation mode based on the numeric type W.
func ActivateMode[W Numeric]() {
	gopool.SetCap(int32(runtime.NumCPU()))

	var zero W
	switch any(zero).(type) {
	case float64:
		UseInt32 = false
		UseInt64 = false
	case int32:
		UseInt32 = true
	case int64:
		UseInt32 = false
		UseInt64 = true
	default:
		UseInt32 = true
	}
}

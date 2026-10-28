// Package util provides general utility functions used throughout the project.
package util

import (
	"context"
	"errors"
	"fmt"
	stdLog "log"
	"math"
	"strconv"

	"time"
)

// Sleep pauses execution until the duration has elapsed or ctx is cancelled.
func Sleep(ctx context.Context, duration time.Duration) error {
	select {
	case <-time.After(duration):
		return nil
	case <-ctx.Done():
		return ctx.Err()
	}
}

// GetCurrentSeconds returns the total seconds elapsed since midnight today.
func GetCurrentSeconds() float64 {
	now := time.Now()
	hour := now.Hour()
	minute := now.Minute()
	seconds := now.Second()

	totalDaySeconds := float64(hour*3600 + minute*60 + seconds)
	return totalDaySeconds
}

// Error error
type Error struct {
	orig error
	msg  string
	code error
}

func (e *Error) Error() string {
	if e.orig != nil {
		return e.msg
	}

	return e.msg
}

func (e *Error) Unwrap() error {
	return e.orig
}

// WrapErrorf creates a new Error wrapping orig with code and formatted message.
func WrapErrorf(orig error, code error, format string, a ...any) error {
	return &Error{
		code: code,
		orig: orig,
		msg:  fmt.Sprintf(format, a...),
	}
}

// Code returns the underlying sentinel error code.
func (e *Error) Code() error {
	return e.code
}

// Common sentinel errors returned across packages.
var (
	// ErrInternalServerError indicates an internal server failure.
	ErrInternalServerError = errors.New("internal Server Error")
	// ErrNotFound indicates the requested item was not found.
	ErrNotFound = errors.New("your requested Item is not found")
	// ErrConflict indicates the item already exists.
	ErrConflict = errors.New("your Item already exist")
	// ErrBadParamInput indicates invalid parameter input.
	ErrBadParamInput = errors.New("given Param is not valid")
)

// MessageInternalServerError is the default message for internal server error responses.
var MessageInternalServerError = "internal server error"

// KMHToMSeconds converts speed from kilometers per hour to meters per second.
func KMHToMSeconds(speedKmh float64) float64 {
	return speedKmh * 1000 / (60 * 60)
}

// KilometerToMeter converts kilometers to meters.
func KilometerToMeter(d float64) float64 {
	return d * 1000
}

// MeterToKilometer converts meters to kilometers.
func MeterToKilometer(d float64) float64 {
	return d / 1000.0
}

// SecondsToMinutes converts seconds to minutes.
func SecondsToMinutes(seconds float64) float64 {
	return seconds / 60
}

// Abs returns the absolute value of integer a.
func Abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

// DegreeToRadians converts degrees to radians.
func DegreeToRadians(angle float64) float64 {
	return angle * (math.Pi / 180.0)
}

// RadiansToDegree converts radians to degrees.
func RadiansToDegree(rad float64) float64 {
	return 180.0 * rad / math.Pi
}

// RemoveDuplicates returns a new slice with duplicate elements removed preserving first occurrence.
func RemoveDuplicates[T comparable](arr []T) []T {
	set := make(map[T]struct{}, len(arr)*2)
	newarr := make([]T, 0, len(arr))

	for _, v := range arr {
		if _, ok := set[v]; !ok {
			set[v] = struct{}{}
			newarr = append(newarr, v)
		}
	}
	return newarr
}

// RoundFloat rounds a float64 value to the specified decimal precision.
func RoundFloat(val float64, precision uint) float64 {
	ratio := math.Pow(10, float64(precision))
	return math.Round(val*ratio) / ratio
}

// ReverseG reverses the elements of slice arr in place.
func ReverseG[T any](arr []T) {

	for i, j := 0, len(arr)-1; i < j; i, j = i+1, j-1 {
		arr[i], arr[j] = arr[j], arr[i]
	}
}

// AssertFatal logs a fatal error and terminates execution if cond is false.
func AssertFatal(cond bool, msg string) {
	if !cond {
		stdLog.Fatal(msg)
	}
}

// ToFloat64Map converts a map[string]any to map[float64]float64, parsing keys as float64.
func ToFloat64Map(input any) (map[float64]float64, float64) {
	result := make(map[float64]float64)
	defaultVal := 0.0
	for k, v := range input.(map[string]any) {
		if k == "default" {
			defaultVal = v.(float64)
			continue
		}

		key, err := strconv.ParseFloat(k, 64)
		if err != nil {
			continue
		}

		result[key] = v.(float64)
	}

	return result, defaultVal
}

// ToFloat64IntMap converts a map[string]any to map[float64]int, parsing keys as float64.
func ToFloat64IntMap(input any) (map[float64]int, int) {
	result := make(map[float64]int)
	defaultVal := 0

	for k, v := range input.(map[string]any) {
		if k == "default" {
			defaultVal = v.(int)
			continue
		}

		key, err := strconv.ParseFloat(k, 64)
		if err != nil {
			continue
		}

		result[key] = int(v.(int))
	}

	return result, defaultVal
}

// IsTimeout checks whether the context is cancelled or timed out.
func IsTimeout(ctx context.Context) bool {
	select {
	case <-ctx.Done():
		return true
	default:
		return false
	}
}

// Flatten concatenates a 2D slice into a single 1D slice.
func Flatten[T any](container [][]T) []T {
	finalSize := 0
	for _, part := range container {
		finalSize += len(part)
	}

	result := make([]T, finalSize)
	idx := 0
	for _, part := range container {
		for _, elem := range part {
			result[idx] = elem
			idx++
		}
	}
	return result
}

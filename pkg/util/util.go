package util

import (
	"context"
	"errors"
	"fmt"
	"math"
	"strconv"
	"strings"
)

// error

type Error struct {
	orig error
	msg  string
	code error
}

func (e *Error) Error() string {
	if e.orig != nil {
		return fmt.Sprintf("%s", e.msg)
	}

	return e.msg
}

func (e *Error) Unwrap() error {
	return e.orig
}

func WrapErrorf(orig error, code error, format string, a ...interface{}) error {
	return &Error{
		code: code,
		orig: orig,
		msg:  fmt.Sprintf(format, a...),
	}
}

func (e *Error) Code() error {
	return e.code
}

var (
	ErrInternalServerError = errors.New("internal Server Error")
	ErrNotFound            = errors.New("your requested Item is not found")
	ErrConflict            = errors.New("your Item already exist")
	ErrBadParamInput       = errors.New("given Param is not valid")
)

var MessageInternalServerError string = "internal server error"

func SecondsToMinutes(seconds float64) float64 {
	return seconds / 60
}
func Abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

func DegreeToRadians(angle float64) float64 {
	return angle * (math.Pi / 180.0)
}

func RadiansToDegree(rad float64) float64 {
	return 180.0 * rad / math.Pi
}

func StringToFloat64(str string) (float64, error) {
	val, err := strconv.ParseFloat(str, 64)
	if err != nil {
		return 0, err
	}
	return val, nil
}

func RoundFloat(val float64, precision uint) float64 {
	ratio := math.Pow(10, float64(precision))
	return math.Round(val*ratio) / ratio
}

func CountDecimalPlacesF64(value float64) int {
	strValue := strconv.FormatFloat(value, 'f', -1, 64)

	parts := strings.Split(strValue, ".")

	if len(parts) < 2 {
		return 0
	}

	return len(parts[1])
}

func ReverseG[T any](arr []T) []T {
	copyArr := make([]T, len(arr)) // should do on the copy )
	copy(copyArr, arr)
	for i, j := 0, len(copyArr)-1; i < j; i, j = i+1, j-1 {
		copyArr[i], copyArr[j] = copyArr[j], copyArr[i]
	}
	return copyArr
}

func StopConcurrentOperation(ctx context.Context) bool {
	select {
	case <-ctx.Done():
		return true
	default:
		return false
	}
}

func AssertPanic(cond bool, msg string) {
	if !cond {
		panic(msg)
	}
}

func MinInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

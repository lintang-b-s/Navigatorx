package util

import (
	"bufio"
	"context"
	"fmt"
	"math"
	"strconv"
	"strings"

	"github.com/cockroachdb/errors"

	"time"
)

func Bitpack(i, j uint32) uint64 {
	return uint64(i) | (uint64(j) << 32)
}

func ReadLine(br *bufio.Reader) (string, error) {
	line, err := br.ReadString('\n')
	if err != nil {
		return "", err
	}
	return strings.TrimRight(line, "\r\n"), nil
}

func ParseInt(s string) (int, error) {
	return strconv.Atoi(s)
}

func ParseUInt32(s string) (uint32, error) {
	num, err := strconv.ParseUint(s, 10, 32)
	return uint32(num), err
}

func ParseUInt64(s string) (uint64, error) {
	num, err := strconv.ParseUint(s, 10, 64)
	return uint64(num), err
}

func Fields(s string) []string {

	return strings.Fields(s)
}

func GetCurrentSeconds() float64 {
	now := time.Now()
	hour := now.Hour()
	min := now.Minute()
	seconds := now.Second()

	totalDaySeconds := float64(hour*3600 + min*60 + seconds)
	return totalDaySeconds
}

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

func KMHToMMin(speedKmh float64) float64 {
	return speedKmh * 1000 / 60
}

func KilometerToMeter(d float64) float64 {
	return d * 1000
}

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

func ReverseG[T any](arr []T) {

	for i, j := 0, len(arr)-1; i < j; i, j = i+1, j-1 {
		arr[i], arr[j] = arr[j], arr[i]
	}
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

func MinInt64(a, b int64) int64 {
	if a < b {
		return a
	}
	return b
}

func MaxInt(a, b int64) int64 {
	if a > b {
		return a
	}
	return b
}

const (
	EPS = 1e-9
)

// equal operator
func Eq(a, b float64) bool {
	return math.Abs(a-b) <= EPS
}

// equal operator
func EqEps(a, b, eps float64) bool {
	return math.Abs(a-b) <= eps
}

// less than operator
func Lt(a, b float64) bool {
	return a+EPS < b
}

// greater than or equal to operator
func Ge(a, b float64) bool {
	return Le(b, a)
}

func Gt(a, b float64) bool {
	return Lt(b, a)
}

// less than or equal operator
func Le(a, b float64) bool {
	return a <= b+EPS
}

// MaxFloat. maximum operator
func MaxFloat(a, b float64) float64 {
	if Gt(a, b) {
		return a
	}
	return b
}

func MinFloat(a, b float64) float64 {
	if Lt(a, b) {
		return a
	}
	return b
}

func ToFloat64Map(input interface{}) (map[float64]float64, float64) {
	result := make(map[float64]float64)
	defaultVal := 0.0
	for k, v := range input.(map[string]interface{}) {
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

func ToFloat64IntMap(input interface{}) (map[float64]int, int) {
	result := make(map[float64]int)
	defaultVal := 0

	for k, v := range input.(map[string]interface{}) {
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

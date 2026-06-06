// Package util provides general utility functions used throughout the project.
package util

import (
	"bufio"
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"strconv"
	"strings"

	"errors"

	"time"
)

func Sleep(ctx context.Context, duration time.Duration) error {
	select {
	case <-time.After(duration):
		return nil
	case <-ctx.Done():
		return ctx.Err()
	}
}

func Bitpack(i, j uint32) uint64 {
	return uint64(i) | (uint64(j) << 32)
}

func ReadLine(br *bufio.Reader) (string, error) {
	line, err := br.ReadString('\n')
	if err != nil && (err != io.EOF || len(line) == 0) {
		return "", err
	}
	return strings.TrimRight(line, "\r\n"), nil
}

func requireSize(value []byte, size int, kind string) error {
	if len(value) != size {
		return fmt.Errorf("%s requires %d bytes, got %d", kind, size, len(value))
	}
	return nil
}

func ParseInt32(value []byte) (int32, error) {
	if err := requireSize(value, 4, "int32"); err != nil {
		return 0, err
	}
	return int32(binary.LittleEndian.Uint32(value)), nil
}

func ParseInt64(value []byte) (int64, error) {
	if err := requireSize(value, 8, "int64"); err != nil {
		return 0, err
	}
	return int64(binary.LittleEndian.Uint64(value)), nil
}

func ParseBool(value []byte) (bool, error) {
	if err := requireSize(value, 1, "bool"); err != nil {
		return false, err
	}
	switch value[0] {
	case 0:
		return false, nil
	case 1:
		return true, nil
	default:
		return false, fmt.Errorf("invalid bool byte %d", value[0])
	}
}

func ParseUInt8(value []byte) (uint8, error) {
	if err := requireSize(value, 1, "uint8"); err != nil {
		return 0, err
	}
	return value[0], nil
}

func ParseUInt32(value []byte) (uint32, error) {
	if err := requireSize(value, 4, "uint32"); err != nil {
		return 0, err
	}
	return binary.LittleEndian.Uint32(value), nil
}

func ParseUInt64(value []byte) (uint64, error) {
	if err := requireSize(value, 8, "uint64"); err != nil {
		return 0, err
	}
	return binary.LittleEndian.Uint64(value), nil
}

func Fields(s string) []string {
	return strings.Fields(s)
}

func ParseFloat64(value []byte) (float64, error) {
	if err := requireSize(value, 8, "float64"); err != nil {
		return 0, err
	}
	return math.Float64frombits(binary.LittleEndian.Uint64(value)), nil
}

func ParseTextInt(value string) (int, error) {
	return strconv.Atoi(value)
}

func ParseTextInt32(value string) (int32, error) {
	parsed, err := strconv.ParseInt(value, 10, 32)
	return int32(parsed), err
}

func ParseTextInt64(value string) (int64, error) {
	return strconv.ParseInt(value, 10, 64)
}

func ParseTextUInt8(value string) (uint8, error) {
	parsed, err := strconv.ParseUint(value, 10, 8)
	return uint8(parsed), err
}

func ParseTextUInt32(value string) (uint32, error) {
	parsed, err := strconv.ParseUint(value, 10, 32)
	return uint32(parsed), err
}

func ParseTextUInt64(value string) (uint64, error) {
	parsed, err := strconv.ParseUint(value, 10, 64)
	return parsed, err
}

func ParseTextBool(value string) (bool, error) {
	return strconv.ParseBool(value)
}

func ParseTextFloat64(value string) (float64, error) {
	return strconv.ParseFloat(value, 64)
}

func GetCurrentSeconds() float64 {
	now := time.Now()
	hour := now.Hour()
	min := now.Minute()
	seconds := now.Second()

	totalDaySeconds := float64(hour*3600 + min*60 + seconds)
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

func KMHToMSeconds(speedKmh float64) float64 {
	return speedKmh * 1000 / (60 * 60)
}

func KilometerToMeter(d float64) float64 {
	return d * 1000
}

func MeterToKilometer(d float64) float64 {
	return d / 1000.0
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

func RoundFloat(val float64, precision uint) float64 {
	ratio := math.Pow(10, float64(precision))
	return math.Round(val*ratio) / ratio
}

func ReverseG[T any](arr []T) {

	for i, j := 0, len(arr)-1; i < j; i, j = i+1, j-1 {
		arr[i], arr[j] = arr[j], arr[i]
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
	EPS = 1e-7
)

// equal operator
func Eq(a, b float64) bool {
	return math.Abs(a-b) <= EPS
}

// equal operator
func EqEps(a, b, eps float64) bool {
	return math.Abs(a-b) <= eps
}

// less than operator. a<b
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

func ClampMin(a, b int) int {
	if a < b {
		return b
	}
	return a
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

func IsTimeout(ctx context.Context) bool {
	select {
	case <-ctx.Done():
		return true
	default:
		return false
	}
}

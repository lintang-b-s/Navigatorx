package util

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"runtime"
	"runtime/debug"
	"strconv"
	"strings"
)

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

func FreeMemory() {
	runtime.GC()
	debug.FreeOSMemory()
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

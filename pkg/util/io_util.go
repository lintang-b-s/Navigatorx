package util

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"strconv"
	"strings"
)

// Bitpack packs two uint32 values into a single uint64.
func Bitpack(i, j uint32) uint64 {
	return uint64(i) | (uint64(j) << 32)
}

// ReadLine reads a line from the buffered reader, trimming trailing CRLF.
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

// ParseInt32 decodes a little-endian int32 from a 4-byte slice.
func ParseInt32(value []byte) (int32, error) {
	if err := requireSize(value, 4, "int32"); err != nil {
		return 0, err
	}
	return int32(binary.LittleEndian.Uint32(value)), nil
}

// ParseInt64 decodes a little-endian int64 from an 8-byte slice.
func ParseInt64(value []byte) (int64, error) {
	if err := requireSize(value, 8, "int64"); err != nil {
		return 0, err
	}
	return int64(binary.LittleEndian.Uint64(value)), nil
}

// ParseBool decodes a boolean from a single byte (0 for false, 1 for true).
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

// ParseUInt8 decodes a uint8 from a single-byte slice.
func ParseUInt8(value []byte) (uint8, error) {
	if err := requireSize(value, 1, "uint8"); err != nil {
		return 0, err
	}
	return value[0], nil
}

// ParseUInt32 decodes a little-endian uint32 from a 4-byte slice.
func ParseUInt32(value []byte) (uint32, error) {
	if err := requireSize(value, 4, "uint32"); err != nil {
		return 0, err
	}
	return binary.LittleEndian.Uint32(value), nil
}

// ParseUInt64 decodes a little-endian uint64 from an 8-byte slice.
func ParseUInt64(value []byte) (uint64, error) {
	if err := requireSize(value, 8, "uint64"); err != nil {
		return 0, err
	}
	return binary.LittleEndian.Uint64(value), nil
}

// Fields splits the string s around each instance of one or more consecutive white space characters.
func Fields(s string) []string {
	return strings.Fields(s)
}

// ParseFloat64 decodes a little-endian float64 from an 8-byte slice.
func ParseFloat64(value []byte) (float64, error) {
	if err := requireSize(value, 8, "float64"); err != nil {
		return 0, err
	}
	return math.Float64frombits(binary.LittleEndian.Uint64(value)), nil
}

// ParseTextInt parses a decimal integer string into an int.
func ParseTextInt(value string) (int, error) {
	return strconv.Atoi(value)
}

// ParseTextInt32 parses a decimal integer string into an int32.
func ParseTextInt32(value string) (int32, error) {
	parsed, err := strconv.ParseInt(value, 10, 32)
	return int32(parsed), err
}

// ParseTextInt64 parses a decimal integer string into an int64.
func ParseTextInt64(value string) (int64, error) {
	return strconv.ParseInt(value, 10, 64)
}

// ParseTextUInt8 parses a decimal unsigned integer string into a uint8.
func ParseTextUInt8(value string) (uint8, error) {
	parsed, err := strconv.ParseUint(value, 10, 8)
	return uint8(parsed), err
}

// ParseTextUInt32 parses a decimal unsigned integer string into a uint32.
func ParseTextUInt32(value string) (uint32, error) {
	parsed, err := strconv.ParseUint(value, 10, 32)
	return uint32(parsed), err
}

// ParseTextUInt64 parses a decimal unsigned integer string into a uint64.
func ParseTextUInt64(value string) (uint64, error) {
	parsed, err := strconv.ParseUint(value, 10, 64)
	return parsed, err
}

// ParseTextBool parses a boolean string into a bool.
func ParseTextBool(value string) (bool, error) {
	return strconv.ParseBool(value)
}

// ParseTextFloat64 parses a floating-point string into a float64.
func ParseTextFloat64(value string) (float64, error) {
	return strconv.ParseFloat(value, 64)
}

package datastructure

import (
	"fmt"
	"strconv"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// Index represents index.
type Index uint32

// ParseTextIndex parses text index.
func ParseTextIndex(value string) (Index, error) {
	parsed, err := util.ParseTextUInt32(value)
	if err != nil {
		return 0, fmt.Errorf("parse text index %q: %w", value, err)
	}
	return Index(parsed), nil
}

// ParseIndex parses index.
func ParseIndex(value string) (Index, error) {
	parsed, err := strconv.ParseUint(value, 10, 32)
	if err != nil {
		return 0, fmt.Errorf("parse text index %q: %w", value, err)
	}
	return Index(parsed), nil
}

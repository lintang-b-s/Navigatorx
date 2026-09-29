package datastructure

import (
	"fmt"
	"strconv"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Index uint32

func ParseTextIndex(value string) (Index, error) {
	parsed, err := util.ParseTextUInt32(value)
	if err != nil {
		return 0, fmt.Errorf("parse text index %q: %w", value, err)
	}
	return Index(parsed), nil
}

func ParseIndex(value string) (Index, error) {
	parsed, err := strconv.ParseUint(value, 10, 32)
	if err != nil {
		return 0, fmt.Errorf("parse text index %q: %w", value, err)
	}
	return Index(parsed), nil
}

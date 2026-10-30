// Package controllers defines the HTTP controllers for the routing engine.
package controllers

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

const (
	// JSONBufPoolSize is the jsonbuf pool size value.
	JSONBufPoolSize = 1 << 14
	// MbToBytes is the mb to bytes value.
	MbToBytes = 1000000
)

// GetMapMatchingTransitionFile returns map matching transition file.
func GetMapMatchingTransitionFile() string {
	return fmt.Sprintf("./data/profiles/%s/%s_transition_matrix.ntm", pkg.ProfileName, pkg.RegionName)
}

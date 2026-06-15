// Package controllers defines the HTTP controllers for the routing engine.
package controllers

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

const (
	JSON_BUF_POOL_SIZE = 32 << 10
	MB_TO_BYTES        = 1000000
)

func GetMapMatchingTransitionFile() string {
	return fmt.Sprintf("./data/profiles/%s/%s_transition_matrix.ntm", pkg.ProfileName, pkg.RegionName)
}

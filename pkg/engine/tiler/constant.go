package tiler

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

const (
	GeohashPrecision = 6
	GeohashBits      = 6 * 5
)

func MapTileFilePathPrefix() string {
	return fmt.Sprintf("./data/profiles/%s/tiles/%s", pkg.ProfileName, pkg.RegionName)
}

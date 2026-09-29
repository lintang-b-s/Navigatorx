// Package usecases contains application business logic and service interfaces.
package usecases

import (
	"errors"

	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

var (
	ErrPathNotFound = errors.New("no path found from origin to destination")
)

const (
	UPPERBOUND_SHORTEST_PATH    = 1.0
	MAX_SEARCH_RADIUS           = 1.0 // 1km
	SEARCH_RADIUS_MULTIPLIER    = 2.0
	COORDINATES_POOL_SIZE       = 256
	DRIVING_DIRECTION_POOL_SIZE = 128
	candidatePairCapacity       = (spatialindex.MAX_CANDIDATES * spatialindex.MAX_CANDIDATES) / 10
)

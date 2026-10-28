// Package usecases contains application business logic and service interfaces.
package usecases

import (
	"errors"

	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

var (
	// ErrPathNotFound is the err path not found value.
	ErrPathNotFound = errors.New("no path found from origin to destination")
)

const (
	// UpperboundShortestPath is the upperbound shortest path value.
	UpperboundShortestPath = 1.0
	// MaxSearchRadius is the max search radius value.
	MaxSearchRadius = 1.0 // 1km
	// SearchRadiusMultiplier is the search radius multiplier value.
	SearchRadiusMultiplier = 2.0

	// CoordinatesPoolSize is the coordinates pool size value.
	CoordinatesPoolSize = 256

	// DrivingDirectionPoolSize is the driving direction pool size value.
	DrivingDirectionPoolSize = 128
	candidatePairCapacity    = (spatialindex.MaxCandidates * spatialindex.MaxCandidates) / 10
)

// Package usecases contains application business logic and service interfaces.
package usecases

import (
	"github.com/golang/geo/s2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// AlternativeRouteAlgorithm represents alternative route algorithm.
type AlternativeRouteAlgorithm[W util.Numeric] interface {
	ComputeAlternativeRoutes(s, t da.Index, k int, reroute bool, startEdgeId da.Index, optCost float64, optPath []da.Index,
		crpQuery *routing.CRPQuery[W], param routing.AlternativeRouteParameters) ([]routing.AlternativeRoute, float64, int64)
}

// SpatialIndex represents spatial index.
type SpatialIndex interface {
	SearchWithinRadius(qLat, qLon, radius float64, mode uint8) []da.Index
}

// OnlineMapMatcherEngine represents online map matcher engine.
type OnlineMapMatcherEngine interface {
	OnlineMapMatch(gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64)
}

// MapAttributesEngine represents map attributes engine.
type MapAttributesEngine interface {
	GetMapAttributes(s2CellId s2.CellID, radius float64) ([]byte, error)
}

// OfflineMapMatcherEngine represents offline map matcher engine.
type OfflineMapMatcherEngine interface {
	MapMatch(gpsTraj []*da.GPSPoint) []*da.MatchedGPSPoint
	MapMatchWithGPSRadiuses(gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.Coordinate)
}

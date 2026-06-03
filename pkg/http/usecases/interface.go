// Package usecases contains application business logic and service interfaces.
package usecases

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

type AlternativeRouteAlgorithm interface {
	FindAlternativeRoutes(sp, tp da.PhantomNode, k int, reroute bool, startEdgeId da.Index) ([]routing.AlternativeRoute, float64, int64)
}

type SpatialIndex interface {
	SearchWithinRadius(qLat, qLon, radius float64, mode uint8) []da.Index
}

type OnlineMapMatcherEngine interface {
	OnlineMapMatch(gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64)
}

type TilingEngine interface {
	GetTileFilePath(userGeohash string) string
	GetNumberOfVertices() int
}

type OfflineMapMatcherEngine interface {
	MapMatch(gpsTraj []*da.GPSPoint) []*da.MatchedGPSPoint
	MapMatchWithGPSRadiuses(gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.Coordinate)
}

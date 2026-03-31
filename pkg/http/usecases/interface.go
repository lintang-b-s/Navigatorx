package usecases

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

type RoutingEngine interface {
	GetGraph() *da.Graph
	GetHaversineDistanceFromUtoV(u, v da.Index) float64
	GetVertexCoordinatesFromOutEdge(u da.Index) (float64, float64)
	GetVertexCoordinatesFromInEdge(u da.Index) (float64, float64)
	PathExists(u, v da.Index) bool
	DoneQuery(edgePath []da.OutEdge, path []da.Coordinate)
}

type AlternativeRouteAlgorithm interface {
	FindAlternativeRoutes(asId, atId da.Index, k int) ([]*routing.AlternativeRoute, float64, int64)
}

type SpatialIndex interface {
	SearchWithinRadius(qLat, qLon, radius float64) []da.Index
}

type OnlineMapMatcherEngine interface {
	OnlineMapMatch(gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64)
}

type OfflineMapMatcherEngine interface {
	MapMatch(gpsTraj []*da.GPSPoint) []*datastructure.MatchedGPSPoint
}

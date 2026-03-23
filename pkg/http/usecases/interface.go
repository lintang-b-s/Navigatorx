package usecases

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

type RoutingEngine interface {
	GetGraph() *da.Graph
	GetHaversineDistanceFromUtoV(u, v da.Index) float64
	GetVertexCoordinatesFromOutEdge(u da.Index) (float64, float64)
	GetVertexCoordinatesFromInEdge(u da.Index) (float64, float64)
	VerticeUToVConnected(u, v da.Index) bool
}

type SpatialIndex interface {
	SearchWithinRadius(float64, float64, float64) []spatialindex.ArcEndpoint
}

type OnlineMapMatcherEngine interface {
	OnlineMapMatch(gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64)
}

type OfflineMapMatcherEngine interface {
	MapMatch(gpsTraj []*da.GPSPoint) []*datastructure.MatchedGPSPoint
}

package usecases

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

type RoutingEngine interface {
	GetGraph() *da.Graph
	PathExists(u, v da.Index) bool
	GetWeight(eId da.Index, outEdge bool) float64
	GetWeightFromLength(eId da.Index, eLength float64, outEdge bool) float64
	IsDummyOutEdge(eId da.Index) bool
	IsDummyInEdge(eId da.Index) bool
}

type AlternativeRouteAlgorithm interface {
	FindAlternativeRoutes(sp, tp da.PhantomNode, k int) ([]routing.AlternativeRoute, float64, int64)
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

package controllers

import (
	"context"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

type RoutingService interface {
	ShortestPath(ctx context.Context, origLat, origLon, dstLat, dstLon float64, reroute bool, startEdgeId da.Index) (float64, float64, string, []da.DrivingDirection, bool, error)
	AlternativeRouteSearch(ctx context.Context, origLat, origLon, dstLat, dstLon float64, k int, reroute bool, startEdgeId da.Index) ([]routing.AlternativeRoute, error)
	GetRoutingEngine() RoutingEngine
	Close()
	InitBackgroundWorker(ctx context.Context)
	GetBoundingBox(ctx context.Context) da.BoundingBox
	OfflineMapMatch(ctx context.Context, gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.Coordinate, error)
}

type RoutingEngine interface {
	GetGraph() *da.Graph
	PathExists(u, v da.Index) bool
	GetWeight(eId da.Index, outEdge bool) float64
	GetWeightFromLength(eId da.Index, eLength float64, outEdge bool) float64
	IsDummyOutEdge(eId da.Index) bool
	IsDummyInEdge(eId da.Index) bool
	InitBackgroundWorker(ctx context.Context)
	ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, *da.Coordinates, []da.Index, bool)
	Close()
	GetSegmentSpeed(eId da.Index, outEdge bool) float64
}

type MapMatcherService interface {
	OnlineMapMatch(ctx context.Context, gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64, error)
}

type TilingService interface {
	GetTileFilePath(ctx context.Context, userGeohash string) string
	GetNumberOfVertices(ctx context.Context) int
}

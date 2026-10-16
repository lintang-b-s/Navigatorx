package controllers

import (
	"context"

	"github.com/golang/geo/s2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
)

type RoutingService interface {
	ShortestPath(ctx context.Context, origLat, origLon, dstLat, dstLon float64, reroute bool, startEdgeId da.Index, useAnnotation, useSteps bool) (float64, float64, string, []da.DrivingDirection, bool, error)
	AlternativeRouteSearch(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64, k int, reroute bool, startSegId da.Index, useAnnotation bool, useSteps bool) ([]float64, []float64, []string, [][]da.DrivingDirection, error)
	GetRoutingEngine() RoutingEngine
	Close()
	InitBackgroundWorker(ctx context.Context)
	GetBoundingBox(ctx context.Context) da.BoundingBox
	OfflineMapMatch(ctx context.Context, gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.FloatCoordinate, error)
}

type RoutingEngine interface {
	GetGraph() *da.Graph
	PathExists(u, v da.Index) bool
	GetMetrics() *met.Metric[int32]
	InitBackgroundWorker(ctx context.Context)
	ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, *da.Coordinates, []da.Index, bool)
	ShortestPathWithAlternativesSearch(sp, tp da.PhantomNode, k int, reroute bool, startSegId da.Index) ([]float64, []float64, []*da.Coordinates, [][]da.Index, bool)
	Close()
}

type Metrics interface {
	GetDurationSeconds(segId da.Index) float64
	GetSegmentSpeed(segId da.Index) float64
	GetSegmentLength(segId da.Index) float64
	GetDurationFromLength(segId da.Index, length float64) float64
}

type MapMatcherService interface {
	OnlineMapMatch(ctx context.Context, gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64, error)
}

// MapAttributesService  https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type MapAttributesService interface {
	GetMapAttributes(ctx context.Context, s2CellId s2.CellID, radius float64) ([]byte, error)
}

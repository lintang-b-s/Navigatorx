package controllers

import (
	"context"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

type RoutingService interface {
	ShortestPath(ctx context.Context, origLat, origLon, dstLat, dstLon float64) (float64, float64, string, []da.DrivingDirection, bool, error)
	AlternativeRouteSearch(ctx context.Context, origLat, origLon, dstLat, dstLon float64, k int) ([]routing.AlternativeRoute, bool, error)
	GetRoutingEngine() *routing.CRPRoutingEngine
	Close()
}

type MapMatcherService interface {
	OnlineMapMatch(ctx context.Context, gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64, error)
}

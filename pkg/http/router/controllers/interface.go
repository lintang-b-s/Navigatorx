package controllers

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

type RoutingService interface {
	ShortestPath(origLat, origLon, dstLat, dstLon float64) (float64, float64, string, []da.DrivingDirection, bool, error)
	AlternativeRouteSearch(origLat, origLon, dstLat, dstLon float64, k int) ([]routing.AlternativeRoute, bool, error)
	DoneDrivingDirection(drivingDirection []da.DrivingDirection)
}

type MapMatcherService interface {
	OnlineMapMatch(gps *da.GPSPoint, k int,
		candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64)
}

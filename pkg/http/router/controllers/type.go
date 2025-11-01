package controllers

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/mapmatcher/online"
)

type RoutingService interface {
	ShortestPath(origLat, origLon, dstLat, dstLon float64) (float64, float64, string, []datastructure.DrivingDirection, bool, error)
	AlternativeRouteSearch(origLat, origLon, dstLat, dstLon float64, k int) ([]*routing.AlternativeRoute, bool, error)
}

type MapMatcherService interface {
	OnlineMapMatch(gps *datastructure.GPSPoint, k int,
		candidates []*online.Candidate, speedMeanK, speedStdK float64) (*datastructure.MatchedGPSPoint, []*online.Candidate, float64, float64)
}

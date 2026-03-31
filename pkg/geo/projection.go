package geo

import (
	"github.com/golang/geo/s2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// return in meter
func PointLinePerpendicularDistance(pointA da.Coordinate, pointB da.Coordinate,
	snap da.Coordinate) float64 {
	projectionPoint := ProjectPointOnSegment(pointA, pointB, snap)

	dist := CalculateGreatCircleDistance(snap.GetLat(), snap.GetLon(), projectionPoint.GetLat(), projectionPoint.GetLon())
	return util.KilometerToMeter(dist)
}

func ProjectPointOnSegment(pointA da.Coordinate, pointB da.Coordinate,
	qCoord da.Coordinate) da.Coordinate {

	qLat := qCoord.GetLat()
	qLon := qCoord.GetLon()

	pointAS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(pointA.GetLat(), pointA.GetLon()))
	pointBS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(pointB.GetLat(), pointB.GetLon()))
	queryPointS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(qLat, qLon))
	projection := s2.Project(queryPointS2, pointAS2, pointBS2)
	projectLatLng := s2.LatLngFromPoint(projection)
	snap := da.NewCoordinate(projectLatLng.Lat.Degrees(), projectLatLng.Lng.Degrees())
	return snap
}

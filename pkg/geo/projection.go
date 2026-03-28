package geo

import (
	"github.com/golang/geo/s2"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// return in meter
func PointLinePerpendicularDistance(pointA Coordinate, pointB Coordinate,
	snap Coordinate) float64 {
	projectionPoint := ProjectPointOnSegment(pointA, pointB, snap)

	dist := CalculateHaversineDistance(snap.GetLat(), snap.GetLon(), projectionPoint.GetLat(), projectionPoint.GetLon())
	return util.KilometerToMeter(dist)
}

func ProjectPointOnSegment(pointA Coordinate, pointB Coordinate,
	qCoord Coordinate) Coordinate {

	qLat := qCoord.GetLat()
	qLon := qCoord.GetLon()

	pointAS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(pointA.GetLat(), pointA.GetLon()))
	pointBS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(pointB.GetLat(), pointB.GetLon()))
	snapS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(qLat, qLon))
	projection := s2.Project(snapS2, pointAS2, pointBS2)
	projectLatLng := s2.LatLngFromPoint(projection)
	snap := NewCoordinate(projectLatLng.Lat.Degrees(), projectLatLng.Lng.Degrees())
	return snap
}

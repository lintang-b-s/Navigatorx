package geo

import (
	"github.com/lintang-b-s/Navigatorx/pkg/util"

	"github.com/golang/geo/s2"
)

func ProjectPointToLineCoord(pointA Coordinate, pointB Coordinate,
	snap Coordinate) Coordinate {

	snapLat := snap.Lat
	snapLon := snap.Lon

	pointAS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(pointA.Lat, pointA.Lon))
	pointBS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(pointB.Lat, pointB.Lon))
	snapS2 := s2.PointFromLatLng(s2.LatLngFromDegrees(snapLat, snapLon))
	projection := s2.Project(snapS2, pointAS2, pointBS2)
	projectLatLng := s2.LatLngFromPoint(projection)
	return NewCoordinate(projectLatLng.Lat.Degrees(), projectLatLng.Lng.Degrees())
}

// return in meter
func PointLinePerpendicularDistance(pointA Coordinate, pointB Coordinate,
	snap Coordinate) float64 {
	projectionPoint := ProjectPointToLineCoord(pointA, pointB, snap)

	dist := CalculateHaversineDistance(snap.GetLat(), snap.GetLon(), projectionPoint.GetLat(), projectionPoint.GetLon())

	return util.KilometerToMeter(dist)
}

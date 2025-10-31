package geo

import (
	"github.com/lintang-b-s/Navigatorx/pkg/util"

	"github.com/golang/geo/s2"
)

func ProjectPointToLineCoord(pointA Coordinate, pointB Coordinate,
	snap Coordinate) Coordinate {
	pointA = MakeSixDigitsAfterComa2(pointA, 6)
	pointB = MakeSixDigitsAfterComa2(pointB, 6)
	snap = MakeSixDigitsAfterComa2(snap, 6)
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

	return dist * 1000
}

func MakeSixDigitsAfterComa2(n Coordinate, precision int) Coordinate {

	if util.CountDecimalPlacesF64(n.Lat) != precision {
		n.Lat = util.RoundFloat(n.Lat+0.000001, 6)
	}
	if util.CountDecimalPlacesF64(n.Lon) != precision {
		n.Lon = util.RoundFloat(n.Lon+0.000001, 6)
	}
	return n
}

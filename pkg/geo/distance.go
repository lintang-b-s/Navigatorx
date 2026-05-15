package geo

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	earthRadiusKM = 6371.0
)

func havFunction(angleRad float64) float64 {
	return (1 - math.Cos(angleRad)) / 2.0
}

// CalculateGreatCircleDistance. calculate haversine distance in km
func CalculateGreatCircleDistance(latOne, longOne, latTwo, longTwo float64) float64 {
	latOne = util.DegreeToRadians(latOne)
	longOne = util.DegreeToRadians(longOne)
	latTwo = util.DegreeToRadians(latTwo)
	longTwo = util.DegreeToRadians(longTwo)

	a := havFunction(latOne-latTwo) + math.Cos(latOne)*math.Cos(latTwo)*havFunction(longOne-longTwo)
	c := 2.0 * math.Asin(math.Sqrt(a))
	return earthRadiusKM * c
}

// CalculateEuclideanDistMercatorProj. calculate euclidean distance (in meter) using mercator projected coordinates.
// https://gis.stackexchange.com/questions/14528/better-distance-measurements-in-web-mercator-projection
func CalculateEuclideanDistMercatorProj(latOne, longOne, latTwo, longTwo float64) float64 {
	xOne := CalcLonToX(longOne)
	yOne := CalcLatToY(latOne)
	xTwo := CalcLonToX(longTwo)
	yTwo := CalcLatToY(latTwo)

	meanLat := (latOne + latTwo) / 2.0
	meanLat = util.DegreeToRadians(meanLat)

	xx := xTwo - xOne
	yy := yTwo - yOne
	return math.Sqrt(xx*xx+yy*yy) * math.Cos(meanLat)
}

func radToDeg(r float64) float64 {
	return 180.0 * r / math.Pi
}

// GetDestinationPoint returns the destination point given the starting point, bearing and distance
// dist in km
// https://www.movable-type.co.uk/scripts/latlong.html
func GetDestinationPoint(lat1, lon1 float64, bearing float64, dist float64) (float64, float64) {

	dr := dist / earthRadiusKM

	bearing = util.DegreeToRadians(bearing)

	lat1 = util.DegreeToRadians(lat1)
	lon1 = util.DegreeToRadians(lon1)

	lat2Part1 := math.Sin(lat1) * math.Cos(dr)
	lat2Part2 := math.Cos(lat1) * math.Sin(dr) * math.Cos(bearing)

	lat2 := math.Asin(lat2Part1 + lat2Part2)

	lon2Part1 := math.Sin(bearing) * math.Sin(dr) * math.Cos(lat1)
	lon2Part2 := math.Cos(dr) - (math.Sin(lat1) * math.Sin(lat2))

	lon2 := lon1 + math.Atan2(lon2Part1, lon2Part2)

	return radToDeg(lat2), normalizeLongitude(radToDeg(lon2))
}

// normalizeLongitude. long in degree
func normalizeLongitude(long float64) float64 {
	return math.Mod((long+540), 360) - 180.0
}

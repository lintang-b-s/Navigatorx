package geo

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
BearingTo. menghitung sudut initial bearing untuk edge (p1,p2).
https://www.movable-type.co.uk/scripts/latlong.html
*/
func BearingTo(p1Lat, p1Lon, p2Lat, p2Lon float64) float64 {

	dLon := util.DegreeToRadians(p2Lon - p1Lon)

	lat1 := util.DegreeToRadians(p1Lat)
	lat2 := util.DegreeToRadians(p2Lat)

	y := math.Sin(dLon) * math.Cos(lat2)
	x := math.Cos(lat1)*math.Sin(lat2) -
		math.Sin(lat1)*math.Cos(lat2)*math.Cos(dLon)
	brng := math.Mod(util.RadiansToDegree(math.Atan2(y, x))+360, 360.0)

	return brng
}

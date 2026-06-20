package geo

import (
	"math"
)

/*
mercator
project  wgs84 ellipsoidal datum  (lon, lat) coord to (x,y) cartesian coord
conformal

ref:
[1] https://www.hydrometronics.com/downloads/Web%20Mercator%20-%20Non-Conformal,%20Non-Mercator%20%28notes%29.pdf
[2] https://en.wikipedia.org/wiki/Mercator_projection#Properties
[3] https://proj.org/en/stable/operations/projections/webmerc.html
[4] https://wiki.openstreetmap.org/wiki/Mercator

*/

const (
	// taken from: https://en.wikipedia.org/wiki/Web_Mercator_projection
	//	Because the Mercator projects the poles at infinity,
	//  a map using the Web Mercator projection cannot show the poles.
	//  Services such as Google Maps cut off coverage at 85.051129° north and south.

	minLatDeg = -85.05112
	maxLatDeg = 85.05112
	maxError  = 0.04
	R         = 6378.0
)

// https://en.wikipedia.org/wiki/Mercator_projection#Properties
func clampLat(lat float64) float64 {
	return max(min(maxLatDeg, lat), minLatDeg)
}

func Radians(deg float64) float64 {
	return deg * math.Pi / 180
}

func Degrees(rad float64) float64 {
	return rad * 180 / math.Pi
}

func CalcYToLat(y float64) float64 {
	return Degrees(2*math.Atan(math.Exp(y/R)) - math.Pi/2)
}

func CalcLatToY(lat float64) float64 {
	lat = clampLat(lat)
	return R * math.Log(math.Tan(math.Pi/4+Radians(lat)/2))
}

func CalcXToLon(x float64) float64 {
	return Degrees(x / R)
}

func CalcLonToX(lon float64) float64 {
	return R * Radians(lon)
}

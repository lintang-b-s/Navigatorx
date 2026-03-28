package geo

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
web mercator projected coordinate reference system
project  wgs84 ellipsoidal datum  (lon, lat) coord to (x,y) cartesian coord
not-conformal
faster than spherical mercator & ellipsoidal mercator

ref:
[1] https://www.hydrometronics.com/downloads/Web%20Mercator%20-%20Non-Conformal,%20Non-Mercator%20%28notes%29.pdf
[2] https://en.wikipedia.org/wiki/Web_Mercator_projection
[3] https://proj.org/en/stable/operations/projections/webmerc.html

*/

const (
	// taken from: https://en.wikipedia.org/wiki/Web_Mercator_projection
	//	Because the Mercator projects the poles at infinity,
	//  a map using the Web Mercator projection cannot show the poles.
	//  Services such as Google Maps cut off coverage at 85.051129° north and south.

	minLatDeg = -85.05112
	maxLatDeg = 85.05112
	maxError  = 0.1
)

// https://en.wikipedia.org/wiki/Web_Mercator_projection
func clampLat(lat float64) float64 {
	return util.MaxFloat(util.MinFloat(maxLatDeg, lat), minLatDeg)
}

// calcLatToYExact. calculate northing projected web mercator coordinate of lat using the formula
// https://proj.org/en/stable/operations/projections/webmerc.html
func CalcLatToY(lat float64) float64 {
	lat = clampLat(lat)
	lat = util.DegreeToRadians(lat)
	return math.Log(math.Tan(math.Pi/4 + lat/2))
}

// calcLatToYExact. calculate easting projected web mercator coordinate of lat
func CalcLonToX(lon float64) float64 {
	return util.DegreeToRadians(lon)
}

// invGudermanMaclaurinSeries. evaluate inverse gudemannian function at lat using its maclaurin series expansion with deg terms
// https://mathworld.wolfram.com/InverseGudermannian.html
// lat in radian
func invGudermanMaclaurinSeries(lat float64, deg int) float64 {
	approx := 0.0
	for i := 1; i <= deg; i++ {
		numerator := invGudermanMaclaurinNumerators[i-1] * math.Pow(lat, 2*float64(i)-1)
		denominator := invGudermanMaclaurinDenominators[i-1]
		term := numerator / denominator
		approx += term
	}
	return approx
}

// calcLatToYApprox. calculate northing projected web mercator coordinate of lat using inverse guderman maclaurin series expansion
// lat in wgs84 ellipsoidal datum coordinates
func CalcLatToYApprox(lat float64) float64 {
	lat = clampLat(lat)
	return invGudermanMaclaurinSeries(util.DegreeToRadians(lat), bestNumMaclaurinTerms)
}

// calcBestNumsOfTermsInvGudermanMaclaurinSeries. compute number of terms in inv guderman maclaurin series exp s.t
// the returned degree max error is less than maxError
func calcBestNumsOfTermsInvGudermanMaclaurinSeries() int {

	for deg := 1; deg <= maxNumMaclaurinTerms; deg++ {
		maxRemainder := 0.0
		for lat := minLatDeg; util.Lt(lat, maxLatDeg); lat += 0.01 {
			approx := invGudermanMaclaurinSeries(util.DegreeToRadians(lat), deg)
			exact := CalcLatToY(lat)
			remainder := math.Abs(exact - approx)
			maxRemainder = util.MaxFloat(maxRemainder, remainder)
		}

		if util.Lt(maxRemainder, maxError) {
			return deg
		}
	}

	return maxNumMaclaurinTerms
}

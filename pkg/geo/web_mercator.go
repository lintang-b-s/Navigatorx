package geo

import (
	"math"
	"math/rand"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
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
	maxError  = 0.04
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

// https://rosettacode.org/wiki/Horner%27s_rule_for_polynomial_evaluation
func horner(x float64, coeffs []float64, deg int) (acc float64) {
	for i := deg - 1; i >= 0; i-- {
		acc = acc*x + coeffs[i]
	}

	return acc
}

// invGudermanMaclaurinSeries. evaluate inverse gudemannian function at lat using its maclaurin series approximation with deg terms (radius of convergence is pi/2 rad)
// https://mathworld.wolfram.com/InverseGudermannian.html
// https://en.wikipedia.org/wiki/Gudermannian_function#Taylor_series
// lat in radian
func invGudermanMaclaurinSeries(lat float64, deg int) float64 {
	approx := horner(lat, invGudermanMaclaurinCoeffs, deg)
	return approx
}

func factorial(n float64) float64 {
	if util.Eq(n, 0) {
		return 1
	}

	return n * factorial(n-1)
}

// horner rule coefficients  https://rosettacode.org/wiki/Horner%27s_rule_for_polynomial_evaluation
// inverse guderman taylor series: https://en.wikipedia.org/wiki/Gudermannian_function#Taylor_series
func invGudermanMaclaurinSeriesCoefficients() []float64 {
	coeffs := make([]float64, maxNumMaclaurinTerms+1)
	for i := 0; i < maxNumMaclaurinTerms; i++ {

		numerator := eulerSecantNumbers[i]
		denominator := factorial(float64(i + 1))
		coeffs[i+1] = numerator / denominator
	}

	return coeffs
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
		for lat := minLatDeg; util.Lt(lat, maxLatDeg); lat += 0.001 {
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

func RandomCoordinate(bb *da.BoundingBox, rd *rand.Rand) da.Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return da.NewCoordinate(lat, lon)
}

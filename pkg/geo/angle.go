package geo

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
BearingTo. menghitung sudut initial bearing untuk edge (p1,p2).
https://www.movable-type.co.uk/scripts/latlong.html
return in degree
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

const (
	COLLINEAR_EPS = 1e-10
)

// cross. returns the cross product of two vectors a and b
func cross(ax, ay, bx, by float64) float64 {
	return ax*by - ay*bx
}

// collinear. returns true if point r is on the same line as the line pq
func collinear(px, py, qx, qy, rx, ry float64) bool {
	cp := math.Abs(cross(qx-px, qy-py, rx-px, ry-py))
	return cp < COLLINEAR_EPS
}

// PolylineCollinear. return true jika semua points diantara endpoint (tail,head) dari coords is on the same line as the line (tail,head)
func IsPolylineCollinear(coords da.Coordinates) bool {
	if len(coords) == 2 {
		return true
	}
	tailX, tailY := CalcLonToX(coords[0].GetLon()), CalcLatToYApprox(coords[0].GetLat())
	headX, headY := CalcLonToX(coords[len(coords)-1].GetLon()), CalcLatToYApprox(coords[len(coords)-1].GetLat())

	for i := 1; i < len(coords)-1; i++ {
		rx, ry := CalcLonToX(coords[i].GetLon()), CalcLatToYApprox(coords[i].GetLat())
		if !collinear(tailX, tailY, headX, headY, rx, ry) {
			return false
		}
	}

	return true
}

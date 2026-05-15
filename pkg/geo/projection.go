package geo

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// return in meter
func PointLinePerpendicularDistance(pointA da.Coordinate, pointB da.Coordinate,
	snap da.Coordinate) float64 {
	projectionPoint := ProjectPointOnSegment(pointA, pointB, snap)

	dist := CalculateEuclideanDistMercatorProj(snap.GetLat(), snap.GetLon(), projectionPoint.GetLat(), projectionPoint.GetLon())
	return dist
}

func ProjectPointOnSegment(pointA da.Coordinate, pointB da.Coordinate,
	qCoord da.Coordinate) da.Coordinate {

	ax, ay := CalcLonToX(pointA.GetLon()), CalcLatToY(pointA.GetLat())
	bx, by := CalcLonToX(pointB.GetLon()), CalcLatToY(pointB.GetLat())
	qx, qy := CalcLonToX(qCoord.GetLon()), CalcLatToY(qCoord.GetLat())

	abx, aby := bx-ax, by-ay
	aqx, aqy := qx-ax, qy-ay

	numerator := abx*aqx + aby*aqy
	denominator := abx*abx + aby*aby

	var t float64
	if util.Eq(denominator, 0) {
		t = 0
	} else {
		t = numerator / denominator
	}

	t = util.MaxFloat(0, util.MinFloat(1, t))

	projx, projy := ax+t*abx, ay+t*aby

	projLon, projLat := CalcXToLon(projx), CalcYToLat(projy)

	return da.NewCoordinate(projLat, projLon)
}

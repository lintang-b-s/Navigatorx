package geo

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// PointLinePerpendicularDistance handles point line perpendicular distance.
// return in meter
func PointLinePerpendicularDistance(pointA da.Coordinate, pointB da.Coordinate,
	snap da.Coordinate) float64 {
	projectionPoint := ProjectPointOnSegment(pointA, pointB, snap)

	dist := CalculateEuclideanDistMercatorProj(snap.GetLat(), snap.GetLon(), projectionPoint.GetLat(), projectionPoint.GetLon())
	return dist
}

// ProjectPointOnSegment handles project point on segment.
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

	t = max(0, min(1, t))

	projx, projy := ax+t*abx, ay+t*aby

	projLon, projLat := CalcXToLon(projx), CalcYToLat(projy)

	return da.NewCoordinate(projLat, projLon)
}

// ProjectPointOnSegmentDirect handles project point on segment.
func ProjectPointOnSegmentDirect(ax, ay, bx, by,
	qx, qy float64) (float64, float64) {

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

	t = max(0, min(1, t))

	projx, projy := ax+t*abx, ay+t*aby
	return projx, projy
}

// ProjectPointOnSegmentGeometry handles project point on segment geometry.
func ProjectPointOnSegmentGeometry(segGeometry []da.Coordinate, lat, lon float64) (da.Index, da.Coordinate, float64, float64) {
	minDist := pkg.InfWeight
	var bestpx, bestpy float64 //  best projected point
	n := len(segGeometry)

	stailDist := pkg.InfWeight //  dist dari tail  vertex dari this road segment id ke titik proyeksi (lat,lon) to this road segment
	cumDist := 0.0
	qx, qy := CalcLonToX(lon), CalcLatToY(lat)
	lastIndex := 0
	for i := 0; i < n-1; i++ {
		tail := segGeometry[i]
		head := segGeometry[i+1]
		tailx, taily := CalcLonToX(tail.GetLon()), CalcLatToY(tail.GetLat())
		headx, heady := CalcLonToX(head.GetLon()), CalcLatToY(head.GetLat())

		px, py := ProjectPointOnSegmentDirect(
			tailx, taily,
			headx, heady,
			qx, qy,
		)

		dist := CalculateEuclideanDist(px, py,
			qx, qy) // dist dari (lat,lon) ke titik proyeksi

		if util.Lt(dist, minDist) {
			minDist = dist
			bestpx, bestpy = px, py
			lastIndex = i
			cumDist += CalculateEuclideanDist(tailx, taily,
				px, py) // dist dari tail (lat,lon) ke titik proyeksi
			stailDist = cumDist
		}

		cumDist += CalculateEuclideanDist(tailx, taily, headx, heady)
	}

	pPoint := da.NewCoordinate(CalcYToLat(bestpy), CalcXToLon(bestpx))
	return da.Index(lastIndex), pPoint, minDist, stailDist
}

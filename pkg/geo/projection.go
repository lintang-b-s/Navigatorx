package geo

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
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

	t = max(0, min(1, t))

	projx, projy := ax+t*abx, ay+t*aby

	projLon, projLat := CalcXToLon(projx), CalcYToLat(projy)

	return da.NewCoordinate(projLat, projLon)
}

func ProjectPointOnSegmentGeometry(segGeometry []da.Coordinate, lat, lon float64) (da.Index, da.Coordinate, float64, float64) {
	minDist := pkg.INF_WEIGHT
	var pPoint da.Coordinate //  best projected point
	n := len(segGeometry)

	stailDist := pkg.INF_WEIGHT //  dist dari tail  vertex dari this road segment id ke titik proyeksi (lat,lon) to this road segment
	cumDist := 0.0

	lastIndex := 0
	for i := 0; i < n-1; i++ {
		tail := segGeometry[i]
		head := segGeometry[i+1]
		projectedPoint := ProjectPointOnSegment(
			tail,
			head,
			da.Coordinate(da.NewCoordinate(lat, lon)),
		)

		plat, plon := projectedPoint.GetLat(), projectedPoint.GetLon()
		dist := CalculateEuclideanDistMercatorProj(plat, plon,
			lat, lon) // dist dari (lat,lon) ke titik proyeksi

		if util.Lt(dist, minDist) {
			minDist = dist
			pPoint = projectedPoint
			lastIndex = i
			cumDist += CalculateEuclideanDistMercatorProj(tail.GetLat(), tail.GetLon(),
				plat, plon) // dist dari (lat,lon) ke titik proyeksi
			stailDist = cumDist
		}

		cumDist += CalculateEuclideanDistMercatorProj(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
	}
	return da.Index(lastIndex), pPoint, minDist, stailDist

}

// func Project2DPointOnSegment(geom []da.Coordinate, qx, qy float64) (float64, float64, float64) {

// 	px, py := 0.0, 0.0
// 	minDist := math.MaxFloat64
// 	for i := 0; i < len(geom)-1; i++ {
// 		a := geom[i]
// 		b := geom[i+1]
// 		ax, ay := CalcLonToX(a.GetLon()), CalcLatToY(a.GetLat())
// 		bx, by := CalcLonToX(b.GetLon()), CalcLatToY(b.GetLat())
// 		abx, aby := bx-ax, by-ay
// 		aqx, aqy := qx-ax, qy-ay

// 		numerator := abx*aqx + aby*aqy
// 		denominator := abx*abx + aby*aby

// 		var t float64
// 		if util.Eq(denominator, 0) {
// 			t = 0
// 		} else {
// 			t = numerator / denominator
// 		}
// 		t = max(0, min(1, t))
// 		projx, projy := ax+t*abx, ay+t*aby

// 		dx, dy := (qx - projx), (qy - projy)
// 		dist := dx*dx + dy*dy

// 		if util.Lt(dist, minDist) {
// 			minDist = dist
// 			px, py = projx, projy
// 		}
// 	}

// 	return px, py, minDist
// }

package guidance

import (
	"iter"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

func (db *DirectionBuilder) buildSimplifiedAnnotation(segIds []da.Index) da.Annotation {
	avgSpeed := 0.0
	n := 0
	for _, segId := range segIds {
		avgSpeed += db.metrics.GetSegmentSpeed(segId)
		geomLength := db.rn.GetSegmentGeometryLength(segId)
		n += int(geomLength) - 1
	}
	avgSpeed /= max(float64(len(segIds)), 1)

	simplifiedDistance := make([]float64, 0, n)
	simplifiedDuration := make([]float64, 0, n)

	m := len(segIds)
	geomOffset := make([]uint32, 0, m+1)
	geomCount := uint32(0)
	for _, segId := range segIds {
		j := 0
		prevc := da.NewCoordinate(0, 0)
		for c := range db.rn.ForSegmentGeometry(segId) {
			geomOffset = append(geomOffset, geomCount)
			if j > 0 {
				dist := geo.CalculateEuclideanDistMercatorProj(prevc.GetLat(), prevc.GetLon(), c.GetLat(), c.GetLon())
				simplifiedDistance = append(simplifiedDistance, dist)
				simplifiedDuration = append(simplifiedDuration, dist/avgSpeed)
				geomCount++
			}
			prevc = c
			j++
		}
	}

	geometry := da.GooglePolylineFromCoords(db.forRouteCoordinates(segIds), int(geomCount)+n)
	return da.NewAnnotation(simplifiedDuration, simplifiedDistance, geometry, geomOffset)
}

func (db *DirectionBuilder) forRouteCoordinates(segIds []da.Index) iter.Seq[da.Coordinate] {
	return func(yield func(da.Coordinate) bool) {
		db.rn.YieldRouteCoordinates(segIds, yield)
	}
}

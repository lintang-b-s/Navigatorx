package guidance

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

func (db *DirectionBuilder) buildSimplifiedAnnotation(segIds []da.Index, geometry da.Coordinates) da.Annotation {
	avgSpeed := 0.0
	for _, segID := range segIds {
		avgSpeed += db.engine.GetSegmentSpeed(segID)
	}
	avgSpeed /= max(float64(len(segIds)), 1)

	m := len(segIds)
	if m > 0 {
		lSegId := segIds[m-1]
		l := int(db.rn.GetSegmentGeometryLength(lSegId)) - 1
		if l >= 0 {
			lPoint := db.rn.GetSegmentGeometryPoint(lSegId, int(l))
			geometry = append(geometry, lPoint)
		}
	}
	n := len(geometry)

	if n <= 1 {
		segGeomOffset := db.buildEdgeGeomOffsetFromGeometry(segIds, geometry)
		return da.NewAnnotation([]float64{}, []float64{}, geometry, segGeomOffset)
	}

	simplifiedDistance := make([]float64, 0, n)
	simplifiedDuration := make([]float64, 0, n)
	for i := 0; i < n-1; i++ { // O(n), n=len(geometry)
		curr := geometry[i]
		next := geometry[i+1]
		dist := geo.CalculateEuclideanDistMercatorProj(curr.GetLat(), curr.GetLon(), next.GetLat(), next.GetLon())
		simplifiedDistance = append(simplifiedDistance, dist)
		simplifiedDuration = append(simplifiedDuration, dist/avgSpeed)
	}

	segGeomOffset := db.buildEdgeGeomOffsetFromGeometry(segIds, geometry)
	return da.NewAnnotation(simplifiedDuration, simplifiedDistance, geometry, segGeomOffset)
}

func (db *DirectionBuilder) buildEdgeGeomOffsetFromGeometry(segIds []da.Index, geometry da.Coordinates) []da.Index {
	if len(segIds) == 0 || len(geometry) == 0 {
		return []da.Index{}
	}

	segGeomOffset := make([]da.Index, 0, len(segIds))

	offset := da.Index(0)
	for i := 0; i < len(segIds); i++ { // O(n)
		segId := segIds[i]
		geomLength := db.rn.GetSegmentGeometryLength(segId)
		segGeomOffset = append(segGeomOffset, offset)
		offset += da.Index(geomLength - 1)
	}

	return segGeomOffset
}

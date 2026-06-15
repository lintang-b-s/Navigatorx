package guidance

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

func (db *DirectionBuilder) buildSimplifiedAnnotation(edgeIds []da.Index, geometry da.Coordinates) da.Annotation {
	avgSpeed := 0.0

	for _, edgeID := range edgeIds {
		avgSpeed += db.engine.GetSegmentSpeed(edgeID, true)
	}
	avgSpeed /= max(float64(len(edgeIds)), 1)
	avgSpeed = max(avgSpeed, 1.0)

	m := len(edgeIds)
	if m > 0 {
		lastEdgeID := edgeIds[m-1]
		lastEdgeGeomIdx := db.graph.GetEdgeGeometryLength(lastEdgeID) - 1
		if lastEdgeGeomIdx >= 0 {
			lastEdgeGeomPoint := db.graph.GetEdgeGeometryPoint(lastEdgeID, lastEdgeGeomIdx)
			geometry = append(geometry, lastEdgeGeomPoint)
		}
	}
	n := len(geometry)

	if n <= 1 {
		edgeGeomOffset := db.buildEdgeGeomOffsetFromGeometry(edgeIds, geometry)
		return da.NewAnnotation([]float64{}, []float64{}, geometry, edgeGeomOffset)
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

	edgeGeomOffset := db.buildEdgeGeomOffsetFromGeometry(edgeIds, geometry)
	return da.NewAnnotation(simplifiedDuration, simplifiedDistance, geometry, edgeGeomOffset)
}

func (db *DirectionBuilder) buildEdgeGeomOffsetFromGeometry(edgeIds []da.Index, geometry da.Coordinates) []da.Index {
	if len(edgeIds) == 0 || len(geometry) == 0 {
		return []da.Index{}
	}

	edgeGeomOffset := make([]da.Index, 0, len(edgeIds))

	offset := da.Index(0)
	for i := 0; i < len(edgeIds); i++ { // O(n)
		edgeId := edgeIds[i]
		geomLength := db.graph.GetEdgeGeometryLength(edgeId)
		edgeGeomOffset = append(edgeGeomOffset, offset)
		offset += da.Index(geomLength - 1)
	}

	return edgeGeomOffset
}

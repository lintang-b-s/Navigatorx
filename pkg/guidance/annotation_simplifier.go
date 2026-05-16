package guidance

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

func (db *DirectionBuilder) buildSimplifiedAnnotation(edgeIDs []da.Index, geometry da.Coordinates) da.Annotation {
	simplifiedGeometry := dedupeConsecutivePoints(geometry)
	if len(simplifiedGeometry) == 0 {
		return da.NewAnnotation([]float64{}, []float64{}, da.Coordinates{}, []da.Index{})
	}

	avgSpeed := 1.0

	for _, edgeID := range edgeIDs {
		avgSpeed += db.engine.GetSegmentSpeed(edgeID, true)
	}
	avgSpeed /= float64(len(edgeIDs))
	avgSpeed = max(avgSpeed, 1.0)

	simplifiedDistance := make([]float64, 0, max(len(simplifiedGeometry)-1, 0))
	simplifiedDuration := make([]float64, 0, max(len(simplifiedGeometry)-1, 0))

	for i := 0; i < len(simplifiedGeometry)-1; i++ { // O(n), n=len(simplifiedGeometry)
		curr := simplifiedGeometry[i]
		next := simplifiedGeometry[i+1]
		dist := geo.CalculateEuclideanDistMercatorProj(curr.GetLat(), curr.GetLon(), next.GetLat(), next.GetLon())
		simplifiedDistance = append(simplifiedDistance, dist)

		simplifiedDuration = append(simplifiedDuration, dist/avgSpeed)
	}

	simplifiedEdgeGeomOffset := db.buildEdgeGeomOffsetFromSimplifiedGeometry(edgeIDs, simplifiedGeometry)
	return da.NewAnnotation(simplifiedDuration, simplifiedDistance, simplifiedGeometry, simplifiedEdgeGeomOffset)
}

func dedupeConsecutivePoints(geometry da.Coordinates) da.Coordinates {
	if len(geometry) == 0 {
		return da.Coordinates{}
	}

	deduped := make(da.Coordinates, 0, len(geometry))
	deduped = append(deduped, geometry[0])
	for i := 1; i < len(geometry); i++ { // O(n)
		if !da.IsSameCoordinate(geometry[i], geometry[i-1]) {
			deduped = append(deduped, geometry[i])
		}
	}
	return deduped
}

func (db *DirectionBuilder) buildEdgeGeomOffsetFromSimplifiedGeometry(edgeIDs []da.Index, simplifiedGeometry da.Coordinates) []da.Index {
	if len(edgeIDs) == 0 || len(simplifiedGeometry) == 0 {
		return []da.Index{}
	}

	simplifiedEdgeGeomOffset := make([]da.Index, 0, len(edgeIDs))
	lastMatch := 0
	for _, edgeID := range edgeIDs { // O(n*m*s), n=len(edgeIDs), m=len(eGeom), s=len(simplifiedGeometry)
		eGeom := db.graph.GetEdgeGeometry(edgeID)
		matched := lastMatch
		for i := lastMatch; i < len(simplifiedGeometry); i++ {
			if containsCoordinate(eGeom, simplifiedGeometry[i]) {
				matched = i
				break
			}
		}
		simplifiedEdgeGeomOffset = append(simplifiedEdgeGeomOffset, da.Index(matched))
		lastMatch = matched
	}
	return simplifiedEdgeGeomOffset
}

func containsCoordinate(geometry []da.Coordinate, c da.Coordinate) bool {
	for _, p := range geometry {
		if da.IsSameCoordinate(p, c) {
			return true
		}
	}
	return false
}

package usecases

import (
	"errors"
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

func (rs *RoutingService) snapOrigDestToNearbyEdges(origLat, origLon, dstLat, dstLon float64) (datastructure.Index,
	datastructure.Index, error) {
	// find nearest orig edge (inEdgeOffset) to origLat, origLon
	origCandidates := rs.spatialIndex.SearchWithinRadius(origLat, origLon, rs.searchRadius)
	if len(origCandidates) == 0 {
		return 0, 0, errors.New("no origin candidates found")
	}

	// find nearest dst edge (outEdgeOffset) to dstLat, dstLon
	dstCandidates := rs.spatialIndex.SearchWithinRadius(dstLat, dstLon, rs.searchRadius)
	if len(dstCandidates) == 0 {
		return 0, 0, errors.New("no destination candidates found")
	}

	// sort origCandidates by distance to origLat, origLon
	// sort dstCandidates by distance to dstLat, dstLon
	origDist := make([]float64, len(origCandidates))
	dstDist := make([]float64, len(dstCandidates))
	for i, c := range origCandidates {
		cLat, cLon := rs.engine.GetVertexCoordinatesFromOutEdge(c.GetExitOffset())
		origDist[i] = geo.CalculateHaversineDistance(origLat, origLon, cLat, cLon)
	}
	for i, c := range dstCandidates {
		cLat, cLon := rs.engine.GetVertexCoordinatesFromInEdge(c.GetEntryOffset())
		dstDist[i] = geo.CalculateHaversineDistance(dstLat, dstLon, cLat, cLon)
	}

	sortedOrigId := make([]int, len(origCandidates))
	for i := range sortedOrigId {
		sortedOrigId[i] = i
	}

	sortedDestId := make([]int, len(dstCandidates))
	for i := range sortedDestId {
		sortedDestId[i] = i
	}

	sort.Slice(sortedOrigId, func(i, j int) bool {
		return origDist[sortedOrigId[i]] < origDist[sortedOrigId[j]]
	})

	sort.Slice(sortedDestId, func(i, j int) bool {
		return dstDist[sortedDestId[i]] < dstDist[sortedDestId[j]]
	})

	sortedOrig := make([]spatialindex.ArcEndpoint, len(origCandidates))
	sortedDest := make([]spatialindex.ArcEndpoint, len(dstCandidates))
	for i, newId := range sortedOrigId {
		sortedOrig[i] = origCandidates[newId]
	}

	for i, newId := range sortedDestId {
		sortedDest[i] = dstCandidates[newId]
	}

	// for _, o := range origCandidates {
	// 	for _, d := range dstCandidates {
	// 		if rs.engine.VerticeUandVAreConnected(o.GetExitOffset(), d.GetEntryOffset()) {
	// 			return o.GetExitOffset(), d.GetEntryOffset(), nil
	// 		}
	// 	}
	// }

	return sortedOrig[0].GetExitOffset(), sortedDest[0].GetEntryOffset(), nil
}

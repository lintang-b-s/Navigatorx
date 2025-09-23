package usecases

import (
	"errors"
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
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

	sort.Slice(origCandidates, func(i, j int) bool {
		return origDist[i] < origDist[j]
	})

	sort.Slice(dstCandidates, func(i, j int) bool {
		return dstDist[i] < dstDist[j]
	})

	// for _, o := range origCandidates {
	// 	for _, d := range dstCandidates {
	// 		if rs.engine.VerticeUandVAreConnected(o.GetExitOffset(), d.GetEntryOffset()) {
	// 			return o.GetExitOffset(), d.GetEntryOffset(), nil
	// 		}
	// 	}
	// }

	return origCandidates[0].GetExitOffset(), dstCandidates[0].GetEntryOffset(), nil
	// return 0, 0, errors.New("no connected origin and destination candidates found")
}

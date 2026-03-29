package usecases

import (
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (rs *RoutingService) SnapOrigDestToNearbyEdges(origLat, origLon, dstLat, dstLon, searchRad float64,
	prevPairSet map[uint64]struct{}) (da.Index,
	da.Index, da.Coordinate, da.Coordinate) {
	var (
		projectedLat, projectedLon float64
	)

	// find nearest orig edge (inEdgeOffset) to origLat, origLon
	origCandidates := rs.spatialIndex.SearchWithinRadius(origLat, origLon, searchRad)

	// find nearest dst edge (outEdgeOffset) to dstLat, dstLon
	dstCandidates := rs.spatialIndex.SearchWithinRadius(dstLat, dstLon, searchRad)

	// sort origCandidates by distance to origLat, origLon
	// sort dstCandidates by distance to dstLat, dstLon
	origDist := make([]float64, len(origCandidates))
	dstDist := make([]float64, len(dstCandidates))
	origCoord := make([]da.Coordinate, len(origCandidates))
	dstCoord := make([]da.Coordinate, len(dstCandidates))

	for i, c := range origCandidates {
		projectedLat, projectedLon, origDist[i] = rs.ProjectCoordinateToEdge(origLat, origLon, c.GetId())
		origCoord[i] = da.NewCoordinate(projectedLat, projectedLon)
	}

	for i, c := range dstCandidates {
		projectedLat, projectedLon, dstDist[i] = rs.ProjectCoordinateToEdge(dstLat, dstLon, c.GetId())
		dstCoord[i] = da.NewCoordinate(projectedLat, projectedLon)
	}

	sortedOrigId := make([]int, len(origCandidates))
	for i := range sortedOrigId {
		sortedOrigId[i] = i
	}

	sortedDstId := make([]int, len(dstCandidates))
	for i := range sortedDstId {
		sortedDstId[i] = i
	}

	sort.Slice(sortedOrigId, func(i, j int) bool {
		return origDist[sortedOrigId[i]] < origDist[sortedOrigId[j]]
	})

	sort.Slice(sortedDstId, func(i, j int) bool {
		return dstDist[sortedDstId[i]] < dstDist[sortedDstId[j]]
	})

	sortedOrig := make([]spatialindex.ArcEndpoint, len(origCandidates))
	sortedDst := make([]spatialindex.ArcEndpoint, len(dstCandidates))
	sortedOrigCoord := make([]da.Coordinate, len(origCandidates))
	sortedDstCoord := make([]da.Coordinate, len(dstCandidates))

	for i, oldId := range sortedOrigId {
		sortedOrig[i] = origCandidates[oldId]
		sortedOrigCoord[i] = origCoord[oldId]
	}

	for i, oldId := range sortedDstId {
		sortedDst[i] = dstCandidates[oldId]
		sortedDstCoord[i] = dstCoord[oldId]
	}

	g := rs.engine.GetGraph()

	// origDestination

	minDist := pkg.INF_WEIGHT

	bestPair := newOriginDestination(da.INVALID_EDGE_ID, da.INVALID_EDGE_ID,
		da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON), da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON))
	for i, o := range sortedOrig {
		for j, d := range sortedDst {
			destinationTail, dstInEdge := g.GetTailOfOutedgeWithInEdge(d.GetId())
			originHead := g.GetHeadOfOutEdge(o.GetId())
			if rs.isAlreadyEvaluated(o.GetId(), d.GetId(), prevPairSet) {
				continue
			}
			
			if !rs.engine.VerticeUToVConnected(originHead, destinationTail) {
				continue
			}

			origDestSnapDist := origDist[sortedOrigId[i]] + dstDist[sortedDstId[j]]

			if origDestSnapDist < minDist {
				minDist = origDestSnapDist
				bestPair = newOriginDestination(o.GetId(), dstInEdge.GetEdgeId(), sortedOrigCoord[i], sortedDstCoord[j])
			}
		}
	}

	return bestPair.origEdgeId, bestPair.destEdgeId, bestPair.origCoord, bestPair.destCoord
}

type originDestination struct {
	origEdgeId, destEdgeId da.Index
	origCoord, destCoord   da.Coordinate
}

func newOriginDestination(origEdgeId, destEdgeId da.Index, origCoord, destCoord da.Coordinate) originDestination {
	return originDestination{
		origEdgeId: origEdgeId,
		destEdgeId: destEdgeId,
		origCoord:  origCoord,
		destCoord:  destCoord,
	}
}

func (rs *RoutingService) ProjectCoordinateToEdge(lat, lon float64, edgeId da.Index) (float64, float64, float64) {

	eGeometry := rs.engine.GetGraph().GetEdgeGeometry(edgeId)
	minDist := pkg.INF_WEIGHT
	var bestProjectedPoint geo.Coordinate

	for i := 0; i < len(eGeometry)-1; i++ {
		tail := eGeometry[i]
		head := eGeometry[i+1]
		projectedPoint := geo.ProjectPointOnSegment(
			tail.ToGeoCoordinate(),
			head.ToGeoCoordinate(),
			geo.Coordinate(da.NewCoordinate(lat, lon)),
		)

		dist := geo.CalculateEuclidianDistWebMercatorProj(projectedPoint.Lat, projectedPoint.Lon,
			lat, lon)

		if dist < minDist {
			minDist = dist
			bestProjectedPoint = projectedPoint
		}
	}

	return bestProjectedPoint.GetLat(), bestProjectedPoint.GetLon(), minDist
}

func (rs *RoutingService) notFoundOriginDestinationWithinRadius(seId, teId da.Index) bool {
	if seId == da.INVALID_EDGE_ID && teId == da.INVALID_EDGE_ID {
		return true
	}
	return false
}

func (rs *RoutingService) isAlreadyEvaluated(orig, dest da.Index, prevPairSet map[uint64]struct{}) bool {
	pairKey := util.Bitpack(uint32(orig), uint32(dest))
	if _, alreadyEvaluated := prevPairSet[pairKey]; alreadyEvaluated {
		return true
	}

	prevPairSet[pairKey] = struct{}{}
	return false
}

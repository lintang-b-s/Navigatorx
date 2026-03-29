package usecases

import (
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
SnapOrigDestToNearbyRoadSegments. snap origin dan destination query ke road segment terdekatnya.
serta terdapat path dari head dari road segment origin ke tail dari road segment destination hasil snap.

see: https://blog.mapbox.com/robust-navigation-with-smart-nearest-neighbor-search-dbc1f6218be8

let w adalah head dari kandidat road segment origin dan q adalah tail dari kandidat road segment destination
kalau tidak ada nearby road segments (dari source dan destination query) atau setiap pair road segments yang dievaluate dari round ini gak ada path (dari w ke q),
jalanin lagi SnapOrigDestToNearbyRoadSegmentsByradius() dengan search radius 2x dari radius sebelumnya dan kita gak evaluate lagi evaluated candidate pairs di all previous rounds.
kenapa??
1. karena kita tahu evaluated candidate pairs gak ada path (dari w ke q) di all previous rounds.
2. masih ada kemungkinan terdapat path dari old origCandidates ke new dstCandidates

let q=number of rounds until searchRead exceeds MAX_SEARCH_RADIUS
let M=number of road segments/edges in the graph
let c=max number of road segments/edges returned by rtree spatial index
let V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
worst case: O(q*(M + c^2 * (V_G+O_G)))
*/
func (rs *RoutingService) SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon float64,
) (da.Index,
	da.Index, da.Coordinate, da.Coordinate) {
	searchRad := rs.searchRadius
	var (
		as, at                  da.Index      = da.INVALID_EDGE_ID, da.INVALID_EDGE_ID
		snappedOrig, snappedDst da.Coordinate = da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON),
			da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON)
		pairSize           int             = spatialindex.MAX_CANDIDATES * spatialindex.MAX_CANDIDATES
		removedPrevPairSet *da.Set[uint64] = da.NewSet[uint64](pairSize)
	)

	for util.Le(searchRad, MAX_SEARCH_RADIUS) {
		// https://blog.mapbox.com/robust-navigation-with-smart-nearest-neighbor-search-dbc1f6218be8

		as, at, snappedOrig, snappedDst = rs.SnapOrigDestToNearbyRoadSegmentsByradius(qOrigLat, qOrigLon, qDstLat, qDstLon, searchRad, removedPrevPairSet)
		if !rs.notFoundOriginDestinationWithinRadius(as, at) {
			// break loop early if found connected origin and destination
			break
		}
		searchRad *= SEARCH_RADIUS_MULTIPLIER
	}

	return as, at, snappedOrig, snappedDst
}

/*
SnapOrigDestToNearbyRoadSegmentsByradius. snap origin dan destination query ke road segment terdekatnya dalam radius=searchRad,
serta terdapat path dari head dari road segment origin ke tail dari road segment destination hasil snap.
edge (u,v) dari road segment. tail = u, head = v.

let w adalah head dari kandidat road segment origin dan q adalah tail dari kandidat road segment destination
kalau tidak ada nearby road segments (dari source dan destination query) atau setiap pair road segments yang dievaluate dari round ini gak ada path (dari w ke q),
jalanin lagi SnapOrigDestToNearbyRoadSegmentsByradius() dengan search radius 2x dari radius sebelumnya dan kita gak evaluate lagi evaluated candidate pairs di all previous rounds.
kenapa??
1. karena kita tahu evaluated candidate pairs di all previous rounds gak ada path (dari w ke q).
2. masih ada kemungkinan terdapat path dari old origCandidates ke new dstCandidates

let M=number of road segments/edges in the graph
let c=max number of road segments/edges returned by rtree spatial index
let V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
worst case: O(M + c^2 * (V_G+O_G))
*/
func (rs *RoutingService) SnapOrigDestToNearbyRoadSegmentsByradius(qOrigLat, qOrigLon, qDstLat, qDstLon, searchRad float64,
	removedPrevPairSet *da.Set[uint64]) (da.Index,
	da.Index, da.Coordinate, da.Coordinate) {
	var (
		projectedLat, projectedLon float64
	)

	// let M=number of road segments/edges in the graph
	// R-tree search worst case is O(M), avg case is O(logM)
	// find nearest orig edge (inEdgeOffset) to qOrigLat, qOrigLon
	origCandidates := rs.spatialIndex.SearchWithinRadius(qOrigLat, qOrigLon, searchRad)

	// find nearest dst edge (outEdgeOffset) to qDstLat, qDstLon
	dstCandidates := rs.spatialIndex.SearchWithinRadius(qDstLat, qDstLon, searchRad)

	// sort origCandidates by distance to qOrigLat, qOrigLon
	// sort dstCandidates by distance to qDstLat, qDstLon
	origDist := make([]float64, len(origCandidates))
	dstDist := make([]float64, len(dstCandidates))
	origCoord := make([]da.Coordinate, len(origCandidates))
	dstCoord := make([]da.Coordinate, len(dstCandidates))

	for i, c := range origCandidates {
		projectedLat, projectedLon, origDist[i] = rs.ProjectCoordinateToEdge(qOrigLat, qOrigLon, c.GetId())
		origCoord[i] = da.NewCoordinate(projectedLat, projectedLon)
	}

	for i, c := range dstCandidates {
		projectedLat, projectedLon, dstDist[i] = rs.ProjectCoordinateToEdge(qDstLat, qDstLon, c.GetId())
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

	// let c=max number of road segments/edges returned by rtree spatial index

	// worst case of this loop: O(c^2 * (V_G+O_G)), V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
	minDist := pkg.INF_WEIGHT
	bestPair := newOriginDestination(da.INVALID_EDGE_ID, da.INVALID_EDGE_ID,
		da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON), da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON))
	for i, o := range sortedOrig {
		for j, d := range sortedDst {
			destinationTail, dstInEdge := g.GetTailOfOutedgeWithInEdge(d.GetId())
			originHead := g.GetHeadOfOutEdge(o.GetId())
			if rs.isPairAlreadyEvaluated(o.GetId(), d.GetId(), removedPrevPairSet) {
				continue
			}

			// kita set (o,d) evaluated=true mau ada path atau gak ada path dari o ke d.
			rs.evaluate(o.GetId(), d.GetId(), removedPrevPairSet)

			// O(V_G + E_G), V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
			if !rs.engine.PathExists(originHead, destinationTail) {
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

// isPairAlreadyEvaluated. check (orig,dest) pair evaluated==true
func (rs *RoutingService) isPairAlreadyEvaluated(orig, dest da.Index, removedPrevPairSet *da.Set[uint64]) bool {
	pairKey := util.Bitpack(uint32(orig), uint32(dest))
	return removedPrevPairSet.Test(pairKey)
}

// evaluate. set (orig,dest) pair evaluated=true
func (rs *RoutingService) evaluate(orig, dest da.Index, removedPrevPairSet *da.Set[uint64]) {
	pairKey := util.Bitpack(uint32(orig), uint32(dest))
	removedPrevPairSet.Set(pairKey)
}

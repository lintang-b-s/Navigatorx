package usecases

import (
	"github.com/bytedance/gopkg/collection/hashset"
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

return:
outEdgeId dari source road segment.
inEdgeId dari destination road segment.
snapped point (proyeksi titik query source ke road segment terdekat) of source.
snapped point (proyeksi titik query destination ke road segment terdekat) of destination.
edgeGeometry dari source road segment setelah snappedPoint of destination.
edgeGeometry dari source road segment sebelum snappedPoint of destination.
*/
func (rs *RoutingService) SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon float64,
) (da.PhantomNode, da.PhantomNode) {
	searchRad := rs.searchRadius
	var (
		sp, tp             da.PhantomNode    = da.NewInvalidPhantomNode(), da.NewInvalidPhantomNode()
		pairSize           int               = spatialindex.MAX_CANDIDATES * spatialindex.MAX_CANDIDATES
		removedPrevPairSet hashset.Uint64Set = hashset.NewUint64WithSize(pairSize)
	)

	for util.Le(searchRad, MAX_SEARCH_RADIUS) {
		// https://blog.mapbox.com/robust-navigation-with-smart-nearest-neighbor-search-dbc1f6218be8

		sp, tp = rs.SnapOrigDestToNearbyRoadSegmentsByradius(qOrigLat, qOrigLon, qDstLat, qDstLon, searchRad, removedPrevPairSet)
		if !rs.notFoundOriginDestinationWithinRadius(sp, tp) {
			// break loop early if found connected origin and destination
			break
		}
		searchRad *= SEARCH_RADIUS_MULTIPLIER
	}

	return sp, tp
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
	removedPrevPairSet hashset.Uint64Set) (da.PhantomNode, da.PhantomNode) {
	var (
		projectedLat, projectedLon float64
	)

	// let M=number of road segments/edges in the graph
	// R-tree search worst case is O(M), avg case is O(logM)
	// find nearest orig edge (inEdgeOffset) to qOrigLat, qOrigLon
	origCandidates := rs.spatialIndex.SearchWithinRadius(qOrigLat, qOrigLon, searchRad)

	// find nearest dst edge (outEdgeOffset) to qDstLat, qDstLon
	dstCandidates := rs.spatialIndex.SearchWithinRadius(qDstLat, qDstLon, searchRad)

	origDist := make([]float64, len(origCandidates))
	dstDist := make([]float64, len(dstCandidates))
	origCoord := make([]da.Coordinate, len(origCandidates))
	origNextCoords := make([][]da.Coordinate, len(origCandidates))
	dstCoord := make([]da.Coordinate, len(dstCandidates))
	dstNextCoords := make([][]da.Coordinate, len(dstCandidates))

	for i, c := range origCandidates {
		projectedLat, projectedLon, origDist[i], origNextCoords[i] = rs.ProjectCoordinateToEdge(qOrigLat, qOrigLon, c, true)
		origCoord[i] = da.NewCoordinate(projectedLat, projectedLon)
	}

	for i, c := range dstCandidates {
		projectedLat, projectedLon, dstDist[i], dstNextCoords[i] = rs.ProjectCoordinateToEdge(qDstLat, qDstLon, c, false)
		dstCoord[i] = da.NewCoordinate(projectedLat, projectedLon)
	}

	g := rs.engine.GetGraph()

	// origDestination

	// let c=max number of road segments/edges returned by rtree spatial index

	// worst case of this loop: O(c^2 * (V_G+O_G)), V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
	minDist := pkg.INF_WEIGHT
	bestPair := newOriginDestination(da.INVALID_EDGE_ID, da.INVALID_EDGE_ID,
		da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON), da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON))
	bestOriginNextCoords := make([]da.Coordinate, 0)
	bestDestBefCoords := make([]da.Coordinate, 0)

	for i, o := range origCandidates {
		for j, d := range dstCandidates {
			destinationTail, dstInEdge := g.GetTailOfOutedgeWithInEdge(d)
			originHead := g.GetHeadOfOutEdge(o)
			if rs.isPairAlreadyEvaluated(o, d, removedPrevPairSet) {
				continue
			}

			// kita set (o,d) evaluated=true mau ada path atau gak ada path dari o ke d.
			rs.evaluate(o, d, removedPrevPairSet)

			// O(V_G + E_G), V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
			if !rs.engine.PathExists(originHead, destinationTail) {
				continue
			}

			origDestSnapDist := origDist[i] + dstDist[j]

			if origDestSnapDist < minDist {
				minDist = origDestSnapDist
				bestPair = newOriginDestination(o, dstInEdge, origCoord[i], dstCoord[j])
				bestOriginNextCoords = origNextCoords[i]
				bestDestBefCoords = dstNextCoords[j]
			}
		}
	}

	if bestPair.origEdgeId == da.INVALID_EDGE_ID && bestPair.destEdgeId == da.INVALID_EDGE_ID {
		return da.NewInvalidPhantomNode(), da.NewInvalidPhantomNode()
	}

	sForwardTravelTime := rs.engine.GetWeight(bestPair.origEdgeId, true)

	sp := da.NewPhantomNode(bestPair.origCoord, sForwardTravelTime, 0, bestPair.origEdgeId, da.INVALID_EDGE_ID, bestOriginNextCoords,
		make([]da.Coordinate, 0))

	tReverseTravelTime := rs.engine.GetWeight(bestPair.destEdgeId, false)

	tp := da.NewPhantomNode(bestPair.destCoord, 0.0, tReverseTravelTime, da.INVALID_EDGE_ID, bestPair.destEdgeId, make([]da.Coordinate, 0),
		bestDestBefCoords)

	return sp, tp
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

func (rs *RoutingService) ProjectCoordinateToEdge(lat, lon float64, edgeId da.Index, origin bool) (float64, float64, float64, []da.Coordinate) {

	eGeometry := rs.engine.GetGraph().GetEdgeGeometry(edgeId)
	minDist := pkg.INF_WEIGHT
	var bestProjectedPoint da.Coordinate

	lastIndex := 0
	for i := 0; i < len(eGeometry)-1; i++ {
		tail := eGeometry[i]
		head := eGeometry[i+1]
		projectedPoint := geo.ProjectPointOnSegment(
			tail,
			head,
			da.Coordinate(da.NewCoordinate(lat, lon)),
		)

		dist := geo.CalculateEuclidianDistWebMercatorProj(projectedPoint.Lat, projectedPoint.Lon,
			lat, lon)

		if util.Lt(dist, minDist) {
			minDist = dist
			bestProjectedPoint = projectedPoint
			lastIndex = i
		}
	}

	/*
		misal untuk origin: out edge (u,v) paling dekat dengan origin
			u - - - - - s - - - - - > v

		kita juga harus return edge geometry dari setelah s ke v buat nampilin path dari origin

		untuk destination: out edge (w,q) paling dekat dengan destination

		   w - - - - - t - - - - - - > q

		   karena kita query shortest path/rute alternatif dari s ke t, kita return edge geometry dari w ke t
	*/

	nextEdgeGeometry := make([]da.Coordinate, 0, len(eGeometry))
	if !origin {
		// kalau destination
		nextEdgeGeometry = append(nextEdgeGeometry, eGeometry[:lastIndex]...)
	} else {
		// kalau origin
		// range lastIndex [0,len(geometry)-2]
		nextEdgeGeometry = append(nextEdgeGeometry, eGeometry[lastIndex+1:]...)
	}

	return bestProjectedPoint.GetLat(), bestProjectedPoint.GetLon(), minDist, nextEdgeGeometry
}

func (rs *RoutingService) notFoundOriginDestinationWithinRadius(sp, tp da.PhantomNode) bool {
	if da.IsPhantomNodeInvalid(sp) || da.IsPhantomNodeInvalid(tp) {
		return true
	}
	return false
}

// isPairAlreadyEvaluated. check (orig,dest) pair evaluated==true
func (rs *RoutingService) isPairAlreadyEvaluated(orig, dest da.Index, removedPrevPairSet hashset.Uint64Set) bool {
	pairKey := util.Bitpack(uint32(orig), uint32(dest))
	return removedPrevPairSet.Contains(pairKey)
}

// evaluate. set (orig,dest) pair evaluated=true
func (rs *RoutingService) evaluate(orig, dest da.Index, removedPrevPairSet hashset.Uint64Set) {
	pairKey := util.Bitpack(uint32(orig), uint32(dest))
	removedPrevPairSet.Add(pairKey)
}

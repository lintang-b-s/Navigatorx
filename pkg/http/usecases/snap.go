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

	origToEndpointDist := make([]float64, len(origCandidates))
	destToEndpointDist := make([]float64, len(dstCandidates))

	for i, c := range origCandidates {
		projectedLat, projectedLon, origDist[i], origToEndpointDist[i], origNextCoords[i] = rs.ProjectCoordinateToEdge(qOrigLat, qOrigLon, c, true)
		origCoord[i] = da.NewCoordinate(projectedLat, projectedLon)
	}

	for i, c := range dstCandidates {
		projectedLat, projectedLon, dstDist[i], destToEndpointDist[i], dstNextCoords[i] = rs.ProjectCoordinateToEdge(qDstLat, qDstLon, c, false)
		dstCoord[i] = da.NewCoordinate(projectedLat, projectedLon)
	}

	g := rs.graph

	// origDestination

	// let c=max number of road segments/edges returned by rtree spatial index

	// worst case of this loop: O(c^2 * (V_G+O_G)), V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
	minDist := pkg.INF_WEIGHT
	minDistToEndpoint := pkg.INF_WEIGHT
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

			origDestSnapToEndpointDist := origToEndpointDist[i] + destToEndpointDist[j]

			if util.Lt(origDestSnapDist, minDist) {
				minDist = origDestSnapDist
				minDistToEndpoint = origDestSnapToEndpointDist
				bestPair = newOriginDestination(o, dstInEdge, origCoord[i], dstCoord[j])
				bestOriginNextCoords = origNextCoords[i]
				bestDestBefCoords = dstNextCoords[j]
			} else if util.Eq(origDestSnapDist, minDist) && util.Lt(origDestSnapToEndpointDist, minDistToEndpoint) {
				minDist = origDestSnapDist
				minDistToEndpoint = origDestSnapToEndpointDist
				bestPair = newOriginDestination(o, dstInEdge, origCoord[i], dstCoord[j])
				bestOriginNextCoords = origNextCoords[i]
				bestDestBefCoords = dstNextCoords[j]
			}
		}
	}

	if bestPair.origEdgeId == da.INVALID_EDGE_ID && bestPair.destEdgeId == da.INVALID_EDGE_ID {
		return da.NewInvalidPhantomNode(), da.NewInvalidPhantomNode()
	}

	sEdgeLength := rs.graph.GetOutEdgeLength(bestPair.origEdgeId)
	sForwardTravelTime := rs.engine.GetWeightFromLength(bestPair.origEdgeId, sEdgeLength, true)

	sp := da.NewPhantomNode(bestPair.origCoord, sForwardTravelTime, 0, bestPair.origEdgeId, da.INVALID_EDGE_ID, sEdgeLength, 0, bestOriginNextCoords,
		make([]da.Coordinate, 0))

	tEdgeLength := rs.graph.GetInEdgeLength(bestPair.destEdgeId)
	tReverseTravelTime := rs.engine.GetWeightFromLength(bestPair.destEdgeId, tEdgeLength, false)

	destExitId := rs.graph.GetExitIdOfInEdge(bestPair.destEdgeId) // outEdgeId of destination road segment

	tp := da.NewPhantomNode(bestPair.destCoord, 0.0, tReverseTravelTime, destExitId, bestPair.destEdgeId, 0, tEdgeLength, make([]da.Coordinate, 0),
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

func (rs *RoutingService) ProjectCoordinateToEdge(lat, lon float64, edgeId da.Index, origin bool) (float64, float64, float64, float64, []da.Coordinate) {

	eGeometry := rs.graph.GetEdgeGeometry(edgeId)
	minDist := pkg.INF_WEIGHT
	var bestProjectedPoint da.Coordinate
	n := len(eGeometry)

	edgeHeadCoord := eGeometry[n-1]
	edgeTailCoord := eGeometry[0]
	distToEdgeEndpoint := pkg.INF_WEIGHT //  dist dari titik proyeksi ke head dari edge (kalau origin = true), else dist: dari titik proyeksi ke tail dari edge

	lastIndex := 0
	for i := 0; i < n-1; i++ {
		tail := eGeometry[i]
		head := eGeometry[i+1]
		projectedPoint := geo.ProjectPointOnSegment(
			tail,
			head,
			da.Coordinate(da.NewCoordinate(lat, lon)),
		)

		dist := geo.CalculateEuclidianDistMercatorProj(projectedPoint.Lat, projectedPoint.Lon,
			lat, lon) // dist dari (lat,lon) ke titik proyeksi

		if util.Lt(dist, minDist) {
			minDist = dist
			bestProjectedPoint = projectedPoint
			lastIndex = i
			if origin {
				distToEdgeEndpoint = geo.CalculateEuclidianDistMercatorProj(projectedPoint.Lat, projectedPoint.Lon,
					edgeHeadCoord.GetLat(), edgeHeadCoord.GetLon())
			} else {
				distToEdgeEndpoint = geo.CalculateEuclidianDistMercatorProj(projectedPoint.Lat, projectedPoint.Lon,
					edgeTailCoord.GetLat(), edgeTailCoord.GetLon())
			}
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

	/*
		harusnya kalau openstreetmap bisa support lane level routing (geometry dari setiap osm way yang two-way dibedain jadi dua sesuai arah dan lanenya ):
		kita bisa snap ke edge yang lane osm way nya lebih deket ke titik query, kaya di gmaps berikut (lihat road segment destination):

		https://www.google.com/maps/dir/Sans+Guest+House+2,+Jl.+Mulwo,+Karangasem,+Kec.+Laweyan,+Kota+Surakarta,+Jawa+Tengah+57145/-7.5541728,110.8270471/@-7.5542505,110.8247047,18.47z/data=!4m9!4m8!1m5!1m1!1s0x2e7a14403c5830dd:0x5a2e99d453ee8b46!2m2!1d110.7819826!2d-7.5504398!1m0!3e0?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


		tapi di osrm juga gak support beginian sih (lihat road segment destination):
		https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.550317%2C110.782131%3B-7.554244%2C110.827106
		
		karena osm way yang two-way edge geometry untuk arah forward dan backward sama di openstreetmap. 
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

	return bestProjectedPoint.GetLat(), bestProjectedPoint.GetLon(), minDist, distToEdgeEndpoint, nextEdgeGeometry
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

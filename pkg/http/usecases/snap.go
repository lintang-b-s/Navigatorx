package usecases

import (
	"github.com/bytedance/gopkg/collection/hashset"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type candidateSnap struct {
	coord      da.Coordinate
	dist       float64
	tailDist   float64
	nextCoords []da.Coordinate
}

/*
SnapOrigDestToNearbyRoadSegments. snap origin dan destination query ke road segment terdekatnya.
serta terdapat path dari head dari road segment origin ke tail dari road segment destination hasil snap.

see: https://blog.mapbox.com/robust-navigation-with-smart-nearest-neighbor-search-dbc1f6218be8

let w adalah head dari kandidat road segment origin dan q adalah tail dari kandidat road segment destination
kalau tidak ada nearby road segments (dari source dan destination query) atau setiap pair road segments yang dievaluate dari round ini gak ada path (dari w ke q),
jalanin lagi snapOrigDestToNearbyRoadSegmentsByradius() dengan search radius 2x dari radius sebelumnya dan kita gak evaluate lagi evaluated candidate pairs di all previous rounds.
kenapa??
1. karena kita tahu evaluated candidate pairs gak ada path (dari w ke q) di all previous rounds.
2. masih ada kemungkinan terdapat path dari old origCands ke new dstCands

let q=number of rounds until searchRead exceeds MAX_SEARCH_RADIUS
let M=number of road segments/edges in the graph
let c=max number of road segments/edges returned by rtree spatial index (max 35)
avg case: O(q*(logM + c^2))

return:
s dari source road segment.
t dari destination road segment.
snapped point (proyeksi titik query source ke road segment terdekat) of source.
snapped point (proyeksi titik query destination ke road segment terdekat) of destination.
edgeGeometry dari source road segment setelah snappedPoint of destination.
edgeGeometry dari source road segment sebelum snappedPoint of destination.
*/
func (rs *RoutingService) SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon float64, reroute bool, startSegmentId da.Index,
) (da.PhantomNode, da.PhantomNode) {
	searchRad := rs.searchRadius
	var (
		sp = da.NewInvalidPhantomNode()
		tp = da.NewInvalidPhantomNode()
	)
	removedPrevPairSet := hashset.NewUint64WithSize(candidatePairCapacity)
	for util.Le(searchRad, MAX_SEARCH_RADIUS) {
		// https://blog.mapbox.com/robust-navigation-with-smart-nearest-neighbor-search-dbc1f6218be8

		sp, tp = rs.snapOrigDestToNearbyRoadSegmentsByradius(qOrigLat, qOrigLon, qDstLat, qDstLon, searchRad, removedPrevPairSet, reroute, startSegmentId)
		if !rs.notFoundOriginDestinationWithinRadius(sp, tp) {
			// break loop early if found connected origin and destination
			break
		}
		searchRad *= SEARCH_RADIUS_MULTIPLIER
	}

	return sp, tp
}

/*
snapOrigDestToNearbyRoadSegmentsByradius. snap origin dan destination query ke road segment terdekatnya dalam radius=searchRad,
serta terdapat path dari head dari road segment origin ke tail dari road segment destination hasil snap.
edge (u,v) dari road segment. tail = u, head = v.

let w adalah head dari kandidat road segment origin dan q adalah tail dari kandidat road segment destination
kalau tidak ada nearby road segments (dari source dan destination query) atau setiap pair road segments yang dievaluate dari round ini gak ada path (dari w ke q),
jalanin lagi snapOrigDestToNearbyRoadSegmentsByradius() dengan search radius 2x dari radius sebelumnya dan kita gak evaluate lagi evaluated candidate pairs di all previous rounds.
kenapa??
1. karena kita tahu evaluated candidate pairs di all previous rounds gak ada path (dari w ke q).
2. masih ada kemungkinan terdapat path dari old origCands ke new dstCands

let M=number of road segments/edges in the graph, MAX_CANDIDATES (see spatial_index/constant.go dan rtree.go) adalah jumlah leafs data maksimum yang direturn oleh Search() nya r-tree
let c=max number of road segments/edges returned by rtree spatial index
avg case: O(logM + c^2)
*/
func (rs *RoutingService) snapOrigDestToNearbyRoadSegmentsByradius(qOrigLat, qOrigLon, qDstLat, qDstLon, searchRad float64,
	removedPrevPairSet hashset.Uint64Set, reroute bool, startSegmentId da.Index) (da.PhantomNode, da.PhantomNode) {
	var (
		pLat, pLon float64
		origCands  []da.Index
	)

	// let M=number of road segments/edges in the graph, MAX_CANDIDATES (see spatial_index/constant.go dan rtree.go) adalah jumlah leafs data maksimum yang direturn oleh Search() nya r-tree
	// SearchWithinRadius worst case is O(M), avg case is O(logM)
	// find nearest orig edge (inSegmentOffset) to qOrigLat, qOrigLon
	if !reroute {
		origCands = rs.spatialIndex.SearchWithinRadius(qOrigLat, qOrigLon, searchRad, 0)
	} else {
		origCands = append(origCands, startSegmentId)
	}

	// find nearest dst edge (outSegmentOffset) to qDstLat, qDstLon
	dstCands := rs.spatialIndex.SearchWithinRadius(qDstLat, qDstLon, searchRad, 1)

	origSnaps := make([]candidateSnap, len(origCands))
	dstSnaps := make([]candidateSnap, len(dstCands))

	for i, c := range origCands {
		pLat, pLon, origSnaps[i].dist, origSnaps[i].tailDist, origSnaps[i].nextCoords = rs.projectCoordinateToSegment(qOrigLat, qOrigLon, c, true)
		origSnaps[i].coord = da.NewCoordinate(pLat, pLon)
	}

	for i, c := range dstCands {
		pLat, pLon, dstSnaps[i].dist, dstSnaps[i].tailDist, dstSnaps[i].nextCoords = rs.projectCoordinateToSegment(qDstLat, qDstLon, c, false)
		dstSnaps[i].coord = da.NewCoordinate(pLat, pLon)
	}

	// origDestination

	// let c=max number of road segments/edges returned by rtree spatial index

	minDist := pkg.INF_WEIGHT
	minEndpointDist := pkg.INF_WEIGHT
	bestPair := newOriginDestination(da.INVALID_SEGMENT_ID, da.INVALID_SEGMENT_ID,
		da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON), da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON))
	var oCoords []da.Coordinate
	var dRevCoords []da.Coordinate
	var oLength, dLength float64

	// worst case of this loop: O(c^2)
	for i, o := range origCands {
		for j, d := range dstCands {

			if rs.isPairAlreadyEvaluated(o, d, removedPrevPairSet) {
				continue
			}

			// kita set (o,d) evaluated=true mau ada path atau gak ada path dari o ke d.
			rs.evaluate(o, d, removedPrevPairSet)

			// O(1)
			if !rs.engine.PathExists(o, d) {
				continue
			}

			odSnapDist := origSnaps[i].dist + dstSnaps[j].dist

			oSegLength := rs.engine.GetSegmentLength(o)
			odEndDist := (oSegLength - origSnaps[i].tailDist) + dstSnaps[j].tailDist

			if util.Lt(odSnapDist, minDist) {
				minDist = odSnapDist
				minEndpointDist = odEndDist
				bestPair = newOriginDestination(o, d, origSnaps[i].coord, dstSnaps[j].coord)
				oCoords = origSnaps[i].nextCoords
				dRevCoords = dstSnaps[j].nextCoords
				oLength = origSnaps[i].tailDist
				dLength = dstSnaps[j].tailDist
			} else if util.Eq(odSnapDist, minDist) && util.Lt(odEndDist, minEndpointDist) {
				minDist = odSnapDist
				minEndpointDist = odEndDist
				bestPair = newOriginDestination(o, d, origSnaps[i].coord, dstSnaps[j].coord)
				oCoords = origSnaps[i].nextCoords
				dRevCoords = dstSnaps[j].nextCoords
				oLength = origSnaps[i].tailDist
				dLength = dstSnaps[j].tailDist
			}
		}
	}

	if bestPair.s == da.INVALID_SEGMENT_ID && bestPair.t == da.INVALID_SEGMENT_ID {
		return da.NewInvalidPhantomNode(), da.NewInvalidPhantomNode()
	}

	sfCost := rs.engine.GetDurationFromLength(bestPair.s, oLength)

	sp := da.NewPhantomNode(bestPair.s, bestPair.spCoord, sfCost, 0, oLength, 0, oCoords,
		make([]da.Coordinate, 0))

	trCost := rs.engine.GetDurationFromLength(bestPair.t, dLength)

	tp := da.NewPhantomNode(bestPair.t, bestPair.dpCoord, 0.0, trCost, 0, dLength, make([]da.Coordinate, 0),
		dRevCoords)

	// handle case when bestPair.s == bestPair.t
	if rs.isSameSourceDestinationSegment(sp, tp) {
		sp, tp = rs.handleSameSourceDestinationSegment(sp, tp)
	}

	return sp, tp
}

func (rs *RoutingService) isSameSourceDestinationSegment(sp, tp da.PhantomNode) bool {
	return sp.GetVId() == tp.GetVId()
}

func (rs *RoutingService) handleSameSourceDestinationSegment(sp, tp da.PhantomNode) (da.PhantomNode, da.PhantomNode) {
	var newSourceForwardGeom []da.Coordinate

	spForwardGeom := sp.GetForwardGeometry()
	/*
		case 1:
		misal segment jalan yang source dan destination ke snap:

		u---s----------------t--->v
		misal edgeGometry cuma (uCoord, vCoord)

		jadi kita cuma return geometry (sCoord, tCoord) (dihandle di rs.AppendPhantomNodesToPath()) buat shortest path nya (kalau source dan destination road segment sama)....

		case 2:
		misal segment jalan yang source dan destination ke snap:

		u---s----x------w--z---t--->v
		misal edgeGometry ada (uCoord,xCoord,wCoord,zCoord,vCoord)

		di case ini, kita return geometry (sCoord, xCoord, wCoord, zCoord, tCoord)  buat shortest path nya (kalau source dan destination road segment sama dan edgeGeometry > 2)....
	*/

	edgeId := sp.GetVId()
	spProjectedCoord := sp.GetSnappedCoord()
	lastIndexForward, _, _, _, _ := rs.project(spProjectedCoord.GetLat(), spProjectedCoord.GetLon(), edgeId, true)
	tpProjectedCoord := tp.GetSnappedCoord()
	lastIndexBackward, _, _, _, _ := rs.project(tpProjectedCoord.GetLat(), tpProjectedCoord.GetLon(), edgeId, true)

	newSPLength := 0.0

	tp.SetReverseCost(0)
	tp.SetReverseDistance(0)

	if lastIndexForward != lastIndexBackward {
		// case 2
		newSourceForwardGeom = spForwardGeom[:lastIndexBackward+1]

		for i := 0; i < len(newSourceForwardGeom)-1; i++ {
			curCo := newSourceForwardGeom[i]
			nextCo := newSourceForwardGeom[i+1]
			newSPLength += geo.CalculateGreatCircleDistance(curCo.GetLat(), curCo.GetLon(),
				nextCo.GetLat(), nextCo.GetLon())
		}

		// dist (sp, newSourceForwardGeom[0])
		fCoord := newSourceForwardGeom[0]
		newSPLength += geo.CalculateGreatCircleDistance(spProjectedCoord.GetLat(), spProjectedCoord.GetLon(),
			fCoord.GetLat(), fCoord.GetLon())

		// dist (newSourceForwardGeom[len(newSourceForwardGeom)-1], tp)
		lastCoord := newSourceForwardGeom[len(newSourceForwardGeom)-1]
		newSPLength += geo.CalculateGreatCircleDistance(lastCoord.GetLat(), lastCoord.GetLon(),
			tpProjectedCoord.GetLat(), tpProjectedCoord.GetLon())

		newSPCost := rs.engine.GetDurationFromLength(sp.GetVId(), newSPLength)
		newSP := da.NewPhantomNode(sp.GetVId(), sp.GetSnappedCoord(), newSPCost, 0,
			newSPLength, 0.0, newSourceForwardGeom, make([]da.Coordinate, 0))

		return newSP, tp
	}

	// case 1 tinggal return empty newSourceForwardGeom, geometry dist & traveltime (sCoord, tCoord) dihandle di sini
	newSPLength += geo.CalculateGreatCircleDistance(spProjectedCoord.GetLat(), spProjectedCoord.GetLon(),
		tpProjectedCoord.GetLat(), tpProjectedCoord.GetLon())
	newSPCost := rs.engine.GetDurationFromLength(sp.GetVId(), newSPLength)
	newSP := da.NewPhantomNode(sp.GetVId(), sp.GetSnappedCoord(), newSPCost, 0,
		newSPLength, 0.0, newSourceForwardGeom, make([]da.Coordinate, 0))

	return newSP, tp
}

type originDestination struct {
	s, t             da.Index
	spCoord, dpCoord da.Coordinate // projected origin query coordinate to road segment, projected destination query coordinate to road segment.
}

func newOriginDestination(s, t da.Index, spCoord, dpCoord da.Coordinate) originDestination {
	return originDestination{
		s:       s,
		t:       t,
		spCoord: spCoord,
		dpCoord: dpCoord,
	}
}

func (rs *RoutingService) projectCoordinateToSegment(lat, lon float64, id da.Index, origin bool) (float64, float64, float64, float64, []da.Coordinate) {

	lastIndex, eGeometry, pPoint, minDist, tailDist := rs.project(lat, lon, id, origin)

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

		https://www.google.com/maps/dir/Sans+Guest+House+2,+Jl.+Mulwo,+Karangasem,+Kec.+Laweyan,+Kota+Surakarta,+Jawa+Tengah+57145/-7.5541728,110.8270471/@-7.5542505,110.8247047,18.47z/data=!4m1!4b1!4m3!4m2!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwOC4wIKXMDSoASAFQAw%3D%3D


		tapi di osrm juga gak support beginian sih (lihat road segment destination):
		https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.550317%2C110.782131%3B-7.554244%2C110.827106

		karena osm way yang two-way edge geometry untuk arah forward dan backward sama di openstreetmap.
	*/

	var nextSegGeometry []da.Coordinate
	if !origin {
		nextSegGeometry = eGeometry[:lastIndex+1]
	} else {
		nextSegGeometry = eGeometry[lastIndex+1:]
	}

	return pPoint.GetLat(), pPoint.GetLon(), minDist, tailDist, nextSegGeometry
}

func (rs *RoutingService) project(lat, lon float64, id da.Index, origin bool) (da.Index, []da.Coordinate, da.Coordinate, float64, float64) {

	segGeometry := rs.rn.GetSegmentGeometry(id)
	minDist := pkg.INF_WEIGHT
	var pPoint da.Coordinate //  best projected point

	lastIndex, pPoint, minDist, stailDist := geo.ProjectPointOnSegmentGeometry(segGeometry, lat, lon)
	return lastIndex, segGeometry, pPoint, minDist, stailDist
}

func (rs *RoutingService) notFoundOriginDestinationWithinRadius(sp, tp da.PhantomNode) bool {
	if da.IsPhantomNodeInvalid(sp) || da.IsPhantomNodeInvalid(tp) {
		return true
	}
	return false
}

// isPairAlreadyEvaluated. check (orig,dest) pair evaluated==true. avg case O(1) hashtable
func (rs *RoutingService) isPairAlreadyEvaluated(orig, dest da.Index, removedPrevPairSet hashset.Uint64Set) bool {
	pairKey := util.Bitpack(uint32(orig), uint32(dest))
	return removedPrevPairSet.Contains(pairKey)
}

// evaluate. set (orig,dest) pair evaluated=true. avg case O(1) hashtable
func (rs *RoutingService) evaluate(orig, dest da.Index, removedPrevPairSet hashset.Uint64Set) {
	pairKey := util.Bitpack(uint32(orig), uint32(dest))
	removedPrevPairSet.Add(pairKey)
}

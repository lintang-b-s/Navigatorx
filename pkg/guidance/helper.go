// Package guidance provides logic for generating driving directions (turn-by-turn navigation instructions).
package guidance

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// lookForward. traverse dari db.path[lastPathId] ke beberapa next db.path[lastPathId+1:...] until found edge dengan name != prevName.
// return forward edge di path dengan name != prevName, true jika ada, dan number of step lookForward.
// todo: ini bisa dipake buat kasus Dual carriageway intersections yang dijelasin di: https://wiki.openstreetmap.org/wiki/Junctions  (DONE)
func (db *DirectionBuilder) lookForward(prevName string, maxStep int) ([]da.Index, bool, int) {
	step := 0
	nextSegmentIds := make([]da.Index, 0, maxStep)
	for i := db.lastPathId + 1; step < maxStep && i < len(db.path); i++ {
		segName := db.rn.GetStreetName(db.path[i])
		step++
		nextSegmentIds = append(nextSegmentIds, db.path[i])
		if segName != prevName {
			return nextSegmentIds, true, step
		}
	}

	return make([]da.Index, 0), false, 0
}

//
/*
GetPrevPoint. ini buat get prevPoint, point sebelum tail vertex/intersection vertex.

ingat bentuk turn adalah seperti ini:

prevPoint----prevEdge----tail
						|
						|
						currentEdge
						|
						|
						headPoint

ada dua kasus untuk getPrevPoint:
1. geometry dari prevEdge gak ngecurve (contoh Jalan Brigadir Jenderal Slamet Riyadi: openstreetmap.org/way/300751601), untuk kasus ini kita bisa return point pertama dari geometrynya prevEdge sebagai prevPoint.

2. geometry dari prevEdge ngecurve (contoh: Jalan Yos Sudarso https://www.openstreetmap.org/way/357667665) ,  ini agak ribet,
perhatikan saat kita pakai commercial maps (gmaps, apple maps, etc), panduan turn right/turn left itu saat posisi kita dekat dari titik intersection,
kita dapat tipe turn dari relative bearing dari currentEdge (tail, head) dengan prevEdge (prevPoint, tail) (lihat GetTurnDirection() di turn.go).
yang berarti kita harus pakai prevPoint yang sangat dekat dengan titik intersection (tail), yang mana kalo prevEdgenya ngecurve kita gak bisa langsung return point pertama
dari geometry prevEdge.

untuk curved prevEdge, kita bisa pakai point dari geometry prevEdge yang jaraknya atleast 25m dari intersection (tail).

kenapa gak pake last point dari prevEdge geometry?? karena last point coord nya equal to tail coord

argument:
eGeom prevEdge geometry
tailCoord tail node coordinate
atLeastDist in meter
*/ // nolint: gofmt
func (db *DirectionBuilder) GetPrevPoint(segmentId da.Index, tailCoord da.Coordinate, atLeastDist float64) da.Coordinate {

	curved := db.rn.IsCurved(segmentId)
	n := db.rn.GetSegmentGeometryLength(segmentId)
	if !curved || n == 2 {
		return db.rn.GetSegmentGeometryPoint(segmentId, 0)
	}

	v := db.rn.GetSegmentGeometryPoint(segmentId, int(n-2))
	for i := int(n - 2); int(i) >= 0; i-- {
		p := db.rn.GetSegmentGeometryPoint(segmentId, int(i))
		dist := geo.CalculateEuclideanDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon())
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}

// GetHeadPoint. ini buat get headPoint, point setelah tail vertex/intersection vertex.
// mirip kaya GetPrevPoint, tapi pakai geometry dari currentEdge.
func (db *DirectionBuilder) GetHeadPoint(eId da.Index, tailCoord da.Coordinate, atLeastDist float64) da.Coordinate {

	curved := db.rn.IsCurved(eId)
	n := db.rn.GetSegmentGeometryLength(eId)
	if !curved || n == 2 {
		return db.rn.GetSegmentGeometryPoint(eId, 1)
	}

	v := db.rn.GetSegmentGeometryPoint(eId, 1)
	for i := 1; i < int(n); i++ {
		p := db.rn.GetSegmentGeometryPoint(eId, i)
		dist := geo.CalculateEuclideanDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon())
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}

// lookForwardSameOsmWay. cek apakah segment dengan id segId dan head vertex dengan id headId berada di osm way yang sama.
// contoh dari tail: https://www.openstreetmap.org/node/11294649720
// dari tail osm node diatas ada 2 edge ke head: https://www.openstreetmap.org/node/11294649718
// dan edge satunya ke head: https://www.openstreetmap.org/node/11294649719  (dari jalan curved/uturn ke kanan)
func (db *DirectionBuilder) lookForwardSameOsmWay(segId da.Index, headOsmId uint64, maxStep int) bool {
	step := 0
	osmWayOne := db.rn.GetOsmWayId(segId)
	for i := db.lastPathId + 1; step < maxStep && i < len(db.path); i++ {
		nSegOsmWay := db.rn.GetOsmWayId(db.path[i])
		_, nHeadOsmId := db.rn.GetTailHeadOsmNodeId(db.path[i])
		step++
		if nSegOsmWay == osmWayOne && nHeadOsmId == headOsmId {
			return true
		}
	}

	return false
}

// IsSuggestAlternatives. cek apakah tipe highway dari segmentId in "primary", "secondary", "trunk"
// biasanya kalau kita pakai google map, ketika kita di dekat persimpangan jalan besar, gmaps ngasih rute alternatives
// nah ini tujuannya buat nandain routeStep (kumpulan segmen jalan sebelum titik belok) bisa disuggest alternative routes
// ingat kita menambahkan turnSign && routeStep ketika turnSign dari segmentId yang diproses buildInstruction(segmentId) bukan IGNORE
// kalau segmentId tipenya "primary", "secondary", "trunk" (yang biasanya jadi Jalan nasional, Jalan provinsi, Jalan kabupaten/kota)
// kita set suggestAlternative=true untuk current routeStep...
// ntar di frontend nya ketika ketika hasil matched_edge_id dari online map matchingnya ada di last 3 segmentIds dari currentRouteStep (routeStep yang next beloknya ke jalan dari ketiga tipe diatas)
// request ke endpoint alternativeRoutes, tambahin polyline alternative routes ke map
func (db *DirectionBuilder) IsSuggestAlternatives(segmentId da.Index) bool {

	edgeRoadClass := db.rn.GetRoadClass(segmentId)
	edgeRoadClassLink := db.rn.GetRoadClassLink(segmentId)

	switch edgeRoadClass {
	case pkg.PRIMARY, pkg.SECONDARY, pkg.TRUNK:
		return true
	default:
		switch edgeRoadClassLink {
		case pkg.PRIMARY_LINK, pkg.SECONDARY_LINK, pkg.TRUNK_LINK:
			return true
		}
	}
	return false
}

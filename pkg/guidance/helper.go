package guidance

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// lookForward. traverse dari db.path[lastPathId] ke beberapa next db.path[lastPathId+1:...] until found edge dengan name != prevName.
// return forward edge di path dengan name != prevName, true jika ada, dan number of step lookForward.
// todo: ini bisa dipake buat kasus Dual carriageway intersections yang dijelasin di: https://wiki.openstreetmap.org/wiki/Junctions
func (db *DirectionBuilder) lookForward(prevName string, maxStep int) (string, bool, int) {
	step := 0
	for i := db.lastPathId + 1; step < maxStep && i < len(db.path); i++ {
		currEdgeStreetname := db.graph.GetStreetName(db.path[i])
		step++
		if currEdgeStreetname != prevName {
			return currEdgeStreetname, true, step
		}
	}

	return "", false, 0
}

//
/*
GetPrevPoint. ini buat get prevPoint, point sebelum tail vertex/intersection vertex.

ingat bentuk turn adalah seperti ini:

prevNode----prevEdge----tail
						|
						|
						currentEdge
						|
						|
						head

ada dua kasus untuk getPrevPoint:
1. geometry dari prevEdge gak ngecurve (contoh Jalan Brigadir Jenderal Slamet Riyadi: openstreetmap.org/way/300751601), untuk kasus ini kita bisa return point pertama dari geometrynya prevEdge sebagai prevPoint.

2. geometry dari prevEdge ngecurve (contoh: Jalan Yos Sudarso https://www.openstreetmap.org/way/357667665) ,  ini agak ribet,
perhatikan saat kita pakai commercial maps (gmaps, apple maps, etc), panduan turn right/turn left itu saat posisi kita dekat dari titik intersection,
kita dapat tipe turn dari delta bearing dari currentEdge (tail, head) dengan prevEdge (prevNode, tail) (lihat getTurnDirection() di turn.go).
yang berarti kita harus pakai prevNode yang sangat dekat dengan titik intersection (tail), yang mana kalo prevEdgenya ngecurve kita gak bisa langsung return point pertama
dari geometry prevEdge.

untuk curved prevEdge, kita bisa pakai point dari geometry yang jaraknya atleast 25m

kenapa gak pake last point dari prevEdge geometry?? karena last point coord nya equal to tail coord

argument:
eGeom prevEdge geometry
tailCoord tail node coordinate
atLeastDist in meter
*/ // nolint: gofmt
func (db *DirectionBuilder) GetPrevPoint(eGeom da.Coordinates, tailCoord da.Coordinate, atLeastDist float64) da.Coordinate {

	curved := geo.IsPolylineCurved(eGeom)
	n := len(eGeom)
	if !curved || n == 2 {
		return eGeom[0]
	}

	v := eGeom[n-2]
	for i := n - 2; i >= 0; i-- {
		p := eGeom[i]
		dist := util.KilometerToMeter(geo.CalculateEuclidianDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon()))
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}

// GetHeadPoint. ini buat get prevPoint, point setelah tail vertex/intersection vertex.
// mirip kaya GetPrevPoint, tapi pakai geometry dari currentEdge.
func (db *DirectionBuilder) GetHeadPoint(eGeom da.Coordinates, tailCoord da.Coordinate, atLeastDist float64) da.Coordinate {

	curved := geo.IsPolylineCurved(eGeom)
	n := len(eGeom)
	if !curved || n == 2 {
		return eGeom[1]
	}

	v := eGeom[1]
	for i := 1; i < n; i++ {
		p := eGeom[i]
		dist := util.KilometerToMeter(geo.CalculateEuclidianDistMercatorProj(tailCoord.GetLat(), tailCoord.GetLon(), p.GetLat(), p.GetLon()))
		if util.Ge(dist, atLeastDist) {
			v = p
			break
		}
	}

	return v
}

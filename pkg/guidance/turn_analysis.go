package guidance

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
GetAlternativeTurns. get jumlah belokan alternatif yang bisa dilakukan dari tail sekarang & bukan currentEdge/prevEdge. Misalkan:

		 |
		 |
	 alternative
		 |
--prev-- tail --currentEdge---
		 |
		 |
	alternative
		 |

ada 4 belokan yang bisa dilakukan dari tail.
--- / | = jalan 2 arah
*/ // nolint: gofmt
func (db *DirectionBuilder) GetAlternativeTurns(tailId, headId, prevVertexId da.Index) (int, []da.Index) {
	outEdgesFromTail := make([]da.Index, 0) // reset length, tapi capacity tetep sama

	db.graph.ForOutEdgeIdsOf(tailId, func(eId da.Index) {

		outEdgesFromTail = append(outEdgesFromTail, eId)
	})

	tempAltTurns := make([]da.Index, 0)

	for _, edge := range outEdgesFromTail {
		edgeHead := db.graph.GetHeadOfOutEdge(edge)
		if edgeHead != prevVertexId && edgeHead != headId {
			tempAltTurns = append(tempAltTurns, edge)
		}
	}

	return len(tempAltTurns), tempAltTurns
}

func isSameRoadClassAndLink(prevEdgeRoadClass, prevEdgeRoadClassLink string,
	currEdgeRoadClass, currEdgeRoadClassLink string) bool {
	return prevEdgeRoadClass == currEdgeRoadClass && prevEdgeRoadClassLink == currEdgeRoadClassLink
}

/*
getOtherEdgeContinueDirection. get alternativeEdges lain dari tail yang arahnya continue. Misalkan

				---- currentEdge-----

--prevEdge-- tail

				----alternativeEdge-----

delta bearing antara currentEdge dan alternativeEdge mendekati 0° atau alternativeEdge punya tipe turn CONTINUE or TURN_SLIGHT_*
contoh: https://www.google.com/maps/dir/-7.5484714,110.7825683/-7.7560503,110.3762651/@-7.5446759,110.7824839,17z/am=t/data=!4m6!4m5!3e0!6m3!1i0!2i0!3i2?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
*/ // nolint: gofmt
func (db *DirectionBuilder) getOtherEdgeContinueDirection(prevLat, prevLon, prevInitialBearing float64, alternativeTurns []da.Index) da.Index {
	for _, edge := range alternativeTurns {
		edgeHead := db.graph.GetHeadOfOutEdge(edge)

		node := db.graph.GetVertex(edgeHead)
		lat, lon := node.GetLat(), node.GetLon()

		tmpSign := getTurnDirection(prevLat, prevLon, lat, lon, prevInitialBearing)
		if da.IsTurnSlight(da.TurnType(tmpSign)) {
			return edge
		}
	}
	return da.INVALID_EDGE_ID
}

/*
isStreetMergedSkip. cek apakah 2 street (prevEdge, otherEdge) (yang arahnya berkebalikan) merged ke 1 street. Misal kalau jalan di indo:
--prevEdge-->
				tail <--currentEdge-->
<--otherEdge--

examplenya di jalan solo-semarang, A.Yani : -7.5533505900708455, 110.82338424980728
atau -7.554690293226057, 110.80098414699525

kalau streetMrged == true dan nama jalan dari prevEdge == nama jalan currentEdge, kita gak perlu tambahin turn instruction buat ke currentEdge

delta bearing prevEdge dan otherEdge haruslah > 150 degree, jadi semacam arah nya berkebalikan
note that forward/backward direction dari osm way hanyalah direction dari nodes di simpan di osm way (https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right)

contoh:
https://www.google.com/maps/dir/-7.5502186,110.7820629/-7.5569461,110.8054174/@-7.5547439,110.8005837,19.49z/am=t/data=!4m9!4m8!1m1!4e1!1m0!3e0!6m3!1i0!2i1!3i1?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik  -7.554690293226057, 110.80098414699525 , yang merupakan tail yang merged prevEdge dan otherEdge (yang saling berlawanan arahnya) ke currentEdge, dan nama jalan dari prevEdge  == nama jalan currentEdge
di kasus ini gmaps gak ngasih turn instruction buat ke currentEdge


contoh2:
https://www.google.com/maps/dir/-7.5512339,110.8198894/-7.5542367,110.8268638/@-7.5530866,110.8231349,20.17z/am=t/data=!4m6!4m5!3e0!6m3!1i0!2i0!3i0?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik -7.55331131002897, 110.82335347667555 , yang merupakan tail dari merged prevEdge dan otherEdge (yang saling berlawanan arahnya) ke currentEdge, dan nama jalan dari prevEdge  == nama jalan currentEdge
di kasus ini gmaps juga gak ngasih turn instruction buat ke currentEdge


*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetMergedSkip(currentEdge, prevEdge da.Index, currStreetName, prevEdgeStreetName string,
	prevEdgeRoadClass, currRoadClass string, isSameName func(currStreetName, prevEdgeStreetName string) bool) bool {
	tail := db.graph.GetTailOfOutedge(currentEdge)

	if !isSameName(currStreetName, prevEdgeStreetName) || currRoadClass != prevEdgeRoadClass {
		return false
	}

	otherEdge := da.INVALID_EDGE_ID // outEdge dari tail selain PrevEdge yang mengarah dari tail

	outEdgesFromTail := make([]da.Index, 0) // reset length, tapi capacity tetep sama

	db.graph.ForOutEdgeIdsOf(tail, func(eId da.Index) {

		outEdgesFromTail = append(outEdgesFromTail, eId)
	})

	tailOfPrevEdge := db.graph.GetTailOfOutedge(prevEdge)
	currHead := db.graph.GetHeadOfOutEdge(currentEdge)

	tailOfPrevEdgeCoord := db.graph.GetVertexCoordinate(tailOfPrevEdge)

	tailCoord := db.graph.GetVertexCoordinate(tail)

	prevInitialBearing := computeInitialBearing(tailOfPrevEdgeCoord.GetLat(), tailOfPrevEdgeCoord.GetLon(), tailCoord.GetLat(),
		tailCoord.GetLon())

	for _, outEdgeId := range outEdgesFromTail {
		edgeStreetName := db.graph.GetStreetName(outEdgeId)

		edgeHead := db.graph.GetHeadOfOutEdge(outEdgeId)
		edgeHeadCoord := db.graph.GetVertexCoordinate(edgeHead)

		relativeBearing := util.RadiansToDegree(math.Abs(computeRelativeBearing(tailCoord.GetLat(),
			tailCoord.GetLon(), edgeHeadCoord.GetLat(), edgeHeadCoord.GetLon(), prevInitialBearing)))

		isOtherEdgeBidirectional := db.graph.IsStreetBidirectional(outEdgeId)

		if outEdgeId != currentEdge &&
			edgeHead != currHead && edgeHead != tailOfPrevEdge && relativeBearing > DELTA_BEARING_U_TURN &&
			isSameName(currStreetName, edgeStreetName) && !isOtherEdgeBidirectional {
			otherEdge = outEdgeId
		}
	}

	if otherEdge == da.INVALID_EDGE_ID {
		return false
	}

	otherEdgeLanes := db.graph.GetRoadLanes(otherEdge)

	// harus convert uint8 ke int, karena kalau gak diconvert pas hasil subtraction negatif jadi 255
	laneDiff := int(db.graph.GetRoadLanes(currentEdge)) - int((db.graph.GetRoadLanes(prevEdge) + otherEdgeLanes)) // lane dari other 1 & lane dari prev  1
	return laneDiff <= 1
}

/*
	isStreetSplit.	cek apakah edge sekarang hasil dari split edge sebelumnya.

	 							--currentEdge-->
	 	<--prevEdge-->tail
							   	<--otherEdge--
examplenya di -7.559777239220366, 110.83649946865347

jika streetSplit==true dan nama jalan dari prevEdge == nama jalan currentEdge , kita gak perlu kasih turn instruction buat ke currentEdge

contoh kedua: -7.555498615741148, 110.80275937109323
https://www.google.com/maps/dir/Warung+Makan+Mas+Eng+Bebek+dan+Ayam+Goreng+Kremes,+Jl.+Adi+Sucipto+No.133,+Jajar,+Kec.+Colomadu,+Kabupaten+Karanganyar,+Jawa+Tengah+57174/-7.5569461,110.8054174/@-7.5478686,110.7846665,16.7z/data=!4m9!4m8!1m5!1m1!1s0x2e7a146abedbd01d:0x6e1d57d5149dc641!2m2!1d110.7825854!2d-7.5485275!1m0!3e0?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik  -7.555498615741148, 110.80275937109323 ,  yang merupakan tail dan ada split ke 2 edge currentEdge dan  otherEdge (yang arahnya saling berlawanan), dan  nama jalan prevEdge == nama jalan currentEdge
gmaps gak ngasih turn instruction buat ke currentEdge

contoh2:
https://www.google.com/maps/dir/-7.5675585,110.8262857/-7.5717457,110.8241106/@-7.5701915,110.8246814,19.26z/am=t/data=!4m7!4m6!3e0!5i1!6m3!1i0!2i0!3i4?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik split  -7.570920015762306, 110.8244399952867 ,  karena setelah titik split nama jalan ganti gmaps tetep ngasih turn instruction "Continue onto Jl. Tj. Anom/Jl. Yos Sudarso"


*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetSplitSkip(currentEdge, prevEdge da.Index, currStreetName, prevEdgeStreetName string,
	prevEdgeRoadClass, currRoadClass string, isSameName func(currStreetName, prevEdgeStreetName string) bool) bool {
	tail := db.graph.GetTailOfOutedge(currentEdge)

	if !isSameName(currStreetName, prevEdgeStreetName) || currRoadClass != prevEdgeRoadClass {
		return false
	}

	otherEdge := da.INVALID_EDGE_ID // inEdge dari tail selain PrevEdge yang mengarah ke tail

	inEdgesFromTail := make([]da.Index, 0)

	db.graph.ForInEdgeIdsOf(tail, func(eId da.Index) {
		inEdgesFromTail = append(inEdgesFromTail, eId)
	})
	prevEdgeTail := db.graph.GetTailOfOutedge(prevEdge)

	currHead := db.graph.GetHeadOfOutEdge(currentEdge)

	tailCoord := db.graph.GetVertexCoordinate(tail)
	currHeadCoord := db.graph.GetVertexCoordinate(currHead)

	prevInitialBearing := computeInitialBearing(tailCoord.GetLat(), tailCoord.GetLon(), currHeadCoord.GetLat(),
		currHeadCoord.GetLon())

	for _, inEdgeId := range inEdgesFromTail {

		outEdgeId := db.graph.GetExitIdOfInEdge(inEdgeId)

		inEdgeStreetName := db.graph.GetStreetName(outEdgeId)
		inEdgeTail := db.graph.GetTailOfOutedge(outEdgeId)

		relativeBearing := util.RadiansToDegree(math.Abs(computeRelativeBearing(currHeadCoord.GetLat(),
			currHeadCoord.GetLon(), tailCoord.GetLat(), tailCoord.GetLon(), prevInitialBearing)))

		isOtherEdgeBidirectional := db.graph.IsStreetBidirectional(outEdgeId)

		if inEdgeTail != currHead && inEdgeTail != prevEdgeTail &&
			isSameName(currStreetName, inEdgeStreetName) && relativeBearing > DELTA_BEARING_U_TURN &&
			!isOtherEdgeBidirectional {

			otherEdge = outEdgeId
		}
	}

	if otherEdge == da.INVALID_EDGE_ID {
		return false
	}

	otherEdgeOutEdgeId := db.graph.GetExitIdOfInEdge(otherEdge)

	otherEdgeLanes := db.graph.GetRoadLanes(otherEdgeOutEdgeId)

	laneDiff := int(db.graph.GetRoadLanes(prevEdge)) - int((otherEdgeLanes + db.graph.GetRoadLanes(currentEdge))) // lane dari  otherEdge  & currentEdge cuma 1
	return laneDiff <= 1
}

/*
isStreetMerged. cek apakah 2 street (prevEdge, otherEdge) (yang satu arah) merged ke 1 street. Misal kalau jalan di indo:
--prevEdge-->
				tail <--currentEdge-->
--otherEdge-->

contohnya di tail node: https://www.openstreetmap.org/node/7298595963

kalau merged gini kita harus output Merge Onto jl. .....

contoh2:
https://www.google.com/maps/dir/-7.568126,110.8386365/-7.5752331,110.8287711/@-7.5696958,110.8304612,20.45z/am=t/data=!4m7!4m6!3e0!5i2!6m3!1i0!2i0!3i2?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D

di tail node -7.569596062971082, 110.83036862554624 , turn instruction gmaps  "Merge onto Jl. Jend. Sudirman"
meskipun sign dari relativeBearing nya TURN_SLIGHT_LEFT

*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetMerged(currentEdge, prevEdge da.Index, currStreetName, prevEdgeStreetName string,
	isSameName func(currStreetName, prevEdgeStreetName string) bool) bool {
	tail := db.graph.GetTailOfOutedge(currentEdge)

	if isSameName(currStreetName, prevEdgeStreetName) {
		return false
	}

	otherEdge := da.INVALID_EDGE_ID // outEdge dari tail selain PrevEdge yang mengarah dari tail

	inEdgesFromTail := make([]da.Index, 0) // reset length, tapi capacity tetep sama

	db.graph.ForInEdgeIdsOf(tail, func(eId da.Index) {

		inEdgesFromTail = append(inEdgesFromTail, eId)
	})

	tailOfPrevEdge := db.graph.GetTailOfOutedge(prevEdge)

	tailOfPrevEdgeCoord := db.graph.GetVertexCoordinate(tailOfPrevEdge)

	tailCoord := db.graph.GetVertexCoordinate(tail)

	prevInitialBearing := computeInitialBearing(tailOfPrevEdgeCoord.GetLat(), tailOfPrevEdgeCoord.GetLon(), tailCoord.GetLat(),
		tailCoord.GetLon())

	for _, inEdgeId := range inEdgesFromTail {
		outEdgeId := db.graph.GetExitIdOfInEdge(inEdgeId)

		edgeStreetName := db.graph.GetStreetName(outEdgeId)

		outEdgeHead := db.graph.GetHeadOfOutEdge(outEdgeId)
		outEdgeTail := db.graph.GetTailOfOutedge(outEdgeId)

		outEdgeHeadCoord := db.graph.GetVertexCoordinate(outEdgeHead)
		outEdgeTailCoord := db.graph.GetVertexCoordinate(outEdgeTail)

		relativeBearing := util.RadiansToDegree(math.Abs(computeRelativeBearing(outEdgeTailCoord.GetLat(),
			outEdgeTailCoord.GetLon(), outEdgeHeadCoord.GetLat(), outEdgeHeadCoord.GetLon(), prevInitialBearing)))

		isOtherEdgeBidirectional := db.graph.IsStreetBidirectional(outEdgeId)

		if outEdgeId != currentEdge &&
			outEdgeHead == tail && outEdgeTail != tailOfPrevEdge && relativeBearing < MERGE_DELTA_BEARING &&
			isSameName(currStreetName, edgeStreetName) && !isOtherEdgeBidirectional {
			otherEdge = outEdgeId
		}
	}

	if otherEdge == da.INVALID_EDGE_ID {
		return false
	}

	otherEdgeLanes := db.graph.GetRoadLanes(otherEdge)

	// harus convert uint8 ke int, karena kalau gak diconvert pas hasil subtraction negatif jadi 255
	laneDiff := int(db.graph.GetRoadLanes(currentEdge)) - int((db.graph.GetRoadLanes(prevEdge) + otherEdgeLanes)) // lane dari other 1 & lane dari prev  1
	return laneDiff <= 1
}

package guidance

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
GetAlternativeTurns. get jumlah belokan alternatif yang bisa dilakukan dari tail sekarang & bukan segmentId/prevSegmentId. Misalkan:

		 |
		 |
	 alternative
		 |
--prev-- tail --segmentId---
		 |
		 |
	alternative
		 |

ada 4 belokan yang bisa dilakukan dari tail.
--- / | = jalan 2 arah
*/ // nolint: gofmt
func (db *DirectionBuilder) GetAlternativeTurns(prevSegmentId, segmentId da.Index) (int, []da.Index) {
	db.alternativeTurns = db.alternativeTurns[:0]

	db.graph.ForOutEdgeIdsOf(prevSegmentId, func(eId da.Index) {

		nSegmentId := db.graph.GetHead(eId)

		if db.rn.IsParallelVia(nSegmentId) {
			return
		}

		if nSegmentId != segmentId {
			db.alternativeTurns = append(db.alternativeTurns, nSegmentId)
		}
	})

	return len(db.alternativeTurns), db.alternativeTurns
}

/*
getOtherEdgeContinueDirection. get alternativeEdges lain dari tail yang arahnya continue. Misalkan

				---- segmentId-----

--prevSegment-- tail

				----alternativeEdge-----

relative bearing antara segmentId dan alternativeEdge mendekati 0° atau alternativeEdge punya tipe turn CONTINUE or TURN_SLIGHT_*
contoh: https://www.google.com/maps/dir/-7.5484714,110.7825683/-7.7560503,110.3762651/@-7.5446759,110.7824839,17z/am=t/data=!4m6!4m5!3e0!6m3!1i0!2i0!3i2?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
*/ // nolint: gofmt
func (db *DirectionBuilder) getOtherEdgeContinueDirection(tail da.Coordinate, prevInitBearing float64, alternativeTurns []da.Index) (da.Index, da.Coordinate) {
	for _, altSegId := range alternativeTurns {

		altHead := db.GetHeadPoint(altSegId, tail, 8)
		tmpSign := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), altHead.GetLat(), altHead.GetLon(), prevInitBearing)
		if da.IsTurnSlight(tmpSign) {
			return altSegId, altHead
		}
	}
	return da.INVALID_SEGMENT_ID, da.NewInvalidCoordinate()
}

/*
isStreetMergedSkip. cek apakah 2 street (prevSegment, otherSegment) (yang arahnya berkebalikan) merged ke 1 street. Misal kalau jalan di indo:
--prevSegment-->
				tail <--segmentId-->
<--otherSegment--

examplenya di jalan solo-semarang, A.Yani : -7.5533505900708455, 110.82338424980728
atau -7.554690293226057, 110.80098414699525

kalau streetMrged == true dan nama jalan dari prevSegment == nama jalan segmentId, kita gak perlu tambahin turn instruction buat ke segmentId

relative bearing prevSegment dan otherSegment haruslah > 150 degree, jadi semacam arah nya berkebalikan
note that forward/backward direction dari osm way hanyalah direction dari nodes di simpan di osm way (https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right)

contoh:
https://www.google.com/maps/dir/-7.5502186,110.7820629/-7.5569461,110.8054174/@-7.5547439,110.8005837,19.49z/am=t/data=!4m9!4m8!1m1!4e1!1m0!3e0!6m3!1i0!2i1!3i1?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik  -7.554690293226057, 110.80098414699525 , yang merupakan tail yang merged prevSegment dan otherSegment (yang saling berlawanan arahnya) ke segmentId, dan nama jalan dari prevSegment  == nama jalan segmentId
di kasus ini gmaps gak ngasih turn instruction buat ke segmentId


contoh2:
https://www.google.com/maps/dir/-7.5512339,110.8198894/-7.5542367,110.8268638/@-7.5530866,110.8231349,20.17z/am=t/data=!4m6!4m5!3e0!6m3!1i0!2i0!3i0?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik -7.55331131002897, 110.82335347667555 , yang merupakan tail dari merged prevSegment dan otherSegment (yang saling berlawanan arahnya) ke segmentId, dan nama jalan dari prevSegment  == nama jalan segmentId
di kasus ini gmaps juga gak ngasih turn instruction buat ke segmentId

contoh3:
tai node: https://www.openstreetmap.org/node/517642592

*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetMergedSkip(prevSegment, segmentId da.Index, streetName, prevSegmentStreetName string,
	prevSegmentRoadClass, currRoadClass pkg.OsmHighwayType, isSameName func(streetName, prevSegmentStreetName string) bool) bool {
	tail := db.rn.GetSegmentTailCoord(segmentId)

	if !isSameName(streetName, prevSegmentStreetName) || currRoadClass != prevSegmentRoadClass {
		return false
	}

	otherSegment := da.INVALID_SEGMENT_ID // outEdge dari tail selain PrevEdge yang mengarah dari tail

	prevTail := db.rn.GetSegmentTailCoord(prevSegment)

	prevInitBearing := geo.ComputeInitialBearing(prevTail.GetLat(), prevTail.GetLon(), tail.GetLat(),
		tail.GetLon())

	db.graph.ForOutEdgesOf(prevSegment, func(_, oSegmentId, _ da.Index) {
		if oSegmentId == segmentId || oSegmentId == prevSegment {
			return
		}
		oStreetName := db.rn.GetStreetName(oSegmentId)
		oHead := db.GetHeadPoint(oSegmentId, tail, 10)

		relBearing := util.RadiansToDegree(math.Abs(geo.ComputeRelativeBearing(tail.GetLat(),
			tail.GetLon(), oHead.GetLat(), oHead.GetLon(), prevInitBearing)))

		isOSegmentBidirectional := db.rn.IsStreetBidirectional(oSegmentId)

		if relBearing > RELATIVE_BEARING_U_TURN &&
			isSameName(prevSegmentStreetName, oStreetName) && !isOSegmentBidirectional {
			otherSegment = oSegmentId
		}
	})

	if otherSegment == da.INVALID_SEGMENT_ID {
		return false
	}

	oSegmentLanes := db.rn.GetRoadLanes(otherSegment)

	// harus convert uint8 ke int, karena kalau gak diconvert pas hasil subtraction negatif jadi 255
	laneDiff := int(db.rn.GetRoadLanes(segmentId)) - int((db.rn.GetRoadLanes(prevSegment) + oSegmentLanes)) // lane dari other 1 & lane dari prev  1
	return laneDiff <= 1
}

/*
	isStreetSplit.	cek apakah edge sekarang hasil dari split edge sebelumnya.

	 							--segmentId-->
	 	<--prevSegment-->tail
							   	<--otherSegment--
examplenya di -7.559777239220366, 110.83649946865347

jika streetSplit==true dan nama jalan dari otherSegment == nama jalan segmentId , kita gak perlu kasih turn instruction buat ke segmentId

contoh kedua: -7.555498615741148, 110.80275937109323
https://www.google.com/maps/dir/Warung+Makan+Mas+Eng+Bebek+dan+Ayam+Goreng+Kremes,+Jl.+Adi+Sucipto+No.133,+Jajar,+Kec.+Colomadu,+Kabupaten+Karanganyar,+Jawa+Tengah+57174/-7.5569461,110.8054174/@-7.5478686,110.7846665,16.7z/data=!4m9!4m8!1m5!1m1!1s0x2e7a146abedbd01d:0x6e1d57d5149dc641!2m2!1d110.7825854!2d-7.5485275!1m0!3e0?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik  -7.555498615741148, 110.80275937109323 ,  yang merupakan tail dan ada split ke 2 edge segmentId dan  otherSegment (yang arahnya saling berlawanan), dan  nama jalan prevSegment == nama jalan segmentId
gmaps gak ngasih turn instruction buat ke segmentId

contoh2:
https://www.google.com/maps/dir/-7.5675585,110.8262857/-7.5717457,110.8241106/@-7.5701915,110.8246814,19.26z/am=t/data=!4m7!4m6!3e0!5i1!6m3!1i0!2i0!3i4?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
di titik split  -7.570920015762306, 110.8244399952867 ,  karena setelah titik split nama jalan ganti gmaps tetep ngasih turn instruction "Continue onto Jl. Tj. Anom/Jl. Yos Sudarso"


*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetSplitSkip(prevSegment, segmentId da.Index, currStreetName, prevSegmentStreetName string,
	prevSegmentRoadClass, currRoadClass pkg.OsmHighwayType, isSameName func(currStreetName, prevSegmentStreetName string) bool,
	head da.Coordinate) bool {
	tail := db.rn.GetSegmentTailCoord(segmentId)

	if !isSameName(currStreetName, prevSegmentStreetName) || currRoadClass != prevSegmentRoadClass {
		return false
	}

	otherSegment := da.INVALID_SEGMENT_ID // inEdge dari tail selain PrevEdge yang mengarah ke tail

	db.graph.ForInEdgesOf(segmentId, func(_, oSegmentId, _ da.Index) {
		if oSegmentId == prevSegment {
			return
		}

		oSegmentName := db.rn.GetStreetName(oSegmentId)
		oTail := db.rn.GetSegmentTailCoord(oSegmentId)

		oHead := db.rn.GetSegmentHeadCoord(oSegmentId)

		prevInitBearing := geo.ComputeInitialBearing(oTail.GetLat(), oTail.GetLon(), oHead.GetLat(), oHead.GetLon())

		relBearing := util.RadiansToDegree(math.Abs(geo.ComputeRelativeBearing(tail.GetLat(), tail.GetLon(), head.GetLat(),
			head.GetLon(), prevInitBearing)))

		isOSegmentBidirectional := db.rn.IsStreetBidirectional(oSegmentId)

		if isSameName(currStreetName, oSegmentName) && relBearing > RELATIVE_BEARING_U_TURN &&
			!isOSegmentBidirectional {

			otherSegment = oSegmentId
		}
	})

	if otherSegment == da.INVALID_SEGMENT_ID {
		return false
	}

	otherSegmentOutEdgeId := db.graph.GetOutId(otherSegment)

	oSegmentLanes := db.rn.GetRoadLanes(otherSegmentOutEdgeId)

	laneDiff := int(db.rn.GetRoadLanes(prevSegment)) - int((oSegmentLanes + db.rn.GetRoadLanes(segmentId))) // lane dari  otherSegment  & segmentId cuma 1
	return laneDiff <= 1
}

/*
isStreetMerged. cek apakah 2 street (prevSegment, otherSegment) (yang satu arah) merged ke 1 street. Misal kalau jalan di indo:
--prevSegment-->
				tail --segmentId-->
--otherSegment-->

contohnya di tail node: https://www.openstreetmap.org/node/7298595963

kalau merged gini kita harus output Merge Onto jl. .....

contoh2:
https://www.google.com/maps/dir/-7.568126,110.8386365/-7.5752331,110.8287711/@-7.5696958,110.8304612,20.45z/am=t/data=!4m7!4m6!3e0!5i2!6m3!1i0!2i0!3i2?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D

di tail node -7.569596062971082, 110.83036862554624 , turn instruction gmaps  "Merge onto Jl. Jend. Sudirman"
meskipun sign dari relativeBearing nya TURN_SLIGHT_LEFT

*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetMerged(prevSegment, segmentId da.Index, currStreetName, prevSegmentStreetName string,
	isSameName func(currStreetName, prevSegmentStreetName string) bool) bool {
	tail := db.rn.GetSegmentTailCoord(segmentId)

	if isSameName(currStreetName, prevSegmentStreetName) {
		return false
	}

	otherSegment := da.INVALID_SEGMENT_ID // outEdge dari tail selain PrevEdge yang mengarah dari tail

	prevTail := db.rn.GetSegmentTailCoord(prevSegment)

	prevInitBearing := geo.ComputeInitialBearing(prevTail.GetLat(), prevTail.GetLon(), tail.GetLat(),
		tail.GetLon())

	db.graph.ForInEdgesOf(segmentId, func(_, oSegmentId, _ da.Index) {
		if oSegmentId == prevSegment {
			return
		}
		edgeStreetName := db.rn.GetStreetName(oSegmentId)

		oTail := db.rn.GetSegmentTailCoord(oSegmentId)
		oHead := db.GetHeadPoint(oSegmentId, tail, 10)

		relativeBearing := util.RadiansToDegree(math.Abs(geo.ComputeRelativeBearing(oTail.GetLat(),
			oTail.GetLon(), oHead.GetLat(), oHead.GetLon(), prevInitBearing)))

		isOSegmentBidirectional := db.rn.IsStreetBidirectional(oSegmentId)

		if oSegmentId != segmentId && relativeBearing < MERGE_RELATIVE_BEARING &&
			isSameName(currStreetName, edgeStreetName) && !isOSegmentBidirectional {
			otherSegment = oSegmentId
		}
	})

	if otherSegment == da.INVALID_SEGMENT_ID {
		return false
	}

	oSegmentLanes := db.rn.GetRoadLanes(otherSegment)

	// harus convert uint8 ke int, karena kalau gak diconvert pas hasil subtraction negatif jadi 255
	laneDiff := int(db.rn.GetRoadLanes(segmentId)) - int((db.rn.GetRoadLanes(prevSegment) + oSegmentLanes)) // lane dari other 1 & lane dari prev  1
	return laneDiff <= 1
}

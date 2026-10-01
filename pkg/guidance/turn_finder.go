package guidance

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
todo1: tambahin motorway handler (jalan toll)
todo2: tambahin destination di driving direction (https://wiki.openstreetmap.org/wiki/Key:destination)
todo3: pake tag osm way ini: https://wiki.openstreetmap.org/wiki/Key:turn , https://wiki.openstreetmap.org/wiki/Key:turn:lanes
todo4: add test expected outputnya pake driving direction google map (dengan rute yang sama) (DONE).
*/

// kayake ini turnSign untuk setiap possible (e1, e2) di every intersections bisa di precompute di fase preprocessing.
// jumlah semua turnSign untuk setiap possible (e1, e2) cuma O(n). n= number of intersections.
// hal ini karena outDegree(v)=inDegree(v)=O(1) untuk every intersection/vertex v di road network.
// rata-rata outDegree of any intersection=2.43 untuk graf road network Amerika Serikat 9th DIMACS Implementation Challenge - Shortest Paths(Demetrescu et al. (2006)) (https://www.diag.uniroma1.it/challenge9/download.shtml)

func (db *DirectionBuilder) getTurnSign(segmentId da.Index, name string) da.TurnType {

	key := util.Bitpack(uint32(db.prevSegmentId), uint32(segmentId))

	if val, ok := db.turnSignCache.GetIfPresent(key); ok {
		db.updatePrevInitialBearing(segmentId)
		turn, streetName := unpackCacheVal(val)
		db.nextStreetName = streetName
		return turn
	}

	edgeRoadClass := db.rn.GetRoadClass(segmentId)
	edgeRoadClassLink := db.rn.GetRoadClassLink(segmentId)

	switch edgeRoadClass {
	case pkg.TERTIARY, pkg.RESIDENTIAL, pkg.LIVING_STREET, pkg.SERVICE, pkg.PRIVATE, pkg.ROAD, pkg.TRACK:
		return db.handleResidentialRoadTurn(db.prevSegmentId, segmentId, name)
	case pkg.PRIMARY, pkg.SECONDARY, pkg.TRUNK:
		return db.handlePrimaryRoadTurn(db.prevSegmentId, segmentId, name)
	default:
		switch edgeRoadClassLink {
		case pkg.TERTIARY_LINK, pkg.RESIDENTIAL_LINK:
			return db.handleResidentialRoadTurn(db.prevSegmentId, segmentId, name)
		case pkg.PRIMARY_LINK, pkg.SECONDARY_LINK, pkg.TRUNK_LINK:
			return db.handlePrimaryRoadTurn(db.prevSegmentId, segmentId, name)
		}
		if edgeRoadClass == pkg.UNKNOWN || edgeRoadClass == pkg.UNCLASSIFIED {
			return db.handleResidentialRoadTurn(db.prevSegmentId, segmentId, name)
		}
	}

	return da.IGNORE
}

/*
handleResidentialRoadTurn. get turn instruction untuk edge dengan highway type (osm way) yang biasanya ada di pemukiman/perumahan/pedesaan/parkiran/akses ke bangunan (parkiran mall/akses ke univ,dll).
karena di osm, osm ways dengan tipe pkg.RESIDENTIAL, pkg.LIVING_STREET, pkg.TERTIARY, etc banyak gak ada namanya, kita harus return turn sign (selain CONTINUE_ON_STREET) meskipun nama jalannya empty "".

contoh directions: https://www.google.com/maps/dir/-7.5505556,110.7819106/-7.5531604,110.7634857/@-7.5492422,110.7685478,16z/am=t/data=!3m1!4b1!4m3!4m2!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
contoh osm way pkg.TERTIARY yang gak ada namanya: https://www.openstreetmap.org/way/332233207#map=17/-7.555473/110.769728.

contoh turn right:
prev----prevEdge----tail
						|
						|
						currentEdge
						|
						|
						headPoint
*/ // nolint: gofmt
func (db *DirectionBuilder) handleResidentialRoadTurn(prevSegmentId, segmentId da.Index, currStreetName string) da.TurnType {

	curved := db.rn.IsCurved(segmentId)

	db.nextStreetName = db.rn.GetStreetNameId(segmentId)

	tail := db.rn.GetSegmentGeometryPoint(segmentId, 0)
	head := db.GetHeadPoint(segmentId, tail, 25)

	headLat := head.GetLat()
	headLon := head.GetLon()

	prev := db.GetPrevPoint(prevSegmentId, tail, 25)

	db.prevInitialBearing = geo.ComputeInitialBearing(prev.GetLat(), prev.GetLon(),
		tail.GetLat(), tail.GetLon())

	sign := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headLat, headLon, db.prevInitialBearing)

	currRoadClass := db.rn.GetRoadClass(segmentId)
	currRoadClassLink := db.rn.GetRoadClassLink(segmentId)

	prevStreetName := db.rn.GetStreetName(db.prevSegmentId)
	prevRoadClass := db.rn.GetRoadClass(db.prevSegmentId)

	isTertiary := (currRoadClass == pkg.TERTIARY || currRoadClassLink == pkg.TERTIARY_LINK)

	streetSplitSkip := db.isStreetSplitSkip(db.prevSegmentId, segmentId, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		isSameResidentialName, head) && isTertiary
	streetMergedSkip := db.isStreetMergedSkip(db.prevSegmentId, segmentId, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		isSameResidentialName) && isTertiary

	leavingPrevStreet := !isSameResidentialName(prevStreetName, currStreetName)
	alternativeTurnsCount, alternativeTurns := db.GetAlternativeTurns(prevSegmentId, segmentId)

	if !da.IsTurnSlight(sign) {
		if streetMergedSkip || streetSplitSkip || (alternativeTurnsCount == 0 && curved) {
			return da.IGNORE
		}

		// sign is not CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		return sign
	} else if leavingPrevStreet && currStreetName != "" && prevStreetName != "" && !streetMergedSkip && !streetSplitSkip {
		//  sign CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		// dan ada nama street dari prevEdge dan currentEdge
		return sign
	}

	// disini sign = TURN_SLIGHT_*/CONTINUE dan name == ""
	// kita hanya output TURN_SLIGHT_* jika ada other edge dari tail yang signnya CONTINUE/TURN_SLIGHT_*
	ocSegment, _ := db.getOtherEdgeContinueDirection(tail, db.prevInitialBearing, alternativeTurns)
	if ocSegment != da.INVALID_SEGMENT_ID && leavingPrevStreet {
		return sign
	}

	return da.IGNORE
}

func isSameResidentialName(name1, name2 string) bool {
	if name1 == "" || name2 == "" {
		// seringkali di osm, nama street kosong "" (terutama di residential/living street/tertiary osm ways), better dianggap false
		// biar kalo belok masih ada turn instructionnya
		// contoh tertiary osm way yang gak ada namanya:  https://www.openstreetmap.org/way/332233207#map=17/-7.555473/110.769728
		return false
	}
	return name1 == name2
}

func isSamePrimaryName(name1, name2 string) bool {
	if name1 == "" || name2 == "" {
		// seringkali di osm, nama street kosong "" (terutama di residential/living street/tertiary osm ways), better dianggap false
		// biar kalo belok masih ada turn instructionnya
		// contoh tertiary osm way yang gak ada namanya:  https://www.openstreetmap.org/way/332233207#map=17/-7.555473/110.769728
		return false
	}
	return name1 == name2
}

/*
handlePrimaryRoadTurn. get turn instruction untuk edge dengan highway type (osm way) yang biasanya ada di Jalan nasional, Jalan provinsi (jalan yang menghubungkan ibukota provinsi ke ibukota kabupaten/kota atau antarpusat pemerintahan kabupaten/kota),
Jalan kabupaten (menghubungkan pusat pemerintahan kota/kabupaten dengan kecamatan di sekelilingnya atau antarkecamatan).
see: https://wiki.openstreetmap.org/wiki/Template:Id:Map_Features:highway?hl=id-ID

biasanya di osm ways tipe highway ini (primary,trunk,secondary) udah ada namanya di openstreetmap.

contoh driving directions:
https://www.google.com/maps/dir/-7.5501666,110.7820614/Kasunanan+Palace,+Surakarta+Hadiningrat,+Jl.+Sasono+Mulyo,+Baluwarti,+Pasar+Kliwon,+Surakarta+City,+Central+Java+57144/@-7.5630171,110.7956402,15z/am=t/data=!3m1!5s0x2e7a160578cca9e5:0xfb2dbb81e79af22d!4m11!4m10!1m1!4e1!1m5!1m1!1s0x2e7a1666277a94b3:0xe54ac955c7781a7b!2m2!1d110.8279099!2d-7.5777426!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D

*/ // nolint: gofmt
func (db *DirectionBuilder) handlePrimaryRoadTurn(prevSegmentId, segmentId da.Index, currStreetName string) da.TurnType {
	key := util.Bitpack(uint32(db.prevSegmentId), uint32(segmentId))

	curved := db.rn.IsCurved(segmentId)

	tail := db.rn.GetSegmentGeometryPoint(segmentId, 0)
	head := db.GetHeadPoint(segmentId, tail, 25)

	prev := db.GetPrevPoint(db.prevSegmentId, tail, 25)

	db.nextStreetName = db.rn.GetStreetNameId(segmentId)

	headLat := head.GetLat()
	headLon := head.GetLon()

	db.prevInitialBearing = geo.ComputeInitialBearing(prev.GetLat(), prev.GetLon(),
		tail.GetLat(), tail.GetLon())

	sign := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headLat, headLon, db.prevInitialBearing)

	currRoadClass := db.rn.GetRoadClass(segmentId)

	prevStreetName := db.rn.GetStreetName(db.prevSegmentId)
	prevRoadClass := db.rn.GetRoadClass(db.prevSegmentId)

	streetSplitSkip := db.isStreetSplitSkip(db.prevSegmentId, segmentId, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		isSamePrimaryName, head)

	streetMergedSkip := db.isStreetMergedSkip(db.prevSegmentId, segmentId, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		func(currStreetName, prevStreetName string) bool {
			if prevStreetName == "" && currStreetName == "" {
				return true
			}
			return prevStreetName != currStreetName
		})

	leavingPrevStreet := !isSamePrimaryName(prevStreetName, currStreetName)
	alternativeTurnsCount, alternativeTurns := db.GetAlternativeTurns(prevSegmentId, segmentId)

	if !da.IsTurnSlight(sign) {
		if !leavingPrevStreet || streetMergedSkip || streetSplitSkip || (alternativeTurnsCount == 0 && curved) {
			db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID))
			return da.IGNORE
		}

		if currStreetName == "" {
			// buat handle case Dual carriageway intersections: https://wiki.openstreetmap.org/wiki/Junctions
			nSegmentId, foundNextTurn, step := db.lookForward(currStreetName, 3)

			if foundNextTurn {

				// ini bisa aja uturn:
				// example: dari Jalan Insinyur Sukarno (dari arah the park) -> https://www.openstreetmap.org/way/1419877376 -> ke Jalan Insinyur Sukarno lagi (ke arah the park)

				if step == 1 {

					nSegmentId := nSegmentId[0]

					nTail := db.rn.GetSegmentTailCoord(nSegmentId)
					nHead := db.GetHeadPoint(nSegmentId, nTail, 10)
					currInitialBearing := geo.ComputeInitialBearing(tail.GetLat(), tail.GetLon(),
						nTail.GetLat(), nTail.GetLon())

					nextSign := geo.GetTurnDirection(nTail.GetLat(), nTail.GetLon(),
						nHead.GetLat(), nHead.GetLon(), currInitialBearing)

					if db.isSameConsecutiveTurn(sign, nextSign) {
						db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName))
						return sign
					}
				}

				db.useLookForward = true
				db.lookForwardStep = step
				db.updateState(segmentId, false)
				for i := 0; i < db.lookForwardStep; i++ {
					db.lastPathId++
					nSegmentId := db.path[db.lastPathId]
					db.updateState(nSegmentId, false)
				}
			}
		}

		// sign is not CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName))
		return sign
	} else if leavingPrevStreet && (sign == da.TURN_SLIGHT_LEFT || sign == da.TURN_SLIGHT_RIGHT) && !streetMergedSkip && !streetSplitSkip {
		//  sign TURN_SLIGHT_*  & leavingPrevStreet &  not split/merged street -> output sign
		// dan ada nama street dari prevEdge dan currentEdge
		// contoh:
		// google.com/maps/dir/-7.5501666,110.7820614/Kasunanan+Palace,+Surakarta+Hadiningrat,+Jl.+Sasono+Mulyo,+Baluwarti,+Pasar+Kliwon,+Surakarta+City,+Central+Java+57144/@-7.5724056,110.8273447,17z/am=t/data=!4m15!4m14!1m1!4e1!1m5!1m1!1s0x2e7a1666277a94b3:0xe54ac955c7781a7b!2m2!1d110.8279099!2d-7.5777426!3e0!5i2!6m3!1i0!2i1!3i5?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D
		// di titik -7.572258961155632, 110.82862613980265 , turn instructionnya Slight right to stay on Jl. Slamet Riyadi

		if db.isStreetMerged(db.prevSegmentId, segmentId, currStreetName, prevStreetName, isSamePrimaryName) {
			sign = da.MERGE_ONTO
			db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName))
			return sign
		}

		if alternativeTurnsCount >= 1 {
			if streetMergedSkip {
				db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID))
				return da.IGNORE
			}
			db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName))
			return sign
		}
	}

	if (sign == da.TURN_SLIGHT_LEFT || sign == da.TURN_SLIGHT_RIGHT) && (streetMergedSkip || streetSplitSkip) {

		db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID))
		return da.IGNORE
	}

	// disini sign = CONTINUE/TURN_SLIGHT_* && !(streetMergedSkip || streetSplitSkip)
	// kita bisa output CONTINUE / KEEP_LEFT / KEEP_RIGHT , tergantung dari relative bearingnya alternative turn
	// contoh:
	// https://www.google.com/maps/dir/-7.5501666,110.7820614/Kasunanan+Palace,+Surakarta+Hadiningrat,+Jl.+Sasono+Mulyo,+Baluwarti,+Pasar+Kliwon,+Surakarta+City,+Central+Java+57144/@-7.5630171,110.7956402,15z/am=t/data=!3m1!5s0x2e7a160578cca9e5:0xfb2dbb81e79af22d!4m11!4m10!1m1!4e1!1m5!1m1!1s0x2e7a1666277a94b3:0xe54ac955c7781a7b!2m2!1d110.8279099!2d-7.5777426!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D
	// pas mau masuk flyover manahan di titik -7.557800121677021, 110.80655549226334 , turn instruction nya KEEP_RIGHT...
	// kenapa??
	// karena ada alternative turn yang sama sama TURN_SLIGHT_LEFT sign nya (yang ke arah MT Haryono)

	ocSegment, oHead := db.getOtherEdgeContinueDirection(tail, db.prevInitialBearing, alternativeTurns)
	if ocSegment != da.INVALID_SEGMENT_ID {

		relBearing := geo.ComputeRelativeBearing(tail.GetLat(), tail.GetLon(), headLat, headLon, db.prevInitialBearing)
		altTurnRelBearing := geo.ComputeRelativeBearing(tail.GetLat(), tail.GetLon(), oHead.GetLat(), oHead.GetLon(), db.prevInitialBearing) // bearing difference antara prev->tail->ocSegment.GetHead()

		altTurnRelBearingDeg := util.RadiansToDegree(math.Abs(altTurnRelBearing))
		relBearingDeg := util.RadiansToDegree(math.Abs(relBearing))

		if util.Lt(relBearingDeg, CONTINUE_ALT_CURRENT_RELATIVE_BEARING) && util.Gt(altTurnRelBearingDeg, CONTINUE_ALT_TURN_RELATIVE_BEARING) {
			// bearing difference antara prevEdge dan currentEdge < 7° (CONTINUE Direction), Edge ocSegment > 8.6 (TURN SLIGHT or more direction).
			if db.nextStreetName == da.INVALID_STREET_NAME_ID || !leavingPrevStreet {
				db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID))
				return da.IGNORE
			}

			db.turnSignCache.Set(key, makeCacheVal(da.CONTINUE_ON_STREET, db.nextStreetName))
			return da.CONTINUE_ON_STREET
		}

		if util.Lt(altTurnRelBearingDeg, KEEP_LEFT_RIGHT_ALT_TURN_RELATIVE_BEARING) {
			_, foundNextTurn, step := db.lookForward(currStreetName, 2)

			if foundNextTurn {
				db.useLookForward = true
				db.lookForwardStep = step
				db.updateState(segmentId, false)
				for i := 0; i < db.lookForwardStep; i++ {
					db.lastPathId++
					nSegmentId := db.path[db.lastPathId]
					db.updateState(nSegmentId, false)
				}
			}

			_, headOsmId := db.rn.GetTailHeadOsmNodeId(ocSegment)
			oHeadOntheSameWay := db.lookForwardSameOsmWay(segmentId, headOsmId, 4)

			if oHeadOntheSameWay {
				// 2 edge searah tapi gak pindah jalan
				// contoh dari tail: https://www.openstreetmap.org/node/11294649720
				// dari tail osm node diatas ada 2 edge ke head: https://www.openstreetmap.org/node/11294649718
				// dan edge satunya ke head: https://www.openstreetmap.org/node/11294649719  (dari jalan curved/uturn ke kanan)
				db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID))
				return da.IGNORE
			}

			/*
				jika dari tail ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction KEEP_LEFT/KEEP_RIGHT ke currEdge. Example:

									-----currentEdge---------
				---prevEdge--- tail
									-----ocSegment---
				pada case diatas karena relBearing <= prevOtherEdgeInitialBearing, output keep left


			*/ // nolint: gofmt
			if relBearing > altTurnRelBearing {

				if !db.useLookForward { // only set cache kalo gak pake lookForward, ribet buat correctness nya wkwk
					db.turnSignCache.Set(key, makeCacheVal(da.KEEP_RIGHT, db.nextStreetName))
				}
				return da.KEEP_RIGHT
			} else {
				if !db.useLookForward {
					db.turnSignCache.Set(key, makeCacheVal(da.KEEP_LEFT, db.nextStreetName))
				}
				return da.KEEP_LEFT
			}
		}
	}

	// lagi karena di if diatas kita update leavingPrevStreet = leavingPrevStreet || foundNextTurn
	leavingPrevStreet = !isSamePrimaryName(prevStreetName, currStreetName)

	currStreetNameId := db.rn.GetStreetNameId(segmentId)

	// kalau gak ada ocSegment
	// kita cuma output CONTINUE_ON_STREET jika current edge street name beda dari street name prev edge
	if leavingPrevStreet && currStreetName != "" && prevStreetName != "" {
		if db.isStreetMerged(db.prevSegmentId, segmentId, currStreetName, prevStreetName, isSamePrimaryName) {
			sign = da.MERGE_ONTO
			db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName))
			return sign
		}

		db.turnSignCache.Set(key, makeCacheVal(da.CONTINUE_ON_STREET, currStreetNameId))
		return da.CONTINUE_ON_STREET
	}

	db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID))
	return da.IGNORE
}

/*
updateState. update state dari DirectionBuilder.

contoh:
prev----prevEdge----tail
							|
							|
							currentEdge
							|
							|
							headPoint

setelah evaluate turn dari currentEdge:
kita update prev, doublePrevPoint, prevNode, prevEdge, doublePrevNode, etc..
*/ // nolint: gofmt
func (db *DirectionBuilder) updateState(segmentId da.Index, isInRoundabout bool) {
	if db.prevSegmentId != da.INVALID_SEGMENT_ID {
		db.doublePrevInitialBearing = db.prevInitialBearing
		db.doublePrevStreetName = db.rn.GetStreetName(db.prevSegmentId)
	}

	db.doublePrevSegmentId = db.prevSegmentId
	db.prevInRoundabout = isInRoundabout
	db.prevSegmentId = segmentId

	db.cumulativeDistance += db.engine.GetSegmentLength(segmentId)
	db.cumulativeCost += db.engine.GetDurationSeconds(segmentId)

	if db.useAnnotation {
		db.segmentIds = append(db.segmentIds, segmentId)
		segGeometry := db.rn.GetSegmentGeometry(segmentId)
		l := max(0, len(segGeometry)-1)
		db.geometry = append(db.geometry, segGeometry[:l]...)
	}

	db.nextStreetName = db.rn.GetStreetNameId(segmentId)
}

func makeCacheVal(sign da.TurnType, streetName uint32) uint64 {
	return uint64(sign) | uint64(streetName)<<8
}

func unpackCacheVal(val uint64) (da.TurnType, uint32) {
	return da.TurnType(val & 0xff), uint32(val >> 8)
}

func (db *DirectionBuilder) updatePrevInitialBearing(segmentId da.Index) {
	tail := db.rn.GetSegmentGeometryPoint(segmentId, 0)
	prev := db.GetPrevPoint(db.prevSegmentId, tail, 25)
	db.prevInitialBearing = geo.ComputeInitialBearing(prev.GetLat(), prev.GetLon(),
		tail.GetLat(), tail.GetLon())
}

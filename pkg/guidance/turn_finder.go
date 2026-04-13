package guidance

import (
	"encoding/binary"
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
todo4: add test expected outputnya pake driving direction google map (dengan rute yang sama).
*/

func (db *DirectionBuilder) getTurnSign(edgeId da.Index, tailId, prevNodeId, headId da.Index, name string) da.TurnType {
	key := util.Bitpack(uint32(db.prevEdge), uint32(edgeId))

	if sign, ok := db.turnSignCache.Get(key); ok {
		eGeom := db.graph.GetEdgeGeometry(edgeId)
		db.updateState(eGeom, edgeId, false)

		db.nextStreetName = binary.LittleEndian.Uint32(sign[1:])
		turn := da.TurnType(sign[0])
		return turn
	}

	edgeRoadClass := db.graph.GetRoadClass(edgeId)
	edgeRoadClassLink := db.graph.GetRoadClassLink(edgeId)

	switch edgeRoadClass {
	case "tertiary", "residential", "living_street", "service", "private", "road", "track", "unclassified", "unknown", "undefined":
		return db.handleResidentialRoadTurn(edgeId, tailId, prevNodeId, headId, name)
	case "primary", "secondary", "trunk":
		return db.handlePrimaryRoadTurn(edgeId, tailId, prevNodeId, headId, name)
	default:
		switch edgeRoadClassLink {
		case "tertiary_link", "residential_link":
			return db.handleResidentialRoadTurn(edgeId, tailId, prevNodeId, headId, name)
		case "primary_link", "secondary_link", "trunk_link":
			return db.handlePrimaryRoadTurn(edgeId, tailId, prevNodeId, headId, name)
		}
	}

	return da.IGNORE
}

/*
handleResidentialRoadTurn. get turn instruction untuk edge dengan highway type (osm way) yang biasanya ada di pemukiman/perumahan/pedesaan/parkiran/akses ke bangunan (parkiran mall/akses ke univ,dll).
karena di osm, osm ways dengan tipe "residential", "living_street", "tertiary", etc banyak gak ada namanya, kita harus return turn sign (selain CONTINUE_ON_STREET) meskipun nama jalannya empty "".

contoh directions: https://www.google.com/maps/dir/-7.5505556,110.7819106/-7.5531604,110.7634857/@-7.5492422,110.7685478,16z/am=t/data=!3m1!4b1!4m3!4m2!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwNS4wIKXMDSoASAFQAw%3D%3D
contoh osm way "tertiary" yang gak ada namanya: https://www.openstreetmap.org/way/332233207#map=17/-7.555473/110.769728.

contoh turn right:
prevPoint----prevEdge----tail
						|
						|
						currentEdge
						|
						|
						headPoint
*/ // nolint: gofmt
func (db *DirectionBuilder) handleResidentialRoadTurn(edgeId da.Index, tailId, prevNodeId, headId da.Index, currStreetName string) da.TurnType {

	eGeom := db.graph.GetEdgeGeometry(edgeId)
	curved := geo.IsPolylineCurved(eGeom)

	db.nextStreetName = db.graph.GetStreetNameId(edgeId)
	defer func() {
		db.updateState(eGeom, edgeId, false)
	}()

	tailCoord := eGeom[0]
	headCoord := db.GetHeadPoint(eGeom, tailCoord, 25)

	headLat := headCoord.GetLat()
	headLon := headCoord.GetLon()

	db.prevInitialBearing = computeInitialBearing(db.prevPoint.GetLat(), db.prevPoint.GetLon(),
		tailCoord.GetLat(), tailCoord.GetLon())

	sign := getTurnDirection(tailCoord.GetLat(), tailCoord.GetLon(), headLat, headLon, db.prevInitialBearing)

	currRoadClass := db.graph.GetRoadClass(edgeId)
	currRoadClassLink := db.graph.GetRoadClassLink(edgeId)

	prevStreetName := db.graph.GetStreetName(db.prevEdge)
	prevRoadClass := db.graph.GetRoadClass(db.prevEdge)

	isTertiary := (currRoadClass == "tertiary" || currRoadClassLink == "tertiary_link")

	streetSplitSkip := db.isStreetSplitSkip(edgeId, db.prevEdge, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		isSameResidentialName) && isTertiary
	streetMergedSkip := db.isStreetMergedSkip(edgeId, db.prevEdge, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		isSameResidentialName) && isTertiary

	prevEdgeStreetName := db.graph.GetStreetName(db.prevEdge)
	leavingPrevStreet := !isSameResidentialName(prevEdgeStreetName, currStreetName)
	alternativeTurnsCount, alternativeTurns := db.GetAlternativeTurns(tailId, headId, prevNodeId)

	if !da.IsTurnSlight(sign) {
		if streetMergedSkip || streetSplitSkip || (alternativeTurnsCount == 0 && curved) {
			return da.IGNORE
		}

		// sign is not CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		return sign
	} else if leavingPrevStreet && currStreetName != "" && prevEdgeStreetName != "" && !streetMergedSkip && !streetSplitSkip {
		//  sign CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		// dan ada nama street dari prevEdge dan currentEdge
		return sign
	}

	// disini sign = TURN_SLIGHT_*/CONTINUE dan name == ""
	// kita hanya output TURN_SLIGHT_* jika ada other edge dari tail yang signnya CONTINUE/TURN_SLIGHT_*
	otherContinueEdge := db.getOtherEdgeContinueDirection(tailCoord.GetLat(), tailCoord.GetLon(), db.prevInitialBearing, alternativeTurns)
	if otherContinueEdge != da.INVALID_EDGE_ID && leavingPrevStreet {
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

func isSameNameByRoadClass(name1, name2, currRoadClass, currRoadClassLink string) bool {

	switch currRoadClass {
	case "tertiary", "residential", "living_street", "service", "private", "road", "track", "unclassified", "unknown", "undefined":
		return isSameResidentialName(name1, name2)
	default:
		switch currRoadClassLink {
		case "tertiary_link", "residential_link":
			return isSameResidentialName(name1, name2)
		}

		return name1 == name2
	}
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
handleResidentialRoadTurn. get turn instruction untuk edge dengan highway type (osm way) yang biasanya ada di Jalan nasional, Jalan provinsi (jalan yang menghubungkan ibukota provinsi ke ibukota kabupaten/kota atau antarpusat pemerintahan kabupaten/kota),
Jalan kabupaten (menghubungkan pusat pemerintahan kota/kabupaten dengan kecamatan di sekelilingnya atau antarkecamatan).
see: https://wiki.openstreetmap.org/wiki/Template:Id:Map_Features:highway?hl=id-ID

biasanya di osm ways tipe highway ini (primary,trunk,secondary) udah ada namanya di openstreetmap.

contoh driving directions:
https://www.google.com/maps/dir/-7.5501666,110.7820614/Kasunanan+Palace,+Surakarta+Hadiningrat,+Jl.+Sasono+Mulyo,+Baluwarti,+Pasar+Kliwon,+Surakarta+City,+Central+Java+57144/@-7.5630171,110.7956402,15z/am=t/data=!3m1!5s0x2e7a160578cca9e5:0xfb2dbb81e79af22d!4m11!4m10!1m1!4e1!1m5!1m1!1s0x2e7a1666277a94b3:0xe54ac955c7781a7b!2m2!1d110.8279099!2d-7.5777426!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D

*/ // nolint: gofmt
func (db *DirectionBuilder) handlePrimaryRoadTurn(edgeId da.Index, tailId, prevNodeId, headId da.Index, currStreetName string) da.TurnType {
	eGeom := db.graph.GetEdgeGeometry(edgeId)

	key := util.Bitpack(uint32(db.prevEdge), uint32(edgeId))

	curved := geo.IsPolylineCurved(eGeom)

	tailCoord := eGeom[0]
	headCoord := db.GetHeadPoint(eGeom, tailCoord, 25)

	useLookForwad := false

	db.nextStreetName = db.graph.GetStreetNameId(edgeId)

	defer func() {
		if !useLookForwad {
			db.updateState(eGeom, edgeId, false)
		}
	}()

	headLat := headCoord.GetLat()
	headLon := headCoord.GetLon()

	db.prevInitialBearing = computeInitialBearing(db.prevPoint.GetLat(), db.prevPoint.GetLon(),
		tailCoord.GetLat(), tailCoord.GetLon())

	sign := getTurnDirection(tailCoord.GetLat(), tailCoord.GetLon(), headLat, headLon, db.prevInitialBearing)

	currRoadClass := db.graph.GetRoadClass(edgeId)

	prevStreetName := db.graph.GetStreetName(db.prevEdge)
	prevRoadClass := db.graph.GetRoadClass(db.prevEdge)

	streetSplitSkip := db.isStreetSplitSkip(edgeId, db.prevEdge, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		isSamePrimaryName)

	streetMergedSkip := db.isStreetMergedSkip(edgeId, db.prevEdge, currStreetName, prevStreetName, prevRoadClass, currRoadClass,
		func(currStreetName, prevEdgeStreetName string) bool {
			return true
		})

	prevEdgeStreetName := db.graph.GetStreetName(db.prevEdge)
	leavingPrevStreet := !isSamePrimaryName(prevEdgeStreetName, currStreetName)
	alternativeTurnsCount, alternativeTurns := db.GetAlternativeTurns(tailId, headId, prevNodeId)

	if !da.IsTurnSlight(sign) {
		if !leavingPrevStreet || streetMergedSkip || streetSplitSkip || (alternativeTurnsCount == 0 && curved) {
			db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID), 1)
			return da.IGNORE
		}

		if currStreetName == "" {
			// buat handle case Dual carriageway intersections: https://wiki.openstreetmap.org/wiki/Junctions
			nextEdgeIds, foundNextTurn, step := db.lookForward(currStreetName, 3)

			if foundNextTurn {

				useLookForwad = true
				db.lastPathId = db.lastPathId + step
				for _, nexteId := range nextEdgeIds {
					nextEGeom := db.graph.GetEdgeGeometry(nexteId)
					db.updateState(nextEGeom, nexteId, false)
				}
			}
		}

		// sign is not CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName), 1)
		return sign
	} else if leavingPrevStreet && (sign == da.TURN_SLIGHT_LEFT || sign == da.TURN_SLIGHT_RIGHT) && !streetMergedSkip && !streetSplitSkip {
		//  sign TURN_SLIGHT_*  & leavingPrevStreet &  not split/merged street -> output sign
		// dan ada nama street dari prevEdge dan currentEdge
		// contoh:
		// google.com/maps/dir/-7.5501666,110.7820614/Kasunanan+Palace,+Surakarta+Hadiningrat,+Jl.+Sasono+Mulyo,+Baluwarti,+Pasar+Kliwon,+Surakarta+City,+Central+Java+57144/@-7.5724056,110.8273447,17z/am=t/data=!4m15!4m14!1m1!4e1!1m5!1m1!1s0x2e7a1666277a94b3:0xe54ac955c7781a7b!2m2!1d110.8279099!2d-7.5777426!3e0!5i2!6m3!1i0!2i1!3i5?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D
		// di titik -7.572258961155632, 110.82862613980265 , turn instructionnya Slight right to stay on Jl. Slamet Riyadi

		if db.isStreetMerged(edgeId, db.prevEdge, currStreetName, prevStreetName, isSamePrimaryName) {
			sign = da.MERGE_ONTO
			db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName), 1)
			return sign
		}

		if alternativeTurnsCount >= 1 {
			db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName), 1)
			return sign
		}
	}

	if (sign == da.TURN_SLIGHT_LEFT || sign == da.TURN_SLIGHT_RIGHT) && (streetMergedSkip || streetSplitSkip) {

		db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID), 1)
		return da.IGNORE
	}

	// disini sign = CONTINUE/TURN_SLIGHT_* && !(streetMergedSkip || streetSplitSkip)
	// kita bisa output CONTINUE / KEEP_LEFT / KEEP_RIGHT , tergantung dari relative bearingnya alternative turn
	// contoh:
	// https://www.google.com/maps/dir/-7.5501666,110.7820614/Kasunanan+Palace,+Surakarta+Hadiningrat,+Jl.+Sasono+Mulyo,+Baluwarti,+Pasar+Kliwon,+Surakarta+City,+Central+Java+57144/@-7.5630171,110.7956402,15z/am=t/data=!3m1!5s0x2e7a160578cca9e5:0xfb2dbb81e79af22d!4m11!4m10!1m1!4e1!1m5!1m1!1s0x2e7a1666277a94b3:0xe54ac955c7781a7b!2m2!1d110.8279099!2d-7.5777426!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D
	// pas mau masuk flyover manahan di titik -7.557800121677021, 110.80655549226334 , turn instruction nya KEEP_RIGHT...
	// kenapa??
	// karena ada alternative turn yang sama sama TURN_SLIGHT_LEFT sign nya (yang ke arah MT Haryono)

	otherContinueEdge := db.getOtherEdgeContinueDirection(tailCoord.GetLat(), tailCoord.GetLon(), db.prevInitialBearing, alternativeTurns)
	if otherContinueEdge != da.INVALID_EDGE_ID {

		otherContinueEdgeHead := db.graph.GetHeadOfOutEdge(otherContinueEdge)
		otherHeadCoord := db.graph.GetVertexCoordinate(otherContinueEdgeHead)
		otherHeadLat, otherHeadLon := otherHeadCoord.GetLat(), otherHeadCoord.GetLon()

		currRelativeBearing := computeRelativeBearing(tailCoord.GetLat(), tailCoord.GetLon(), headLat, headLon, db.prevInitialBearing)
		alternativeTurnRelativeBearing := computeRelativeBearing(tailCoord.GetLat(), tailCoord.GetLon(), otherHeadLat, otherHeadLon, db.prevInitialBearing) // bearing difference antara prevPoint->tail->otherContinueEdge.GetHead()

		alternativeTurnRelativeBearingDeg := util.RadiansToDegree(math.Abs(alternativeTurnRelativeBearing))
		currRelativeBearingDeg := util.RadiansToDegree(math.Abs(currRelativeBearing))

		if util.Lt(currRelativeBearingDeg, CONTINUE_ALT_CURRENT_DELTA_BEARING) && util.Gt(alternativeTurnRelativeBearingDeg, CONTINUE_ALT_TURN_DELTA_BEARING) {
			// bearing difference antara prevEdge dan currentEDge < 7° (CONTINUE Direction), Edge otherContinueEdge > 8.6 (TURN SLIGHT or more direction).
			if db.nextStreetName == da.INVALID_STREET_NAME_ID || !leavingPrevStreet {
				db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID), 1)
				return da.IGNORE
			}
			db.turnSignCache.Set(key, makeCacheVal(da.CONTINUE_ON_STREET, db.nextStreetName), 1)
			return da.CONTINUE_ON_STREET
		}

		if util.Lt(alternativeTurnRelativeBearingDeg, KEEP_LEFT_RIGHT_ALT_TURN_DELTA_BEARING) {
			nextEdgeIds, foundNextTurn, step := db.lookForward(currStreetName, 2)

			if foundNextTurn {

				useLookForwad = true
				db.lastPathId = db.lastPathId + step
				for _, nexteId := range nextEdgeIds {
					nextEGeom := db.graph.GetEdgeGeometry(nexteId)
					db.updateState(nextEGeom, nexteId, false)
				}
			}

			otherHeadOntheSameWay := db.lookForwardSameOsmWay(edgeId, otherContinueEdgeHead, 4)

			if otherHeadOntheSameWay {
				// 2 edge searah tapi gak pindah jalan
				// contoh dari tail: https://www.openstreetmap.org/node/11294649720
				// dari tail osm node diatas ada 2 edge ke head: https://www.openstreetmap.org/node/11294649718
				// dan edge satunya ke head: https://www.openstreetmap.org/node/11294649719  (dari jalan curved/uturn ke kanan)
				db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID), 1)
				return da.IGNORE
			}

			/*
				jika dari tail ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction KEEP_LEFT/KEEP_RIGHT ke currEdge. Example:

									-----currentEdge---------
				---prevEdge--- tail
									-----otherContinueEdge---
				pada case diatas karena currRelativeBearing <= prevOtherEdgeInitialBearing, output keep left


			*/ // nolint: gofmt
			if currRelativeBearing > alternativeTurnRelativeBearing {
				if !useLookForwad { // only set cache kalo gak pake lookForward, ribet buat correctness nya wkwk
					db.turnSignCache.Set(key, makeCacheVal(da.KEEP_RIGHT, db.nextStreetName), 1)
				}
				return da.KEEP_RIGHT
			} else {
				if !useLookForwad {
					db.turnSignCache.Set(key, makeCacheVal(da.KEEP_LEFT, db.nextStreetName), 1)
				}
				return da.KEEP_LEFT
			}
		}
	}

	// lagi karena di if diatas kita update leavingPrevStreet = leavingPrevStreet || foundNextTurn
	leavingPrevStreet = !isSamePrimaryName(prevEdgeStreetName, currStreetName)

	currStreetNameId := db.graph.GetStreetNameId(edgeId)

	// kalau gak ada otherContinueEdge
	// kita cuma output CONTINUE_ON_STREET jika current edge street name beda dari street name prev edge
	if leavingPrevStreet && currStreetName != "" && prevStreetName != "" {
		db.turnSignCache.Set(key, makeCacheVal(da.CONTINUE_ON_STREET, currStreetNameId), 1)
		return da.CONTINUE_ON_STREET
	}

	db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, da.INVALID_STREET_NAME_ID), 1)
	return da.IGNORE
}

/*
updateState. update state dari DirectionBuilder.

contoh:
prevPoint----prevEdge----tail
							|
							|
							currentEdge
							|
							|
							headPoint

setelah evaluate turn dari currentEdge:
kita update prevPoint, doublePrevPoint, prevNode, prevEdge, doublePrevNode, etc..
*/ // nolint: gofmt
func (db *DirectionBuilder) updateState(eGeom da.Coordinates, edgeId da.Index, isInRoundabout bool) {

	headId := db.graph.GetHeadOfOutEdge(edgeId)
	tailId := db.graph.GetTailOfOutedge(edgeId)
	head := db.graph.GetVertexCoordinate(headId)

	n := len(eGeom)
	db.doublePrevPoint = db.prevPoint
	db.prevPoint = db.GetPrevPoint(eGeom, eGeom[n-1], 25)

	db.doublePrevNode = db.prevNode
	db.prevInRoundabout = isInRoundabout
	db.prevNode = tailId
	db.prevEdge = edgeId

	db.cumulativeDistance += db.graph.GetOutEdgeLength(edgeId)
	db.cumulativeTravelTime += db.engine.GetWeight(edgeId, true)

	db.edgeIds = append(db.edgeIds, edgeId)
	db.points = append(db.points,
		da.NewCoordinate(head.GetLat(), head.GetLon()))

	db.nextStreetName = db.graph.GetStreetNameId(edgeId)

	isHeadTrafficLight := db.graph.IsTrafficLight(headId)
	if isHeadTrafficLight {
		db.cumulativeTravelTime += util.SecondsToMinutes(pkg.TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND)
	}
}

func makeCacheVal(sign da.TurnType, streetName uint32) []byte {
	buf := make([]byte, 4+1)
	buf[0] = byte(sign)
	binary.LittleEndian.PutUint32(buf[1:], streetName)
	return buf
}

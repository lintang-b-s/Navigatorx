package guidance

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
todo1: tambahin motorway handler (jalan toll)
todo2: tambahin destination di driving direction (https://wiki.openstreetmap.org/wiki/Key:destination)
todo3: pake tag osm way ini: https://wiki.openstreetmap.org/wiki/Key:turn

*/

func (db *DirectionBuilder) getTurnSign(edgeId da.Index, tailId, prevNodeId, headId da.Index, name string) da.TurnType {
	key := util.Bitpack(uint32(db.prevEdge), uint32(edgeId))

	if sign, ok := db.turnSignCache.Get(key); ok {

		db.nextStreetName = string(sign[1:])
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
prevNode----prevEdge----tail
						|
						|
						currentEdge
						|
						|
						head
*/ // nolint: gofmt
func (db *DirectionBuilder) handleResidentialRoadTurn(edgeId da.Index, tailId, prevNodeId, headId da.Index, currStreetName string) da.TurnType {
	var (
		tailCoord, headCoord da.Coordinate
	)
	key := util.Bitpack(uint32(db.prevEdge), uint32(edgeId))
	db.nextStreetName = currStreetName

	eGeom := db.graph.GetEdgeGeometry(edgeId)
	if len(eGeom) > 3 {
		tailCoord = eGeom[1]
		headCoord = eGeom[len(eGeom)-2]
	} else {
		tailCoord = eGeom[0]
		headCoord = eGeom[len(eGeom)-1]
	}

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
	if !da.IsTurnSlight(sign) {
		if !leavingPrevStreet || streetMergedSkip || streetSplitSkip {
			db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, ""), 1)
			return da.IGNORE
		}

		// sign is not CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName), 1)
		return sign
	} else if leavingPrevStreet && currStreetName != "" && prevEdgeStreetName != "" && !streetMergedSkip && !streetSplitSkip {
		//  sign CONTINUE/TURN_SLIGHT_* & street name berubah dari prev edge ke curr edge & not split/merged street -> output sign
		// dan ada nama street dari prevEdge dan currentEdge
		db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName), 1)
		return sign
	}

	// disini sign = TURN_SLIGHT_*/CONTINUE dan name == ""
	// kita hanya output TURN_SLIGHT_* jika ada other edge yang signnya CONTINUE
	_, alternativeTurns := db.GetAlternativeTurns(tailId, headId, prevNodeId)
	otherContinueEdge := db.getOtherEdgeContinueDirection(tailCoord.GetLat(), tailCoord.GetLon(), db.prevInitialBearing, alternativeTurns)
	if otherContinueEdge != da.INVALID_EDGE_ID {
		db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName), 1)
		return sign
	}

	db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, ""), 1)
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
	var (
		tailCoord, headCoord da.Coordinate
	)
	key := util.Bitpack(uint32(db.prevEdge), uint32(edgeId))

	db.nextStreetName = currStreetName

	eGeom := db.graph.GetEdgeGeometry(edgeId)
	if len(eGeom) > 3 {
		tailCoord = eGeom[1]
		headCoord = eGeom[len(eGeom)-2]
	} else {
		tailCoord = eGeom[0]
		headCoord = eGeom[len(eGeom)-1]
	}
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
	if !da.IsTurnSlight(sign) {
		if !leavingPrevStreet || streetMergedSkip || streetSplitSkip {
			db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, ""), 1)
			return da.IGNORE
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

		db.turnSignCache.Set(key, makeCacheVal(sign, db.nextStreetName), 1)
		return sign
	}

	if (sign == da.TURN_SLIGHT_LEFT || sign == da.TURN_SLIGHT_RIGHT) && (streetMergedSkip || streetSplitSkip) {

		db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, ""), 1)
		return da.IGNORE
	}

	// disini sign = CONTINUE/TURN_SLIGHT_* && !(streetMergedSkip || streetSplitSkip)
	// kita bisa output CONTINUE / KEEP_LEFT / KEEP_RIGHT , tergantung dari delta bearingnya alternative turn
	// contoh:
	// https://www.google.com/maps/dir/-7.5501666,110.7820614/Kasunanan+Palace,+Surakarta+Hadiningrat,+Jl.+Sasono+Mulyo,+Baluwarti,+Pasar+Kliwon,+Surakarta+City,+Central+Java+57144/@-7.5630171,110.7956402,15z/am=t/data=!3m1!5s0x2e7a160578cca9e5:0xfb2dbb81e79af22d!4m11!4m10!1m1!4e1!1m5!1m1!1s0x2e7a1666277a94b3:0xe54ac955c7781a7b!2m2!1d110.8279099!2d-7.5777426!3e0!5i2?entry=ttu&g_ep=EgoyMDI2MDQwNy4wIKXMDSoASAFQAw%3D%3D
	// pas mau masuk flyover manahan di titik -7.557800121677021, 110.80655549226334 , turn instruction nya KEEP_RIGHT...
	// kenapa??
	// karena ada alternative turn yang sama sama TURN_SLIGHT_LEFT sign nya (yang ke arah MT Haryono)

	_, alternativeTurns := db.GetAlternativeTurns(tailId, headId, prevNodeId)
	otherContinueEdge := db.getOtherEdgeContinueDirection(tailCoord.GetLat(), tailCoord.GetLon(), db.prevInitialBearing, alternativeTurns)
	if otherContinueEdge != da.INVALID_EDGE_ID {

		otherContinueEdgeHead := db.graph.GetHeadOfOutEdge(otherContinueEdge)
		otherHeadCoord := db.graph.GetVertexCoordinate(otherContinueEdgeHead)
		otherHeadLat, otherHeadLon := otherHeadCoord.GetLat(), otherHeadCoord.GetLon()

		currDeltaBearing := computeDeltaBearing(tailCoord.GetLat(), tailCoord.GetLon(), headLat, headLon, db.prevInitialBearing)
		alternativeTurnDeltaBearing := computeDeltaBearing(tailCoord.GetLat(), tailCoord.GetLon(), otherHeadLat, otherHeadLon, db.prevInitialBearing) // bearing difference antara prevNode->tail->otherContinueEdge.GetHead()

		nextStreetName, foundNextTurn, step := db.lookForward(currStreetName, 2)
		leavingPrevStreet = leavingPrevStreet || foundNextTurn

		if foundNextTurn {
			db.nextStreetName = nextStreetName
			db.lastPathId = db.lastPathId + step - 1
		}

		if util.RadiansToDegree(math.Abs(currDeltaBearing)) < 7 && util.RadiansToDegree(math.Abs(alternativeTurnDeltaBearing)) > 8.6 {
			// bearing difference antara prevEdge dan currentEDge < 7° (CONTINUE Direction), Edge otherContinueEdge > 8.6 (TURN SLIGHT or more direction).
			if db.nextStreetName == "" {
				db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, ""), 1)
				return da.IGNORE
			}
			db.turnSignCache.Set(key, makeCacheVal(da.CONTINUE_ON_STREET, db.nextStreetName), 1)
			return da.CONTINUE_ON_STREET
		}

		if util.RadiansToDegree(math.Abs(alternativeTurnDeltaBearing)) < 40 {
			/*
				jika dari tail ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction KEEP_LEFT/KEEP_RIGHT ke currEdge. Example:

									-----currentEdge---------
				---prevEdge--- tail
									-----otherContinueEdge---
				pada case diatas karena currDeltaBearing <= prevOtherEdgeInitialBearing, output keep left


			*/ // nolint: gofmt
			if currDeltaBearing > alternativeTurnDeltaBearing {
				db.turnSignCache.Set(key, makeCacheVal(da.KEEP_RIGHT, db.nextStreetName), 1)
				return da.KEEP_RIGHT
			} else {
				db.turnSignCache.Set(key, makeCacheVal(da.KEEP_LEFT, db.nextStreetName), 1)
				return da.KEEP_LEFT
			}
		}

	}

	// kalau gak ada otherContinueEdge
	// kita cuma output CONTINUE_ON_STREET jika current edge street name beda dari street name prev edge
	if leavingPrevStreet {
		db.turnSignCache.Set(key, makeCacheVal(da.CONTINUE_ON_STREET, db.nextStreetName), 1)
		return da.CONTINUE_ON_STREET
	}

	db.turnSignCache.Set(key, makeCacheVal(da.IGNORE, ""), 1)
	return da.IGNORE
}

// lookForward. traverse dari db.path[lastPathId] ke beberapa next db.path[lastPathId+1:...] until found edge dengan name != prevName.
// return forward edge di path dengan name != prevName, true jika ada, dan number of step lookForward.
func (db *DirectionBuilder) lookForward(prevName string, maxStep int) (string, bool, int) {
	step := 0
	for i := db.lastPathId + 1; step < maxStep; i++ {
		currEdgeStreetname := db.graph.GetStreetName(db.path[i])
		step++
		if currEdgeStreetname != prevName {
			return currEdgeStreetname, true, step
		}
	}

	return "", false, 0
}

func makeCacheVal(sign da.TurnType, streetName string) []byte {
	buf := make([]byte, len(streetName)+1)
	buf[0] = byte(sign)
	buf = append(buf, []byte(streetName)...)
	return buf
}

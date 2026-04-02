package guidance

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

/*
GetAlternativeTurns. get jumlah belokan alternatif yang bisa dilakukan dari tail sekarang & bukan currentEdge/prevEdge. Misalkan:

		 |
		 |
	 alternative
		 |
--prev-- B --currentEdge---
		 |
		 |
	alternative
		 |

ada 4 belokan yang bisa dilakukan dari tail B.
--- / | = jalan 2 arah
*/ // nolint: gofmt
func (db *DirectionBuilder) GetAlternativeTurns(tailId, headId, prevVertexId da.Index) (int, []da.Index) {
	db.tempOutEdges = db.tempOutEdges[:0] // reset length, tapi capacity tetep sama

	db.graph.ForOutEdgeIdsOf(tailId, func(eId da.Index) {

		db.tempOutEdges = append(db.tempOutEdges, eId)
	})

	db.tempAltTurns = db.tempAltTurns[:0]

	for _, edge := range db.tempOutEdges {
		if edge != prevVertexId && edge != headId {
			db.tempAltTurns = append(db.tempAltTurns, edge)
		}
	}

	return 1 + len(db.tempAltTurns), db.tempAltTurns
}

func (db *DirectionBuilder) isLeavingCurrentStreet(prevStreetName, currentStreetName string, prevEdge, currEdge da.Index) bool {
	if isSameName(currentStreetName, prevStreetName) {
		// isSameName == false - bisa ketika nama street kosong di osm.
		return false
	}

	prevEdgeRoadClass, prevEdgeRoadClassLink := db.graph.GetRoadClass(prevEdge), db.graph.GetRoadClassLink(prevEdge)
	currEdgeRoadClass, currEdgeRoadClassLink := db.graph.GetRoadClass(currEdge), db.graph.GetRoadClassLink(currEdge)

	if ok := isSameRoadClassAndLink(prevEdgeRoadClass, prevEdgeRoadClassLink,
		currEdgeRoadClass, currEdgeRoadClassLink); !ok {
		// leave current street jika prevStreetName  != currentStreetName && roadclass/roadclassLink prevEdge,currentEdge beda.
		return true
	}
	return false
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

delta bearing antara currentEdge dan alternativeEdge mendekati 0°
*/ // nolint: gofmt
func (db *DirectionBuilder) getOtherEdgeContinueDirection(prevLat, prevLon, prevInitialBearing float64, alternativeTurns []da.Index) da.Index {
	var tmpSign int
	for _, edge := range alternativeTurns {
		edgeHead := db.graph.GetHeadOfOutEdge(edge)

		node := db.graph.GetVertex(edgeHead)
		lat, lon := node.GetLat(), node.GetLon()

		tmpSign = getTurnDirection(prevLat, prevLon, lat, lon, prevInitialBearing)
		if math.Abs(float64(tmpSign)) <= 1 {
			return edge
		}
	}
	return da.INVALID_EDGE_ID
}

/*
	isStreetMerged. cek apakah 2 street (prevEdge, otherEdge) merged ke 1 street. Misal kalau jalan di indo:
	--prevEdge-->
					tail <--currentEdge-->
	<--otherEdge--

examplenya di jalan solo-semarang, A.Yani : -7.5533505900708455, 110.82338424980728
*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetMerged(currentEdge, prevEdge da.Index, currStreetName, prevEdgeStreetName string,
	prevEdgeRoadClass, currRoadClass string) bool {
	tail := db.graph.GetTailOfOutedge(currentEdge)

	if currRoadClass != prevEdgeRoadClass {
		return false
	}

	otherEdge := da.INVALID_EDGE_ID // outEdge dari tail selain PrevEdge yang mengarah dari tail

	db.tempOutEdges = db.tempOutEdges[:0] // reset length, tapi capacity tetep sama

	db.graph.ForOutEdgeIdsOf(tail, func(eId da.Index) {

		db.tempOutEdges = append(db.tempOutEdges, eId)
	})

	for _, edge := range db.tempOutEdges {

		edgeStreetName := db.graph.GetStreetName(edge)

		edgeRoadClass := db.graph.GetRoadClass(edge)

		currentEdgeHead := db.graph.GetHeadOfOutEdge(currentEdge)
		edgeHead := db.graph.GetHeadOfOutEdge(edge)

		if edge != currentEdge && edge != prevEdge &&
			edgeHead != currentEdgeHead && edgeHead != db.graph.GetTailOfOutedge(prevEdge) &&
			currRoadClass == edgeRoadClass &&
			isSameName(currStreetName, edgeStreetName) {
			if otherEdge != da.INVALID_EDGE_ID {
				return false
			}
			otherEdge = edge
		}
	}

	if otherEdge == da.INVALID_EDGE_ID {
		return false
	}

	currentEdgeDirection := db.graph.GetStreetDirection(currentEdge) // [0] forward, [1] reversed
	if currentEdgeDirection[1] {

		prevEdgeDirection := db.graph.GetStreetDirection(prevEdge)
		if prevEdgeDirection[1] {
			// prevEdge harus tidak punya reversed direction
			return false
		}

		otherEdgeLanes := db.graph.GetRoadLanes(otherEdge)

		otherEdgeDirection := db.graph.GetStreetDirection(otherEdge)
		if !otherEdgeDirection[0] || otherEdgeDirection[1] {
			// otherEdge harus only forward edge (bukan bidirectional)
			return false
		}

		laneDiff := db.graph.GetRoadLanes(currentEdge) - db.graph.GetRoadLanes(prevEdge) + otherEdgeLanes // setidaknya lane dari currentEdge 2. lane dari other & prev 1
		return laneDiff <= 1
	}
	return false
}

/*
	isStreetSplit.	cek apakah edge sekarang hasil dari split edge sebelumnya.

	 							--currentEdge-->
	 	<--prevEdge-->tail
							   	<--otherEdge--
		examplenya di -7.559777239220366, 110.83649946865347
*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetSplit(currentEdge, prevEdge da.Index, currStreetName, prevEdgeStreetName string,
	prevEdgeRoadClass, currRoadClass string) bool {
	tail := db.graph.GetTailOfOutedge(currentEdge)

	if !isSameName(currStreetName, prevEdgeStreetName) || currRoadClass != prevEdgeRoadClass {
		return false
	}

	otherEdge := da.INVALID_EDGE_ID // inEdge dari tail selain PrevEdge yang mengarah ke tail

	db.tempInEdges = db.tempInEdges[:0]

	db.graph.ForInEdgeIdsOf(tail, func(eId da.Index) {
		db.tempInEdges = append(db.tempInEdges, eId)
	})
	prevEdgeTail := db.graph.GetTailOfOutedge(prevEdge)

	for _, inEdge := range db.tempInEdges {

		edgeStreetName := db.graph.GetStreetName(inEdge)
		edgeRoadClass := db.graph.GetRoadClass(inEdge)
		currentEdgeHead := db.graph.GetHeadOfOutEdge(currentEdge)
		inEdgeTail := db.graph.GetTailOfInedge(inEdge)

		if inEdge != currentEdge && inEdge != prevEdge &&
			inEdgeTail != currentEdgeHead && inEdgeTail != prevEdgeTail &&
			currRoadClass == edgeRoadClass &&
			isSameName(currStreetName, edgeStreetName) {
			if otherEdge != da.INVALID_EDGE_ID {
				return false
			}
			otherEdge = inEdge
		}
	}

	if otherEdge == da.INVALID_EDGE_ID {
		return false
	}

	otherEdgeLanes := db.graph.GetRoadLanes(otherEdge)

	prevEdgeDirection := db.graph.GetStreetDirection(prevEdge) // [0] forward, [1] reversed
	if !prevEdgeDirection[1] {
		// jika prevEdge bukan bidirectional
		return false
	}

	laneDiff := db.graph.GetRoadLanes(prevEdge) -
		(otherEdgeLanes + db.graph.GetRoadLanes(currentEdge)) // setidak nya lane dari PrevEdge 2. lane dari  otherEdge  & currentEdge cuma 1
	return laneDiff <= 1
}

func isSameName(name1, name2 string) bool {
	if name1 == "" || name2 == "" {
		// seringkali di osm, nama street kosong "", better dianggap false
		return false
	}
	return name1 == name2
}

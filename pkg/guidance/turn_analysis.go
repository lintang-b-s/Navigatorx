package guidance

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
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
func (db *DirectionBuilder) GetAlternativeTurns(tailId, headId, prevVertexId datastructure.Index) (int, []*datastructure.OutEdge) {
	alternativeTurns := []*datastructure.OutEdge{}
	var (
		edges []*datastructure.OutEdge = make([]*datastructure.OutEdge, 0)
	)

	db.graph.ForOutEdgesOfWithId(tailId, func(e *datastructure.OutEdge, id datastructure.Index) {
		edges = append(edges, e)
	})

	for _, edge := range edges {
		if edge.GetHead() != prevVertexId && edge.GetHead() != headId {
			alternativeTurns = append(alternativeTurns, edge)
		}
	}

	return 1 + len(alternativeTurns), alternativeTurns
}

func (db *DirectionBuilder) isLeavingCurrentStreet(prevStreetName, currentStreetName string, prevEdge, currEdge datastructure.OutEdge) bool {
	if isSameName(currentStreetName, prevStreetName) {
		// isSameName == false - bisa ketika nama street kosong di osm.
		return false
	}

	prevEdgeRoadClass, prevEdgeRoadClassLink := db.graph.GetRoadClass(prevEdge.GetEdgeId()), db.graph.GetRoadClassLink(prevEdge.GetEdgeId())
	currEdgeRoadClass, currEdgeRoadClassLink := db.graph.GetRoadClass(currEdge.GetEdgeId()), db.graph.GetRoadClassLink(currEdge.GetEdgeId())

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
func (db *DirectionBuilder) getOtherEdgeContinueDirection(prevLat, prevLon, prevInitialBearing float64, alternativeTurns []*datastructure.OutEdge) *datastructure.OutEdge {
	var tmpSign int
	for _, edge := range alternativeTurns {

		node := db.graph.GetVertex(edge.GetHead())
		lat, lon := node.GetLat(), node.GetLon()

		tmpSign = getTurnDirection(prevLat, prevLon, lat, lon, prevInitialBearing)
		if math.Abs(float64(tmpSign)) <= 1 {
			return edge
		}
	}
	return nil
}

/*
	isStreetMerged. cek apakah 2 street (prevEdge, otherEdge) merged ke 1 street. Misal kalau jalan di indo:
	--prevEdge-->
					tail <--currentEdge-->
	<--otherEdge--

examplenya di jalan solo-semarang, A.Yani : -7.5533505900708455, 110.82338424980728
*/ // nolint: gofmt
func (db *DirectionBuilder) isStreetMerged(currentEdge, prevEdge datastructure.OutEdge) bool {
	tail := db.graph.GetTailOfOutedge(currentEdge.GetEdgeId())

	currStreetName := db.graph.GetStreetName(currentEdge.GetEdgeId())
	currRoadClass := db.graph.GetRoadClass(currentEdge.GetEdgeId())
	prevEdgeRoadClass := db.graph.GetRoadClass(prevEdge.GetEdgeId())
	if currRoadClass != prevEdgeRoadClass {
		return false
	}

	var otherEdge *datastructure.OutEdge = nil // outEdge dari tail selain PrevEdge yang mengarah dari tail
	outEdges := make([]*datastructure.OutEdge, 0)

	db.graph.ForOutEdgesOfWithId(tail, func(e *datastructure.OutEdge, id datastructure.Index) {
		outEdges = append(outEdges, e)
	})

	for _, edge := range outEdges {

		edgeStreetName := db.graph.GetStreetName(edge.GetEdgeId())

		edgeRoadClass := db.graph.GetRoadClass(edge.GetEdgeId())

		if edge.GetEdgeId() != currentEdge.GetEdgeId() && edge.GetEdgeId() != prevEdge.GetEdgeId() &&
			edge.GetHead() != currentEdge.GetHead() && edge.GetHead() != db.graph.GetTailOfOutedge(prevEdge.GetEdgeId()) &&
			currRoadClass == edgeRoadClass &&
			isSameName(currStreetName, edgeStreetName) {
			if otherEdge != nil {
				return false
			}
			otherEdge = edge
		}
	}

	if otherEdge == nil {
		return false
	}

	currentEdgeDirection := db.graph.GetStreetDirection(currentEdge.GetEdgeId()) // [0] forward, [1] reversed
	if currentEdgeDirection[1] {

		prevEdgeDirection := db.graph.GetStreetDirection(prevEdge.GetEdgeId())
		if prevEdgeDirection[1] {
			// prevEdge harus tidak punya reversed direction
			return false
		}

		otherEdgeLanes := db.graph.GetRoadLanes(otherEdge.GetEdgeId())

		otherEdgeDirection := db.graph.GetStreetDirection(otherEdge.GetEdgeId())
		if !otherEdgeDirection[0] || otherEdgeDirection[1] {
			// otherEdge harus only forward edge (bukan bidirectional)
			return false
		}

		laneDiff := db.graph.GetRoadLanes(currentEdge.GetEdgeId()) - db.graph.GetRoadLanes(prevEdge.GetEdgeId()) + otherEdgeLanes // setidaknya lane dari currentEdge 2. lane dari other & prev 1
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
func (db *DirectionBuilder) isStreetSplit(currentEdge, prevEdge datastructure.OutEdge) bool {
	tail := db.graph.GetTailOfOutedge(currentEdge.GetEdgeId())

	currStreetName := db.graph.GetStreetName(currentEdge.GetEdgeId())
	currRoadClass := db.graph.GetRoadClass(currentEdge.GetEdgeId())
	prevEdgeRoadClass := db.graph.GetRoadClass(prevEdge.GetEdgeId())
	prevEdgeStreetName := db.graph.GetStreetName(prevEdge.GetEdgeId())
	if !isSameName(currStreetName, prevEdgeStreetName) || currRoadClass != prevEdgeRoadClass {
		return false
	}

	var otherEdge *datastructure.InEdge = nil // inEdge dari tail selain PrevEdge yang mengarah ke tail
	var (
		inEdges []*datastructure.InEdge = make([]*datastructure.InEdge, 0)
	)

	db.graph.ForInEdgesOfWithId(tail, func(e *datastructure.InEdge, id datastructure.Index) {
		inEdges = append(inEdges, e)
	})
	prevEdgeTail := db.graph.GetTailOfOutedge(prevEdge.GetEdgeId())

	for _, inEdge := range inEdges {

		edgeStreetName := db.graph.GetStreetName(inEdge.GetEdgeId())
		edgeRoadClass := db.graph.GetRoadClass(inEdge.GetEdgeId())

		if inEdge.GetEdgeId() != currentEdge.GetEdgeId() && inEdge.GetEdgeId() != prevEdge.GetEdgeId() &&
			inEdge.GetTail() != currentEdge.GetHead() && inEdge.GetTail() != prevEdgeTail &&
			currRoadClass == edgeRoadClass &&
			isSameName(currStreetName, edgeStreetName) {
			if otherEdge != nil {
				return false
			}
			otherEdge = inEdge
		}
	}
	if otherEdge == nil {
		return false
	}

	otherEdgeLanes := db.graph.GetRoadLanes(otherEdge.GetEdgeId())

	prevEdgeDirection := db.graph.GetStreetDirection(prevEdge.GetEdgeId()) // [0] forward, [1] reversed
	if !prevEdgeDirection[1] {
		// jika prevEdge bukan bidirectional
		return false
	}

	laneDiff := db.graph.GetRoadLanes(prevEdge.GetEdgeId()) -
		(otherEdgeLanes + db.graph.GetRoadLanes(currentEdge.GetEdgeId())) // setidak nya lane dari PrevEdge 2. lane dari  otherEdge  & currentEdge cuma 1
	return laneDiff <= 1
}

func isSameName(name1, name2 string) bool {
	if name1 == "" || name2 == "" {
		// seringkali di osm, nama street kosong "", better dianggap false
		return false
	}
	return name1 == name2
}

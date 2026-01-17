package guidance

import (
	"math"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type DirectionBuilder struct {
	graph                    Graph
	instructions             []*datastructure.Instruction
	prevEdge                 datastructure.OutEdge
	prevNode                 datastructure.Index
	prevInitialBearing       float64
	doublePrevInitialBearing float64
	prevInstruction          *datastructure.Instruction
	prevInRoundabout         bool
	doublePrevStreetName     string
	doublePrevNode           datastructure.Index
	cumulativeDistance       float64
	cumulativeTravelTime     float64
	edgeIds                  []datastructure.Index
	points                   []datastructure.Coordinate
	clockwise                bool // clockwise roundabout (like in indonesia) or counter-clockwise roundabout
	lefthand                 bool // left hand traffic (like in indonesia) or right hand traffic
}

func NewDirectionBuilder(graph Graph, clockwise, lefthand bool) *DirectionBuilder {
	return &DirectionBuilder{
		graph:                    graph,
		instructions:             make([]*datastructure.Instruction, 0),
		prevNode:                 math.MaxUint32,
		prevInRoundabout:         false,
		doublePrevInitialBearing: 0,
		clockwise:                clockwise,
		lefthand:                 lefthand,
	}
}

func (db *DirectionBuilder) GetDrivingDirections(path []datastructure.OutEdge) []datastructure.DrivingDirection {

	for _, edge := range path {
		db.buildInstruction(edge)
	}
	db.buildFinalInstruction()

	var turnDescriptions = make([]string, len(db.instructions))
	for i, ins := range db.instructions {
		desc := ins.GetTurnDescription(db.clockwise)
		turnDescriptions[i] = desc
	}

	drivingDirections := make([]datastructure.DrivingDirection, len(db.instructions))

	for i, ins := range db.instructions {
		var (
			currStepTravelTime, currStepDistance float64
		)
		if i > 0 {
			currStepTravelTime = db.instructions[i].GetCumulativeTravelTime() - db.instructions[i-1].GetCumulativeTravelTime()
			currStepDistance = db.instructions[i].GetCumulativeDistance() - db.instructions[i-1].GetCumulativeDistance()
		}
		way := *db.instructions[i]
		currPolyline := geo.PoylineFromCoords(datastructure.NewGeoCoordinates(way.GetPoints()))
		drivingDirections[i] = datastructure.NewDrivingDirection(way, turnDescriptions[i],
			currStepTravelTime, currStepDistance, way.GetEdgeIds(), currPolyline, ins.GetTurnBearing())
	}

	return drivingDirections
}

func (db *DirectionBuilder) buildInstruction(edge datastructure.OutEdge) {
	headId := edge.GetHead()
	tailId := db.graph.GetTailOfOutedge(edge.GetEdgeId())

	tail := db.graph.GetVertex(tailId)
	head := db.graph.GetVertex(headId)

	isRoundabout := db.graph.IsRoundabout(edge.GetEdgeId())

	var prevNode datastructure.Vertex
	if db.prevNode != math.MaxUint32 {
		prevNode = *db.graph.GetVertex(db.prevNode)
	}

	streetName := db.graph.GetStreetName(edge.GetEdgeId())
	if db.prevInstruction == nil && !isRoundabout {
		// start point dari shortetest path & bukan bundaran (roundabout)
		sign := datastructure.START
		point := datastructure.NewCoordinate(
			tail.GetLat(), tail.GetLon(),
		)
		turnBearing := computeInitialBearing(tail.GetLat(), tail.GetLon(),
			head.GetLat(), head.GetLon())
		newIns := datastructure.NewInstruction(sign, streetName, point, false,
			[]datastructure.Index{edge.GetEdgeId()}, db.cumulativeDistance, db.cumulativeTravelTime,
			db.points, turnBearing, db.clockwise)
		db.prevInstruction = newIns

		db.edgeIds = make([]datastructure.Index, 0)
		db.points = make([]datastructure.Coordinate, 0)

		db.prevInstruction.SetExtraInfo("heading", turnBearing)
		db.instructions = append(db.instructions, db.prevInstruction)
	} else if isRoundabout {
		// current edge bundaran
		if !db.prevInRoundabout {
			sign := datastructure.USE_ROUNDABOUT
			point := datastructure.NewCoordinate(tail.GetLat(), tail.GetLon())
			roundaboutInstruction := datastructure.NewRoundaboutInstruction()

			db.doublePrevInitialBearing = db.prevInitialBearing
			if db.prevInstruction != nil {
				db.prevInitialBearing = computeInitialBearing(prevNode.GetLat(), prevNode.GetLon(), tail.GetLat(), tail.GetLon())

			} else {
				// start point dari shortetest path & dan bundaran (roundabout)
				db.prevInitialBearing = computeInitialBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
			}

			turnBearing := computeFinalBearing(prevNode.GetLat(), prevNode.GetLon(), tail.GetLat(), tail.GetLon())
			prevIns := datastructure.NewInstructionWithRoundabout(sign, streetName, point, true, roundaboutInstruction, db.cumulativeDistance,
				db.cumulativeTravelTime, db.edgeIds, turnBearing)
			db.prevInstruction = &prevIns

			// reset edgeIDs and points
			db.edgeIds = []datastructure.Index{}
			db.points = []datastructure.Coordinate{}

			db.instructions = append(db.instructions, db.prevInstruction)
		}

		db.graph.ForOutEdgesOfWithId(headId, func(e *datastructure.OutEdge, id datastructure.Index) {
			eIsRoundabout := db.graph.IsRoundabout(e.GetEdgeId())
			if !eIsRoundabout {
				roundaboutInstruction := db.prevInstruction
				roundaboutInstruction.IncrementExitNumber()
			}
		})
	} else if db.prevInRoundabout {
		db.prevInstruction.SetStreetName(streetName)
		roundaboutInstruction := db.prevInstruction
		roundaboutInstruction.SetExited()
		db.doublePrevStreetName = db.graph.GetStreetName(db.prevEdge.GetEdgeId())
	} else {
		turnSign := db.getTurnSign(edge, tailId, db.prevNode, headId, streetName)
		if turnSign != datastructure.IGNORE {
			uTurn, uturnType := db.checkUTurn(turnSign, streetName, edge)
			if uTurn {
				db.prevInstruction.SetSign(uturnType)
				db.prevInstruction.SetStreetName(streetName)
			} else {
				// bukan U-turn -> continue/right/left
				tail := datastructure.NewCoordinate(tail.GetLat(), tail.GetLon())
				turnBearing := computeFinalBearing(prevNode.GetLat(), prevNode.GetLon(),
					tail.GetLat(), tail.GetLon())
				prevIns := datastructure.NewInstruction(turnSign, streetName, tail, false, db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime,
					db.points, turnBearing, db.clockwise)
				db.prevInstruction = prevIns

				db.edgeIds = make([]datastructure.Index, 0)
				db.points = make([]datastructure.Coordinate, 0)

				db.doublePrevInitialBearing = db.prevInitialBearing
				db.doublePrevStreetName = db.graph.GetStreetName(db.prevEdge.GetEdgeId())
				db.instructions = append(db.instructions, prevIns)
			}
		}
	}

	db.doublePrevNode = db.prevNode
	db.prevInRoundabout = isRoundabout
	db.prevNode = tailId
	db.prevEdge = edge
	db.cumulativeDistance += edge.GetLength()
	db.cumulativeTravelTime += edge.GetWeight()
	db.edgeIds = append(db.edgeIds, edge.GetEdgeId())
	db.points = append(db.points,
		datastructure.NewCoordinate(tail.GetLat(), tail.GetLon()))
	db.points = append(db.points,
		datastructure.NewCoordinate(head.GetLat(), head.GetLon()))

	isHeadTrafficLight := db.graph.IsTrafficLight(headId)
	if isHeadTrafficLight {
		db.cumulativeTravelTime += util.SecondsToMinutes(pkg.TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND)
	}
}

func (db *DirectionBuilder) buildFinalInstruction() {

	doublePrevNode := db.graph.GetVertex(db.doublePrevNode)

	tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(db.prevEdge.GetEdgeId()))

	node := db.graph.GetVertex(db.prevEdge.GetHead())
	point := datastructure.NewCoordinate(node.GetLat(), node.GetLon())

	prevNodeData := db.graph.GetVertex(db.prevNode)
	turnBearing := computeFinalBearing(prevNodeData.GetLat(), prevNodeData.GetLon(), tail.GetLat(), tail.GetLon())

	finishInstruction := datastructure.NewInstruction(datastructure.FINISH, db.graph.GetStreetName(db.prevEdge.GetEdgeId()), point, false,
		db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime, db.points, turnBearing, db.clockwise)
	finishInstruction.SetExtraInfo("heading", geo.BearingTo(doublePrevNode.GetLat(), doublePrevNode.GetLon(), tail.GetLat(), tail.GetLon()))

	db.instructions = append(db.instructions, finishInstruction)

}

/*
checkUTurn. check if current edge is U-turn. Example:

A --doublePrevEdge-->B
				    |
					|
				PrevEdge
					|
					|
					|
D <--currentEdge---C

If from A->B turn right, and from B->C turn right, and the delta bearing between A->B and C->D is close to 180 degrees, then it can be considered a U-turn.
*/ // nolint: gofmt
func (db *DirectionBuilder) checkUTurn(sign int, name string, edge datastructure.OutEdge) (bool, int) {
	isUTurn := false
	uTurnType := datastructure.U_TURN_UNKNOWN

	if db.doublePrevInitialBearing != 0 && (sign > 0) == (db.prevInstruction.GetTurnSign() > 0) &&
		((db.lefthand && (util.Abs(sign) == datastructure.TURN_SLIGHT_RIGHT || util.Abs(sign) == datastructure.TURN_RIGHT || util.Abs(sign) == datastructure.TURN_SHARP_RIGHT)) ||
			(!db.lefthand && (util.Abs(sign) == datastructure.TURN_SLIGHT_LEFT || util.Abs(sign) == datastructure.TURN_LEFT || util.Abs(sign) == datastructure.TURN_SHARP_LEFT))) &&
		((db.lefthand && util.Abs(db.prevInstruction.GetTurnSign()) == datastructure.TURN_SLIGHT_RIGHT || util.Abs(db.prevInstruction.GetTurnSign()) == datastructure.TURN_RIGHT || util.Abs(db.prevInstruction.GetTurnSign()) == datastructure.TURN_SHARP_RIGHT) ||
			(!db.lefthand && util.Abs(db.prevInstruction.GetTurnSign()) == datastructure.TURN_SLIGHT_LEFT || util.Abs(db.prevInstruction.GetTurnSign()) == datastructure.TURN_LEFT || util.Abs(db.prevInstruction.GetTurnSign()) == datastructure.TURN_SHARP_LEFT)) &&
		isSameName(db.doublePrevStreetName, name) {
		head := db.graph.GetVertex(edge.GetHead())
		headLat, headLon := head.GetLat(), head.GetLon()
		tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(edge.GetEdgeId()))
		currentInitialBearing := computeInitialBearing(tail.GetLat(), tail.GetLon(), headLat, headLon)
		diff := math.Abs(db.doublePrevInitialBearing - currentInitialBearing)
		diffAngle := util.RadiansToDegree(diff)
		if diffAngle > 155 && diffAngle < 205 {
			isUTurn = true
			if sign < 0 {
				uTurnType = datastructure.U_TURN_LEFT
			} else {
				uTurnType = datastructure.U_TURN_RIGHT
			}
		}
	}
	return isUTurn, uTurnType
}

/*
getTurnSign.Obtain the turn sign of every two adjacent edges on the shortest path based on the initial bearing difference. For example:

prevNode----prevEdge----tail
							|
							|
						currentEdge
							|
							|
						head

*/ // nolint: gofmt
func (db *DirectionBuilder) getTurnSign(edge datastructure.OutEdge, tailId, prevNodeId, headId datastructure.Index, name string) int {
	tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(edge.GetEdgeId()))
	point := db.graph.GetVertex(edge.GetHead())
	lat := point.GetLat()
	lon := point.GetLon()

	prevNode := db.graph.GetVertex(db.prevNode)

	db.prevInitialBearing = computeInitialBearing(prevNode.GetLat(), prevNode.GetLon(),
		tail.GetLat(), tail.GetLon())
	sign := getTurnDirection(tail.GetLat(), tail.GetLon(), lat, lon, db.prevInitialBearing)

	alternativeTurnsCount, alternativeTurns := db.GetAlternativeTurns(tailId, headId, prevNodeId)

	isStreetSplit := db.isStreetSplit(edge, db.prevEdge)

	isStreetMerged := db.isStreetMerged(edge, db.prevEdge)

	if alternativeTurnsCount == 1 {
		// there are only one alternative turns (other than this current edge)
		if math.Abs(float64(sign)) > 1 && !(isStreetMerged || isStreetSplit) {
			// there are only one alternative turns (other than this current edge) and sign is not CONTINUE/TURN_SLIGHT_*
			return sign
		}
		return datastructure.IGNORE
	}

	prevEdgeStreetName := db.graph.GetStreetName(db.prevEdge.GetEdgeId())
	if math.Abs(float64(sign)) > 1 {
		if (isSameName(name, prevEdgeStreetName)) ||
			isStreetMerged || isStreetSplit {
			return datastructure.IGNORE
		}
		// sign is not CONTINUE/TURN_SLIGHT_*  & street name changed from prev edge to curr edge & not split/merged street, so output sign
		return sign
	}

	if db.prevEdge.GetWeight() == 0 {
		return sign
	}

	// get another edge from the tail that have CONTINUE direction
	otherContinueEdge := db.getOtherEdgeContinueDirection(tail.GetLat(), tail.GetLon(), db.prevInitialBearing, alternativeTurns)

	prevCurrEdgeInitialBearingDiff := computeDeltaBearing(tail.GetLat(), tail.GetLon(), lat, lon, db.prevInitialBearing) // bearing difference antara prevNode->tail->head
	if otherContinueEdge != nil {
		// there is another edge (connected to the tail) in the same direction and have CONTINUE direction.
		if !isSameName(name, prevEdgeStreetName) {
			// current Street Name != prevEdge Street Name
			roadClass := db.graph.GetRoadClass(edge.GetEdgeId())
			prevRoadClass := db.graph.GetRoadClass(db.prevEdge.GetEdgeId())
			otherRoadClass := db.graph.GetRoadClass(otherContinueEdge.GetEdgeId())

			link := db.graph.GetRoadClassLink(edge.GetEdgeId())
			prevLink := db.graph.GetRoadClassLink(db.prevEdge.GetEdgeId())
			otherLink := db.graph.GetRoadClassLink(otherContinueEdge.GetEdgeId())

			node := db.graph.GetVertex(otherContinueEdge.GetHead())
			tmpLat, tmpLon := node.GetLat(), node.GetLon()

			if isMajorRoad(roadClass) {
				if (roadClass == prevRoadClass && link == prevLink) && (otherRoadClass != prevRoadClass || otherLink != prevLink) {
					// current road class == major road class && prevRoadClass sama dg current edge roadClass
					return datastructure.IGNORE
				}
			}

			prevOtherEdgeInitialBearing := computeDeltaBearing(tail.GetLat(), tail.GetLon(), tmpLat, tmpLon, db.prevInitialBearing) // bearing difference antara prevNode->tail->otherContinueEdge.GetHead()

			if util.RadiansToDegree(math.Abs(prevCurrEdgeInitialBearingDiff)) < 6 && util.RadiansToDegree(math.Abs(prevOtherEdgeInitialBearing)) > 8.6 && isSameName(name, prevEdgeStreetName) {
				// bearing difference antara prevEdge dan currentEDge < 6° (CONTINUE Direction), Edge otherContinueEdge > 8.6 (TURN SLIGHT or more direction). Nama prevEdge street == current Street
				return datastructure.CONTINUE_ON_STREET
			}

			if roadClass == "residential" || prevRoadClass == "residential" || (roadClass == "unclassified" && prevRoadClass == "unclassified") {
				// skip roadclass residential untuk mengurangi instructions.
				return datastructure.IGNORE
			}

			/*
				jika dari tail ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction ke currEdge. Example:

						-----currentEdge---------
				tail
						-----otherContinueEdge---
				pada case diatas, output keep left karena prevCurrEdgeInitialBearingDiff <= prevOtherEdgeInitialBearing
			*/ // nolint: gofmt
			if prevCurrEdgeInitialBearingDiff > prevOtherEdgeInitialBearing {
				return datastructure.KEEP_RIGHT
			} else {
				return datastructure.KEEP_LEFT
			}
		}
	}

	if ok := db.isLeavingCurrentStreet(prevEdgeStreetName, name, db.prevEdge, edge); !(isStreetMerged || isStreetSplit) &&
		(util.RadiansToDegree(math.Abs(prevCurrEdgeInitialBearingDiff)) > 34 || ok) {
		// current streetname != prev streetname & not street split & not street merged, so output sign other than CONTINUE_ON_STREET
		return sign
	}
	return datastructure.IGNORE
}

func isMajorRoad(roadClass string) bool {
	return roadClass == "motorway" || roadClass == "trunk" || roadClass == "primary" || roadClass == "secondary" || roadClass == "tertiary"
}

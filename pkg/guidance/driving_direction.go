package guidance

import (
	"math"
	"sync"

	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type DirectionBuilder struct {
	turnSignCache *lru.Cache[uint64, int]

	instructions             []*da.Instruction
	prevInstruction          *da.Instruction
	turnDescriptions         []string
	tempOutEdges             []da.OutEdge
	tempInEdges              []da.InEdge
	tempAltTurns             []da.OutEdge
	points                   []da.Coordinate
	edgeIds                  []da.Index
	drivingInstructionPool   *sync.Pool
	coordinatesPool          *sync.Pool
	drivingEdgeIdsPool       *sync.Pool
	graph                    Graph
	prevEdge                 da.OutEdge
	doublePrevStreetName     string
	prevInitialBearing       float64
	doublePrevInitialBearing float64
	cumulativeDistance       float64
	cumulativeTravelTime     float64
	doublePrevNode           da.Index
	prevNode                 da.Index
	clockwise                bool // clockwise roundabout (like in indonesia) or counter-clockwise roundabout
	lefthand                 bool // left hand traffic (like in indonesia) or right hand traffic
	prevInRoundabout         bool
}

func NewDirectionBuilder(graph Graph, clockwise, lefthand bool, drivingInstructionPool, coordinatesPool, drivingEdgeIdsPool *sync.Pool,
	turnSignCache *lru.Cache[uint64, int]) *DirectionBuilder {
	db := &DirectionBuilder{
		graph:                    graph,
		prevNode:                 math.MaxUint32,
		prevInRoundabout:         false,
		doublePrevInitialBearing: 0,
		clockwise:                clockwise,
		lefthand:                 lefthand,
		drivingInstructionPool:   drivingInstructionPool,
		coordinatesPool:          coordinatesPool,
		drivingEdgeIdsPool:       drivingEdgeIdsPool,
		turnDescriptions:         make([]string, 256),
		tempOutEdges:             make([]da.OutEdge, 0, 8),
		tempInEdges:              make([]da.InEdge, 0, 8),
		tempAltTurns:             make([]da.OutEdge, 0, 8),
		turnSignCache:            turnSignCache,
	}

	return db
}

func (db *DirectionBuilder) reset() {
	db.coordinatesPool.Put(db.points)
	db.drivingEdgeIdsPool.Put(db.edgeIds)

	db.points = db.coordinatesPool.Get().([]da.Coordinate)
	db.points = db.points[:0]

	db.edgeIds = db.drivingEdgeIdsPool.Get().([]da.Index)
	db.edgeIds = db.edgeIds[:0]
}

func (db *DirectionBuilder) done() {
	db.drivingInstructionPool.Put(db.instructions)
	db.coordinatesPool.Put(db.points)
	db.drivingEdgeIdsPool.Put(db.edgeIds)

}

func (db *DirectionBuilder) Reset() {

	db.prevNode = math.MaxUint32
	db.prevInRoundabout = false
	db.prevInstruction = nil
	db.prevEdge = da.OutEdge{}
	db.doublePrevStreetName = ""
	db.prevInitialBearing = 0
	db.doublePrevInitialBearing = 0
	db.cumulativeDistance = 0
	db.cumulativeTravelTime = 0
	db.doublePrevNode = 0

	db.instructions = db.drivingInstructionPool.Get().([]*da.Instruction)
	db.instructions = db.instructions[:0]
}

func (db *DirectionBuilder) GetDrivingDirections(path []da.OutEdge, drivingDirections []da.DrivingDirection) []da.DrivingDirection {
	defer db.done()

	for _, edge := range path {
		db.buildInstruction(edge)
	}
	db.buildFinalInstruction()

	n := len(db.instructions)
	if len(db.turnDescriptions) < n {
		db.turnDescriptions = make([]string, n)
	}

	for i, ins := range db.instructions {
		desc := ins.GetTurnDescription(db.clockwise)
		db.turnDescriptions[i] = desc
	}

	for i, ins := range db.instructions {
		var (
			currStepTravelTime, currStepDistance float64
		)
		if i > 0 {
			currStepTravelTime = db.instructions[i].GetCumulativeTravelTime() - db.instructions[i-1].GetCumulativeTravelTime()
			currStepDistance = db.instructions[i].GetCumulativeDistance() - db.instructions[i-1].GetCumulativeDistance()
		}
		way := *db.instructions[i]
		currPolyline := geo.PoylineFromCoords(da.NewGeoCoordinates(way.GetPoints()))
		drivingDirections = append(drivingDirections, da.NewDrivingDirection(way, db.turnDescriptions[i],
			currStepTravelTime, currStepDistance, way.GetEdgeIds(), currPolyline, ins.GetTurnBearing()))
	}

	return drivingDirections
}

func (db *DirectionBuilder) buildInstruction(edge da.OutEdge) {
	headId := edge.GetHead()
	tailId := db.graph.GetTailOfOutedge(edge.GetEdgeId())

	tail := db.graph.GetVertex(tailId)
	head := db.graph.GetVertex(headId)

	isRoundabout := db.graph.IsRoundabout(edge.GetEdgeId())

	var prevNode da.Vertex
	if db.prevNode != math.MaxUint32 {
		prevNode = *db.graph.GetVertex(db.prevNode)
	}

	streetName := db.graph.GetStreetName(edge.GetEdgeId())
	if db.prevInstruction == nil && !isRoundabout {
		// start point dari shortetest path & bukan bundaran (roundabout)
		sign := da.START
		point := da.NewCoordinate(
			tail.GetLat(), tail.GetLon(),
		)
		turnBearing := computeInitialBearing(tail.GetLat(), tail.GetLon(),
			head.GetLat(), head.GetLon())
		newIns := da.NewInstruction(sign, streetName, point, false,
			[]da.Index{edge.GetEdgeId()}, db.cumulativeDistance, db.cumulativeTravelTime,
			db.points, turnBearing, db.clockwise)
		db.prevInstruction = newIns

		db.points = db.coordinatesPool.Get().([]da.Coordinate)
		db.points = db.points[:0]

		db.edgeIds = db.drivingEdgeIdsPool.Get().([]da.Index)
		db.edgeIds = db.edgeIds[:0]

		db.prevInstruction.SetExtraInfo("heading", turnBearing)
		db.instructions = append(db.instructions, db.prevInstruction)
	} else if isRoundabout {
		// current edge bundaran
		if !db.prevInRoundabout {
			sign := da.USE_ROUNDABOUT
			point := da.NewCoordinate(tail.GetLat(), tail.GetLon())
			roundaboutInstruction := da.NewRoundaboutInstruction()

			db.doublePrevInitialBearing = db.prevInitialBearing
			if db.prevInstruction != nil {
				db.prevInitialBearing = computeInitialBearing(prevNode.GetLat(), prevNode.GetLon(), tail.GetLat(), tail.GetLon())

			} else {
				// start point dari shortetest path & dan bundaran (roundabout)
				db.prevInitialBearing = computeInitialBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
			}

			turnBearing := computeFinalBearing(prevNode.GetLat(), prevNode.GetLon(), tail.GetLat(), tail.GetLon())
			prevIns := da.NewInstructionWithRoundabout(sign, streetName, point, true, roundaboutInstruction, db.cumulativeDistance,
				db.cumulativeTravelTime, db.edgeIds, turnBearing)
			db.prevInstruction = &prevIns

			// reset edgeIDs and points
			db.reset()

			db.instructions = append(db.instructions, db.prevInstruction)
		}

		db.graph.ForOutEdgesOfWithId(headId, func(e *da.OutEdge, id da.Index) {
			if da.SkipDummyEdge(e) {
				return
			}
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
		if turnSign != da.IGNORE {
			uTurn, uturnType := db.checkUTurn(turnSign, streetName, edge)
			if uTurn {
				db.prevInstruction.SetSign(uturnType)
				db.prevInstruction.SetStreetName(streetName)
			} else {
				// bukan U-turn -> continue/right/left
				tail := da.NewCoordinate(tail.GetLat(), tail.GetLon())
				turnBearing := computeFinalBearing(prevNode.GetLat(), prevNode.GetLon(),
					tail.GetLat(), tail.GetLon())
				prevIns := da.NewInstruction(turnSign, streetName, tail, false, db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime,
					db.points, turnBearing, db.clockwise)
				db.prevInstruction = prevIns

				db.reset()

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
		da.NewCoordinate(tail.GetLat(), tail.GetLon()))
	db.points = append(db.points,
		da.NewCoordinate(head.GetLat(), head.GetLon()))

	isHeadTrafficLight := db.graph.IsTrafficLight(headId)
	if isHeadTrafficLight {
		db.cumulativeTravelTime += util.SecondsToMinutes(pkg.TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND)
	}
}

func (db *DirectionBuilder) buildFinalInstruction() {

	doublePrevNode := db.graph.GetVertex(db.doublePrevNode)

	tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(db.prevEdge.GetEdgeId()))

	node := db.graph.GetVertex(db.prevEdge.GetHead())
	point := da.NewCoordinate(node.GetLat(), node.GetLon())

	prevNodeData := db.graph.GetVertex(db.prevNode)
	turnBearing := computeFinalBearing(prevNodeData.GetLat(), prevNodeData.GetLon(), tail.GetLat(), tail.GetLon())

	finishInstruction := da.NewInstruction(da.FINISH, db.graph.GetStreetName(db.prevEdge.GetEdgeId()), point, false,
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
func (db *DirectionBuilder) checkUTurn(sign int, name string, edge da.OutEdge) (bool, int) {
	isUTurn := false
	uTurnType := da.U_TURN_UNKNOWN

	if db.doublePrevInitialBearing != 0 && (sign > 0) == (db.prevInstruction.GetTurnSign() > 0) &&
		((db.lefthand && (util.Abs(sign) == da.TURN_SLIGHT_RIGHT || util.Abs(sign) == da.TURN_RIGHT || util.Abs(sign) == da.TURN_SHARP_RIGHT)) ||
			(!db.lefthand && (util.Abs(sign) == da.TURN_SLIGHT_LEFT || util.Abs(sign) == da.TURN_LEFT || util.Abs(sign) == da.TURN_SHARP_LEFT))) &&
		((db.lefthand && util.Abs(db.prevInstruction.GetTurnSign()) == da.TURN_SLIGHT_RIGHT || util.Abs(db.prevInstruction.GetTurnSign()) == da.TURN_RIGHT || util.Abs(db.prevInstruction.GetTurnSign()) == da.TURN_SHARP_RIGHT) ||
			(!db.lefthand && util.Abs(db.prevInstruction.GetTurnSign()) == da.TURN_SLIGHT_LEFT || util.Abs(db.prevInstruction.GetTurnSign()) == da.TURN_LEFT || util.Abs(db.prevInstruction.GetTurnSign()) == da.TURN_SHARP_LEFT)) &&
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
				uTurnType = da.U_TURN_LEFT
			} else {
				uTurnType = da.U_TURN_RIGHT
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
func (db *DirectionBuilder) getTurnSign(edge da.OutEdge, tailId, prevNodeId, headId da.Index, name string) int {

	key := util.Bitpack(uint32(db.prevEdge.GetEdgeId()), uint32(edge.GetEdgeId()))

	if sign, ok := db.turnSignCache.Get(key); ok {
		return sign
	}

	tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(edge.GetEdgeId()))
	point := db.graph.GetVertex(edge.GetHead())
	lat := point.GetLat()
	lon := point.GetLon()

	prevNode := db.graph.GetVertex(db.prevNode)

	db.prevInitialBearing = computeInitialBearing(prevNode.GetLat(), prevNode.GetLon(),
		tail.GetLat(), tail.GetLon())

	sign := getTurnDirection(tail.GetLat(), tail.GetLon(), lat, lon, db.prevInitialBearing)

	alternativeTurnsCount, alternativeTurns := db.GetAlternativeTurns(tailId, headId, prevNodeId)

	currStreetName := db.graph.GetStreetName(edge.GetEdgeId())
	currRoadClass := db.graph.GetRoadClass(edge.GetEdgeId())

	prevStreetName := db.graph.GetStreetName(db.prevEdge.GetEdgeId())
	prevRoadClass := db.graph.GetRoadClass(db.prevEdge.GetEdgeId())

	isStreetSplit := db.isStreetSplit(edge, db.prevEdge, currStreetName, prevStreetName, prevRoadClass, currRoadClass)

	isStreetMerged := db.isStreetMerged(edge, db.prevEdge, currStreetName, prevStreetName, prevRoadClass, currRoadClass)

	if alternativeTurnsCount == 1 {
		// there are only one alternative turns (other than this current edge)
		if math.Abs(float64(sign)) > 1 && !(isStreetMerged || isStreetSplit) {
			// there are only one alternative turns (other than this current edge) and sign is not CONTINUE/TURN_SLIGHT_*
			db.turnSignCache.Add(key, sign)
			return sign
		}
		db.turnSignCache.Add(key, da.IGNORE)
		return da.IGNORE
	}

	prevEdgeStreetName := db.graph.GetStreetName(db.prevEdge.GetEdgeId())
	if math.Abs(float64(sign)) > 1 {
		if (isSameName(name, prevEdgeStreetName)) ||
			isStreetMerged || isStreetSplit {
			db.turnSignCache.Add(key, da.IGNORE)
			return da.IGNORE
		}
		// sign is not CONTINUE/TURN_SLIGHT_*  & street name changed from prev edge to curr edge & not split/merged street, so output sign
		db.turnSignCache.Add(key, sign)
		return sign
	}

	if db.prevEdge.GetWeight() == 0 {
		db.turnSignCache.Add(key, sign)
		return sign
	}

	// get another edge from the tail that have CONTINUE direction
	otherContinueEdge := db.getOtherEdgeContinueDirection(tail.GetLat(), tail.GetLon(), db.prevInitialBearing, alternativeTurns)

	prevCurrEdgeInitialBearingDiff := computeDeltaBearing(tail.GetLat(), tail.GetLon(), lat, lon, db.prevInitialBearing) // bearing difference antara prevNode->tail->head
	if otherContinueEdge.GetEdgeId() != da.INVALID_EDGE_ID {
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
					db.turnSignCache.Add(key, da.IGNORE)
					return da.IGNORE
				}
			}

			prevOtherEdgeInitialBearing := computeDeltaBearing(tail.GetLat(), tail.GetLon(), tmpLat, tmpLon, db.prevInitialBearing) // bearing difference antara prevNode->tail->otherContinueEdge.GetHead()

			if util.RadiansToDegree(math.Abs(prevCurrEdgeInitialBearingDiff)) < 6 && util.RadiansToDegree(math.Abs(prevOtherEdgeInitialBearing)) > 8.6 && isSameName(name, prevEdgeStreetName) {
				// bearing difference antara prevEdge dan currentEDge < 6° (CONTINUE Direction), Edge otherContinueEdge > 8.6 (TURN SLIGHT or more direction). Nama prevEdge street == current Street
				db.turnSignCache.Add(key, da.CONTINUE_ON_STREET)
				return da.CONTINUE_ON_STREET
			}

			if roadClass == "residential" || prevRoadClass == "residential" || (roadClass == "unclassified" && prevRoadClass == "unclassified") {
				// skip roadclass residential untuk mengurangi instructions.
				db.turnSignCache.Add(key, da.IGNORE)
				return da.IGNORE
			}

			/*
				jika dari tail ada 2 jalan yang arahnya sama sama lurus/sedikit belok, tambah turn instruction ke currEdge. Example:

						-----currentEdge---------
				tail
						-----otherContinueEdge---
				pada case diatas, output keep left karena prevCurrEdgeInitialBearingDiff <= prevOtherEdgeInitialBearing
			*/ // nolint: gofmt
			if prevCurrEdgeInitialBearingDiff > prevOtherEdgeInitialBearing {
				db.turnSignCache.Add(key, da.KEEP_RIGHT)

				return da.KEEP_RIGHT
			} else {
				db.turnSignCache.Add(key, da.KEEP_LEFT)
				return da.KEEP_LEFT
			}
		}
	}

	if ok := db.isLeavingCurrentStreet(prevEdgeStreetName, name, db.prevEdge, edge); !(isStreetMerged || isStreetSplit) &&
		(util.RadiansToDegree(math.Abs(prevCurrEdgeInitialBearingDiff)) > 34 || ok) {
		// current streetname != prev streetname & not street split & not street merged, so output sign other than CONTINUE_ON_STREET
		db.turnSignCache.Add(key, sign)
		return sign
	}

	db.turnSignCache.Add(key, da.IGNORE)
	return da.IGNORE
}

func isMajorRoad(roadClass string) bool {
	return roadClass == "motorway" || roadClass == "trunk" || roadClass == "primary" || roadClass == "secondary" || roadClass == "tertiary"
}

package guidance

import (
	"math"

	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://wiki.openstreetmap.org/wiki/Sample_driving_instructions/template_TEMPLATE
// https://wiki.openstreetmap.org/wiki/Key:turn
// https://wiki.openstreetmap.org/wiki/Lanes
// https://wiki.openstreetmap.org/wiki/Key:destination
// https://wiki.openstreetmap.org/wiki/Highway_link

type DirectionBuilder struct {
	turnSignCache *ristretto.Cache[uint64, []byte]

	instructions     []*da.Instruction
	prevInstruction  *da.Instruction
	turnDescriptions []string

	points         []da.Coordinate
	edgeIds        []da.Index
	path           []da.Index
	lastPathId     int
	nextStreetName string

	graph                    Graph
	prevEdge                 da.Index
	doublePrevStreetName     string
	prevInitialBearing       float64
	doublePrevInitialBearing float64
	cumulativeDistance       float64
	cumulativeTravelTime     float64
	doublePrevNode           da.Index
	prevNode                 da.Index
	prevPoint                da.Coordinate
	doublePrevPoint          da.Coordinate
	clockwise                bool // clockwise roundabout (like in indonesia) or counter-clockwise roundabout
	lefthand                 bool // left hand traffic (like in indonesia) or right hand traffic
	prevInRoundabout         bool
}

func NewDirectionBuilder(graph Graph, clockwise, lefthand bool,
	turnSignCache *ristretto.Cache[uint64, []byte]) *DirectionBuilder {
	db := &DirectionBuilder{
		graph:                    graph,
		prevNode:                 math.MaxUint32,
		prevInRoundabout:         false,
		doublePrevInitialBearing: 0,
		clockwise:                clockwise,
		lefthand:                 lefthand,
		prevPoint:                da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON),
		prevEdge:                 da.Index(da.INVALID_EDGE_ID),
		turnDescriptions:         make([]string, 256),
		points:                   make([]da.Coordinate, 0),
		instructions:             make([]*da.Instruction, 0),
		edgeIds:                  make([]da.Index, 0),
		lastPathId:               0,
		turnSignCache:            turnSignCache,
	}

	return db
}

func (db *DirectionBuilder) reset() {
	db.points = db.points[:0]
	db.edgeIds = db.edgeIds[:0]
}

func (db *DirectionBuilder) done() {
	db.instructions = db.instructions[:0]
	db.points = db.points[:0]
	db.edgeIds = db.edgeIds[:0]
}

func (db *DirectionBuilder) Reset() {

	db.prevNode = math.MaxUint32
	db.prevInRoundabout = false
	db.prevInstruction = nil
	db.prevEdge = da.Index(da.INVALID_EDGE_ID)
	db.prevPoint = da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON)
	db.doublePrevPoint = da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON)
	db.doublePrevStreetName = ""
	db.prevInitialBearing = 0
	db.doublePrevInitialBearing = 0
	db.cumulativeDistance = 0
	db.cumulativeTravelTime = 0
	db.doublePrevNode = 0
	db.lastPathId = 0
	db.nextStreetName = ""
	db.path = make([]da.Index, 0)
}

func (db *DirectionBuilder) GetDrivingDirections(path []da.Index) []da.DrivingDirection {
	defer db.done()

	if len(path) == 0 {
		return make([]da.DrivingDirection, 0)
	}

	db.path = path

	for db.lastPathId < len(db.path) {
		edgeId := db.path[db.lastPathId]

		db.buildInstruction(edgeId)
		db.lastPathId++
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

	drivingDirections := make([]da.DrivingDirection, 0)
	for i, ins := range db.instructions {
		var (
			currStepTravelTime, currStepDistance float64
		)
		if i > 0 {
			currStepTravelTime = db.instructions[i].GetCumulativeTravelTime() - db.instructions[i-1].GetCumulativeTravelTime()
			currStepDistance = db.instructions[i].GetCumulativeDistance() - db.instructions[i-1].GetCumulativeDistance()
		}
		way := *db.instructions[i]
		currPolyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(way.GetPoints()))
		drivingDirections = append(drivingDirections, da.NewDrivingDirection(way, db.turnDescriptions[i],
			currStepTravelTime, currStepDistance, way.GetEdgeIds(), currPolyline, ins.GetTurnBearing()))
	}

	return drivingDirections
}

func (db *DirectionBuilder) buildInstruction(edgeId da.Index) {

	headId := db.graph.GetHeadOfOutEdge(edgeId)

	tailId := db.graph.GetTailOfOutedge(edgeId)

	tail := db.graph.GetVertex(tailId)
	head := db.graph.GetVertex(headId)

	isRoundabout := db.graph.IsRoundabout(edgeId)

	var prevPoint da.Coordinate
	if db.prevPoint.GetLat() != pkg.INVALID_LAT {
		prevPoint = db.prevPoint
	}

	streetName := db.graph.GetStreetName(edgeId)
	if db.prevInstruction == nil && !isRoundabout {
		// start point dari shortetest path & bukan bundaran (roundabout)
		sign := da.START
		point := da.NewCoordinate(
			tail.GetLat(), tail.GetLon(),
		)
		turnBearing := computeInitialBearing(tail.GetLat(), tail.GetLon(),
			head.GetLat(), head.GetLon())
		newIns := da.NewInstruction(sign, streetName, point, false,
			[]da.Index{edgeId}, db.cumulativeDistance, db.cumulativeTravelTime,
			db.points, turnBearing, db.clockwise)
		db.prevInstruction = newIns

		db.points = make([]da.Coordinate, 0)
		db.edgeIds = make([]da.Index, 0)

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
				db.prevInitialBearing = computeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())

			} else {
				// start point dari shortetest path & dan bundaran (roundabout)
				db.prevInitialBearing = computeInitialBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
			}

			turnBearing := computeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())
			prevIns := da.NewInstructionWithRoundabout(sign, streetName, point, true, roundaboutInstruction, db.cumulativeDistance,
				db.cumulativeTravelTime, db.edgeIds, turnBearing)
			db.prevInstruction = &prevIns

			// reset edgeIDs and points
			db.reset()

			db.instructions = append(db.instructions, db.prevInstruction)
		}

		db.graph.ForOutEdgeIdsOf(headId, func(eId da.Index) {

			eIsRoundabout := db.graph.IsRoundabout(eId)
			if !eIsRoundabout {
				roundaboutInstruction := db.prevInstruction
				roundaboutInstruction.IncrementExitNumber()
			}
		})
	} else if db.prevInRoundabout {
		db.prevInstruction.SetStreetName(streetName)
		roundaboutInstruction := db.prevInstruction
		roundaboutInstruction.SetExited()
		db.doublePrevStreetName = db.graph.GetStreetName(db.prevEdge)
	} else {
		turnSign := db.getTurnSign(edgeId, tailId, db.prevNode, headId, streetName)
		if turnSign != da.IGNORE {
			uTurn, uturnType := db.checkUTurn(turnSign, streetName, edgeId)
			if uTurn {
				db.prevInstruction.SetSign(uturnType)
				db.prevInstruction.SetStreetName(streetName)
			} else {
				// bukan U-turn -> continue/right/left
				tail := da.NewCoordinate(tail.GetLat(), tail.GetLon())
				turnBearing := computeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(),
					tail.GetLat(), tail.GetLon())
				prevIns := da.NewInstruction(turnSign, db.nextStreetName, tail, false, db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime,
					db.points, turnBearing, db.clockwise)
				db.prevInstruction = prevIns

				db.reset()

				db.doublePrevInitialBearing = db.prevInitialBearing
				db.doublePrevStreetName = db.graph.GetStreetName(db.prevEdge)
				db.instructions = append(db.instructions, prevIns)
			}
		}
	}

	db.doublePrevPoint = db.prevPoint
	eGeom := db.graph.GetEdgeGeometry(edgeId)
	n := len(eGeom)
	db.prevPoint = db.GetPrevPoint(eGeom, eGeom[n-1], 25)

	db.doublePrevNode = db.prevNode
	db.prevInRoundabout = isRoundabout
	db.prevNode = tailId
	db.prevEdge = edgeId

	db.cumulativeDistance += db.graph.GetOutEdgeLength(edgeId)
	db.cumulativeTravelTime += db.graph.GetOutEdgeWeight(edgeId) // todo: ganti metrics.GetWeight()
	db.edgeIds = append(db.edgeIds, edgeId)
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

	tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(db.prevEdge))

	prevEdgeHead := db.graph.GetHeadOfOutEdge(db.prevEdge)
	node := db.graph.GetVertex(prevEdgeHead)
	point := da.NewCoordinate(node.GetLat(), node.GetLon())

	prevNodeData := db.graph.GetVertex(db.prevNode)
	turnBearing := computeFinalBearing(prevNodeData.GetLat(), prevNodeData.GetLon(), tail.GetLat(), tail.GetLon())

	finishInstruction := da.NewInstruction(da.FINISH, db.graph.GetStreetName(db.prevEdge), point, false,
		db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime, db.points, turnBearing, db.clockwise)
	finishInstruction.SetExtraInfo("heading", geo.BearingTo(doublePrevNode.GetLat(), doublePrevNode.GetLon(), tail.GetLat(), tail.GetLon()))

	db.instructions = append(db.instructions, finishInstruction)
}

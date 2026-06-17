package guidance

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/maypok86/otter/v2"
)

// https://wiki.openstreetmap.org/wiki/Sample_driving_instructions/template_TEMPLATE
// https://wiki.openstreetmap.org/wiki/Key:turn
// https://wiki.openstreetmap.org/wiki/Lanes
// https://wiki.openstreetmap.org/wiki/Key:destination
// https://wiki.openstreetmap.org/wiki/Highway_link

type DirectionBuilder struct {
	turnSignCache *otter.Cache[uint64, uint64] // cache untuk turn sign dari (prevEdgeId, currEdgeId) untuk di jalan Nasional, Jalan provinsi, dan Jalan kabupaten. Uses uint64 value to avoid the per-call byte-slice allocation of the previous []byte encoding

	instructions      []da.Instruction
	prevInstruction   int
	turnDescriptions  []string
	drivingDirections []da.DrivingDirection

	edgeIds          []da.Index
	path             []da.Index
	geometry         da.Coordinates
	alternativeTurns []da.Index

	lastPathId     int
	nextStreetName uint32 // streetName Id

	engine   RoutingEngine
	graph    Graph
	prevEdge da.Index

	doublePrevStreetName     string
	prevInitialBearing       float64
	doublePrevInitialBearing float64
	cumulativeDistance       float64
	cumulativeTravelTime     float64
	doublePrevNode           da.Index
	prevNode                 da.Index
	prevPoint                da.Coordinate
	doublePrevPoint          da.Coordinate
	prevSign                 da.TurnType
	clockwise                bool // clockwise roundabout (like in indonesia) or counter-clockwise roundabout
	lefthand                 bool // left hand traffic (like in indonesia) or right hand traffic
	prevInRoundabout         bool
	useLookForward           bool
	lookForwardStep          int

	// reroute
	reroute       bool
	startEdgeId   da.Index
	useAnnotation bool
}

func NewDirectionBuilder(engine RoutingEngine, graph Graph, lefthand bool,
	turnSignCache *otter.Cache[uint64, uint64]) *DirectionBuilder {

	clockwise := lefthand

	db := &DirectionBuilder{
		engine:                   engine,
		graph:                    graph,
		prevNode:                 math.MaxUint32,
		prevInRoundabout:         false,
		doublePrevInitialBearing: 0,
		clockwise:                clockwise,
		lefthand:                 lefthand,
		prevPoint:                da.NewCoordinate(pkg.INVALID_LAT, pkg.INVALID_LON),
		prevEdge:                 da.Index(da.INVALID_EDGE_ID),
		turnDescriptions:         make([]string, 0, 16),
		drivingDirections:        make([]da.DrivingDirection, 0, 16),
		instructions:             make([]da.Instruction, 0, 16),
		prevInstruction:          -1,
		edgeIds:                  make([]da.Index, 0, 16),
		geometry:                 da.Coordinates{},
		alternativeTurns:         make([]da.Index, 0),
		lastPathId:               0,
		turnSignCache:            turnSignCache,
		prevSign:                 da.IGNORE,
	}

	return db
}

// reset releases annotation-owned slices and otherwise reuses builder capacity.
func (db *DirectionBuilder) reset() {
	db.edgeIds = db.edgeIds[:0]
	db.geometry = db.geometry[:0]
}

// done clears request-owned references before the builder returns to its pool.
func (db *DirectionBuilder) done() {
	db.instructions = db.instructions[:0]
	db.turnDescriptions = db.turnDescriptions[:0]
	db.drivingDirections = db.drivingDirections[:0]
	db.edgeIds = db.edgeIds[:0]
	db.geometry = db.geometry[:0]
	db.prevInstruction = -1
	db.alternativeTurns = db.alternativeTurns[:0]
}

func (db *DirectionBuilder) Reset() {

	db.prevNode = math.MaxUint32
	db.prevInRoundabout = false
	db.prevInstruction = -1
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
	db.nextStreetName = da.INVALID_STREET_NAME_ID
	db.path = db.path[:0]
	db.prevSign = da.IGNORE
	db.useLookForward = false
	db.lookForwardStep = 0
	db.startEdgeId = da.INVALID_EDGE_ID
	db.reroute = false
	db.useAnnotation = false
}

func (db *DirectionBuilder) SetReroute(startEdgeId da.Index) {
	db.reroute = true
	db.startEdgeId = startEdgeId
}

// GetDrivingDirections. generate driving directions dairi path (list of edgeIds)
// worst case: O(n*q + n*l), n = number of edgeIds in path arr, q=max outDegree of any vertex in the graph, l=maximum length of any edge geometry
func (db *DirectionBuilder) GetDrivingDirections(
	path []da.Index,
	sp, tp da.PhantomNode,
	useAnnotation bool,
) []da.DrivingDirection {
	defer db.done()
	db.useAnnotation = useAnnotation

	if len(path) == 0 {
		return nil
	}

	if !db.graph.IsDummyOutEdge(sp.GetOutEdgeId()) && !db.reroute {
		db.path = append(db.path, sp.GetOutEdgeId())
	} else if db.reroute {
		db.path = append(db.path, db.startEdgeId)
	}
	db.path = append(db.path, path...)

	if !db.graph.IsDummyOutEdge(tp.GetOutEdgeId()) {
		db.path = append(db.path, tp.GetOutEdgeId())
	}

	m := len(db.path)
	for db.lastPathId < m {
		edgeId := db.path[db.lastPathId]

		db.buildInstruction(edgeId, sp)
		db.lastPathId++
		db.useLookForward = false
		db.lookForwardStep = 0
	}

	if !db.graph.IsDummyOutEdge(tp.GetOutEdgeId()) {
		lastEdgeId := tp.GetOutEdgeId()
		db.buildFinalInstruction(lastEdgeId, tp)
	} else {
		db.buildFinalInstruction(db.path[m-1], tp)
	}

	db.turnDescriptions = db.turnDescriptions[:0]

	for i := range db.instructions {
		desc := db.instructions[i].GetTurnDescription(db.clockwise)
		db.turnDescriptions = append(db.turnDescriptions, desc)
	}

	db.drivingDirections = db.drivingDirections[:0]

	for i := range db.instructions {
		var (
			currStepTravelTime, currStepDistance float64
		)
		if i > 0 {
			currStepTravelTime = db.instructions[i].GetCumulativeTravelTime() - db.instructions[i-1].GetCumulativeTravelTime()
			currStepDistance = db.instructions[i].GetCumulativeDistance() - db.instructions[i-1].GetCumulativeDistance()
		}

		db.drivingDirections = append(db.drivingDirections, da.NewDrivingDirection(db.instructions[i], db.turnDescriptions[i],
			currStepTravelTime, currStepDistance, db.instructions[i].GetEdgeIds(), db.instructions[i].GetTurnBearing(), db.instructions[i].GetAnnotation()))
	}

	return db.drivingDirections
}

func (db *DirectionBuilder) buildInstruction(edgeId da.Index, sp da.PhantomNode) {

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
	if db.prevInstruction < 0 && !isRoundabout {
		// start point dari shortetest path & bukan bundaran (roundabout) & bukan reroute
		sign := da.START
		tailCoord := sp.GetSnappedCoord()
		point := da.NewCoordinate(
			tailCoord.GetLat(), tailCoord.GetLon(),
		)

		turnBearing := geo.ComputeInitialBearing(tailCoord.GetLat(), tailCoord.GetLon(),
			head.GetLat(), head.GetLon())

		db.updateState(edgeId, false)
		ann := db.annotation()

		var edgeIds []da.Index
		if db.useAnnotation {
			edgeIds = []da.Index{edgeId}
		}
		newIns := da.NewInstruction(sign, streetName, point, false,
			edgeIds, db.cumulativeDistance, db.cumulativeTravelTime,
			turnBearing, ann, db.clockwise)

		newIns.SetHeading(turnBearing)
		db.instructions = append(db.instructions, newIns)
		db.prevInstruction = len(db.instructions) - 1

		db.reset()
	} else if isRoundabout {
		// current edge bundaran
		if !db.prevInRoundabout {
			sign := da.USE_ROUNDABOUT
			point := da.NewCoordinate(tail.GetLat(), tail.GetLon())
			roundaboutInstruction := da.NewRoundaboutInstruction()

			db.doublePrevInitialBearing = db.prevInitialBearing
			if db.prevInstruction >= 0 {
				db.prevInitialBearing = geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())

			} else {
				// start point dari shortetest path & dan bundaran (roundabout)
				db.prevInitialBearing = geo.ComputeInitialBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
			}

			turnBearing := geo.ComputeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())
			ann := db.annotation()
			prevIns := da.NewInstructionWithRoundabout(sign, streetName, point, true, roundaboutInstruction, db.cumulativeDistance,
				db.cumulativeTravelTime, db.edgeIds, ann, turnBearing)
			db.instructions = append(db.instructions, prevIns)
			db.prevInstruction = len(db.instructions) - 1

			// reset edgeIDs and points
			db.reset()
		}

		db.graph.ForOutEdgeIdsOf(headId, func(eId da.Index) {

			eIsRoundabout := db.graph.IsRoundabout(eId)
			if !eIsRoundabout {
				db.previousInstruction().IncrementExitNumber()
			}
		})
	} else if db.prevInRoundabout {
		db.previousInstruction().SetStreetName(streetName)
		roundaboutInstruction := db.previousInstruction()
		roundaboutInstruction.SetExited()
		db.doublePrevStreetName = db.graph.GetStreetName(db.prevEdge)
	} else {
		turnSign := db.getTurnSign(edgeId, tailId, db.prevNode, headId, streetName)

		if turnSign != da.IGNORE {
			uTurn, uturnType := db.checkUTurn(turnSign, streetName, edgeId)
			if uTurn {
				db.previousInstruction().SetSign(uturnType)
				db.previousInstruction().SetStreetName(streetName)
			} else {
				// bukan U-turn -> continue/right/left
				tailCoord := tail.GetCoordinate()
				turnBearing := geo.ComputeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(),
					tail.GetLat(), tail.GetLon())
				nextStreetName := db.graph.GetStrFromId(db.nextStreetName)
				suggestAlternatives := db.IsSuggestAlternatives(edgeId)
				ann := db.annotation()
				ins := da.NewInstruction(turnSign, nextStreetName, tailCoord, false, db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime,
					turnBearing, ann, db.clockwise)
				ins.SetSuggestAlternatives(suggestAlternatives)

				db.reset()

				db.instructions = append(db.instructions, ins)
				db.prevInstruction = len(db.instructions) - 1
			}
		}
		db.prevSign = turnSign
	}

	if !db.useLookForward {
		db.updateState(edgeId, isRoundabout)
	}

}

func (db *DirectionBuilder) buildFinalInstruction(edgeId da.Index, tp da.PhantomNode) {

	doublePrevNode := db.graph.GetVertex(db.doublePrevNode)
	tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(edgeId))

	var head da.Coordinate
	if !db.graph.IsDummyOutEdge(tp.GetOutEdgeId()) {
		head = tp.GetSnappedCoord()
	} else {
		headId := db.graph.GetHeadOfOutEdge(edgeId)
		head = db.graph.GetVertexCoordinate(headId)
	}

	point := da.NewCoordinate(head.GetLat(), head.GetLon())

	turnBearing := geo.ComputeFinalBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
	ann := db.annotation()

	finishInstruction := da.NewInstruction(da.FINISH, db.graph.GetStreetName(edgeId), point, false,
		db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime, turnBearing, ann, db.clockwise)
	finishInstruction.SetHeading(geo.BearingTo(doublePrevNode.GetLat(), doublePrevNode.GetLon(), tail.GetLat(), tail.GetLon()))

	db.instructions = append(db.instructions, finishInstruction)
}

// previousInstruction returns the previous instruction
func (db *DirectionBuilder) previousInstruction() *da.Instruction {
	return &db.instructions[db.prevInstruction]
}

// annotation avoids constructing annotation data when the request did not ask for it.
func (db *DirectionBuilder) annotation() da.Annotation {
	if !db.useAnnotation {
		return da.Annotation{}
	}
	return db.buildSimplifiedAnnotation(db.edgeIds, db.geometry)
}

package guidance

import (
	"math"

	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

// https://wiki.openstreetmap.org/wiki/Sample_driving_instructions/template_TEMPLATE
// https://wiki.openstreetmap.org/wiki/Key:turn
// https://wiki.openstreetmap.org/wiki/Lanes
// https://wiki.openstreetmap.org/wiki/Key:destination
// https://wiki.openstreetmap.org/wiki/Highway_link

type DirectionBuilder struct {
	turnSignCache *ristretto.Cache[uint64, []byte] // cache untuk turn sign dari (prevEdgeId, currEdgeId) untuk di jalan Nasional, Jalan provinsi, dan Jalan kabupaten

	instructions     []*da.Instruction
	prevInstruction  *da.Instruction
	turnDescriptions []string

	edgeIds        []da.Index
	edgeGeomOffset []da.Index
	path           []da.Index
	geometry       da.Coordinates
	distance       []float64
	duration       []float64

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
	reroute     bool
	startEdgeId da.Index
}

func NewDirectionBuilder(engine RoutingEngine, graph Graph, lefthand bool,
	turnSignCache *ristretto.Cache[uint64, []byte]) *DirectionBuilder {

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
		turnDescriptions:         make([]string, 256),
		instructions:             make([]*da.Instruction, 0),
		edgeIds:                  make([]da.Index, 0),
		lastPathId:               0,
		turnSignCache:            turnSignCache,
		prevSign:                 da.IGNORE,
	}

	return db
}

func (db *DirectionBuilder) reset() {
	db.edgeIds = db.edgeIds[:0]
	db.geometry = db.geometry[:0]
	db.distance = db.distance[:0]
	db.duration = db.duration[:0]
	db.edgeGeomOffset = db.edgeGeomOffset[:0]
}

func (db *DirectionBuilder) done() {
	db.instructions = db.instructions[:0]
	db.edgeIds = db.edgeIds[:0]
	db.geometry = db.geometry[:0]
	db.distance = db.distance[:0]
	db.duration = db.duration[:0]
	db.edgeGeomOffset = db.edgeGeomOffset[:0]
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
	db.nextStreetName = da.INVALID_STREET_NAME_ID
	db.path = make([]da.Index, 0)
	db.prevSign = da.IGNORE
	db.useLookForward = false
	db.lookForwardStep = 0
	db.startEdgeId = da.INVALID_EDGE_ID
	db.reroute = false
}

func (db *DirectionBuilder) SetReroute(startEdgeId da.Index) {
	db.reroute = true
	db.startEdgeId = startEdgeId
}

// GetDrivingDirections. generate driving directions dairi path (list of edgeIds)
// worst case: O(n*q + n*l), n = number of edgeIds in path arr, q=max outDegree of any vertex in the graph, l=maximum length of any edge geometry
func (db *DirectionBuilder) GetDrivingDirections(path []da.Index, sp, tp da.PhantomNode) []da.DrivingDirection {
	defer db.done()

	if len(path) == 0 {
		return make([]da.DrivingDirection, 0)
	}

	if !db.graph.IsDummyOutEdge(sp.GetOutEdgeId()) && !db.reroute {
		firstEdgeId := sp.GetOutEdgeId()
		path = append([]da.Index{firstEdgeId}, path...)
	} else if db.reroute {
		firstEdgeId := db.startEdgeId
		path = append([]da.Index{firstEdgeId}, path...)
	}

	if !db.graph.IsDummyOutEdge(tp.GetOutEdgeId()) {
		lastEdgeId := tp.GetOutEdgeId()
		path = append(path, lastEdgeId)
	}

	db.path = path

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

		currPolyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(ins.GetPoints()))
		drivingDirections = append(drivingDirections, da.NewDrivingDirection(*ins, db.turnDescriptions[i],
			currStepTravelTime, currStepDistance, ins.GetEdgeIds(), currPolyline, ins.GetTurnBearing(), ins.GetAnnotation()))
	}

	return drivingDirections
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
	if db.prevInstruction == nil && !isRoundabout {
		// start point dari shortetest path & bukan bundaran (roundabout) & bukan reroute
		sign := da.START
		tailCoord := sp.GetSnappedCoord()
		point := da.NewCoordinate(
			tailCoord.GetLat(), tailCoord.GetLon(),
		)

		turnBearing := geo.ComputeInitialBearing(tailCoord.GetLat(), tailCoord.GetLon(),
			head.GetLat(), head.GetLon())

		db.updateState(edgeId, false)

		newIns := da.NewInstruction(sign, streetName, point, false,
			[]da.Index{edgeId}, db.cumulativeDistance, db.cumulativeTravelTime,
			db.GetEdgePoints(edgeId), turnBearing, db.duration, db.distance, db.geometry, db.edgeGeomOffset, db.clockwise)

		db.prevInstruction = newIns

		db.edgeIds = make([]da.Index, 0)

		db.prevInstruction.SetExtraInfo("heading", turnBearing)
		db.instructions = append(db.instructions, db.prevInstruction)

		db.reset()
	} else if isRoundabout {
		// current edge bundaran
		if !db.prevInRoundabout {
			sign := da.USE_ROUNDABOUT
			point := da.NewCoordinate(tail.GetLat(), tail.GetLon())
			roundaboutInstruction := da.NewRoundaboutInstruction()

			db.doublePrevInitialBearing = db.prevInitialBearing
			if db.prevInstruction != nil {
				db.prevInitialBearing = geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())

			} else {
				// start point dari shortetest path & dan bundaran (roundabout)
				db.prevInitialBearing = geo.ComputeInitialBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
			}

			turnBearing := geo.ComputeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())
			prevIns := da.NewInstructionWithRoundabout(sign, streetName, point, true, roundaboutInstruction, db.cumulativeDistance,
				db.cumulativeTravelTime, db.edgeIds, db.duration, db.distance, db.geometry, db.edgeGeomOffset, turnBearing)
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
				tailCoord := tail.GetCoordinate()
				turnBearing := geo.ComputeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(),
					tail.GetLat(), tail.GetLon())
				nextStreetName := db.graph.GetStrFromId(db.nextStreetName)
				suggestAlternatives := db.IsSuggestAlternatives(edgeId)
				ins := da.NewInstruction(turnSign, nextStreetName, tailCoord, false, db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime,
					db.GetEdgePoints(edgeId), turnBearing, db.duration, db.distance, db.geometry, db.edgeGeomOffset, db.clockwise)
				ins.SetSuggestAlternatives(suggestAlternatives)

				db.prevInstruction = ins

				db.reset()

				db.instructions = append(db.instructions, ins)
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

	finishInstruction := da.NewInstruction(da.FINISH, db.graph.GetStreetName(edgeId), point, false,
		db.edgeIds, db.cumulativeDistance, db.cumulativeTravelTime, db.GetEdgePoints(edgeId), turnBearing, db.duration, db.distance, db.geometry, db.edgeGeomOffset, db.clockwise)
	finishInstruction.SetExtraInfo("heading", geo.BearingTo(doublePrevNode.GetLat(), doublePrevNode.GetLon(), tail.GetLat(), tail.GetLon()))

	db.instructions = append(db.instructions, finishInstruction)
}

package guidance

import (
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
	prevInstruction   int // -1 if no previous instruction
	turnDescriptions  []string
	drivingDirections []da.DrivingDirection

	segmentIds           []da.Index
	path                 []da.Index
	geometry             da.Coordinates
	alternativeTurns     []da.Index
	doublePrevStreetName string

	prevInitialBearing       float64 // previous initial bearing of previous road segment. atau course of the previous road segment.
	doublePrevInitialBearing float64
	cumulativeDistance       float64
	cumulativeCost           float64

	lastPathId     int
	nextStreetName uint32 // streetName Id

	engine RoutingEngine
	graph  Graph
	rn     RoadNetworkDataContainer

	doublePrevSegmentId da.Index
	prevSegmentId       da.Index

	prevSign         da.TurnType
	clockwise        bool // clockwise roundabout (like in indonesia) or counter-clockwise roundabout
	lefthand         bool // left hand traffic (like in indonesia) or right hand traffic
	prevInRoundabout bool
	useLookForward   bool
	lookForwardStep  int

	// reroute
	reroute       bool
	startSegId    da.Index
	useAnnotation bool
}

func NewDirectionBuilder(engine RoutingEngine, graph Graph, rn RoadNetworkDataContainer, lefthand bool,
	turnSignCache *otter.Cache[uint64, uint64]) *DirectionBuilder {

	clockwise := lefthand

	db := &DirectionBuilder{
		engine:              engine,
		graph:               graph,
		rn:                  rn,
		prevSegmentId:       da.INVALID_SEGMENT_ID,
		nextStreetName:      da.INVALID_STREET_NAME_ID,
		doublePrevSegmentId: da.INVALID_SEGMENT_ID,

		prevInRoundabout:         false,
		doublePrevInitialBearing: 0,
		clockwise:                clockwise,
		lefthand:                 lefthand,
		turnDescriptions:         make([]string, 0, 16),
		drivingDirections:        make([]da.DrivingDirection, 0, 16),
		instructions:             make([]da.Instruction, 0, 16),
		prevInstruction:          -1,
		segmentIds:               make([]da.Index, 0, 16),
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
	db.segmentIds = db.segmentIds[:0]
	db.geometry = db.geometry[:0]
}

func (db *DirectionBuilder) SetReroute(startSegId da.Index) {
	db.reroute = true
	db.startSegId = startSegId
}

// GetDrivingDirections. generate driving directions dari path (list of segmentIds)
// worst case: O(n*q + n*l), n = number of segmentIds in path arr, q=max outDegree of any vertex in the graph, l=maximum length of any edge geometry
func (db *DirectionBuilder) GetDrivingDirections(
	segments []da.Index,
	sp, tp da.PhantomNode,
	useAnnotation bool,
) []da.DrivingDirection {
	db.useAnnotation = useAnnotation

	if len(segments) == 0 {
		return nil
	}

	if !db.reroute {
		db.path = append(db.path, sp.GetVId())
	} else if db.reroute {
		db.path = append(db.path, db.startSegId)
	}

	db.path = append(db.path, segments...)

	m := len(db.path)

	db.lastPathId = 1
	for db.lastPathId < m {
		segmentId := db.path[db.lastPathId]

		db.buildInstruction(segmentId, sp)
		db.lastPathId++
		db.useLookForward = false
		db.lookForwardStep = 0
	}

	db.finalInstruction(tp.GetVId(), tp)

	db.turnDescriptions = db.turnDescriptions[:0]

	for i := range db.instructions {
		desc := db.instructions[i].GetTurnDescription(db.clockwise)
		db.turnDescriptions = append(db.turnDescriptions, desc)
	}

	db.drivingDirections = db.drivingDirections[:0]

	for i := range db.instructions {
		var (
			currStepCost, currStepDistance float64
		)
		if i > 0 {
			currStepCost = db.instructions[i].GetCumulativeCost() - db.instructions[i-1].GetCumulativeCost()
			currStepDistance = db.instructions[i].GetCumulativeDistance() - db.instructions[i-1].GetCumulativeDistance()
		}

		db.drivingDirections = append(db.drivingDirections, da.NewDrivingDirection(db.instructions[i], db.turnDescriptions[i],
			currStepCost, currStepDistance, db.instructions[i].GetSegmentIds(), db.instructions[i].GetTurnBearing(), db.instructions[i].GetAnnotation()))
	}

	return db.drivingDirections
}

func (db *DirectionBuilder) buildInstruction(segmentId da.Index, sp da.PhantomNode) {

	tail := db.rn.GetSegmentTailCoord(segmentId)
	head := db.GetHeadPoint(segmentId, tail, 15)
	isRoundabout := db.rn.IsRoundabout(segmentId)

	var prevPoint da.Coordinate
	if db.prevSegmentId != da.INVALID_SEGMENT_ID {
		prevPoint = db.rn.GetSegmentTailCoord(db.prevSegmentId)
	}

	streetName := db.rn.GetStreetName(segmentId)
	if db.prevInstruction == -1 && !isRoundabout {
		db.initialInstruction(segmentId, sp, streetName)
	} else if isRoundabout {
		// current edge bundaran
		db.handleRoundabout(prevPoint, tail, head, streetName, segmentId)
	} else if db.prevInRoundabout {
		db.instructions[db.prevInstruction].SetStreetName(streetName)
		db.instructions[db.prevInstruction].SetExited()
		db.doublePrevStreetName = db.rn.GetStreetName(db.prevSegmentId)
	} else {
		turnSign := db.getTurnSign(segmentId, streetName)
		if turnSign != da.IGNORE {
			db.updateTurnInstruction(turnSign, streetName, segmentId, prevPoint, tail)
		}
		db.prevSign = turnSign
	}

	if !db.useLookForward {
		db.updateState(segmentId, isRoundabout)
	}
}

func (db *DirectionBuilder) finalInstruction(segmentId da.Index, tp da.PhantomNode) {

	tail := db.rn.GetSegmentTailCoord(segmentId)

	head := tp.GetSnappedCoord()

	turnBearing := geo.ComputeFinalBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
	ann := db.annotation()

	finishInstruction := da.NewInstruction(da.FINISH, db.rn.GetStreetName(segmentId), head, false,
		db.segmentIds, db.cumulativeDistance, db.cumulativeCost, turnBearing, ann, db.clockwise)

	db.instructions = append(db.instructions, finishInstruction)
}

func (db *DirectionBuilder) initialInstruction(segmentId da.Index, sp da.PhantomNode, streetName string) {
	// start point dari shortetest path & bukan bundaran (roundabout) & bukan reroute
	sign := da.START
	tailCoord := sp.GetSnappedCoord()
	point := da.NewCoordinate(
		tailCoord.GetLat(), tailCoord.GetLon(),
	)

	head := db.rn.GetSegmentHeadCoord(segmentId) // khusus initial instruction, harus pakai GetSegmentHeadCoord() biar arah mata angin ke head dari road segment segmentId bener

	turnBearing := geo.ComputeInitialBearing(tailCoord.GetLat(), tailCoord.GetLon(),
		head.GetLat(), head.GetLon())

	db.updateState(segmentId, false)
	ann := db.annotation()

	var segmentIds []da.Index
	if db.useAnnotation {
		segmentIds = []da.Index{segmentId}
	}
	newIns := da.NewInstruction(sign, streetName, point, false,
		segmentIds, db.cumulativeDistance, db.cumulativeCost,
		turnBearing, ann, db.clockwise)

	newIns.SetHeading(turnBearing)
	db.instructions = append(db.instructions, newIns)
	db.prevInstruction = len(db.instructions) - 1

	db.reset()
}

func (db *DirectionBuilder) updateTurnInstruction(turnSign da.TurnType, streetName string, segmentId da.Index, prevPoint, tail da.Coordinate) {
	uTurn, uturnType := db.checkUTurn(turnSign, streetName, segmentId)
	if uTurn {
		db.instructions[db.prevInstruction].SetSign(uturnType)
		db.instructions[db.prevInstruction].SetStreetName(streetName)
	} else {
		// bukan U-turn -> continue/right/left
		turnBearing := geo.ComputeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(),
			tail.GetLat(), tail.GetLon())
		nextStreetName := db.rn.GetStrFromId(db.nextStreetName)
		suggestAlternatives := db.IsSuggestAlternatives(segmentId)
		ann := db.annotation()
		ins := da.NewInstruction(turnSign, nextStreetName, tail, false, db.segmentIds, db.cumulativeDistance, db.cumulativeCost,
			turnBearing, ann, db.clockwise)
		ins.SetSuggestAlternatives(suggestAlternatives)
		db.reset()
		db.instructions = append(db.instructions, ins)
		db.prevInstruction = len(db.instructions) - 1
	}
}

// annotation. build annotation
func (db *DirectionBuilder) annotation() da.Annotation {
	if !db.useAnnotation {
		return da.Annotation{}
	}
	return db.buildSimplifiedAnnotation(db.segmentIds, db.geometry)
}

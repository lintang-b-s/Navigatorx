package datastructure

import "github.com/lintang-b-s/Navigatorx/pkg"

// ConditionalBarrierNode https://wiki.openstreetmap.org/wiki/Conditional_restrictions#Turn_restrictions
// todo: ini haruse semuanya bisa dijadiin satu struct aja biar lebih clean
type ConditionalBarrierNode struct {
	osmNodeId    int64  // osm node id (bukan graph vertex id)
	timeRangeVal string // yes @ Dec 25
}

// NewConditionalBarrierNode creates and returns conditional barrier node.
func NewConditionalBarrierNode(osmNodeId int64, timeRangeVal string) ConditionalBarrierNode {
	return ConditionalBarrierNode{osmNodeId, timeRangeVal}
}

// GetOsmNodeId returns osm node id.
func (c ConditionalBarrierNode) GetOsmNodeId() int64 {
	return c.osmNodeId
}

// GetTimeRangeVal returns time range val.
func (c ConditionalBarrierNode) GetTimeRangeVal() string {
	return c.timeRangeVal
}

// ConditionalReversibleEdge represents conditional reversible edge.
type ConditionalReversibleEdge struct {
	edgeId       Index
	timeRangeVal string // yes @ (Sa-Su;PH)
}

// NewConditionalReversibleEdge creates and returns conditional reversible edge.
func NewConditionalReversibleEdge(edgeId Index, timeRangeVal string) ConditionalReversibleEdge {
	return ConditionalReversibleEdge{edgeId: edgeId, timeRangeVal: timeRangeVal}
}

// GetEdgeId returns edge id.
func (c ConditionalReversibleEdge) GetEdgeId() Index {
	return c.edgeId
}

// GetTimeRangeVal returns time range val.
func (c ConditionalReversibleEdge) GetTimeRangeVal() string {
	return c.timeRangeVal
}

// ConditionalSpeedLimit represents conditional speed limit.
type ConditionalSpeedLimit struct {
	edgeId            Index
	timeRangeSpeedVal string // 130 @ 19:00-06:00
}

// NewConditionalSpeedLimit creates and returns conditional speed limit.
func NewConditionalSpeedLimit(edgeId Index, timeRangeSpeedVal string) ConditionalSpeedLimit {
	return ConditionalSpeedLimit{edgeId: edgeId, timeRangeSpeedVal: timeRangeSpeedVal}
}

// GetEdgeId returns edge id.
func (c ConditionalSpeedLimit) GetEdgeId() Index {
	return c.edgeId
}

// GetTimeRangeSpeedVal returns time range speed val.
func (c ConditionalSpeedLimit) GetTimeRangeSpeedVal() string {
	return c.timeRangeSpeedVal
}

// ConditionalTrafficMode represents conditional traffic mode.
type ConditionalTrafficMode struct {
	edgeId       Index
	timeRangeVal string // no @ (Sa 08:00-16:00)
}

// NewConditionalTrafficMode creates and returns conditional traffic mode.
func NewConditionalTrafficMode(edgeId Index, timeRangeVal string) ConditionalTrafficMode {
	return ConditionalTrafficMode{edgeId: edgeId, timeRangeVal: timeRangeVal}
}

// GetEdgeId returns edge id.
func (c ConditionalTrafficMode) GetEdgeId() Index {
	return c.edgeId
}

// GetTimeRangeVal returns time range val.
func (c ConditionalTrafficMode) GetTimeRangeVal() string {
	return c.timeRangeVal
}

// ConditionalTurnRestriction represents conditional turn restriction.
type ConditionalTurnRestriction struct {
	fromVId      Index
	viaVId       Index
	toVId        Index
	viaEIds      []Index
	viaWay       bool // is via-way turn restriction. untuk saat ini (30 april 2026 conditional turn restriction cuma support via-node turn restriction dulu).
	turnType     pkg.TurnType
	timeRangeVal string // Mo-Fr 07:00-09:00, 16:00-18:00  (kita parse pakai IsConditionalRestrictionCurrentlyProhibited()  osmparser/helper.go )
}

// NewConditionalTurnRestriction creates and returns conditional turn restriction.
func NewConditionalTurnRestriction(fromVId Index, viaVId Index, toVId Index, viaEIds []Index, viaWay bool, timeRangeVal string, turnType pkg.TurnType) ConditionalTurnRestriction {
	return ConditionalTurnRestriction{fromVId, viaVId, toVId, viaEIds, viaWay, turnType, timeRangeVal}
}

// GetFromVId returns from vertex id.
func (c ConditionalTurnRestriction) GetFromVId() Index {
	return c.fromVId
}

// GetViaVId returns via vertex id.
func (c ConditionalTurnRestriction) GetViaVId() Index {
	return c.viaVId
}

// GetToVId returns to vertex id.
func (c ConditionalTurnRestriction) GetToVId() Index {
	return c.toVId
}

// GetViaEIds returns via edge ids.
func (c ConditionalTurnRestriction) GetViaEIds() []Index {
	return c.viaEIds
}

// GetViaWay returns is via-way turn restriction.
func (c ConditionalTurnRestriction) GetViaWay() bool {
	return c.viaWay
}

// GetTurnTableId returns turn table id.
func (c ConditionalTurnRestriction) GetTurnTableId() pkg.TurnType {
	return c.turnType
}

// GetTimeRangeVal returns time range val.
func (c ConditionalTurnRestriction) GetTimeRangeVal() string {
	return c.timeRangeVal
}

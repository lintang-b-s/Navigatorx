package datastructure

import "github.com/lintang-b-s/Navigatorx/pkg"

// ConditionalBarrierNode https://wiki.openstreetmap.org/wiki/Conditional_restrictions#Turn_restrictions
type ConditionalBarrierNode struct {
	osmNodeId    int64  // osm node id (bukan graph vertex id)
	timeRangeVal string // yes @ Dec 25
}

func NewConditionalBarrierNode(osmNodeId int64, timeRangeVal string) ConditionalBarrierNode {
	return ConditionalBarrierNode{osmNodeId, timeRangeVal}
}

func (c ConditionalBarrierNode) GetOsmNodeId() int64 {
	return c.osmNodeId
}

func (c ConditionalBarrierNode) GetTimeRangeVal() string {
	return c.timeRangeVal
}

type ConditionalReversibleEdge struct {
	edgeId       Index
	timeRangeVal string // yes @ (Sa-Su;PH)
}

func NewConditionalReversibleEdge(edgeId Index, timeRangeVal string) ConditionalReversibleEdge {
	return ConditionalReversibleEdge{edgeId: edgeId, timeRangeVal: timeRangeVal}
}

func (c ConditionalReversibleEdge) GetEdgeId() Index {
	return c.edgeId
}

func (c ConditionalReversibleEdge) GetTimeRangeVal() string {
	return c.timeRangeVal
}

type ConditionalSpeedLimit struct {
	edgeId            Index
	timeRangeSpeedVal string // 130 @ 19:00-06:00
}

func NewConditionalSpeedLimit(edgeId Index, timeRangeSpeedVal string) ConditionalSpeedLimit {
	return ConditionalSpeedLimit{edgeId: edgeId, timeRangeSpeedVal: timeRangeSpeedVal}
}

func (c ConditionalSpeedLimit) GetEdgeId() Index {
	return c.edgeId
}

func (c ConditionalSpeedLimit) GetTimeRangeSpeedVal() string {
	return c.timeRangeSpeedVal
}

type ConditionalTrafficMode struct {
	edgeId       Index
	timeRangeVal string // no @ (Sa 08:00-16:00)
}

func NewConditionalTrafficMode(edgeId Index, timeRangeVal string) ConditionalTrafficMode {
	return ConditionalTrafficMode{edgeId: edgeId, timeRangeVal: timeRangeVal}
}

func (c ConditionalTrafficMode) GetEdgeId() Index {
	return c.edgeId
}

func (c ConditionalTrafficMode) GetTimeRangeVal() string {
	return c.timeRangeVal
}

type ConditionalTurnRestriction struct {
	fromVId      Index
	viaVId       Index
	toVId        Index
	viaEIds      []Index
	viaWay       bool // is via-way turn restriction. untuk saat ini (30 april 2026 conditional turn restriction cuma support via-node turn restriction dulu).
	turnType     pkg.TurnType
	timeRangeVal string // Mo-Fr 07:00-09:00, 16:00-18:00  (kita parse pakai IsConditionalRestrictionCurrentlyProhibited()  osmparser/helper.go )
}

func NewConditionalTurnRestriction(fromVId Index, viaVId Index, toVId Index, viaEIds []Index, viaWay bool, timeRangeVal string, turnType pkg.TurnType) ConditionalTurnRestriction {
	return ConditionalTurnRestriction{fromVId, viaVId, toVId, viaEIds, viaWay, turnType, timeRangeVal}
}

func (c ConditionalTurnRestriction) GetFromVId() Index {
	return c.fromVId
}

func (c ConditionalTurnRestriction) GetViaVId() Index {
	return c.viaVId
}

func (c ConditionalTurnRestriction) GetToVId() Index {
	return c.toVId
}

func (c ConditionalTurnRestriction) GetViaEIds() []Index {
	return c.viaEIds
}

func (c ConditionalTurnRestriction) GetViaWay() bool {
	return c.viaWay
}

func (c ConditionalTurnRestriction) GetTurnType() pkg.TurnType {
	return c.turnType
}

func (c ConditionalTurnRestriction) GetTimeRangeVal() string {
	return c.timeRangeVal
}

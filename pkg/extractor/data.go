package extractor

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type node struct {
	id    int64
	coord da.Coordinate
}

type restriction struct {
	id              int64           // id dari relation turn restriction
	via             int64           // via-node graph node id
	viaWays         []int64         // via-ways osm id
	to              int64           // to-way osm id
	turnRestriction TurnRestriction // tipe turn restriction
	timeRangeVal    string
	isWay           bool
	conditional     bool
}

func (r restriction) GetId() int64 {
	return r.id
}

func (r restriction) GetVia() int64 {
	return r.via
}

func (r restriction) GetViaWays() []int64 {
	return r.viaWays
}

func (r restriction) GetTo() int64 {
	return r.to
}

func (r restriction) GetTurnRestriction() TurnRestriction {
	return r.turnRestriction
}

func (r restriction) GetTimeRangeVal() string {
	return r.timeRangeVal
}

func (r restriction) GetIsWay() bool {
	return r.isWay
}

func (r restriction) IsWay() bool {
	return r.isWay
}

func (r restriction) GetConditional() bool {
	return r.conditional
}

func (r restriction) IsConditional() bool {
	return r.conditional
}

type osmWay struct {
	nodes      []int64    // osm  nodes dari osm way ini
	graphNodes []da.Index // osm nodes yang jadi graph node dari osm way ini
	oneWay     bool
	hwTag      string
}

func (o osmWay) GetOsmNodes() []int64 {
	return o.nodes
}

type nodeWithCoord struct {
	tipe  NodeType
	coord da.Coordinate
}

func (n nodeWithCoord) GetCoord() da.Coordinate {
	return n.coord
}

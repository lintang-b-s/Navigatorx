package extractor

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Edge[W util.Numeric] struct {
	weight    W
	distance  uint32
	fromOsmId uint64
	toOsmId   uint64
	from      uint32
	to        uint32
	osmwayId  int64
}

func (e *Edge[W]) GetFrom() da.Index {
	return da.Index(e.from)
}

func (e *Edge[W]) GetTo() da.Index {
	return da.Index(e.to)
}

func (e *Edge[W]) GetFromOsmId() uint64 {
	return e.fromOsmId
}

func (e *Edge[W]) GetToOsmId() uint64 {
	return e.toOsmId
}

func (e *Edge[W]) GetWeight() W {
	return e.weight
}

func (e *Edge[W]) GetDistance() uint32 {
	return e.distance
}

func (e *Edge[W]) SetFromOSMId(fromOsmId uint64) {
	e.fromOsmId = fromOsmId
}

func (e *Edge[W]) SetToOSMId(toOsmId uint64) {
	e.toOsmId = toOsmId
}

func (e *Edge[W]) SetOsmWayId(osmWayId int64) {
	e.osmwayId = osmWayId
}

func (e *Edge[W]) GetOsmWayId() int64 {
	return e.osmwayId
}

func NewEdge[W util.Numeric](
	from, to uint32,
	weight W,
	distance uint32,
) Edge[W] {
	return Edge[W]{
		from:     from,
		to:       to,
		weight:   weight,
		distance: distance,
	}
}

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

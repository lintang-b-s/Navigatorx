package osmparser

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Edge struct {
	weight                     float64
	distance                   float64
	fromOsmId                  uint64
	toOsmId                    uint64
	from                       uint32
	to                         uint32
	osmwayId                   int64
	junctionTail, junctionHead bool
	containsTrafficLight       bool
	hwType                     pkg.OsmHighwayType
}

func (e *Edge) GetFrom() da.Index {
	return da.Index(e.from)
}

func (e *Edge) GetTo() da.Index {
	return da.Index(e.to)
}

func (e *Edge) GetFromOsmId() uint64 {
	return e.fromOsmId
}

func (e *Edge) GetToOsmId() uint64 {
	return e.toOsmId
}

func (e *Edge) GetWeight() float64 {
	return e.weight
}

func (e *Edge) GetDistance() float64 {
	return e.distance
}

func (e *Edge) GetHighwayType() pkg.OsmHighwayType {
	return e.hwType
}

func (e *Edge) ContainsTrafficLight() bool {
	return e.containsTrafficLight
}

func (e *Edge) SetFromOSMId(fromOsmId uint64) {
	e.fromOsmId = fromOsmId
}

func (e *Edge) SetToOSMId(toOsmId uint64) {
	e.toOsmId = toOsmId
}

func (e *Edge) SetOsmWayId(osmWayId int64) {
	e.osmwayId = osmWayId
}

func (e *Edge) SetJunctionHead() {
	e.junctionHead = true
}

func (e *Edge) SetJunctionTail() {
	e.junctionTail = true
}

func (e *Edge) SetContainsTrafficLight(containsTrafficLight bool) {
	e.containsTrafficLight = containsTrafficLight
}

func (e *Edge) IsJunctionHead() bool {
	return e.junctionHead
}

func (e *Edge) IsJunctionTail() bool {
	return e.junctionTail
}

func (e *Edge) GetOsmWayId() int64 {
	return e.osmwayId
}

func NewEdge(from, to uint32, weight, distance float64, containsTrafficLight bool, hwType pkg.OsmHighwayType) Edge {
	return Edge{
		from:                 from,
		to:                   to,
		weight:               weight,
		distance:             distance,
		containsTrafficLight: containsTrafficLight,
		hwType:               hwType,
	}
}

type node struct {
	id    int64
	coord NodeCoord
}

type NodeCoord struct {
	lat float64
	lon float64
}

func NewNodeCoord(lat, lon float64) NodeCoord {
	return NodeCoord{lat, lon}
}

type restriction struct {
	via             da.Index
	viaWays         []int64
	to              int64
	turnRestriction TurnRestriction
	timeRangeVal    string
	isWay           bool
	conditional     bool
}

type osmWay struct {
	nodes      []int64    // osm  nodes dari osm way ini
	graphNodes []da.Index // osm nodes yang jadi graph node dari osm way ini
	oneWay     bool
	hwTag      string
}
type nodeWithCoord struct {
	tipe  NodeType
	coord NodeCoord
}

package osmparser

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Edge[W util.RoutingNumber] struct {
	weight                     W
	distance                   uint32
	fromOsmId                  uint64
	toOsmId                    uint64
	from                       uint32
	to                         uint32
	osmwayId                   int64
	junctionTail, junctionHead bool
	containsTrafficLight       bool
	hwType                     pkg.OsmHighwayType
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

func (e *Edge[W]) GetHighwayType() pkg.OsmHighwayType {
	return e.hwType
}

func (e *Edge[W]) ContainsTrafficLight() bool {
	return e.containsTrafficLight
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

func (e *Edge[W]) SetJunctionHead() {
	e.junctionHead = true
}

func (e *Edge[W]) SetJunctionTail() {
	e.junctionTail = true
}

func (e *Edge[W]) SetContainsTrafficLight(containsTrafficLight bool) {
	e.containsTrafficLight = containsTrafficLight
}

func (e *Edge[W]) IsJunctionHead() bool {
	return e.junctionHead
}

func (e *Edge[W]) IsJunctionTail() bool {
	return e.junctionTail
}

func (e *Edge[W]) GetOsmWayId() int64 {
	return e.osmwayId
}

func NewEdge[W util.RoutingNumber](
	from, to uint32,
	weight W,
	distance uint32,
	containsTrafficLight bool,
	hwType pkg.OsmHighwayType,
) Edge[W] {
	return Edge[W]{
		from:                 from,
		to:                   to,
		weight:               weight,
		distance:             distance,
		containsTrafficLight: containsTrafficLight,
		hwType:               hwType,
	}
}

func NewFixedEdge(
	from, to uint32,
	weightSeconds, distanceMeters float64,
	containsTrafficLight bool,
	hwType pkg.OsmHighwayType,
) Edge[int32] {
	weight := util.RoundCentiseconds(weightSeconds)
	distance := uint32(util.RoundCentimeters(distanceMeters))
	return NewEdge(from, to, weight, distance, containsTrafficLight, hwType)
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

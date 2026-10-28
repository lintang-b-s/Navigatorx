package extractor

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type node struct {
	id    int64
	coord da.Coordinate
}

// Restriction stores the OSM identifiers and conditions for a turn restriction.
type Restriction struct {
	id              int64           // id dari relation turn restriction
	via             int64           // via-node graph node id
	viaWays         []int64         // via-ways osm id
	to              int64           // to-way osm id
	turnRestriction TurnRestriction // tipe turn restriction
	timeRangeVal    string
	isWay           bool
	conditional     bool
}

// GetId returns the OSM identifier of the restriction.
func (r Restriction) GetId() int64 {
	return r.id
}

// GetVia returns the via-node identifier of the restriction.
func (r Restriction) GetVia() int64 {
	return r.via
}

// GetViaWays returns the identifiers of the via ways in the restriction.
func (r Restriction) GetViaWays() []int64 {
	return r.viaWays
}

// GetTo returns the identifier of the restriction's destination way.
func (r Restriction) GetTo() int64 {
	return r.to
}

// GetTurnRestriction returns the restriction type.
func (r Restriction) GetTurnRestriction() TurnRestriction {
	return r.turnRestriction
}

// GetTimeRangeVal returns the conditional time range.
func (r Restriction) GetTimeRangeVal() string {
	return r.timeRangeVal
}

// GetIsWay reports whether the restriction uses a via way.
func (r Restriction) GetIsWay() bool {
	return r.isWay
}

// IsWay reports whether the restriction uses a via way.
func (r Restriction) IsWay() bool {
	return r.isWay
}

// GetConditional reports whether the restriction is conditional.
func (r Restriction) GetConditional() bool {
	return r.conditional
}

// IsConditional reports whether the restriction is conditional.
func (r Restriction) IsConditional() bool {
	return r.conditional
}

// OsmWay stores an OSM way's node identifiers.
type OsmWay struct {
	nodes      []int64    // osm  nodes dari osm way ini
	graphNodes []da.Index // osm nodes yang jadi graph node dari osm way ini
	oneWay     bool
	hwTag      string
}

// GetOsmNodes returns the node identifiers in the OSM way.
func (o OsmWay) GetOsmNodes() []int64 {
	return o.nodes
}

// NodeWithCoord pairs an OSM node identifier with its coordinate.
type NodeWithCoord struct {
	tipe  NodeType
	coord da.Coordinate
}

// GetCoord returns the node's coordinate.
func (n NodeWithCoord) GetCoord() da.Coordinate {
	return n.coord
}

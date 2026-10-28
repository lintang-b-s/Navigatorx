package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// Edge represents edge.
type Edge[W util.Numeric] struct {
	from      uint32
	to        uint32
	weight    W
	distance  uint32
	fromOsmId uint64
	toOsmId   uint64
	osmWayId  int64
}

// GetFrom returns from.
func (e *Edge[W]) GetFrom() Index {
	return Index(e.from)
}

// GetTo returns to.
func (e *Edge[W]) GetTo() Index {
	return Index(e.to)
}

// GetFromOsmId returns from osm id.
func (e *Edge[W]) GetFromOsmId() uint64 {
	return e.fromOsmId
}

// GetToOsmId returns to osm id.
func (e *Edge[W]) GetToOsmId() uint64 {
	return e.toOsmId
}

// GetWeight returns weight.
func (e *Edge[W]) GetWeight() W {
	return e.weight
}

// GetDistance returns distance.
func (e *Edge[W]) GetDistance() uint32 {
	return e.distance
}

// SetFromOSMId sets from osmid.
func (e *Edge[W]) SetFromOSMId(fromOsmId uint64) {
	e.fromOsmId = fromOsmId
}

// SetToOSMId sets to osmid.
func (e *Edge[W]) SetToOSMId(toOsmId uint64) {
	e.toOsmId = toOsmId
}

// SetOsmWayId sets osm way id.
func (e *Edge[W]) SetOsmWayId(osmWayId int64) {
	e.osmWayId = osmWayId
}

// GetOsmWayId returns osm way id.
func (e *Edge[W]) GetOsmWayId() int64 {
	return e.osmWayId
}

// NewEdge creates and returns edge.
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

// PairEdge represents pair edge.
type PairEdge struct {
	To     int
	Weight float64
}

// NewPairEdge creates and returns pair edge.
func NewPairEdge(to int, weight float64) PairEdge {
	return PairEdge{To: to, Weight: weight}
}

// FlattenEdges handles flatten edges.
func FlattenEdges(es [][]PairEdge) []Edge[float64] {
	return FlattenEdgesG[float64](es)
}

// FlattenEdgesG handles flatten edges with generic weight type.
func FlattenEdgesG[W util.Numeric](es [][]PairEdge) []Edge[W] {
	totalEdges := 0
	for _, edges := range es {
		totalEdges += len(edges)
	}
	flatten := make([]Edge[W], 0, totalEdges)
	for from, edges := range es {
		for _, e := range edges {
			length := uint32(math.Round(e.Weight * 100))
			flatten = append(flatten, NewEdge[W](uint32(from), uint32(e.To), W(e.Weight), length))
		}
	}
	return flatten
}

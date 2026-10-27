package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Edge[W util.Numeric] struct {
	from      uint32
	to        uint32
	weight    W
	distance  uint32
	fromOsmId uint64
	toOsmId   uint64
	osmWayId  int64
}

func (e *Edge[W]) GetFrom() Index {
	return Index(e.from)
}

func (e *Edge[W]) GetTo() Index {
	return Index(e.to)
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
	e.osmWayId = osmWayId
}

func (e *Edge[W]) GetOsmWayId() int64 {
	return e.osmWayId
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

type PairEdge struct {
	To     int
	Weight float64
}

func NewPairEdge(to int, weight float64) PairEdge {
	return PairEdge{To: to, Weight: weight}
}

func FlattenEdges(es [][]PairEdge) []Edge[float64] {
	return FlattenEdgesG[float64](es)
}

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

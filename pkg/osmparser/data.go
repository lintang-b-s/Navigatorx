package osmparser

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Edge struct {
	from      uint32
	fromOsmId uint64
	to        uint32
	toOsmId   uint64
	weight    float64
	distance  float64
	edgeID    uint32
	hwType    pkg.OsmHighwayType
}

func (e *Edge) GetFrom() datastructure.Index {
	return datastructure.Index(e.from)
}

func (e *Edge) GetTo() datastructure.Index {
	return datastructure.Index(e.to)
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

func NewEdge(from, to uint32, weight, distance float64, edgeID uint32, hwType pkg.OsmHighwayType) Edge {
	return Edge{
		from:     from,
		to:       to,
		weight:   weight,
		distance: distance,
		edgeID:   edgeID,
		hwType:   hwType,
	}
}

// Package spatialindex provides spatial indexing capabilities using R-trees (menggunakan mercator projected coordinate system).
package spatialindex

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"

	"github.com/lintang-b-s/rtree"
	"go.uber.org/zap"
)

type leafData struct {
	edgeId uint64 // (backwardEdgeId << 32) | forwardEdgeId
	flag   uint8
}

func newLeafData(edgeId uint64, flag uint8) leafData {
	return leafData{edgeId: edgeId, flag: flag}
}

type Rtree struct {
	tr *rtree.RTreeGN[int32, leafData]
}

const spatialIndexPrecision = 1e5 // mercator projected 2d coordinate

func spatialRound(value float64) int32 {
	return int32(math.Round(value * spatialIndexPrecision))
}

// our query is compact graph CRP multilevel dijkstra with turn cost
// query input = (as,s, at,t)
// as = incoming edge of source
// at = outgouing edge of target
// so we need to know nearby road segments as & at before run the query
func NewRtree() *Rtree {
	var tr rtree.RTreeGN[int32, leafData]
	return &Rtree{
		tr: &tr,
	}
}

type segmentKey struct {
	osmId int64
	pair  uint64 // (hi << 32) | lo
}

func newSegmentKey(osmId int64, pair uint64) segmentKey {
	return segmentKey{osmId: osmId, pair: pair}
}

type segmentVal struct {
	minX, minY int32
	maxX, maxY int32
	eId        da.Index // forwardEdgeId
	flag       uint8    // flag dari forwardEdgeId
}

func newSegmentVal(minX, minY, maxX, maxY int32, eId da.Index, flag uint8) segmentVal {
	return segmentVal{minX, minY, maxX, maxY, eId, flag}
}

// Build. build r-tree
func (rt *Rtree) Build(graph *da.Graph, logger *zap.Logger) {
	logger.Info("Building R-tree spatial index...")
	m := graph.NumberOfEdges()
	mins := make([][2]int32, 0, m)
	maxs := make([][2]int32, 0, m)
	items := make([]leafData, 0, m)

	segmentSet := make(map[segmentKey]segmentVal, m/2) // segmentKey -> eId (forward direction), mbr

	graph.ForOutEdges(func(exitPoint, head, tail, entryId, entryPoint da.Index,
		percentage float64, eId da.Index) {

		eGeom := graph.GetEdgeGeometry(eId)
		maxLat, maxLon := math.Inf(-1), math.Inf(-1)
		minLat, minLon := math.MaxFloat64, math.MaxFloat64
		for i := 0; i < len(eGeom); i++ {
			point := eGeom[i]
			if point.GetLat() > maxLat {
				maxLat = point.GetLat()
			}

			if point.GetLon() > maxLon {
				maxLon = point.GetLon()
			}

			if point.GetLat() < minLat {
				minLat = point.GetLat()
			}
			if point.GetLon() < minLon {
				minLon = point.GetLon()
			}
		}

		// use mercator projected coordinate
		minY := spatialRound(geo.CalcLatToY(minLat))
		maxY := spatialRound(geo.CalcLatToY(maxLat))
		minX := spatialRound(geo.CalcLonToX(minLon))
		maxX := spatialRound(geo.CalcLonToX(maxLon))

		osmId := graph.GetOsmWayId(eId)
		hi, lo := uint64(tail), uint64(head)
		if hi < lo {
			hi, lo = lo, hi
		}

		segKey := newSegmentKey(osmId, (hi<<32)|lo)
		if otherDirSeg, ok := segmentSet[segKey]; ok {
			flag := rt.getFlag(graph, eId, false)
			flag |= otherDirSeg.flag
			forwardEdgeId := uint64(otherDirSeg.eId)
			backwardEdgeId := uint64(eId)
			mins = append(mins, [2]int32{minX, minY})
			maxs = append(maxs, [2]int32{maxX, maxY})
			packEdgeId := (backwardEdgeId << 32) | forwardEdgeId
			leaf := newLeafData(packEdgeId, flag)
			items = append(items, leaf)
			delete(segmentSet, segKey)
		} else {
			// anggap yang disini sbg forward road segment (kalau road segment contained di two-way osm way)
			flag := rt.getFlag(graph, eId, true)
			segmentSet[segKey] = newSegmentVal(minX, minY, maxX, maxY, eId, flag)
		}
	})

	for _, seg := range segmentSet {
		// sisa road segments yang oneway=true
		mins = append(mins, [2]int32{seg.minX, seg.minY})
		maxs = append(maxs, [2]int32{seg.maxX, seg.maxY})
		items = append(items, newLeafData(uint64(seg.eId), seg.flag))
	}

	rt.tr.Bulk(mins, maxs, items)
	logger.Info("R-tree spatial index built.")
}

// SearchWithinRadius search for all arc endpoints within radius (in km) from the query point (qLat, qLon)
// let M=number of road segmnents/edges in the graph
// R-tree search worst case is O(M) ketika MBR dari query overlap semua mbr leafs data
// tapi karena kita limit leaf data yang kita return sebanyak MAX_CANDIDATES
// SearchWithinRadius worst case is O(M)
// https://www2.cs.sfu.ca/CourseCentral/454/jpei/slides/R-Tree.pdf
// https://dl.acm.org/doi/10.1145/971697.602266
// mode=0  origin, mode=1 destination, mode=2 not both, mode=3 for client-side realtime mapmatching webassembly
func (rt *Rtree) SearchWithinRadius(qLat, qLon, radius float64, mode uint8) []da.Index {

	qy, qx := geo.CalcLatToY(qLat), geo.CalcLonToX(qLon)

	lowerY, lowerX := qy-radius, qx-radius
	upperY, upperX := qy+radius, qx+radius

	results := make([]da.Index, 0, 10)

	rt.tr.Search([2]int32{spatialRound(lowerX), spatialRound(lowerY)}, [2]int32{spatialRound(upperX), spatialRound(upperY)},
		func(min, max [2]int32, data leafData) bool {
			candsLessThanMax := len(results) <= MAX_CANDIDATES
			if mode == 0 && !rt.IsJunctionHead(data) {
				// skip edge yang head nya gak junction
				return true && candsLessThanMax
			} else if mode == 1 && !rt.IsJunctionTail(data) {
				// skip edge yang tail nya gak junction
				return true && candsLessThanMax
			}

			forwEdgeId, backEdgeId := rt.GetEdgeId(data, mode)
			if forwEdgeId != da.INVALID_EDGE_ID {
				results = append(results, forwEdgeId)
			}
			if backEdgeId != da.INVALID_EDGE_ID {
				results = append(results, backEdgeId)
			}
			return candsLessThanMax
		})

	return results
}

func (rt *Rtree) getFlag(graph *da.Graph, eId da.Index, forward bool) uint8 {
	flag := uint8(0)
	if graph.IsJunctionHead(eId) {
		switch forward {
		case true:
			flag |= JUNCTION_FORWARD_HEAD_FLAG
		default:
			flag |= JUNCTION_BACKWARD_HEAD_FLAG
			flag |= BIDIRECTIONAL
		}
	}
	if graph.IsJunctionTail(eId) {
		switch forward {
		case true:
			flag |= JUNCTION_FORWARD_TAIL_FLAG
		default:
			flag |= JUNCTION_BACKWARD_TAIL_FLAG
			flag |= BIDIRECTIONAL
		}
	}
	return flag
}

func (rt *Rtree) IsJunctionHead(data leafData) bool {
	return isForwardJunctionHead(data.flag) ||
		isBackwardJunctionHead(data.flag)
}

func (rt *Rtree) IsJunctionTail(data leafData) bool {
	return isForwardJunctionTail(data.flag) ||
		isBackwardJunctionTail(data.flag)
}

func isForwardJunctionHead(flag uint8) bool {
	return flag&JUNCTION_FORWARD_HEAD_FLAG != 0
}

func isBackwardJunctionHead(flag uint8) bool {
	return flag&JUNCTION_BACKWARD_HEAD_FLAG != 0
}

func isForwardJunctionTail(flag uint8) bool {
	return flag&JUNCTION_FORWARD_TAIL_FLAG != 0
}

func isBackwardJunctionTail(flag uint8) bool {
	return flag&JUNCTION_BACKWARD_TAIL_FLAG != 0
}

func isBidirectional(flag uint8) bool {
	return flag&BIDIRECTIONAL != 0
}

func (rt *Rtree) GetEdgeId(data leafData, mode uint8) (da.Index, da.Index) {
	var (
		backEdgeId, forwEdgeId = da.INVALID_EDGE_ID, da.INVALID_EDGE_ID
	)
	switch mode {
	case 0:

		if isForwardJunctionHead(data.flag) {
			forwEdgeId = da.Index(data.edgeId & 0xFFFFFFFF)
		}
		if isBackwardJunctionHead(data.flag) && isBidirectional(data.flag) {
			backEdgeId = da.Index(data.edgeId >> 32)
		}

		return forwEdgeId, backEdgeId
	case 1:
		if isForwardJunctionTail(data.flag) {
			forwEdgeId = da.Index(data.edgeId & 0xFFFFFFFF)
		}
		if isBackwardJunctionTail(data.flag) && isBidirectional(data.flag) {
			backEdgeId = da.Index(data.edgeId >> 32)
		}

		return forwEdgeId, backEdgeId
	default:
		forwEdgeId = da.Index(data.edgeId & 0xFFFFFFFF)
		if isBidirectional(data.flag) {
			backEdgeId = da.Index(data.edgeId >> 32)
		}
		return forwEdgeId, backEdgeId
	}
}

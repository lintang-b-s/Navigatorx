// Package spatialindex provides spatial indexing capabilities using R-trees (menggunakan mercator projected coordinate system).
package spatialindex

import (
	"math"
	"sort"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"

	"github.com/lintang-b-s/rtree"
	"go.uber.org/zap"
)

type leafData struct {
	id   uint64 // (backward direction road segment id << 32) | forward direction road segment id
	flag uint8
}

func newLeafData(id uint64, flag uint8) leafData {
	return leafData{id: id, flag: flag}
}

type Rtree struct {
	tr *rtree.RTreeGN[int32, leafData]
}

const spatialIndexPrecision = 1e5 // mercator projected 2d coordinate

func spatialRound(value float64) int32 {
	return int32(math.Round(value * spatialIndexPrecision))
}

// we need to know nearby road segments s & t before run the query
func NewRtree() *Rtree {
	var tr rtree.RTreeGN[int32, leafData]
	return &Rtree{
		tr: &tr,
	}
}

type segmentVal struct {
	minX, minY int32
	maxX, maxY int32
	id         da.Index // road segment id
	flag       uint8    // flag dari road segment id
}

func newSegmentVal(minX, minY, maxX, maxY int32, id da.Index, flag uint8) segmentVal {
	return segmentVal{minX, minY, maxX, maxY, id, flag}
}

type segmentKey struct {
	osmId uint64
	tail  da.Coordinate
	head  da.Coordinate
}

func newSegmentKey(osmId uint64, tail, head da.Coordinate) segmentKey {
	return segmentKey{osmId: osmId, tail: tail, head: head}
}

// Build. build r-tree
func (rt *Rtree) Build(g *da.Graph, rn *da.RoadNetworkDataContainer, logger *zap.Logger) {
	logger.Info("Building R-tree spatial index...")
	n := g.NumberOfVertices()
	mins := make([][2]int32, 0, n)
	maxs := make([][2]int32, 0, n)
	items := make([]leafData, 0, n)

	segmentSet := make(map[segmentKey]segmentVal, n/10) // segmentKey -> road segment (forward direction), mbr

	g.ForVertices(func(v da.Vertex, segId da.Index) {
		eGeom := rn.GetSegmentGeometry(segId)
		if len(eGeom) < 2 {
			return
		}

		maxLat, maxLon := math.Inf(-1), math.Inf(-1)
		minLat, minLon := math.MaxFloat64, math.MaxFloat64
		for i := 0; i < len(eGeom); i++ {
			point := eGeom[i]
			maxLat = max(maxLat, point.GetLat())
			maxLon = max(maxLon, point.GetLon())
			minLat = min(minLat, point.GetLat())
			minLon = min(minLon, point.GetLon())
		}

		// use mercator projected coordinate
		minY := spatialRound(geo.CalcLatToY(minLat))
		maxY := spatialRound(geo.CalcLatToY(maxLat))
		minX := spatialRound(geo.CalcLonToX(minLon))
		maxX := spatialRound(geo.CalcLonToX(maxLon))

		tail := eGeom[0]
		head := eGeom[len(eGeom)-1]
		segKey := newSegmentKey(rn.GetOsmWayId(segId), tail, head)

		if da.IsSameCoordinate(tail, head) {
			return
		}

		dir := rn.GetStreetDirection(segId)
		f, b := dir[0], dir[1]

		if odSeg, ok := segmentSet[segKey]; ok {

			fId := uint64(segId)
			bId := uint64(odSeg.id)
			flag := rt.getFlag(rn, segId, f, true)
			flag |= odSeg.flag
			if b {
				fId = uint64(odSeg.id)
				bId = uint64(segId)
			}
			mins = append(mins, [2]int32{minX, minY})
			maxs = append(maxs, [2]int32{maxX, maxY})
			packId := (bId << 32) | fId
			leaf := newLeafData(packId, flag)
			items = append(items, leaf)
			delete(segmentSet, segKey)
		} else {
			flag := rt.getFlag(rn, segId, f, false)
			segmentSet[segKey] = newSegmentVal(minX, minY, maxX, maxY, segId, flag)
		}
	})

	for _, seg := range segmentSet {
		// sisa road segments yang oneway=true
		mins = append(mins, [2]int32{seg.minX, seg.minY})
		maxs = append(maxs, [2]int32{seg.maxX, seg.maxY})
		items = append(items, newLeafData(uint64(seg.id), seg.flag))
	}

	rt.tr.Bulk(mins, maxs, items)
	logger.Info("R-tree spatial index built.")
}

type candidate struct {
	id   da.Index
	dist int64
}

// SearchWithinRadius search for all arc endpoints within radius (in km) from the query point (qLat, qLon)
// let M=number of road segmnents in the graph
// R-tree search worst case is O(M) ketika MBR dari query overlap semua mbr leafs data
// kita limit leaf data yang kita return sebanyak MAX_CANDIDATES < maxLeafEntries (64),
// yaitu MAX_CANDIDATES kandidat yang paling dekat ke query point (bukan potongan acak ditengah traversal).
// dan kita pakai STR (sort-tile-recursive) packed r-tree, bulk insert (.Bulk(..)), space utilization nya ~100% every leaf nodes.
// space utilization ~100% -> total area dan total perimeter dari all nodes in r-tree smaller...
// by lemma 3: https://dl.acm.org/doi/pdf/10.1145/170088.170403
// expected number of nodes visited (disk accesses karena di paper r-tree nodes nya disimpan ke disk page) proporsional dengan total area dan total perimeter dari all nodes in r-tree
// karena kita pake bulk insert, total area + total perimeter lebih kecil -> number of nodes visited lebih kecil saat rectangle/bounding box query.
// karena query bounding box kita juga kecil (radius kecil & jauh lebih kecil dari MBR nya root node), jumlah leaf yang overlap (k) juga kecil -> avg case runtime dari rectangle/bounding box query proporsional dengan height dari r-tree: O(logM)
// ref1: https://ia600709.us.archive.org/13/items/nasa_techdoc_19970016975/19970016975.pdf
// ref2: https://xilinx.github.io/Vitis_Libraries/data_analytics/2022.1/guide_L2/internals/geospatialJoin.html
// ref3: https://www2.cs.sfu.ca/CourseCentral/454/jpei/slides/R-Tree.pdf
// ref4: https://dl.acm.org/doi/10.1145/971697.602266
// OSRM static_rtree: https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/static_rtree.hpp
// OSRM pakai packed Hilbert-R-Tree, dengan alasan yang sama dengan diatas. kita pakai packed Sort-Tile-Recursive (STR) R-tree.
// karena di ref1 table 5, STR punya number of disk accesses (nodes visited) yang sedikit lebih kecil dari HS (packed Hilbert-R-Tree) pada graf road network Long Beach Data.
// mode=0  origin, mode=1 destination, mode=2 not both
func (rt *Rtree) SearchWithinRadius(qLat, qLon, radius float64, mode uint8) []da.Index {

	qy, qx := geo.CalcLatToY(qLat), geo.CalcLonToX(qLon)
	qxR, qyR := spatialRound(qx), spatialRound(qy)

	lowerY, lowerX := spatialRound(qy-radius), spatialRound(qx-radius)
	upperY, upperX := spatialRound(qy+radius), spatialRound(qx+radius)

	cands := make([]candidate, 0, 16)

	rt.tr.Search([2]int32{lowerX, lowerY}, [2]int32{upperX, upperY},
		func(min, max [2]int32, data leafData) bool {
			if mode == 0 && !rt.IsJunctionHead(data) {
				// skip road  segment yang head nya gak junction
				return true
			} else if mode == 1 && !rt.IsJunctionTail(data) {
				// skip road segment yang tail nya gak junction
				return true
			}
			midx := (max[0] + min[0]) / 2
			midy := (max[1] + min[1]) / 2
			dx, dy := int64(qxR-midx), int64(qyR-midy)
			dist := dx*dx + dy*dy
			fId, bId := rt.getId(data, mode)
			if fId != da.INVALID_SEGMENT_ID {
				cands = append(cands, candidate{id: fId, dist: dist})
			}
			if bId != da.INVALID_SEGMENT_ID {
				cands = append(cands, candidate{id: bId, dist: dist})
			}
			return true
		})

	sort.Slice(cands, func(i, j int) bool { return cands[i].dist < cands[j].dist })
	if len(cands) > MAX_CANDIDATES {
		cands = cands[:MAX_CANDIDATES]
	}

	res := make([]da.Index, len(cands))
	for i, c := range cands {
		res[i] = c.id
	}
	return res
}

func (rt *Rtree) getFlag(rn *da.RoadNetworkDataContainer, id da.Index, forward, bidir bool) uint8 {
	flag := uint8(0)
	if bidir {
		flag |= BIDIRECTIONAL
	}
	if rn.IsSegmentFlagBitOn(id, da.FlagJunctionHead) {
		switch forward {
		case true:
			flag |= JUNCTION_FORWARD_HEAD_FLAG
		default:
			flag |= JUNCTION_BACKWARD_HEAD_FLAG
		}
	}
	if rn.IsSegmentFlagBitOn(id, da.FlagJunctionTail) {
		switch forward {
		case true:
			flag |= JUNCTION_FORWARD_TAIL_FLAG
		default:
			flag |= JUNCTION_BACKWARD_TAIL_FLAG
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

func isBitOn(flag uint8, mask uint8) bool {
	return flag&mask != 0
}

func isForwardJunctionHead(flag uint8) bool {
	return isBitOn(flag, JUNCTION_FORWARD_HEAD_FLAG)
}

func isBackwardJunctionHead(flag uint8) bool {
	return isBitOn(flag, JUNCTION_BACKWARD_HEAD_FLAG)
}

func isForwardJunctionTail(flag uint8) bool {
	return isBitOn(flag, JUNCTION_FORWARD_TAIL_FLAG)
}

func isBackwardJunctionTail(flag uint8) bool {
	return isBitOn(flag, JUNCTION_BACKWARD_TAIL_FLAG)
}

func isBidirectional(flag uint8) bool {
	return isBitOn(flag, BIDIRECTIONAL)
}

func (rt *Rtree) getId(data leafData, mode uint8) (da.Index, da.Index) {
	var (
		bId, fId = da.INVALID_SEGMENT_ID, da.INVALID_SEGMENT_ID
	)
	switch mode {
	case 0:

		if isForwardJunctionHead(data.flag) {
			fId = da.Index(data.id & 0xFFFFFFFF)
		}
		if isBidirectional(data.flag) && isBackwardJunctionHead(data.flag) {
			bId = da.Index(data.id >> 32)
		}

		return fId, bId
	case 1:
		if isForwardJunctionTail(data.flag) {
			fId = da.Index(data.id & 0xFFFFFFFF)
		}
		if isBidirectional(data.flag) && isBackwardJunctionTail(data.flag) {
			bId = da.Index(data.id >> 32)
		}

		return fId, bId
	default:
		fId = da.Index(data.id & 0xFFFFFFFF)
		if isBidirectional(data.flag) {
			bId = da.Index(data.id >> 32)
		}
		return fId, bId
	}
}

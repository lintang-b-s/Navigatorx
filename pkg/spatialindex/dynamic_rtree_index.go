package spatialindex

import (
	"math"
	"sort"
	"sync/atomic"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/rtree"
)

// // todo: update kode ini

type DynamicRtree struct {
	tr atomic.Pointer[rtree.RTreeGN[int32, uint64]]
}

func NewDynamicRtree() *DynamicRtree {
	var tr rtree.RTreeGN[int32, uint64]
	dr := &DynamicRtree{}
	dr.tr.Store(&tr)
	return dr
}

func (dr *DynamicRtree) Rebuild(g *da.DynamicGraph) {
	m := g.NumVertices()

	mins := make([][2]int32, 0, m)
	maxs := make([][2]int32, 0, m)
	items := make([]uint64, 0, m)

	for segId := da.Index(0); segId < m; segId++ {
		eGeom := g.GetSegmentGeometry(segId)
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
		minY := geo.CalcLatToY(minLat)
		maxY := geo.CalcLatToY(maxLat)
		minX := geo.CalcLonToX(minLon)
		maxX := geo.CalcLonToX(maxLon)

		rnId := g.GetRoadNetworkSegmentId(segId)
		nSegId := dr.Bitpack(da.Index(segId), rnId)

		mins = append(mins, [2]int32{spatialRound(minX), spatialRound(minY)})
		maxs = append(maxs, [2]int32{spatialRound(maxX), spatialRound(maxY)})
		items = append(items, nSegId)
	}

	tr := &rtree.RTreeGN[int32, uint64]{}
	tr.Bulk(mins, maxs, items)
	dr.tr.Store(tr)
}

func (dr *DynamicRtree) SearchWithinRadius(qLat, qLon, radius float64) []da.Index {

	qy, qx := geo.CalcLatToY(qLat), geo.CalcLonToX(qLon)

	lowerY, lowerX := spatialRound(qy-radius), spatialRound(qx-radius)
	upperY, upperX := spatialRound(qy+radius), spatialRound(qx+radius)

	qxR, qyR := spatialRound(qx), spatialRound(qy)

	cands := make([]candidate, 0, 10)

	rt := dr.tr.Load()
	rt.Search([2]int32{lowerX, lowerY}, [2]int32{upperX, upperY},
		func(min, max [2]int32, data uint64) bool {
			segId := dr.GetSegmentId(data)
			midx := (max[0] + min[0]) / 2
			midy := (max[1] + min[1]) / 2
			dx, dy := int64(qxR-midx), int64(qyR-midy)
			dist := dx*dx + dy*dy
			cands = append(cands, candidate{id: segId, dist: dist})
			return true
		})

	sort.Slice(cands, func(i, j int) bool { return cands[i].dist < cands[j].dist })
	if len(cands) > MAX_CANDIDATES_MAP_MATCHING {
		cands = cands[:MAX_CANDIDATES_MAP_MATCHING]
	}
	res := make([]da.Index, len(cands))
	for i, c := range cands {
		res[i] = c.id
	}
	return res
}

func (dr *DynamicRtree) Bitpack(segId da.Index, rnId da.Index) uint64 {
	id := uint64(0)
	id = uint64(segId) | (uint64(rnId) << 32)
	return id
}

func (dr *DynamicRtree) GetSegmentId(id uint64) da.Index {
	return da.Index(id & 0xFFFFFFFF)
}

func (dr *DynamicRtree) GetRoadNetworkSegmentId(id uint64) da.Index {
	return da.Index(id >> 32)
}

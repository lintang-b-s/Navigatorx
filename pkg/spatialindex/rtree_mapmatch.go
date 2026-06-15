package spatialindex

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/rtree"
	"go.uber.org/zap"
)

type RtreeMapMatch struct {
	tr *rtree.RTreeGN[int32, uint64]
}

func NewRtreeMapMatch() *RtreeMapMatch {
	var tr rtree.RTreeGN[int32, uint64]
	return &RtreeMapMatch{
		tr: &tr,
	}
}

func (rt *RtreeMapMatch) BuildMapMatch(graph *da.MapMatchingGraph, logger *zap.Logger) {
	m := graph.NumofEdges()
	if m == 0 {
		return
	}

	mins := make([][2]int32, 0, m)
	maxs := make([][2]int32, 0, m)
	items := make([]uint64, 0, m)

	for eId, e := range graph.GetEdges() {
		eGeom := e.GetGeometry()
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
		minY := geo.CalcLatToY(minLat)
		maxY := geo.CalcLatToY(maxLat)
		minX := geo.CalcLonToX(minLon)
		maxX := geo.CalcLonToX(maxLon)

		newEId := rt.BitPackOriginalEdgeId(da.Index(eId), e.GetRoadNetworkEdgeId())

		mins = append(mins, [2]int32{spatialRound(minX), spatialRound(minY)})
		maxs = append(maxs, [2]int32{spatialRound(maxX), spatialRound(maxY)})
		items = append(items, newEId)
	}

	rt.tr.Bulk(mins, maxs, items)
}

func (rt *RtreeMapMatch) Reset() {
	var tr rtree.RTreeGN[int32, uint64]
	rt.tr = &tr
}

// SearchWithinRadius search for all arc endpoints within radius (in km) from the query point (qLat, qLon)
// let M=number of road segmnents/edges in the graph
// R-tree search worst case is O(M) ketika MBR dari query overlap semua mbr leafs data
// tapi karena kita limit leaf data yang kita return sebanyak MAX_CANDIDATES
// SearchWithinRadius worst case is O(M)
// https://www2.cs.sfu.ca/CourseCentral/454/jpei/slides/R-Tree.pdf
// https://dl.acm.org/doi/10.1145/971697.602266
// mode=0  origin, mode=1 destination, mode=2 not both, mode=3 for client-side realtime mapmatching webassembly
func (rt *RtreeMapMatch) SearchWithinRadius(qLat, qLon, radius float64) []da.Index {

	qy, qx := geo.CalcLatToY(qLat), geo.CalcLonToX(qLon)

	lowerY, lowerX := qy-radius, qx-radius
	upperY, upperX := qy+radius, qx+radius

	results := make([]da.Index, 0, 10)

	rt.tr.Search([2]int32{spatialRound(lowerX), spatialRound(lowerY)}, [2]int32{spatialRound(upperX), spatialRound(upperY)},
		func(min, max [2]int32, data uint64) bool {

			eId := rt.GetMapMatchEdgeId(data)
			results = append(results, eId)
			return len(results) <= MAX_CANDIDATES_MAP_MATCHING

		})
	return results
}

func (rt *RtreeMapMatch) BitPackOriginalEdgeId(eId da.Index, originalEId da.Index) uint64 {
	newId := uint64(0)
	newId = uint64(eId) | (uint64(originalEId) << 32)
	return newId
}

func (rt *RtreeMapMatch) GetMapMatchEdgeId(id uint64) da.Index {
	return da.Index(id & 0xFFFFFFFF)
}

func (rt *RtreeMapMatch) GetRoadNetworkEdgeId(id uint64) da.Index {
	return da.Index(id >> 32)
}

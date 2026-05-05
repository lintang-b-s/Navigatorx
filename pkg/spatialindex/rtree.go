// Package spatialindex provides spatial indexing capabilities using R-trees (menggunakan mercator projected coordinate system).
package spatialindex

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"

	"github.com/tidwall/rtree"
	"go.uber.org/zap"
)

type Rtree struct {
	tr *rtree.RTreeG[uint64]
}

// our query is compact graph CRP multilevel bidirectional dijkstra
// query input = (as,s, at,t)
// as = entryOffset of source
// at = exitOffset of target
// so we need to know nearby as & at before run the query

func NewRtree() *Rtree {
	var tr rtree.RTreeG[uint64]
	return &Rtree{
		tr: &tr,
	}
}

// Build. build r-tree
func (rt *Rtree) Build(graph *da.Graph, logger *zap.Logger) {
	logger.Info("Building R-tree spatial index...")
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
		minY := geo.CalcLatToY(minLat)
		maxY := geo.CalcLatToY(maxLat)
		minX := geo.CalcLonToX(minLon)
		maxX := geo.CalcLonToX(maxLon)

		id := rt.addFlag(graph, eId)
		rt.tr.Insert([2]float64{minX, minY}, [2]float64{maxX, maxY},
			id)
	})

	logger.Info("R-tree spatial index built.")
}

func (rt *Rtree) BuildMapMatch(graph *da.MapMatchingGraph, logger *zap.Logger) {
	logger.Info("Building R-tree spatial index for map matching...")
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

		newEId := rt.BitPackOriginalEdgeId(da.Index(eId), e.GetOriginalEdgeId())
		rt.tr.Insert([2]float64{minX, minY}, [2]float64{maxX, maxY}, newEId)
	}
	logger.Info("R-tree spatial index for map matching built.")
}

func (rt *Rtree) Reset() {
	var tr rtree.RTreeG[uint64]
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
func (rt *Rtree) SearchWithinRadius(qLat, qLon, radius float64, mode uint8) []da.Index {

	lowerLat, lowerLon := geo.GetDestinationPoint(qLat, qLon, 225, radius)
	upperLat, upperLon := geo.GetDestinationPoint(qLat, qLon, 45, radius)

	lowerY, lowerX := geo.CalcLatToYApprox(lowerLat), geo.CalcLonToX(lowerLon)
	upperY, upperX := geo.CalcLatToYApprox(upperLat), geo.CalcLonToX(upperLon)

	results := make([]da.Index, 0, 10)
	rt.tr.Search([2]float64{lowerX, lowerY}, [2]float64{upperX, upperY},
		func(min, max [2]float64, data uint64) bool {
			if mode == 0 && !rt.IsJunctionHead(data) {
				// skip edge yang head nya gak junction
				return true
			} else if mode == 1 && !rt.IsJunctionTail(data) {
				// skip edge yang tail nya gak junction
				return true
			}
			var eId da.Index
			if mode == 3 {
				eId = rt.GetMapMatchEdgeId(data)
			} else {
				eId = rt.GetEdgeId(data)
			}

			results = append(results, eId)
			return len(results) <= MAX_CANDIDATES
		})
	return results
}

func (rt *Rtree) addFlag(graph *da.Graph, eId da.Index) uint64 {
	id := uint64(eId)
	if graph.IsJunctionHead(eId) {
		id |= JUNCTION_HEAD_FLAG
	}
	if graph.IsJunctionTail(eId) {
		id |= JUNCTION_TAIL_FLAG
	}
	return id
}

func (rt *Rtree) IsJunctionHead(id uint64) bool {
	return id&JUNCTION_HEAD_FLAG != 0
}

func (rt *Rtree) IsJunctionTail(id uint64) bool {
	return id&JUNCTION_TAIL_FLAG != 0
}

func (rt *Rtree) GetEdgeId(id uint64) da.Index {
	id &^= JUNCTION_HEAD_FLAG
	id &^= JUNCTION_TAIL_FLAG
	return da.Index(id)
}

func (rt *Rtree) BitPackOriginalEdgeId(eId da.Index, originalEId da.Index) uint64 {
	newId := uint64(0)
	newId = uint64(eId) | (uint64(originalEId) << 32)
	return newId
}

func (rt *Rtree) GetMapMatchEdgeId(id uint64) da.Index {
	return da.Index(id & 0xFFFFFFFF)
}

func (rt *Rtree) GetOriginalEdgeId(id uint64) da.Index {
	return da.Index(id >> 32)
}

package spatialindex

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/tidwall/rtree"
	"go.uber.org/zap"
)

type Rtree struct {
	tr *rtree.RTreeG[ArcEndpoint]
}

// our query is turn-based multilevel bidirectional dijkstra
// query input = (as,s, at,t)
// as = entryOffset of source
// at = exitOffset of target
// so we need to know nearby as & at before run the query
type ArcEndpoint struct {
	id da.Index
}

func (ae ArcEndpoint) GetId() da.Index {
	return ae.id
}

func newArcEndpoint(id da.Index) ArcEndpoint {
	return ArcEndpoint{
		id: id,
	}
}

func NewRtree() *Rtree {
	var tr rtree.RTreeG[ArcEndpoint]
	return &Rtree{
		tr: &tr,
	}
}

// Build. build r-tree, with each leaf having bounding box with radius boundingBoxRadius (in km)
func (rt *Rtree) Build(graph *da.Graph, boundingBoxRadius float64, log *zap.Logger) {
	log.Info("Building R-tree spatial index...")
	graph.ForOutEdges(func(e *da.OutEdge, exitPoint, head, tail, entryId da.Index,
		percentage float64, id da.Index) {

		if da.SkipDummyEdge(e) {
			return
		}

		eGeom := graph.GetEdgeGeometry(id)
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

		minLat, minLon = geo.GetDestinationPoint(minLat, minLon, 225, boundingBoxRadius)
		maxLat, maxLon = geo.GetDestinationPoint(maxLat, maxLon, 45, boundingBoxRadius)

		// use web mercator projected coordinate
		minY := geo.CalcLatToY(minLat)
		maxY := geo.CalcLatToY(maxLat)
		minX := geo.CalcLonToX(minLon)
		maxX := geo.CalcLonToX(maxLon)

		rt.tr.Insert([2]float64{minX, minY}, [2]float64{maxX, maxY},
			newArcEndpoint(id))
	})

	log.Info("R-tree spatial index built.")
}

// SearchWithinRadius search for all arc endpoints within radius (in km) from the query point (qLat, qLon)
// let M=number of road segmnents/edges in the graph
// R-tree search worst case is O(M), avg case is O(logM)
// https://www2.cs.sfu.ca/CourseCentral/454/jpei/slides/R-Tree.pdf
// https://dl.acm.org/doi/10.1145/971697.602266
func (rt *Rtree) SearchWithinRadius(qLat, qLon, radius float64) []ArcEndpoint {
	lowerLat, lowerLon := geo.GetDestinationPoint(qLat, qLon, 225, radius)
	upperLat, upperLon := geo.GetDestinationPoint(qLat, qLon, 45, radius)

	lowerY, lowerX := geo.CalcLatToYApprox(lowerLat), geo.CalcLonToX(lowerLon)
	upperY, upperX := geo.CalcLatToYApprox(upperLat), geo.CalcLonToX(upperLon)

	results := make([]ArcEndpoint, 0, 10)
	rt.tr.Search([2]float64{lowerX, lowerY}, [2]float64{upperX, upperY},
		func(min, max [2]float64, data ArcEndpoint) bool {
			results = append(results, data)
			if len(results) > MAX_CANDIDATES {
				return false
			}
			return true
		})
	return results
}

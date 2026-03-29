package spatialindex

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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

		from := tail
		to := head

		fromLat, fromLon := graph.GetVertexCoordinates(from)
		toLat, toLon := graph.GetVertexCoordinates(to)
		lowerFromLat, lowerFromLon := geo.GetDestinationPoint(fromLat, fromLon, 225, boundingBoxRadius)
		upperFromLat, upperFromLon := geo.GetDestinationPoint(fromLat, fromLon, 45, boundingBoxRadius)

		lowerToLat, lowerToLon := geo.GetDestinationPoint(toLat, toLon, 225, boundingBoxRadius)
		upperToLat, upperToLon := geo.GetDestinationPoint(toLat, toLon, 45, boundingBoxRadius)

		// use web mercator projected coordinate
		lowerFromY := geo.CalcLatToYApprox(lowerFromLat)
		upperFromY := geo.CalcLatToYApprox(upperFromLat)
		lowerFromX := geo.CalcLonToX(lowerFromLon)
		upperFromX := geo.CalcLonToX(upperFromLon)

		lowerToY := geo.CalcLatToYApprox(lowerToLat)
		upperToY := geo.CalcLatToYApprox(upperToLat)
		lowerToX := geo.CalcLonToX(lowerToLon)
		upperToX := geo.CalcLonToX(upperToLon)

		minY := math.Min(lowerFromY, lowerToY)
		minX := math.Min(lowerFromX, lowerToX)
		maxY := util.MaxFloat(upperFromY, upperToY)
		maxX := util.MaxFloat(upperFromX, upperToX)

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

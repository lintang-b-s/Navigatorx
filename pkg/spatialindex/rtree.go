package spatialindex

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
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
	entryOffset datastructure.Index
	exitOffset  datastructure.Index
}

func (ae ArcEndpoint) GetExitOffset() datastructure.Index {
	return ae.exitOffset
}

func (ae ArcEndpoint) GetEntryOffset() datastructure.Index {
	return ae.entryOffset
}

func newArcEndpoint(exitOffset, entryOffset datastructure.Index) ArcEndpoint {
	return ArcEndpoint{
		exitOffset:  exitOffset,
		entryOffset: entryOffset,
	}
}

func NewRtree() *Rtree {
	var tr rtree.RTreeG[ArcEndpoint]
	return &Rtree{
		tr: &tr,
	}
}

// Build. build r-tree, with each leaf having bounding box with radius boundingBoxRadius (in km)
func (rt *Rtree) Build(graph *datastructure.Graph, boundingBoxRadius float64, log *zap.Logger) {
	log.Info("Building R-tree spatial index...")
	graph.ForOutEdges(func(e *datastructure.OutEdge, exitPoint, head, tail, entryOffset datastructure.Index,
		percentage float64) {
		if math.Mod(percentage, 10) < 0.0001 {
			log.Info("Building R-tree spatial index...", zap.Float64("progress", percentage))
		}
		from := tail
		to := head

		fromLat, fromLon := graph.GetVertexCoordinates(from)
		toLat, toLon := graph.GetVertexCoordinates(to)
		lowerFromLat, lowerFromLon := geo.GetDestinationPoint(fromLat, fromLon, 225, boundingBoxRadius)
		upperFromLat, upperFromLon := geo.GetDestinationPoint(fromLat, fromLon, 45, boundingBoxRadius)

		lowerToLat, lowerToLon := geo.GetDestinationPoint(toLat, toLon, 225, boundingBoxRadius)
		upperToLat, upperToLon := geo.GetDestinationPoint(toLat, toLon, 45, boundingBoxRadius)

		minLat := math.Min(lowerFromLat, lowerToLat)
		minLon := math.Min(lowerFromLon, lowerToLon)
		maxLat := math.Max(upperFromLat, upperToLat)
		maxLon := math.Max(upperFromLon, upperToLon)

		exitOffset := exitPoint + graph.GetExitOffset(tail)

		rt.tr.Insert([2]float64{minLon, minLat}, [2]float64{maxLon, maxLat},
			newArcEndpoint(exitOffset, entryOffset))
	})

	log.Info("R-tree spatial index built.")
}

// SearchWithinRadius search for all arc endpoints within radius (in km) from the query point (qLat, qLon)
func (rt *Rtree) SearchWithinRadius(qLat, qLon, radius float64) []ArcEndpoint {
	lowerLat, lowerLon := geo.GetDestinationPoint(qLat, qLon, 225, radius)
	upperLat, upperLon := geo.GetDestinationPoint(qLat, qLon, 45, radius)

	results := make([]ArcEndpoint, 0, 10)
	rt.tr.Search([2]float64{lowerLon, lowerLat}, [2]float64{upperLon, upperLat},
		func(min, max [2]float64, data ArcEndpoint) bool {
			results = append(results, data)
			if len(results) >= 20 {
				return false
			}
			return true
		})
	return results
}

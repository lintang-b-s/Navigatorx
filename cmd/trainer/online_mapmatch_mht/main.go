package main

import (
	"flag"
	"fmt"
	"os"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"golang.org/x/exp/rand"
)

var (
	leafBoundingBoxRadius = flag.Float64("leaf_bounding_box_radius", 0.05, "leaf node (r-tree) bounding box radius in km")
)

func main() {
	flag.Parse()
	logger, err := logger.New()
	if err != nil {
		panic(err)
	}
	rand.Seed(uint64(time.Now().UnixNano()))
	routingEngine, err := engine.NewEngine("./data/original.graph", "./data/overlay_graph.graph", "./data/metrics.txt", logger)
	if err != nil {
		panic(err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(routingEngine.GetRoutingEngine().GetGraph(), *leafBoundingBoxRadius, logger)
	graph, err := datastructure.ReadGraph("./data/original.graph")
	if err != nil {
		panic(err)
	}
	var N *datastructure.SparseMatrix[int]
	_, err = os.Stat("./data/omm_transition_history_id.mm")
	if err == nil {
		logger.Info("reading transition matrix from file...")

		N, err = datastructure.ReadSparseMatrixFromFile[int]("./data/omm_transition_history_id.mm", int(0),
			func(a, b int) bool { return a == b })
		if err != nil {
			panic(err)
		}
	} else {
		N = datastructure.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(),
			0, func(a, b int) bool { return a == b })
	}

	routingService := usecases.NewRoutingService(logger, routingEngine.GetRoutingEngine(), rtree, 0.04, true, true,
		0.8, 0.25, 0.25, 1.3)

	boundingBox := graph.GetBoundingBox()
	for i := 0; i < 1e5; i++ {
		if (i+1)%5e3 == 0 {
			fmt.Printf("completed query: %v\n", i+1)
			N.WriteToFile("./data/omm_transition_history_id.mm")
		}
		src := RandomCoordinate(boundingBox)
		dst := RandomCoordinate(boundingBox)
		as, at, err := routingService.SnapOrigDestToNearbyEdges(src.GetLat(), src.GetLon(), dst.GetLat(), dst.GetLon())
		// as = exit/outEdge index of origin
		// at = entry/inEdge index of destination
		if err != nil {
			continue
		}

		crpQuery := routing.NewCRPBidirectionalSearch(routingService.GetEngine().(*routing.CRPRoutingEngine), 1.0)
		_, _, _, edgePath, found := crpQuery.ShortestPathSearch(as, at)
		if !found {
			continue
		}

		for j := 0; j < len(edgePath)-1; j++ {
			e := int(edgePath[j].GetEdgeId())
			eNext := int(edgePath[j+1].GetEdgeId())
			N.Set(N.Get(e, eNext)+1, e, eNext)
		}
	}

}

func RandomCoordinate(bb *datastructure.BoundingBox) datastructure.Coordinate {

	lat := bb.GetMinLat() + rand.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rand.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return datastructure.NewCoordinate(lat, lon)
}

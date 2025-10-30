package main

// import (
// 	"context"
// 	"flag"

// 	"github.com/lintang-b-s/Navigatorx/pkg/engine"
// 	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
// 	"github.com/lintang-b-s/Navigatorx/pkg/logger"
// 	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
// )

// var (
// 	leafBoundingBoxRadius = flag.Float64("leaf_bounding_box_radius", 0.05, "leaf node (r-tree) bounding box radius in km")
// )

// func main() {

// 	flag.Parse()
// 	logger, err := logger.New()
// 	if err != nil {
// 		panic(err)
// 	}

// 	routingEngine, err := engine.NewEngine("./data/original.graph", "./data/overlay_graph.graph", "./data/metrics.txt", logger)
// 	if err != nil {
// 		panic(err)
// 	}

// 	rtree := spatialindex.NewRtree()
// 	rtree.Build(routingEngine.GetRoutingEngine().GetGraph(), *leafBoundingBoxRadius, logger)

// 	crp := routing.NewCRPBidirectionalSearch(routingEngine.GetRoutingEngine())

// }

// func NewContext() (context.Context, func(), error) {
// 	ctx, cancel := context.WithCancel(context.Background())
// 	cb := func() {
// 		cancel()
// 	}

// 	return ctx, cb, nil
// }

package main

import (
	"context"
	"flag"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"go.uber.org/zap"
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

	N, err := datastructure.ReadSparseMatrixFromFile[int]("./data/omm_transition_history_id.mm", int(0),
		func(a, b int) bool { return a == b })
	if err != nil {
		panic(err)
	}
	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(graph, rtree, 500.0, 500.0, 0.001, 4.07, 1.0/60.0, 0.00001,
		0.06, 180.0, N) // speed in meter/minute, default sampling interval 1.0 seconds (using seatle dataset)

	api := http.NewServer(logger)

	routingService := usecases.NewRoutingService(logger, routingEngine.GetRoutingEngine(), rtree, 0.04, true, true,
		0.8, 0.25, 0.25, 1.3, 0.1)
	mapmatcherService := usecases.NewMapMatcherService(logger, onlineMapMatcherEngine)
	ctx, cleanup, err := NewContext()
	if err != nil {
		panic(err)
	}
	api.Use(ctx,
		logger, false, routingService, mapmatcherService)

	signal := http.GracefulShutdown()

	logger.Info("Navigatorx Routing Engine Server Stopped", zap.String("signal", signal.String()))
	cleanup()
}

func NewContext() (context.Context, func(), error) {
	ctx, cancel := context.WithCancel(context.Background())
	cb := func() {
		cancel()
	}

	return ctx, cb, nil
}

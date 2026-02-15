package main

import (
	"context"
	"flag"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"go.uber.org/zap"
)

var (
	leafBoundingBoxRadius = flag.Float64("leaf_bounding_box_radius", 0.05, "leaf node (r-tree) bounding box radius in km")
	transitionMHTFile     = flag.String("transmht_file", "./data/omm_transition_history_id.mm", "transition matrix for online map-matching Multiple Hypothesis Technique filepath")
	cpuprofile            = flag.String("cpuprofile", "", "write cpu profile to file")
)

const (
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
	landmarkFile     string = "./data/landmark.lm"
)

func main() {

	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	lm, err := landmark.ReadLandmark(landmarkFile)
	if err != nil {
		panic(err)
	}

	routingEngine, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		panic(err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(routingEngine.GetRoutingEngine().GetGraph(), *leafBoundingBoxRadius, logger)
	graph, err := da.ReadGraph(graphFile)
	if err != nil {
		panic(err)
	}

	N, err := da.ReadSparseMatrixFromFile[int](*transitionMHTFile, int(0),
		func(a, b int) bool { return a == b })
	if err != nil {
		panic(err)
	}
	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(graph, rtree, 500.0, 500.0, 0.001, 4.07, 1.0/60.0, 0.00001,
		0.06, 180.0, N) // speed in meter/minute, default sampling interval 1.0 seconds (using seatle dataset)

	api := http.NewServer(logger)

	routingService := usecases.NewRoutingService(logger, routingEngine.GetRoutingEngine(), rtree, 0.04, true, true,
		0.8, 0.3, 0.35, 1.35, 0.1, lm)
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

package main

import (
	"context"
	"flag"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"go.uber.org/zap"
)

var (
	leafBoundingBoxRadius = flag.Float64("leaf_bounding_box_radius", 0.05, "leaf node (r-tree) bounding box radius in km")
	timeDependent         = flag.Bool("time_dependent", true, "Use Time-Dependent Customizable Route Planning")
	transitionMHTFile     = flag.String("transmht_file", "./data/omm_transition_history_id.mm", "transition matrix for online map-matching Multiple Hypothesis Technique filepath")
)

const (
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	tgprParser := osmparser.NewTPGRParser()

	_, osmWayProfile, err := tgprParser.ParseTGPRFile("./data/NY/NY.coordinate", "./data/NY/NY.tpgr",
		logger)
	routingEngine, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger, *timeDependent, osmWayProfile)
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
		0.8, 0.25, 0.25, 1.3, 0.1, *timeDependent, nil)
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

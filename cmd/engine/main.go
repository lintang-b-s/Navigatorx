package main

import (
	"context"
	"flag"
	"fmt"
	"path/filepath"
	"runtime"
	"strings"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

var (
	profileFilePath = flag.String("profile", "./data/car.yaml", "profile file path")
)

var (
	profileName      string = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile        string = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, profileName)
	overlayGraphFile string = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, profileName)
	metricsFile      string = fmt.Sprintf("./data/profiles/%s/%s_metrics.txt", profileName, profileName)
	landmarkFile     string = fmt.Sprintf("./data/profiles/%s/%s_landmark.lm", profileName, profileName)
)

const (
	transitionMHTFile string = "./data/omm_transition_history_id.mm"
)

func init() {
	flag.Parse()
	util.InitProfileConfig(profileName)
}

func main() {
	runtime.GOMAXPROCS(runtime.NumCPU())

	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	routingEngine, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, logger)
	if err != nil {
		panic(err)
	}
	re := routingEngine.GetRoutingEngine()

	rtree := spatialindex.NewRtree()
	rtree.Build(routingEngine.GetRoutingEngine().GetGraph(), logger)

	N, err := da.ReadSparseMatrixFromFile[int](transitionMHTFile, int(0),
		func(a, b int) bool { return a == b })
	if err != nil {
		panic(err)
	}

	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(re.GetGraph(), rtree, 8.33333, 8.3333, 0.0001, 4.07, 1.0, 0.0000001,
		0.06, 3, N) // speed in meter/s, default sampling interval 1.0 seconds (using seatle dataset)

	api := http.NewServer(logger)

	altSearch := routing.NewAlternativeRouteSearch(re)
	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true, true)
	if err != nil {
		panic(err)
	}

	mapmatcherService := usecases.NewMapMatcherService(logger, onlineMapMatcherEngine)
	ctx, cleanup, err := NewContext(re, routingService)
	if err != nil {
		panic(err)
	}

	util.CleanHeap()

	api.Use(ctx,
		logger, false, routingService, mapmatcherService)

	signal := http.GracefulShutdown()

	logger.Info("Navigatorx Routing Engine Server Stopped", zap.String("signal", signal.String()))
	cleanup()
}

func NewContext(re *routing.CRPRoutingEngine, rs *usecases.RoutingService) (context.Context, func(), error) {
	ctx, cancel := context.WithCancel(context.Background())
	cb := func() {
		re.Close()
		rs.Close()
		cancel()
	}

	return ctx, cb, nil
}

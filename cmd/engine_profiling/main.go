package main

import (
	"context"
	"flag"
	"os"
	"runtime"

	goHttp "net/http"

	_ "net/http/pprof"

	"github.com/grafana/pyroscope-go"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/offline"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"

	"go.uber.org/zap"
)

// ini sama kaya cmd/engine, cuma ditambahin pyroscope client, buat profiling query engine

var (
	leafBoundingBoxRadius = flag.Float64("leaf_bounding_box_radius", 0.06, "leaf node (r-tree) bounding box radius in km")
	transitionMHTFile     = flag.String("transmht_file", "./data/omm_transition_history_id.mm", "transition matrix for online map-matching Multiple Hypothesis Technique filepath")
	cpuprofile            = flag.String("cpuprofile", "", "write cpu profile to file")
)

const (
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
	landmarkFile     string = "./data/landmark.lm"
)

// constants buat alternative routes
const (
	alpha      = 0.25 // every subpath P' of alternative route with l(P') <= T = \alpha* l(Opt) is optimal (shortest path). l(Opt) is the cost/travel time of the shortest path
	gamma      = 0.8  // alternative routes at least 20% different than the shortest path
	epsilon    = 0.25 // alternative routes at most 25% longer than the shortest path
	upperBound = 1.25 // stop search when sum dari priority kedua node (dengan priority minimum) dari priority queue forward and backward search of multilevel-dijkstra (MLD) > \mu * upperBound
)

func main() {
	runtime.GOMAXPROCS(runtime.NumCPU())

	pyroscope.Start(pyroscope.Config{
		ApplicationName: "navigatorx.golang.app",

		// replace this with the address of pyroscope server
		ServerAddress: "http://localhost:4040",

		// you can disable logging by setting this to nil
		Logger: pyroscope.StandardLogger,

		// by default all profilers are enabled,
		// but you can select the ones you want to use:
		ProfileTypes: []pyroscope.ProfileType{
			pyroscope.ProfileCPU,
			pyroscope.ProfileAllocObjects,
			pyroscope.ProfileAllocSpace,
			pyroscope.ProfileInuseObjects,
			pyroscope.ProfileInuseSpace,
		},
	})

	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	go func() { // pprof
		goHttp.ListenAndServe("localhost:6868", nil)
	}()

	workingDir, err := os.Getwd()
	err = util.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}

	routingEngine, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, logger)
	if err != nil {
		panic(err)
	}
	re := routingEngine.GetRoutingEngine()

	rtree := spatialindex.NewRtree()
	rtree.Build(routingEngine.GetRoutingEngine().GetGraph(), *leafBoundingBoxRadius, logger)

	N, err := da.ReadSparseMatrixFromFile[int](*transitionMHTFile, int(0),
		func(a, b int) bool { return a == b })
	if err != nil {
		panic(err)
	}

	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(re.GetGraph(), rtree, 8.33333, 8.3333, 0.0001, 4.07, 1.0, 0.0000001,
		0.06, 3, N) // speed in meter/s, default sampling interval 1.0 seconds (using seatle dataset)

	offlineMapMatcherEngine := offline.NewHiddenMarkovModelMapMatching(re.GetGraph(), routingEngine, rtree) // speed in meter/minute, default sampling interval 1.0 seconds (using seatle dataset)

	api := http.NewServer(logger)

	altSearch := routing.NewAlternativeRouteSearch(re)
	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true, true)
	if err != nil {
		panic(err)
	}

	mapmatcherService := usecases.NewMapMatcherService(logger, onlineMapMatcherEngine, offlineMapMatcherEngine)
	ctx, cleanup, err := NewContext(re, routingService)
	if err != nil {
		panic(err)
	}

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

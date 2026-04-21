package main

import (
	"flag"
	"fmt"
	"path/filepath"
	"runtime"
	"strings"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

var (
	profileFilePath        = flag.String("profile", "./data/car.yaml", "profile file path")
	profileName            string
	regionName             = flag.String("region", "diy_solo_semarang", "region name")
	graphFile              string
	overlayGraphFile       string
	metricsFile            string
	landmarkFile           string
	httpPort               = flag.Int("http_port", 6060, "http port")
	websocketPort          = flag.Int("websocket_port", 6666, "websocket port")
	proxyPort              = flag.Int("proxy_port", 6767, "proxy port")
	gracefulShutdownPeriod = flag.Int("graceful_shutdown_period", 3, "graceful shutdown period") // see https://victoriametrics.com/blog/go-graceful-shutdown/
)

const (
	transitionMHTFile string = "./data/omm_transition_history_id.mm"
)

func init() {
	flag.Parse()
	viper.Set("http_port", *httpPort)
	viper.Set("websocket_port", *websocketPort)
	viper.Set("proxy_port", *proxyPort)

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, *regionName)
	landmarkFile = fmt.Sprintf("./data/profiles/%s/%s_landmark.lm", profileName, *regionName)
	metricsFile = fmt.Sprintf("./data/profiles/%s/%s_metrics.txt", profileName, *regionName)
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
	rtree.Build(re.GetGraph(), logger)

	N, err := da.ReadSparseMatrixFromFile[int](transitionMHTFile, int(0),
		func(a, b int) bool { return a == b })
	if err != nil {
		panic(err)
	}

	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(re.GetGraph(), rtree, 8.33333, 8.3333, 0.0001, 4.07, 1.0, 0.0000001,
		0.06, 3, N) // speed in meter/s, default sampling interval 1.0 seconds (using seatle dataset)

	api := http.NewServer(logger)

	altSearch := routing.NewAlternativeRouteSearch(re)

	leftHandTraffic := viper.GetBool("guidance.left_hand")
	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, leftHandTraffic)
	if err != nil {
		panic(err)
	}

	mapmatcherService := usecases.NewMapMatcherService(logger, onlineMapMatcherEngine)

	util.CleanHeap()

	shutdownPeriod := time.Duration(*gracefulShutdownPeriod)

	serverErr := api.Use(
		logger, false, routingService, mapmatcherService, shutdownPeriod*time.Second)

	if serverErr != nil {
		logger.Error("server exited unexpectedly", zap.Error(err))
	}
	logger.Info("Navigatorx Routing Engine Server Stopped")
}

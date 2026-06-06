package main

import (
	"flag"
	"fmt"
	"path/filepath"
	"runtime"
	"strings"
	"time"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/tiler"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	http_router "github.com/lintang-b-s/Navigatorx/pkg/http/router"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

var (
	profileFilePath  = flag.String("profile", "./data/car.yaml", "profile file path")
	profileName      string
	regionName       = flag.String("region", "diy_solo_semarang", "region name")
	graphFile        string
	overlayGraphFile string
	metricsFile      string
	landmarkFile     string
	timeFunctionFile string
	httpPort         = flag.Int("http-port", 6060, "http port")

	gracefulShutdownPeriod = flag.Int("graceful-shutdown-period", 3, "graceful shutdown period") // see https://victoriametrics.com/blog/go-graceful-shutdown/
	useRateLimiter         = flag.Bool("rate-limit", false, "use rate limiter")
	rateLimitParam         = flag.String("rate-limit-param", "6,10", "rate limit parameters qps,burst")
)

func init() {
	flag.Parse()
	viper.Set("http_port", *httpPort)

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.ngraph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.ngraph", profileName, *regionName)
	landmarkFile = fmt.Sprintf("./data/profiles/%s/%s_landmark.nlm", profileName, *regionName)
	metricsFile = fmt.Sprintf("./data/profiles/%s/%s_metrics.nmt", profileName, *regionName)
	timeFunctionFile = fmt.Sprintf("./data/profiles/%s/%s_timefunction.ntf", profileName, *regionName)

	config.InitProfileConfig(profileName, *regionName)

	if *rateLimitParam != "" {
		var q, b int
		_, err := fmt.Sscanf(*rateLimitParam, "%d,%d", &q, &b)
		if err != nil {
			panic(fmt.Sprintf("invalid rate-limit-param format: %s. expected 'qps,burst' (e.g., '6,10')", *rateLimitParam))
		}
		http_router.SetRateLimit(q, b)
	}

	gopool.SetCap(int32(runtime.NumCPU()))
}

func main() {

	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	eng, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		panic(err)
	}
	re := eng.GetRoutingEngine()

	rtree := spatialindex.NewRtree()
	rtree.Build(re.GetGraph(), logger)

	graph := re.GetGraph()

	api := http.NewServer(logger)

	altSearch := routing.NewAlternativeRouteSearch(re)

	leftHandTraffic := viper.GetBool("guidance.left_hand")
	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, leftHandTraffic)
	if err != nil {
		panic(err)
	}

	cf := re.GetCostFunction()
	shutdownPeriod := time.Duration(*gracefulShutdownPeriod)
	tilingEngine := tiler.NewTilingEngine(graph, logger, cf)
	tilingService := usecases.NewTileService(logger, tilingEngine)

	serverErr := api.Use(
		logger, *useRateLimiter, routingService, tilingService, shutdownPeriod*time.Second)

	if serverErr != nil {
		logger.Error("server exited unexpectedly", zap.Error(err))
	}
	logger.Info("Navigatorx Routing Engine Server Stopped")

}

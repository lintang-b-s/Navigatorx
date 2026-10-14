package main

import (
	"flag"
	"fmt"
	"path/filepath"
	"runtime"
	"strings"
	"time"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapattributes"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http"
	http_router "github.com/lintang-b-s/Navigatorx/pkg/http/router"
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
	httpPort               = flag.Int("http-port", 6060, "http port")
	gracefulShutdownPeriod = flag.Int("graceful-shutdown-period", 3, "graceful shutdown period") // see https://victoriametrics.com/blog/go-graceful-shutdown/
	useRateLimiter         = flag.Bool("rate-limit", false, "use rate limiter")
	rateLimitParam         = flag.String("rate-limit-param", "6,10", "rate limit parameters qps,burst")
	spIndexRadius          = flag.Float64("spatial-index-search-radius", 0.04, "search radius for spatial index")
)

func init() {
	flag.Parse()
	viper.Set("http_port", *httpPort)
	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	config.InitProfileConfig(profileName, *regionName, pkg.ROUTER)

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

	eng, err := engine.NewEngine[int32](logger)
	if err != nil {
		panic(err)
	}
	re := eng.GetRoutingEngine()
	rtree := spatialindex.NewRtree()
	graph := re.GetGraph()
	rn := re.GetRoadNetworkContainer()

	rtree.Build(re.GetGraph(), rn, logger)

	api := http.NewServer(logger)

	altSearch := routing.NewAlternativeRouteSearch(re)

	leftHandTraffic := viper.GetBool("guidance.left_hand")
	routingService, err := usecases.NewRoutingService(logger, re, rn, rtree, altSearch, *spIndexRadius, leftHandTraffic)
	if err != nil {
		panic(err)
	}

	met := re.GetMetrics()
	shutdownPeriod := time.Duration(*gracefulShutdownPeriod)
	s2Index, err := spatialindex.ReadS2RoadSegmentsIndexFromFile()
	if err != nil {
		panic(err)
	}

	mapAttributesEngine := mapattributes.NewMapAttributesEngine(graph, rn, logger, met, s2Index)
	mapAttributesService := usecases.NewMapAttributesService(logger, mapAttributesEngine)
	util.FreeMemory()
	serverErr := api.Use(
		logger, *useRateLimiter, routingService, mapAttributesService, shutdownPeriod*time.Second)

	if serverErr != nil {
		logger.Error("server exited unexpectedly", zap.Error(err))
	}
	logger.Info("Navigatorx Routing Engine Server Stopped")

}

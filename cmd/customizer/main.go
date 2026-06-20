package main

import (
	"fmt"
	"path/filepath"
	"runtime"
	"strings"

	"github.com/bytedance/gopkg/util/gopool"
	flag "github.com/spf13/pflag"

	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
)

var (
	profileName       string
	profileFilePath   = flag.String("profile", "./data/car.yaml", "profile file path")
	regionName        = flag.String("region", "diy_solo_semarang", "region name")
	edgeSpeedsFile    = flag.StringSlice("segment-speed-file", []string{}, "segment speed csv file. example usage --segment-speed-file=blokade.csv,traffic_solo.csv")
	turnPenaltiesFile = flag.StringSlice("turn-penalty-file", []string{}, "turn penaltiy csv file. example usage --turn-penalty-file=tutup_portal.csv")
	graphFile         string
	overlayGraphFile  string
	metricsFile       string
	landmarkFile      string
	timeFunctionFile  string
)

func init() {
	flag.Parse()

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.ngraph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.ngraph", profileName, *regionName)
	landmarkFile = fmt.Sprintf("./data/profiles/%s/%s_landmark.nlm", profileName, *regionName)
	metricsFile = fmt.Sprintf("./data/profiles/%s/%s_metrics.nmt", profileName, *regionName)
	timeFunctionFile = fmt.Sprintf("./data/profiles/%s/%s_timefunction.ntf", profileName, *regionName)

	config.InitProfileConfig(profileName, *regionName)

	gopool.SetCap(int32(runtime.NumCPU()))
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	custom := customizer.NewCustomizer[int32](graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)
	custom.SetEdgeSpeedsFilePath(*edgeSpeedsFile)
	custom.SetTurnPenaltiesFilePath(*turnPenaltiesFile)

	_, err = custom.Customize()
	if err != nil {
		panic(err)
	}
}

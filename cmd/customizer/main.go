package main

import (
	"fmt"
	"path/filepath"
	"strings"

	flag "github.com/spf13/pflag"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	profileName     string
	profileFilePath = flag.String("profile", "./data/car.yaml", "profile file path")
	regionName      = flag.String("region", "diy_solo_semarang", "region name")
	edgeSpeedsFile  = flag.StringSlice("segment-speed-file", []string{}, "segment speed csv file. example usage -segment-speed-file=blokade.csv,traffic_solo.csv")

	graphFile        string
	overlayGraphFile string
	metricsFile      string
	landmarkFile     string
	timeFunctionFile string
)

func init() {
	flag.Parse()

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, *regionName)
	landmarkFile = fmt.Sprintf("./data/profiles/%s/%s_landmark.lm", profileName, *regionName)
	metricsFile = fmt.Sprintf("./data/profiles/%s/%s_metrics.txt", profileName, *regionName)
	timeFunctionFile = fmt.Sprintf("./data/profiles/%s/%s_timefunction.txt", profileName, *regionName)

	util.InitProfileConfig(profileName)
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)
	custom.SetEdgeSpeedsFilePath(*edgeSpeedsFile)
	_, err = custom.Customize()
	if err != nil {
		panic(err)
	}
}

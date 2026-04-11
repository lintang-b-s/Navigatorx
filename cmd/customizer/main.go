package main

import (
	"flag"
	"fmt"
	"path/filepath"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
)

var (
	profileName     string
	profileFilePath = flag.String("profile", "./data/car.yaml", "profile file path")
	regionName       = flag.String("region", "diy_solo_semarang", "region name")
	graphFile        string
	overlayGraphFile string
	metricsFile      string
	landmarkFile     string
)

func init() {
	flag.Parse()

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, *regionName)
	landmarkFile = fmt.Sprintf("./data/profiles/%s/%s_landmark.lm", profileName, *regionName)
	metricsFile = fmt.Sprintf("./data/profiles/%s/%s_metrics.txt", profileName, *regionName)
	util.InitProfileConfig(profileName)
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	m, err := custom.Customize()
	if err != nil {
		panic(err)
	}

	lm := landmark.NewLandmark()
	numberOfLandmarks := viper.GetInt("landmarks")

	err = lm.PreprocessALT(numberOfLandmarks, m, custom.GetGraph(), logger)
	if err != nil {
		panic(err)
	}
	err = lm.WriteLandmark(landmarkFile, custom.GetGraph())
	if err != nil {
		panic(err)
	}
}

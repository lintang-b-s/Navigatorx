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
	profileFilePath = flag.String("profile", "./data/car.yaml", "profile file path")
)

var (
	profileName      string = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile        string = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, profileName)
	overlayGraphFile string = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, profileName)
	metricsFile      string = fmt.Sprintf("./data/profiles/%s/%s_metrics.txt", profileName, profileName)
	landmarkFile     string = fmt.Sprintf("./data/profiles/%s/%s_landmark.lm", profileName, profileName)
)

func init() {
	flag.Parse()
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

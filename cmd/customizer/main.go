package main

import (
	"flag"
	"os"

	"net/http"
	_ "net/http/pprof"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	graphFile         string = "./data/original.graph"
	overlayGraphFile  string = "./data/overlay_graph.graph"
	metricsFile       string = "./data/metrics.txt"
	landmarkFile      string = "./data/landmark.lm"
	numberOfLandmarks        = 16
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	workingDir, err := os.Getwd()
	err = util.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}

	go func() { // pprof
		http.ListenAndServe("localhost:6868", nil)
	}()

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	m, err := custom.Customize()
	if err != nil {
		panic(err)
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(numberOfLandmarks, m, custom.GetGraph(), logger)
	if err != nil {
		panic(err)
	}
	err = lm.WriteLandmark(landmarkFile, custom.GetGraph())
	if err != nil {
		panic(err)
	}
}

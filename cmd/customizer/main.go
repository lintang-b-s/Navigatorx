package main

import (
	"flag"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"

	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
)

var (
	timeDependent = flag.Bool("time_dependent", false, "Use Time-Dependent Customizable Route Planning")
	ttfsFile      = flag.String("ttfs_file", "./data/traveltime_profiles/day_speed_profile_monday.csv", "travel time function for all openstreetmap way filepath")
)

const (
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
	landmarkFile     string = "./data/landmark.lm"
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	var (
		dayEdgeTTFs map[da.Index]*da.PWL = make(map[da.Index]*da.PWL)
	)

	if *timeDependent {
		dayEdgeTTFs, err = da.ReadTravelTimeFunctions(*ttfsFile)
		if err != nil {
			panic(err)
		}
	}

	m, err := custom.Customize(*timeDependent, dayEdgeTTFs)
	if err != nil {
		panic(err)
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(8, m, custom, logger)
	if err != nil {
		panic(err)
	}
	err = lm.WriteLandmark(landmarkFile, custom)
	if err != nil {
		panic(err)
	}
}

package main

import (
	"flag"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

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
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	var (
		dayTTF map[int64]*da.PWL = make(map[int64]*da.PWL)
	)

	if *timeDependent {
		dayTTF, err = da.ReadTravelTimeFunctions(*ttfsFile)
		if err != nil {
			panic(err)
		}
	}

	err = custom.Customize(*timeDependent, dayTTF)
	if err != nil {
		panic(err)
	}
}

package main

import (
	"flag"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
)

var (
	timeDependent = flag.Bool("time_dependent", true, "Use Time-Dependent Customizable Route Planning")
)

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}
	custom := customizer.NewCustomizer("./data/original.graph", "./data/overlay_graph.graph", "./data/metrics.txt", logger)

	tgprParser := osmparser.NewTPGRParser()

	_, osmWayProfile, err := tgprParser.ParseTGPRFile("./data/NY/NY.coordinate", "./data/NY/NY.tpgr",
		logger)

	err = custom.Customize(*timeDependent, osmWayProfile)
	if err != nil {
		panic(err)
	}
}

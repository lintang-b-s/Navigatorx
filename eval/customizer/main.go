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

	_, edgeTTFs, err := tgprParser.ParseTGPRFile("./data/NY/NY.coordinate", "./data/NY/NY.tpgr",
		logger)

	_, err = custom.Customize(*timeDependent, edgeTTFs)
	if err != nil {
		panic(err)
	}
}

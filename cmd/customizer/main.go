package main

import (
	"flag"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
)

var (
	visualize = flag.Bool("visualize", false, "visualize customization")
)

func main() {
	flag.Parse()
	logger, err := logger.New()
	if err != nil {
		panic(err)
	}
	custom := customizer.NewCustomizer("./data/original.graph", "./data/overlay_graph.graph", "./data/metrics.txt", logger)
	err = custom.Customize(*visualize)
	if err != nil {
		panic(err)
	}
}

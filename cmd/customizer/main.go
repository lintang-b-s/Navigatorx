package main

import (
	"github.com/lintang-b-s/navigatorx-crp/pkg/customizer"
)

func main() {
	custom := customizer.NewCustomizer("./data/test.graph", "./data/overlay_graph.graph", "./data/metrics.txt")
	err := custom.Customize()
	if err != nil {
		panic(err)
	}
}

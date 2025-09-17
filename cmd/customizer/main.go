package main

import (
	"github.com/lintang-b-s/navigatorx-crp/pkg/customizer"
)

func main() {
	customizer := customizer.NewCustomizer("./data/solo_jogja.graph", "./data/overlay_graph.graph", "./data/metrics.txt")
	err := customizer.Customize()
	if err != nil {
		panic(err)
	}
}

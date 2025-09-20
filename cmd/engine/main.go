package main

import (
	"errors"

	"github.com/lintang-b-s/navigatorx-crp/pkg/engine"
	"github.com/lintang-b-s/navigatorx-crp/pkg/engine/routing"
)

func main() {
	routingEngine, err := engine.NewEngine("./data/test.graph", "./data/overlay_graph.graph", "./data/metrics.txt")
	if err != nil {
		panic(err)
	}

	bidirSearch := routing.NewCRPBidirectionalSearch(routingEngine.GetRoutingEngine())
	_, _, found := bidirSearch.Search(6715, 43569)
	if found == false {
		panic(errors.New("no path found"))
	}
}

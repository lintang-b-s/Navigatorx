package main

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
)

func main() {
	logger, err := logger.New()
	if err != nil {
		panic(err)
	}
	osmParser := osmparser.NewOSMParserV2()

	graph, err := osmParser.Parse("./data/diy_solo_semarang.osm.pbf", logger, false)
	if err != nil {
		panic(err)
	}

	mp := partitioner.NewMultilevelPartitioner(
		[]int{int(math.Pow(2, 8)), int(math.Pow(2, 11)), int(math.Pow(2, 14))},
		3,
		graph, logger,
	)
	mp.RunMultilevelPartitioning()

	// mlp := mp.BuildMLP()
	err = mp.SaveToFile("./data/diy_solo_semarang.mlp")
	if err != nil {
		panic(err)
	}
}

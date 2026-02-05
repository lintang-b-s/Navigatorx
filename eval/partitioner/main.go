package main

import (
	"math"

	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
)

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	tgprParser := osmparser.NewTPGRParser()
	graph, _, err := tgprParser.ParseTGPRFile("./data/NY/NY.coordinate", "./data/NY/NY.tpgr",
		logger)

	mp := partitioner.NewMultilevelPartitioner(
		[]int{int(math.Pow(2, 8)), int(math.Pow(2, 11)), int(math.Pow(2, 13))},
		3,
		graph, logger,
	)
	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile("new_york_eval")
	if err != nil {
		panic(err)
	}
}

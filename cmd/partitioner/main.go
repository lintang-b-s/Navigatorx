package main

import (
	"flag"
	"fmt"
	"math"

	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
)

var (
	osmFile = flag.String("osm_file", "diy_solo_semarang.osm.pbf", "Openstreetmap .pbf filename")
	mlpFile = flag.String("mlp_file", "diy_solo_semarang", "Multilevel partition filename")
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}
	osmParser := osmparser.NewOSMParserV2()

	graph, err := osmParser.Parse(fmt.Sprintf("./data/%s", *osmFile), logger, false)
	if err != nil {
		panic(err)
	}

	mp := partitioner.NewMultilevelPartitioner(
		[]int{int(math.Pow(2, 9)), int(math.Pow(2, 11)), int(math.Pow(2, 14)), int(math.Pow(2, 16))},
		4,
		graph, logger,
	)
	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(*mlpFile)
	if err != nil {
		panic(err)
	}
}

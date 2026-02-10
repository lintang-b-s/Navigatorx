package main

import (
	"flag"
	"fmt"
	"strconv"
	"strings"

	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
)

var (
	osmFile        = flag.String("osm_file", "diy_solo_semarang.osm.pbf", "Openstreetmap .pbf filename")
	mlpFile        = flag.String("mlp_file", "diy_solo_semarang", "Multilevel partition filename")
	partitionSizes = flag.String("us", "8,11,14,17,18", "Multilevel Partition Sizes")
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

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
		if err != nil {
			panic(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		graph, logger,
	)
	
	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(*mlpFile)
	if err != nil {
		panic(err)
	}
}

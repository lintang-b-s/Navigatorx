package main

import (
	"flag"
	"fmt"
	"strconv"
	"strings"
	"time"

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
	now := time.Now()
	op := osmparser.NewOSMParserV2()

	graph, err := op.Parse(fmt.Sprintf("./data/%s", *osmFile), logger, false)
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
		25,
		graph, logger,
		true,
		true,
	) // i recommend u to use unit-capacity, because it faster, less shorctuts created, faster p2p query runtime

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(*mlpFile)
	if err != nil {
		panic(err)
	}

	duration := time.Now().Sub(now)
	logger.Sugar().Infof("done partitioning... time taken: %v s", duration.Seconds())
}

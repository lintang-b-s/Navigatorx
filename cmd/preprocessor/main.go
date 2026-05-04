package main

import (
	"flag"
	"fmt"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

var (
	profileFilePath        = flag.String("profile", "./data/car.yaml", "profile file path")
	osmFile                = flag.String("osm_file", "./data/diy_solo_semarang.osm.pbf", "Openstreetmap .pbf filename")
	mlpFile                = flag.String("mlp_file", "./data/crp_inertial_flow_diy_solo_semarang.mlp", "Multilevel partition filepath")
	regionName             = flag.String("region", "diy_solo_semarang", "region name")
	partitionSizes         = flag.String("us", "8,11,14,17,18", "Multilevel Partition Sizes")
	directed               = flag.Bool("directed_graph", true, "directed/undirected partition graph")
	prePartitionWithSCC    = flag.Bool("prepartition_with_scc", false, "prepartition graph with strongly connected components")
	inertialFlowIterations = flag.Int("iflow_iterations", 10, "number of iterations of the inertial flow algorithm (schild dan sommer (2015)) (https://link.springer.com/chapter/10.1007/978-3-319-20086-6_22)")
	graphFile              string
	overlayGraphFile       string
	profileName            string
)

func init() {
	flag.Parse()

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, *regionName)

	config.InitProfileConfig(profileName, *regionName)
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	now := time.Now()
	op := osmparser.NewOSMParserV2()

	graph, edgeInfoIds, err := op.Parse(*osmFile, logger)
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
		*inertialFlowIterations,
		graph, logger,
		*prePartitionWithSCC,
		*directed,
	)

	mp.RunMultilevelPartitioning()

	mlpPath := *mlpFile
	err = mp.SaveToFile(mlpPath)
	if err != nil {
		panic(err)
	}

	duration := time.Since(now)
	logger.Sugar().Infof("done partitioning... time taken: %v s", duration.Seconds())

	mlp := datastructure.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpPath)
	if err != nil {
		panic(err)
	}

	prep := prepo.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)

	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}
}

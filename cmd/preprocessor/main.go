package main

import (
	"flag"
	"fmt"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	profileFilePath               = flag.String("profile", "./data/car.yaml", "profile file path")
	osmFile                       = flag.String("osm_file", "diy_solo_semarang.osm.pbf", "Openstreetmap .pbf filename")
	mlpFile                       = flag.String("mlp_file", "crp_inertial_flow_diy_solo_semarang", "Multilevel partition filename")
	partitionSizes                = flag.String("us", "8,11,14,17,18", "Multilevel Partition Sizes")
	directed                      = flag.Bool("directed_graph", true, "directed/undirected graph")
	prePartitionWithSCC           = flag.Bool("prepartition_with_scc", false, "prepartition graph with strongly connected components")
	inertialFlowIterations        = flag.Int("iflow_iterations", 5, "number of iterations of the inertial flow algorithm")
	graphFile              string = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, profileName)
	overlayGraphFile       string = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, profileName)
	profileName            string = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
)

func init() {
	flag.Parse()
	util.InitProfileConfig(profileName)
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	now := time.Now()
	op := osmparser.NewOSMParserV2()

	graph, edgeInfoIds, err := op.Parse(fmt.Sprintf("./data/%s", *osmFile), logger)
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

	err = mp.SaveToFile(*mlpFile)
	if err != nil {
		panic(err)
	}

	duration := time.Now().Sub(now)
	logger.Sugar().Infof("done partitioning... time taken: %v s", duration.Seconds())

	mlp := datastructure.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s.mlp", *mlpFile))
	if err != nil {
		panic(err)
	}

	prep := prepo.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)

	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}
}

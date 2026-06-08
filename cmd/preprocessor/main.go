package main

import (
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
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
	transitionMHTFile      string
)

func init() {
	flag.Parse()

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.ngraph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.ngraph", profileName, *regionName)
	transitionMHTFile = fmt.Sprintf("./data/profiles/%s/%s_transition_matrix.ntm", profileName, *regionName)

	config.InitProfileConfig(profileName, *regionName)
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	now := time.Now()
	op := osmparser.NewOSMParserV2()

	graph, timeFunction, edgeInfoIds, err := op.Parse(*osmFile, logger)
	if err != nil {
		panic(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := util.ParseTextInt(pss[i])
		if err != nil {
			panic(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mlpPath := *mlpFile
	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		*inertialFlowIterations,
		graph, logger,
		*prePartitionWithSCC,
		*directed,
	)

	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(mlpPath); err != nil {
		panic(err)
	}

	duration := time.Since(now)
	logger.Sugar().Infof("done partitioning... time taken: %v s", duration.Seconds())

	mlp := datastructure.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpPath)
	if err != nil {
		panic(err)
	}

	if _, err := os.Stat(transitionMHTFile); err == nil {
		logger.Info("removing existing transition matrix file", zap.String("filename", transitionMHTFile))
		if err := os.Remove(transitionMHTFile); err != nil {
			panic(err)
		}
	}

	prep := prepo.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	prep.SetWriteTiles(true)

	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}
}

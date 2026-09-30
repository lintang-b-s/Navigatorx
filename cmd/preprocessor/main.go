package main

import (
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

var (
	profileFilePath        = flag.String("profile", "./data/car.yaml", "profile file path")
	osmFile                = flag.String("osm_file", "./data/diy_solo_semarang.osm.pbf", "Openstreetmap .pbf filename")
	regionName             = flag.String("region", "diy_solo_semarang", "region name")
	partitionSizes         = flag.String("us", "8,11,14,17,18", "Multilevel Partition Sizes")
	directed               = flag.Bool("directed_graph", true, "directed/undirected partition graph")
	prePartitionWithSCC    = flag.Bool("prepartition_with_scc", false, "prepartition graph with strongly connected components")
	inertialFlowIterations = flag.Int("iflow_iterations", 10, "number of iterations of the inertial flow algorithm (schild dan sommer (2015)) (https://link.springer.com/chapter/10.1007/978-3-319-20086-6_22)")
	visualizationFile      = flag.Bool("visualization", false, "write multilevel partition visualization to json file")
	profileName            string
)

func init() {
	flag.Parse()
	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	config.InitProfileConfig(profileName, *regionName, pkg.ROUTER)
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	now := time.Now()
	op := extractor.NewExtractor[int32]()

	graph, rn, wf, err := op.Extract(*osmFile, logger)
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

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		*inertialFlowIterations,
		graph, logger,
		*prePartitionWithSCC,
		*directed,
	)

	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(); err != nil {
		panic(err)
	}

	if *visualizationFile {
		if err := mp.WriteOverlayVerticesInLevel(); err != nil {
			panic(err)
		}
		if err := mp.WriteMLPVisualizationInLevel(); err != nil {
			panic(err)
		}
	}

	duration := time.Since(now)
	logger.Sugar().Infof("done partitioning... time taken: %v s", duration.Seconds())

	mlp := datastructure.NewPlainMLP()
	err = mlp.ReadMlpFile()
	if err != nil {
		panic(err)
	}

	transitionMHTFile := fmt.Sprintf("./data/profiles/%s/%s_transition_matrix.ntm", profileName, *regionName)
	if _, err := os.Stat(transitionMHTFile); err == nil {
		logger.Info("removing existing transition matrix file", zap.String("filename", transitionMHTFile))
		if err := os.Remove(transitionMHTFile); err != nil {
			panic(err)
		}
	}

	prep := prepo.NewPreprocessor(graph, rn, wf, mlp, logger, pkg.ROUTER)
	prep.SetWriteTiles(true)
	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}
}

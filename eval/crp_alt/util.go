// Package crpalt contains evaluation utilities for Customizable Route Planning (CRP) query phase.
package crpalt

import (
	"fmt"
	"os"
	"path/filepath"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

// ini buat dimacs 9th implmenetation challenge correctness test
func BuildCRP(nodeCoords []da.Coordinate, adjList [][]da.PairEdge, n int, Us []int, name string) (*engine.Engine[int64], *da.Graph,
	[]da.Index, map[da.Index]da.Index) {
	logger, err := logger.New()
	if err != nil {
		logger.Fatal("error: %v", zap.Error(err))
	}
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		logger.Fatal("error: %v", zap.Error(err))
	}

	config.InitRegionName(name, pkg.EVAL)

	outputDir := filepath.Join(workingDir, "data", "eval")
	var (
		// config
		metricsFile = fmt.Sprintf("%s/%s/%s_metrics.nmt", config.ProfilesRoot(), pkg.ProfileName, pkg.RegionName)
		prep        *preprocesser.Preprocessor[int64]
	)

	if err := util.EnsureDirExists(outputDir); err != nil {
		logger.Fatal("error: %v", zap.Error(err))
	}

	doPreprocessCustomize := false
	if _, err := os.Stat(metricsFile); os.IsNotExist(err) {
		doPreprocessCustomize = true
	}

	es := da.FlattenEdgesG[int64](adjList)

	op := extractor.NewExtractor[int64]()
	acceptedNodeMap := make(map[int64]da.Coordinate, n)
	nodeToOsmId := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = nodeCoords[i]
		nodeToOsmId[da.Index(i)] = int64(i)
	}
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmId)

	rn := da.NewRoadNetworkDataContainerWithSize(len(es), n)
	g, timeFunction, _, _ := op.BuildGraph(es, rn, uint32(n), false)

	if doPreprocessCustomize {

		ps := make([]int, len(Us))

		for i := 0; i < len(ps); i++ {
			pow := Us[i]
			ps[i] = 1 << pow // 2^pow
		}

		mp := partitioner.NewMultilevelPartitioner(
			ps,
			len(ps), 15,
			g, logger,
		)
		mp.RunMultilevelPartitioning()

		err = mp.SaveToFile()
		if err != nil {
			logger.Fatal("error: %v", zap.Error(err))
		}

		mlp := da.NewPlainMLP()
		err = mlp.ReadMlpFile()
		if err != nil {
			logger.Fatal("error: %v", zap.Error(err))
		}

		prep = preprocesser.NewPreprocessor(g, rn, timeFunction, mlp, logger)
		prep.SetWriteTiles(false)
		err = prep.PreProcessing(true)
		if err != nil {
			logger.Fatal("error: %v", zap.Error(err))
		}

	} else {
		mlp := da.NewPlainMLP()
		err = mlp.ReadMlpFile()
		if err != nil {
			logger.Fatal("error: %v", zap.Error(err))
		}
		prep = preprocesser.NewPreprocessor(g, rn, timeFunction, mlp, logger)
		prep.SetWriteTiles(false)
		err = prep.PreProcessing(false)
		if err != nil {
			logger.Fatal("error: %v", zap.Error(err))
		}
	}

	cust := customizer.NewCustomizer[int64](logger)

	_, err = cust.Customize()
	if err != nil {
		logger.Fatal("error: %v", zap.Error(err))
	}

	re, err := engine.NewEngine[int64](logger)
	if err != nil {
		logger.Fatal("error: %v", zap.Error(err))
	}

	oldToNewVIdMap := prep.GetOldToNewVId()
	newToOldVidMap := prep.GetNewToOldVId()

	return re, g, oldToNewVIdMap, newToOldVidMap
}

type QueryParam struct {
	i, s, t da.Index
}

func (q *QueryParam) GetId() da.Index {
	return q.i
}

func (q *QueryParam) GetSource() da.Index {
	return q.s
}

func (q *QueryParam) GetTarget() da.Index {
	return q.t
}

func NewQueryParam(i, s, t da.Index) QueryParam {
	return QueryParam{i, s, t}
}

// Package tests provides integration and unit tests for the routing engine.
package tests

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

func init() {
	config.InitProfileConfig("car", "test_region")
}

func Setup(t *testing.T, fileName string) (*engine.Engine[int32], *zap.Logger, *customizer.Customizer[int32]) {
	var (
		mlpFile          = fmt.Sprintf("./data/stress_test_%s.mlp", fileName)
		osmfFile         = fmt.Sprintf("./data/%s.osm.pbf", fileName)
		graphFile        = fmt.Sprintf("./data/original_%s_test.ngraph", fileName)
		overlayGraphFile = fmt.Sprintf("./data/overlay_graph_%s_test.ngraph", fileName)
		metricsFile      = fmt.Sprintf("./data/metrics_%s_test.nmt", fileName)
		landmarkFile     = fmt.Sprintf("./data/landmark_%s_test.nlm", fileName)
		timeFunctionFile = fmt.Sprintf("./data/timefunction_%s_test.ntf", fileName)
	)

	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}

	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	op := osmparser.NewOSMParserV2[int32]()

	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		t.Fatal(err)
	}
	graph, timeFunction, edgeInfoIds, err := op.Parse(filepath.Join(workingDir, osmfFile), logger)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split("8,10,11,12,15", ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := util.ParseTextInt(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger, false, true,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpFile)
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer[int32](graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)

	_, err = custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, logger, custom
}

type PairEdge struct {
	To     int
	Weight float64
}

func NewPairEdge(to int, weight float64) PairEdge {
	return PairEdge{To: to, Weight: weight}
}

func FlattenEdges(es [][]PairEdge) []osmparser.Edge[float64] {
	flatten := make([]osmparser.Edge[float64], 0, len(es))

	eid := 0

	for from, edges := range es {
		for _, e := range edges {
			length := uint32(math.Round(e.Weight * 100))
			flatten = append(flatten, osmparser.NewEdge(uint32(from), uint32(e.To), e.Weight, length, false, 0))
			eid++
		}
	}

	return flatten
}

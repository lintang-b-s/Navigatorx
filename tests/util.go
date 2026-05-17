// Package tests provides integration and unit tests for the routing engine.
package tests

import (
	"fmt"
	"os"
	"strconv"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
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

func Setup(t *testing.T, fileName string) (*engine.Engine, *zap.Logger, *customizer.Customizer) {
	var (
		mlpFile          = fmt.Sprintf("./data/stress_test_%s.mlp", fileName)
		osmfFile         = fmt.Sprintf("../../data/%s.osm.pbf", fileName)
		graphFile        = fmt.Sprintf("./data/original_%s_test.graph", fileName)
		overlayGraphFile = fmt.Sprintf("./data/overlay_graph_%s_test.graph", fileName)
		metricsFile      = fmt.Sprintf("./data/metrics_%s_test.txt", fileName)
		landmarkFile     = fmt.Sprintf("./data/landmark_%s_test.lm", fileName)
		timeFunctionFile = fmt.Sprintf("./data/timefunction_%s_test.txt", fileName)
	)

	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}

	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	op := osmparser.NewOSMParserV2()

	graph, edgeInfoIds, err := op.Parse(osmfFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split("8,10,11,12,15", ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
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
	prep := preprocessor.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)

	_, err = custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
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

func FlattenEdges(es [][]PairEdge) []osmparser.Edge {
	flatten := make([]osmparser.Edge, 0, len(es))

	eid := 0

	for from, edges := range es {
		for _, e := range edges {
			flatten = append(flatten, osmparser.NewEdge(uint32(from), uint32(e.To), e.Weight, e.Weight, false, 0))
			eid++
		}
	}

	return flatten
}

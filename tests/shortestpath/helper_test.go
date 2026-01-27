package shortestpath

import (
	"math"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

func buildCRP(t *testing.T, adjList [][]pairEdge, n int, U1, U2 float64) (*engine.Engine, *datastructure.Graph,
	[]datastructure.Index, map[datastructure.Index]datastructure.Index) {
	es := flattenEdges(adjList)

	op := osmparser.NewOSMParserV2()
	gs := datastructure.NewGraphStorageWithSize(len(es), n)
	g := op.BuildGraph(es, gs, uint32(n), true)

	t.Logf("number of vertices: %v, number of edges: %v", uint32(n), len(es))

	g.SetGraphStorage(gs)

	logger, err := logger.New()
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	mp := partitioner.NewMultilevelPartitioner(
		[]int{int(math.Pow(2, U1)), int(math.Pow(2, U2))},
		2,
		g, logger,
	)
	mp.RunMultilevelPartitioning()

	mlp := mp.BuildMLP()

	prep := preprocesser.NewPreprocessor(g, mlp, logger)
	err = prep.PreProcessing(false)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	og := prep.GetOverlayGraph()
	cust := customizer.NewCustomizerDirect(g, og, logger)
	m, err := cust.CustomizeDirect(false, "monday")
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	cf := costfunction.NewTimeCostFunction()

	re, err := engine.NewEngineDirect(g, og, m, logger, cust, cf, false, "monday")
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	oldToNewVIdMap := prep.GetOldToNewVIdMap()
	newToOldVidMap := prep.GetNewToOldVIdMap()
	return re, g, oldToNewVIdMap, newToOldVidMap
}

// equal operator
func eq(a, b float64) bool {
	return math.Abs(a-b) <= EPS
}

func lt(a, b float64) bool {
	return a+EPS < b
}

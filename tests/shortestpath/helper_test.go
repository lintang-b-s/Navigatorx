package shortestpath

import (
	"os"
	"runtime"
	"strings"
	"testing"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/tests"
)

func TestMain(m *testing.M) {
	gopool.SetCap(int32(runtime.NumCPU()))
	os.Exit(m.Run())
}

const (
	graphFile        string = "./data/original_sp_test.ngraph"
	overlayGraphFile string = "./data/overlay_graph_sp_test.ngraph"
	metricsFile      string = "./data/metrics_sp_test.nmt"
	landmarkFile     string = ""
	timeFunctionFile string = "./data/timefunction_sp_test.ntf"
)

func buildCRP(t *testing.T, nodeCoords []osmparser.NodeCoord, adjList [][]tests.PairEdge, n int, Us []int, pgDirected bool) (*engine.Engine[float64], *da.Graph,
	[]da.Index, map[da.Index]da.Index) {
	da.CoordinatePrecision = 1e6
	if strings.Contains(strings.ToUpper(t.Name()), "KRL") {
		da.CoordinatePrecision = 1e3
	}
	es := tests.FlattenEdges(adjList)

	op := osmparser.NewOSMParserV2()
	acceptedNodeMap := make(map[int64]osmparser.NodeCoord, n)
	nodeToOsmId := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = nodeCoords[i]
		nodeToOsmId[da.Index(i)] = int64(i)
	}

	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmId)

	gs := da.NewGraphStorageWithSize(len(es), n)
	g, timeFunction, edgeInfoIds := op.BuildGraphFloat64(es, gs, uint32(n), false)

	t.Logf("number of vertices: %v, number of edges: %v", uint32(n), len(es))

	g.SetGraphStorage(gs)

	logger, err := logger.New()
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	ps := make([]int, len(Us))

	for i := 0; i < len(ps); i++ {
		pow := Us[i]
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps), 1,
		g, logger, true, true,
	)

	mp.RunMultilevelPartitioning()

	mlp := mp.BuildMLP()

	prep := preprocesser.NewPreprocessor(g, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(false)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	og := prep.GetOverlayGraph()
	cust := customizer.NewCustomizerDirect(
		g, og, prep.GetTimeFunction(), logger,
	)
	m, err := cust.CustomizeDirect()
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	re, err := engine.NewEngineDirect(g, og, m, logger, landmarkFile)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	oldToNewVIdMap := prep.GetOldToNewVIdMap()
	newToOldVidMap := prep.GetNewToOldVIdMap()

	return re, g, oldToNewVIdMap, newToOldVidMap
}

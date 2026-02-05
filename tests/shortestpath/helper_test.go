package shortestpath

import (
	"bufio"
	"errors"
	"io"
	"math"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

func buildCRP(t *testing.T, nodeCoords []osmparser.NodeCoord, adjList [][]pairEdge, n int, U1, U2 float64) (*engine.Engine, *da.Graph,
	[]da.Index, map[da.Index]da.Index) {
	es := flattenEdges(adjList)

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

func readLine(br *bufio.Reader) (string, error) {
	line, err := br.ReadString('\n')
	if err != nil {
		if errors.Is(err, io.EOF) && len(line) > 0 {
		} else {
			return "", err
		}
	}
	return strings.TrimRight(line, "\r\n"), nil
}

func fields(s string) []string {

	return strings.Fields(s)
}

type pairEdge struct {
	to     int
	weight float64
}

func newPairEdge(to int, weight float64) pairEdge {
	return pairEdge{to, weight}
}

func flattenEdges(es [][]pairEdge) []osmparser.Edge {
	flatten := make([]osmparser.Edge, 0, len(es))

	eid := 0

	for from, edges := range es {
		for _, e := range edges {
			flatten = append(flatten, osmparser.NewEdge(uint32(from), uint32(e.to), e.weight, 0, uint32(eid)))
			eid++
		}
	}

	return flatten
}

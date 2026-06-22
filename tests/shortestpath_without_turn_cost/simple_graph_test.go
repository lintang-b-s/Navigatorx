package shortestpath_without_turn_cost

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/Navigatorx/tests"
)

func solveSimpleGraph(t *testing.T, filepath string) {
	var (
		err     error
		line    string
		n, m    int
		f, fOut *os.File
	)

	f, err = os.OpenFile(filepath+".in", os.O_RDONLY, 0644)
	if err != nil {
		t.Fatalf("could not open test file: %v", err)
	}
	defer f.Close()

	br := bufio.NewReader(f)

	line, err = util.ReadLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	ff := util.Fields(line)
	n, err = util.ParseTextInt(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	m, err = util.ParseTextInt(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	var nodeCoords []osmparser.NodeCoord

	for i := 0; i < n; i++ {
		nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(i), float64(i)))
	}

	adjList := make([][]tests.PairEdge, n)
	for i := 0; i < m; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := util.Fields(line)
		u, err := util.ParseTextInt(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		v, err := util.ParseTextInt(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		w, err := util.ParseTextInt(ff[2])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		adjList[u] = append(adjList[u], tests.NewPairEdge(v, float64(w)))
	}

	re, _, oldToNewVIdMap, _, _ := buildCRP(t, nodeCoords, adjList, n, []int{1, 2}, true)

	crpQuery := routing.NewCRPBidirectionalSearchWithoutTurnCost(re.GetRoutingEngine())

	sid := oldToNewVIdMap[da.Index(0)]
	tid := oldToNewVIdMap[da.Index(n-1)]

	t.Logf("calculating shortest path...  \n")

	spLength, _, _ := crpQuery.ShortestPathSearch(sid, tid)

	// assert expected output dari test cases soal

	fOut, err = os.OpenFile(filepath+".ans", os.O_RDONLY, 0644)
	if err != nil {
		t.Fatalf("could not open test file: %v", err)
	}
	defer fOut.Close()

	brOut := bufio.NewReader(fOut)

	line, err = util.ReadLine(brOut)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	var expectedSPLength float64 = 0
	_, err = fmt.Sscanf(line, "%f", &expectedSPLength)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	if !util.Eq(spLength, expectedSPLength) {
		t.Fatalf("FAIL: Expected shortest path length: %v, got: %v", expectedSPLength, spLength)
	}

	t.Logf("solveSimpleGraphd test case: %v", filepath)
}

// "go test ./tests/shortestpath_without_turn_cost  -run TestCRPQuerySimpleGraphMLD  -v -timeout=0  -count=1"
func TestCRPQuerySimpleGraphMLD(t *testing.T) {

	dirPath := "../shortestpath/data/tests/shortestpath/simple_graph"

	files, err := os.ReadDir(dirPath)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	for _, entry := range files {

		name := entry.Name()

		if !strings.HasSuffix(name, ".in") {
			continue
		}

		baseName := strings.TrimSuffix(name, ".in")

		testPath := filepath.Join(dirPath, baseName)

		t.Logf("solving test case: %v", baseName)
		t.Run("Multilevel-Dijkstra without turn cost"+dirPath+"/"+baseName, func(t *testing.T) {
			solveSimpleGraph(t, testPath)

			runtime.GC()
			debug.FreeOSMemory()
		})

	}
}

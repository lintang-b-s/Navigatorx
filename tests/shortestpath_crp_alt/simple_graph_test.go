package shortestpath

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strconv"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
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

	line, err = readLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	ff := fields(line)
	n, err = strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	m, err = strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	var nodeCoords []osmparser.NodeCoord

	for i := 0; i < n; i++ {
		nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(i), float64(i)))
	}

	adjList := make([][]pairEdge, n)
	for i := 0; i < m; i++ {
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := fields(line)
		u, err := strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		v, err := strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		w, err := strconv.Atoi(ff[2])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		adjList[u] = append(adjList[u], pairEdge{v, float64(w)})
	}

	re, g, oldToNewVIdMap, _, lm := buildCRP(t, nodeCoords, adjList, n, []int{1, 2}, true)

	crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)

	sid := oldToNewVIdMap[datastructure.Index(0)]
	tid := oldToNewVIdMap[datastructure.Index(n-1)]

	as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1
	at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1

	t.Logf("calculating shortest path...  \n")

	spLength, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

	// assert expected output dari test cases soal

	fOut, err = os.OpenFile(filepath+".ans", os.O_RDONLY, 0644)
	if err != nil {
		t.Fatalf("could not open test file: %v", err)
	}
	defer fOut.Close()

	brOut := bufio.NewReader(fOut)

	line, err = readLine(brOut)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	var expectedSPLength float64 = 0
	_, err = fmt.Sscanf(line, "%f", &expectedSPLength)
	if !eq(spLength, expectedSPLength) {
		t.Fatalf("FAIL: Expected shortest path length: %v, got: %v", expectedSPLength, spLength)
	}

	t.Logf("solveSimpleGraphd test case: %v", filepath)
}

func TestCRPQuerySimpleGraph(t *testing.T) {

	dirPath := "../shortestpath/data/tests/shortestpath/simple_graph/"

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
		t.Run(dirPath+"/"+baseName, func(t *testing.T) {
			solveSimpleGraph(t, testPath)

			runtime.GC()
			debug.FreeOSMemory()
		})

	}
}

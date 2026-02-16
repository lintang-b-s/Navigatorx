package shortestpath

import (
	"bufio"
	"math"
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

/*
problem source: https://icpcarchive.github.io/Europe%20Contests/Southwestern%20Europe%20Regional%20Contest%20(SWERC)/2013%20Southwestern%20Europe%20Regional%20Contest/problems.pdf
no test cases excluded

test data: https://archive.algo.is/icpc/swerc/2013/
my c++ solution (got AC on kattis: https://open.kattis.com/problems/shoppingmalls):
https://drive.google.com/file/d/1GH9DiI2RRa-U0Ge67wa5O8ACjYIasWce/view?usp=sharing


*/

type place struct {
	floor int
	x, y  int
}

func solveShoppingMalls(t *testing.T, filepath string) {
	var (
		err     error
		line    string
		N, M    int
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

	N, err = strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	M, err = strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	places := make([]place, N)

	for i := 0; i < N; i++ {
		var (
			fl, x, y int
		)
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := fields(line)
		fl, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		x, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		y, err = strconv.Atoi(ff[2])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		places[i] = place{fl * 5, x, y}
	}

	adjList := make([][]pairEdge, N)

	type pairDist struct {
		abDist, baDist float64
	}

	getDist := func(placeA, placeB place, tipe string) pairDist {
		ff := float64(placeA.floor - placeB.floor)
		xx := float64(placeA.x - placeB.x)
		yy := float64(placeA.y - placeB.y)
		dist := math.Sqrt(ff*ff + xx*xx + yy*yy)
		if tipe == "walking" || tipe == "stairs" {

			return pairDist{dist, dist}
		} else if tipe == "lift" {
			return pairDist{1, 1}
		}

		// escalator
		return pairDist{1, dist * 3}
	}

	for i := 0; i < M; i++ {
		var (
			a, b int
			tipe string
		)

		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = fields(line)
		a, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		b, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		tipe = ff[2]

		placeA := places[a]
		placeB := places[b]

		pd := getDist(placeA, placeB, tipe)
		adjList[a] = append(adjList[a], pairEdge{b, pd.abDist})
		adjList[b] = append(adjList[b], pairEdge{a, pd.baDist})
	}

	nodeCoords := make([]osmparser.NodeCoord, 0)
	for i := 0; i < N; i++ {
		placei := places[i]
		nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(placei.x), float64(placei.y*placei.floor)))
	}

	re, g, oldToNewVIdMap, newToOldVidMap, lm := buildCRP(t, nodeCoords, adjList, N, 6, 7)

	line, err = readLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	q, err := strconv.Atoi(line)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	fOut, err = os.OpenFile(filepath+".out", os.O_RDONLY, 0644)
	if err != nil {
		t.Fatalf("could not open test file: %v", err)
	}
	defer fOut.Close()

	brOut := bufio.NewReader(fOut)
	for i := 0; i < q; i++ {
		var (
			a, b int
		)
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = fields(line)

		a, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		b, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		sid := oldToNewVIdMap[datastructure.Index(a)]
		tid := oldToNewVIdMap[datastructure.Index(b)]

		as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1
		at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1

		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)
		_, _, _, spEdges, _ := crpQuery.ShortestPathSearch(as, at)
		path := make([]int, 0)
		path = append(path, a)

		for _, e := range spEdges {
			v := newToOldVidMap[e.GetHead()]
			path = append(path, int(v))
		}

		// assert expected output dari test cases soal

		line, err = readLine(brOut)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		ff = fields(line)

		expectedPath := make([]int, len(ff))
		for k := 0; k < len(ff); k++ {
			expectedPath[k], err = strconv.Atoi(ff[k])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
		}
		if len(expectedPath) != len(path) {
			t.Fatalf("FAIL: Expected shortest number of vertices in shortest path : %v, got: %v", len(expectedPath), len(path))
		}

		for j := 0; j < len(expectedPath); j++ {
			if expectedPath[j] != path[j] {
				t.Fatalf("FAIL: Expected vertex in sp: %v, got: %v", expectedPath[j], path[j])
			}
		}

	}
}

// please run the test using command: "cd tests/shortestpath_crp_alt && go test -run TestCRPQueryShoppingMalls  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode
func TestCRPQueryShoppingMalls(t *testing.T) {

	dirPath := "../shortestpath/data/tests/shortestpath/icpc_swerc2013_shoppingmalls/"
	testDirs := []string{"secret"}

	for _, dir := range testDirs {
		fullDir := filepath.Join(dirPath, dir)

		files, err := os.ReadDir(fullDir)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		for _, entry := range files {

			name := entry.Name()

			if !strings.HasSuffix(name, ".in") {
				continue
			}

			baseName := strings.TrimSuffix(name, ".in")

			testPath := filepath.Join(fullDir, baseName)

			t.Logf("solving test case: %v", baseName)
			t.Run(dir+"/"+baseName, func(t *testing.T) {
				solveShoppingMalls(t, testPath)
				runtime.GC()
				debug.FreeOSMemory()
			})

		}
	}
}

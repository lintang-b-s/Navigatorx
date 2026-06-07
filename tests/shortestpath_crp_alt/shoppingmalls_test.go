package shortestpath

import (
	"bufio"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/Navigatorx/tests"
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

	line, err = util.ReadLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	ff := util.Fields(line)

	N, err = util.ParseTextInt(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	M, err = util.ParseTextInt(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	places := make([]place, N)

	for i := 0; i < N; i++ {
		var (
			fl, x, y int
		)
		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := util.Fields(line)
		fl, err = util.ParseTextInt(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		x, err = util.ParseTextInt(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		y, err = util.ParseTextInt(ff[2])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		places[i] = place{fl * 5, x, y}
	}

	adjList := make([][]tests.PairEdge, N)

	type pairDist struct {
		abDist, baDist float64
	}

	getDist := func(placeA, placeB place, tipe string) pairDist {
		ff := float64(placeA.floor - placeB.floor)
		xx := float64(placeA.x - placeB.x)
		yy := float64(placeA.y - placeB.y)
		dist := math.Sqrt(ff*ff + xx*xx + yy*yy)
		switch tipe {
		case "walking", "stairs":
			return pairDist{dist, dist}
		case "lift":
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

		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = util.Fields(line)
		a, err = util.ParseTextInt(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		b, err = util.ParseTextInt(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		tipe = string(ff[2])

		placeA := places[a]
		placeB := places[b]

		pd := getDist(placeA, placeB, tipe)
		adjList[a] = append(adjList[a], tests.NewPairEdge(b, pd.abDist))
		adjList[b] = append(adjList[b], tests.NewPairEdge(a, pd.baDist))
	}

	nodeCoords := make([]osmparser.NodeCoord, 0)
	for i := 0; i < N; i++ {
		placei := places[i]
		nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(placei.x), float64(placei.y*placei.floor)))
	}

	re, g, oldToNewVIdMap, newToOldVidMap, _ := buildCRP(t, nodeCoords, adjList, N, []int{6, 7}, true)

	line, err = util.ReadLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	q, err := util.ParseTextInt(line)
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
		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = util.Fields(line)

		a, err = util.ParseTextInt(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		b, err = util.ParseTextInt(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		sid := oldToNewVIdMap[da.Index(a)]
		tid := oldToNewVIdMap[da.Index(b)]

		as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1
		at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1

		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0)

		sVertex := g.GetVertex(sid)
		tVertex := g.GetVertex(tid)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		_, _, _, spEdges, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		path := make([]int, 0)
		path = append(path, a)

		for _, eId := range spEdges {
			e := g.GetOutEdge(eId)
			v := newToOldVidMap[e.GetHead()]
			path = append(path, int(v))
		}

		// assert expected output dari test cases soal

		line, err = util.ReadLine(brOut)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		ff = util.Fields(line)

		expectedPath := make([]int, len(ff))
		for k := 0; k < len(ff); k++ {
			expectedPath[k], err = util.ParseTextInt(ff[k])
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

// please run the test using command: "cd tests/shortestpath_crp_alt && go test -run TestCRPQueryShoppingMallsMALT  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode
// selesai dalam 1 detik
func TestCRPQueryShoppingMallsMALT(t *testing.T) {

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

			})

		}
	}
}

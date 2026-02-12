package shortestpath

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strconv"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
)

/*
taken from: https://2023.nwerc.eu/main/problem-set.pdf

test data: https://chipcie.wisv.ch/archive/2023/nwerc/solutions.zip
exclude test cases that contains > 50k edges or > 30k vertices, to speed up the testings,
agar ukuran reponya ga kegedean juga heheh.
kalau mau test pakai semua test cases tinggal download dari link diatas and taruh di "tests/shortestpath/data/tests/shortestpath/icpc_nwerc2023_galaxyquest"

my c++ solution (got AC on kattis: https://open.kattis.com/problems/galaxyquest2):
https://drive.google.com/file/d/1ulMqGc3n89_jfIfiOAAedAhYfL4lo9Pj/view?usp=sharing



*/

type planet struct {
	x, y, z int
}

func abcFormula(a, b, c float64) float64 {
	if b*b+EPS < 4*a*c {
		return -1 // undefined/impossible
	}
	return (-b - math.Sqrt(b*b-4*a*c)) / (2 * a)
}

func getFuel(sumSqrtDist float64, t uint32) float64 {
	if sumSqrtDist-0 < EPS {
		return -1
	}
	q := float64(t) / sumSqrtDist

	cLangrange := abcFormula(1, -q, 1) // sqrt(1- \frac{2}{\lambda}), dimana \lambda adlh langrange multiplier
	if cLangrange == -1 {
		return -1
	}
	fuel := 2 * sumSqrtDist * cLangrange
	return fuel
}

func solveGalaxyQuest(t *testing.T, filepath string) {
	var (
		err     error
		line    string
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

	n, err := strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	m, err := strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	q, err := strconv.Atoi(ff[2])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	galaxy := make([]planet, 0)

	for i := 0; i < n; i++ {
		var (
			x, y, z int
		)
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = fields(line)

		x, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		y, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		z, err = strconv.Atoi(ff[2])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		galaxy = append(galaxy, planet{x, y, z})
	}

	adjList := make([][]pairEdge, n)

	eucDist := func(a, b planet) float64 {
		xx := float64(a.x - b.x)
		yy := float64(a.y - b.y)
		zz := float64(a.z - b.z)
		dist := math.Sqrt(xx*xx + yy*yy + zz*zz)
		return dist
	}

	for i := 0; i < m; i++ {

		var (
			u, v int
		)
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = fields(line)

		u, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		v, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		u--
		v--

		planetU := galaxy[u]
		planetV := galaxy[v]

		dist := math.Sqrt(eucDist(planetU, planetV))
		adjList[u] = append(adjList[u], pairEdge{v, dist})
		adjList[v] = append(adjList[v], pairEdge{u, dist})
	}

	nodeCoords := make([]osmparser.NodeCoord, 0)
	for i := 0; i < n; i++ {
		planet := galaxy[i]
		nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(planet.x+planet.z), float64(planet.y+planet.z)))
	}

	re, g, oldToNewVIdMap, _, lm := buildCRP(t, nodeCoords, adjList, n, 7, 9)
	s := 0
	sid := oldToNewVIdMap[da.Index(s)]
	as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1

	dist := make(map[int]float64)
	dist[s] = 0

	t.Logf("calculating shortest paths from planet 1 to other planets.....\n")

	atIds := make([]da.Index, 0, n)

	atIdTotId := make(map[da.Index]int)
	for v := 1; v < n; v++ {

		tid := oldToNewVIdMap[da.Index(v)]
		at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1
		atIds = append(atIds, at)
		atIdTotId[at] = v
	}

	for _, at := range atIds {
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)
		sp, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

		target := atIdTotId[at]

		dist[target] = sp
	}

	fOut, err = os.OpenFile(filepath+".ans", os.O_RDONLY, 0644)
	if err != nil {
		t.Fatalf("could not open test file: %v", err)
	}
	defer fOut.Close()

	brOut := bufio.NewReader(fOut)

	for i := 0; i < q; i++ {

		var (
			target, time int
		)
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = fields(line)
		target, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		target--

		time, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		fuel := getFuel(dist[target], uint32(time))
		if fuel == -1 {
			ans := "impossible"

			line, err = readLine(brOut)
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			expectedAns := line
			if ans != expectedAns {
				t.Fatalf("FAIL: Expected shortest path length: %v, got: %v", expectedAns, ans)
			}
		} else {
			ans := fuel

			line, err = readLine(brOut)
			if err != nil {
				t.Fatalf("err: %v", err)
			}

			var expectedAns float64 = 0
			_, err = fmt.Sscanf(line, "%f", &expectedAns)
			if !eq(ans, expectedAns) {
				t.Fatalf("FAIL: Expected fuel: %v, got: %v", expectedAns, ans)
			}
		}
	}

	t.Logf("solved test case: %v", filepath)
}

// please run the test using command: "cd tests/shortestpath_crp_alt && go test -run TestCRPQueryGalaxyQuest  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode
func TestCRPQueryGalaxyQuest(t *testing.T) {

	dirPath := "../shortestpath/data/tests/shortestpath/icpc_nwerc2023_galaxyquest/"
	testDirs := []string{"sample", "secret"}

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
				solveGalaxyQuest(t, testPath)

				runtime.GC()
				debug.FreeOSMemory()
			})

		}
	}
}

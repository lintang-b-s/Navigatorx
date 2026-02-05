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
	"sync"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
)

/*
taken from: https://ukiepc.info/2016/

test data: https://drive.google.com/drive/folders/0B8zAOBDFU39pNTRYSm5jN1Njbzg?resourcekey=0-LXcVcbcx4u75vtaiwF0zCw

tests selesai sekitar 5 -7 menit

shortcuts yang dihasilkan graph soal ini (jauh lebih banyak),beda dengan graph openstreetmap road networks

my c++ solution (got AC on kattis: https://open.kattis.com/problems/showroom?tab=metadata):
https://drive.google.com/file/d/1d1SQqB-8Y6EUvTlVPgrNwltpWpCqXmes/view?usp=sharing
*/

func SolveShowroom(t *testing.T, filepath string) {
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

	r, err := strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	c, err := strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	grid := make([][]string, r)

	for i := 0; i < r; i++ {
		grid[i] = make([]string, c)
	}

	for i := 0; i < r; i++ {
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		for j := 0; j < c; j++ {
			grid[i][j] = fmt.Sprintf("%v", string(line[j]))
		}
	}

	adjList := make([][]pairEdge, r*c)

	cellToNId := func(i, j int) int {
		return i*c + j
	}

	getWeight := func(i, j int) int {
		if grid[i][j] == "c" {
			return 1
		} else if grid[i][j] == "D" {
			return 0
		}

		return pkg.INF_WEIGHT
	}

	type pair struct {
		first, second int
	}
	pintuUjungs := make([]pair, 0)

	for i := 0; i < r; i++ {
		for j := 0; j < c; j++ {
			if j+1 < c {
				weight := getWeight(i, j+1)

				adjList[cellToNId(i, j)] = append(adjList[cellToNId(i, j)],
					newPairEdge(cellToNId(i, j+1), float64(weight)))
			}

			if i+1 < r {
				weight := getWeight(i+1, j)

				adjList[cellToNId(i, j)] = append(adjList[cellToNId(i, j)],
					newPairEdge(cellToNId(i+1, j), float64(weight)))
			}

			if j-1 >= 0 {
				weight := getWeight(i, j-1)

				adjList[cellToNId(i, j)] = append(adjList[cellToNId(i, j)],
					newPairEdge(cellToNId(i, j-1), float64(weight)))
			}

			if i-1 >= 0 {
				weight := getWeight(i-1, j)

				adjList[cellToNId(i, j)] = append(adjList[cellToNId(i, j)],
					newPairEdge(cellToNId(i-1, j), float64(weight)))
			}

			if j == c-1 && grid[i][j] == "D" {
				pintuUjungs = append(pintuUjungs, pair{i, j})
			}
			if j == 0 && grid[i][j] == "D" {
				pintuUjungs = append(pintuUjungs, pair{i, j})
			}

			if i == 0 && grid[i][j] == "D" {
				pintuUjungs = append(pintuUjungs, pair{i, j})
			}

			if i == r-1 && grid[i][j] == "D" {
				pintuUjungs = append(pintuUjungs, pair{i, j})
			}
		}
	}

	line, err = readLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	ff = fields(line)

	tx, err := strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	ty, err := strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	tx--
	ty--

	nodeCoords := make([]osmparser.NodeCoord, 0)
	for i := 0; i < r; i++ {
		for j := 0; j < c; j++ {
			nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(i), float64(j)))
		}
	}

	re, g, oldToNewVIdMap, _ := buildCRP(t, nodeCoords, adjList, r*c, 13, 16)

	tnId := cellToNId(tx, ty)
	tid := oldToNewVIdMap[datastructure.Index(tnId)]
	at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1

	t.Log("calculating shortest paths...\n")

	minDist := pkg.INF_WEIGHT

	lock := sync.Mutex{}
	calcSp := func(p pair) any {
		snId := cellToNId(p.first, p.second)
		sid := oldToNewVIdMap[datastructure.Index(snId)]

		as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1

		crpQuery := routing.NewCRPBidirectionalSearch(re.GetRoutingEngine(), 1.0)
		spLength, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

		lock.Lock()
		defer lock.Unlock()
		minDist = math.Min(minDist, spLength)
		return nil
	}
	workers := concurrent.NewWorkerPool[pair, any](50, len(pintuUjungs))

	for _, p := range pintuUjungs {
		workers.AddJob(p)
	}

	workers.Close()
	workers.Start(calcSp)
	workers.Wait()

	ans := minDist
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
	if !eq(ans, expectedSPLength) {
		t.Fatalf("FAIL: Expected shortest path length: %v, got: %v", expectedSPLength, ans)
	}

	t.Logf("solved test case: %v", filepath)
}

// please run the test using command: "cd tests/shortestpath && go test -run TestShowroom  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode
func TestShowroom(t *testing.T) {

	dirPath := "./data/tests/shortestpath/ukiepc2016_showroom/"
	testDirs := []string{"secret", "sample"}

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
				SolveShowroom(t, testPath)

				runtime.GC()
				debug.FreeOSMemory()
			})

		}
	}
}

package maximumflow

import (
	"bufio"
	"os"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strconv"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
)

/*
taken from: https://icpcarchive.github.io/North%20America%20Contests/North%20Central%20Regional%20Contest/2020%20North%20Central%20Regional%20Contest/problems.pdf
problem B Ride-Hailing

test data: https://catsofwywop.com/icpc/data/ncna/2020/

my c++ solution (got AC on kattis: https://open.kattis.com/problems/ridehailing?tab=metadata):
https://drive.google.com/file/d/11PXNeUQA_0q5rB1sW9xFhfNJn4yj6N56/view?usp=sharing
// */

func SolveRideHailing(t *testing.T, filepath string) {
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

	k, err := strconv.Atoi(ff[2])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	adjList := make([][]pairEdge, n+1)

	for i := 0; i < m; i++ {
		var (
			u, v, w int
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

		w, err = strconv.Atoi(ff[2])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		adjList[u] = append(adjList[u], newPairEdge(v, float64(w)))
	}

	bitpack := func(u, v int) int {
		return u | (v << 16)
	}

	dijkstra := func(s int) map[int]int64 {
		djdist := make(map[int]int64)
		pq := da.NewFourAryHeap[int]()

		djdist[s] = 0
		pq.Insert(da.NewPriorityQueueNode(0, s))

		for !pq.IsEmpty() {

			queryKey, _ := pq.ExtractMin()

			u := queryKey.GetItem()
			uDist := int64(queryKey.GetRank())

			for _, vv := range adjList[u] {
				v := vv.to
				vDist := int64(vv.weight)
				_, ok := djdist[v]
				if !ok || (ok && uDist+vDist < djdist[v]) {
					djdist[v] = uDist + vDist
					if !ok {
						pq.Insert(da.NewPriorityQueueNode(float64(djdist[v]), v))
					} else {
						pq.DecreaseKey(da.NewPriorityQueueNode(float64(djdist[v]), v))
					}
				}
			}
		}

		return djdist
	}

	dist := make(map[int]int64)

	t.Logf("calculating all pairs shortest path....")
	for s := 1; s <= n; s++ {

		sdist := dijkstra(s)
		for t := 1; t <= n; t++ {
			dist[bitpack(s, t)] = sdist[t]
		}
	}

	type trip struct {
		u, v, t int
	}

	tripRequest := make([]trip, 0)

	for i := 0; i < k; i++ {
		var (
			u, v, time int
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

		time, err = strconv.Atoi(ff[2])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		tripRequest = append(tripRequest, trip{u, v, time})
	}

	source := da.Index(0)
	sink := da.Index(1)

	dg := da.NewPartitionGraph(k*2 + 2)
	dg.AddVertex(da.NewPartitionVertex(source, source, 0, 0))
	dg.AddVertex(da.NewPartitionVertex(sink, sink, 0, 0))

	for i := da.Index(1); i <= da.Index(len(tripRequest)); i++ {
		dg.AddVertex(da.NewPartitionVertex(i+1, i+1, 0, 0))
		dg.AddVertex(da.NewPartitionVertex(i+da.Index(k)+1, i+da.Index(k)+1, 0, 0))
	}

	for i := da.Index(1); i <= da.Index(len(tripRequest)); i++ {
		tt := tripRequest[i-1]
		u := tt.u
		v := tt.v
		ut := int64(tt.t)

		dg.AddEdge(source, i+1, 1, true)
		dg.AddEdge(i+da.Index(k)+1, sink, 1, true)
		for j := da.Index(1); j <= da.Index(len(tripRequest)); j++ {
			ttj := tripRequest[j-1]
			q := ttj.u
			qt := int64(ttj.t)
			if i == j {
				continue
			}

			if ut+dist[bitpack(u, v)]+dist[bitpack(v, q)] <= qt {
				dg.AddEdge(i+1, j+da.Index(k)+1, 1, true)
			}
		}
	}

	t.Logf("calculating maxflow....")
	dinic := partitioner.NewDinicMaxFlow(dg, false)
	mf := dinic.ComputeMaxflowMinCut(source, sink)
	ans := k - mf.GetMinCut()

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
	expectedAns, err := strconv.Atoi(line)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	if ans != expectedAns {
		t.Fatalf("FAIL: Expected minimum number of drivers: %v, got: %v", expectedAns, ans)
	}
	t.Logf("solved test case: %v", filepath)
}

func TestRideHailing(t *testing.T) {
	dirPath := "./data/ridehailing/"
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
				SolveRideHailing(t, testPath)

				runtime.GC()
				debug.FreeOSMemory()
			})

		}
	}
}

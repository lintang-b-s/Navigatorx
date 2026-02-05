package shortestpath

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"sort"
	"strconv"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
)

/*

taken from: https://tlx.toki.id/problems/osn-2024/2B


test data generated using: https://github.com/ia-toki/osn-2024/blob/main/osn-2024-krl/spec.cpp  && https://tcframe.toki.id/docs/topic-guides/spec/

tests selesai sekitar 2 menitan
shortcuts yang dihasilkan graph soal ini (jauh lebih banyak),beda dengan graph openstreetmap road networks


my c++ solution (got AC di semua subtasks tlx: https://tlx.toki.id/problems/osn-2024/2B):
https://drive.google.com/file/d/1KMMGWNHz4eAvEDppBrRUhN4vCdhzbIQT/view?usp=sharing
*/

type SegmentTree struct {
	n       int
	A       []int64
	st      []int64
	lazy    []int64
	minimum bool
}

func (st *SegmentTree) leftChild(parent int) int {
	return parent << 1
}

func (st *SegmentTree) rightChild(parent int) int {
	return (parent << 1) + 1
}

func (st *SegmentTree) conquer(a, b int64) int64 {
	if a == -1.0 {
		return b
	}
	if b == -1.0 {
		return a
	}
	if st.minimum {
		if a < b {
			return a
		}
		return b
	} else {
		if a > b {
			return a
		}
		return b
	}
}

func (st *SegmentTree) build(parent, L, R int) {
	if L == R {
		st.st[parent] = st.A[L]
	} else {
		mid := (L + R) / 2
		st.build(st.leftChild(parent), L, mid)
		st.build(st.rightChild(parent), mid+1, R)
		st.st[parent] = st.conquer(st.st[st.leftChild(parent)], st.st[st.rightChild(parent)])
	}
}

func (st *SegmentTree) propagate(parent, L, R int) {
	if st.lazy[parent] != -1 {
		// parent has a lazy flag
		st.st[parent] = st.lazy[parent]

		if L != R {
			st.lazy[st.rightChild(parent)] = st.lazy[parent]
			st.lazy[st.leftChild(parent)] = st.lazy[parent]
		} else {
			st.A[L] = st.lazy[parent]
		}

		st.lazy[parent] = -1.0
	}
}

func (st *SegmentTree) rmq(parent, L, R, i, j int) int64 {
	// O(log n)
	st.propagate(parent, L, R)
	if i > j {
		return -1.0
	}
	if (i <= L) && (j >= R) {
		return st.st[parent]
	}
	mid := (L + R) / 2
	return st.conquer(st.rmq(st.leftChild(parent), L, mid, i, min(j, mid)),
		st.rmq(st.rightChild(parent), mid+1, R, max(i, mid+1), j))
}

func (st *SegmentTree) updateRange(parent, L, R, i, j int, val int64) {
	// O(log n)
	st.propagate(parent, L, R)
	if i > j {
		return
	}
	if (i <= L) && (j >= R) {
		st.lazy[parent] = val
		st.propagate(parent, L, R)
	} else {
		mid := (L + R) / 2
		st.updateRange(st.leftChild(parent), L, mid, i, min(mid, j), val)
		st.updateRange(st.rightChild(parent), mid+1, R, max(i, mid+1), j, val)

		var left, right int64
		if st.lazy[st.leftChild(parent)] != -1 {
			left = st.lazy[st.leftChild(parent)]
		} else {
			left = st.st[st.leftChild(parent)]
		}

		if st.lazy[st.rightChild(parent)] != -1 {
			right = st.lazy[st.rightChild(parent)]
		} else {
			right = st.st[st.rightChild(parent)]
		}
		st.st[parent] = st.conquer(left, right)
	}
}

func NewSegmentTree(A []int64, minimum bool) *SegmentTree {
	lazy := make([]int64, 4*len(A))
	for i := range lazy {
		lazy[i] = -1.0
	}
	st := &SegmentTree{
		n:       len(A),
		A:       A,
		st:      make([]int64, 4*len(A)),
		lazy:    lazy,
		minimum: minimum,
	}
	st.build(1, 0, len(A)-1) // O(n)
	return st
}

func (st *SegmentTree) RMQ(i, j int) int64 {
	// O(log n)
	return st.rmq(1, 0, st.n-1, i, j)
}

func (st *SegmentTree) UpdateRange(i, j int, val int64) {
	// O(log n)
	st.updateRange(1, 0, st.n-1, i, j, val)
}

type rute struct {
	a, b int
	c, d int64
}

func newRute(a, b int, c, d int64) rute {
	return rute{a, b, c, d}
}

func SolveOSN2024KRL(t *testing.T, filepath string) {
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

	N, err := strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	M, err := strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	K, err := strconv.Atoi(ff[2])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	T, err := strconv.Atoi(ff[3])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	P, err := strconv.Atoi(ff[4])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	Q, err := strconv.Atoi(ff[5])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	rutes := make([]rute, M)
	for i := 0; i < M; i++ {
		var (
			a, b, c, d int64
		)
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff = fields(line)

		a, err = strconv.ParseInt(ff[0], 10, 64)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		b, err = strconv.ParseInt(ff[1], 10, 64)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		c, err = strconv.ParseInt(ff[2], 10, 64)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		d, err = strconv.ParseInt(ff[3], 10, 64)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		rutes[i] = newRute(int(a), int(b), c, d)
	}

	adjList := make([][]pairEdge, 2*N)

	stA := make([]int64, N)
	st := NewSegmentTree(stA, true)

	sort.Slice(rutes, func(i, j int) bool {
		return rutes[i].c > rutes[j].c
	}) // O(M *logM)

	for i := 0; i < M; i++ {
		// O(M * log N)
		rt := rutes[i]
		// set minimum cost edges dari sistem komuter.
		// A[0] = edge{0,1}, A[1] = edge{1,2},....
		st.UpdateRange(rt.a-1, rt.b-2, int64(rt.c)) // O(logN)

	}

	e := 0
	for i := 0; i < M; i++ {
		rt := rutes[i]
		a := rt.a - 1 + N
		b := rt.b - 1 + N

		// edges sistem ekspress
		adjList[a] = append(adjList[a], newPairEdge(b, float64(rt.d)))
		adjList[b] = append(adjList[b], newPairEdge(a, float64(rt.d)))
		e += 2
	}

	for v := 0; v < N; v++ {
		// edge non-transit vertex ke transit vertex
		adjList[v] = append(adjList[v], newPairEdge(v+N, float64(T)))
		// edge vertex ke non-transit vertex
		adjList[v+N] = append(adjList[v+N], newPairEdge(v, 0))
		e += 2
	}

	for v := 0; v+1 < N; v++ {
		// O(N * log N)
		// angkot
		// stasiun non-transit ke next stasiun non-transit
		adjList[v] = append(adjList[v], newPairEdge(v+1, float64(K)))
		adjList[v+1] = append(adjList[v+1], newPairEdge(v, float64(K)))

		e += 2

		rmqv := st.RMQ(v, v) // O(log N)
		if rmqv == 0 {
			continue
		}

		// commuter edges
		adjList[v+N] = append(adjList[v+N], newPairEdge(v+1+N, float64(rmqv)))
		adjList[v+1+N] = append(adjList[v+1+N], newPairEdge(v+N, float64(rmqv)))

		e += 2
	}

	P--
	Q--

	nodeCoords := make([]osmparser.NodeCoord, 0)
	for i := 0; i < N; i++ {
		nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(i), float64(i)))
	}

	for i := N; i < 2*N; i++ {
		nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(i-N), float64(i-N)))
	}

	u1 := 11.0
	if (2.0*float64(e))/math.Pow(float64(N)*2.0, 2) >= 0.00005 {
		u1 = 20.0
	} // jumlah shortcuts nya kebanyakan aowkwowk > 100jt kalau test case nya dense
	re, g, oldToNewVIdMap, _ := buildCRP(t, nodeCoords, adjList, N*2, u1, 22)

	t.Logf("calculating shortest path from P: %v, to: Q: %v\n", P+1, Q+1)

	sid := oldToNewVIdMap[datastructure.Index(P)]
	tid := oldToNewVIdMap[datastructure.Index(Q)]
	tidTransit := oldToNewVIdMap[datastructure.Index(Q+N)]

	as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1
	at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1
	atTransit := g.GetEntryOffset(tidTransit) + g.GetInDegree(tidTransit) - 1

	crpQuery := routing.NewCRPBidirectionalSearch(re.GetRoutingEngine(), 1.0)
	spLength, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

	crpQuery2 := routing.NewCRPBidirectionalSearch(re.GetRoutingEngine(), 1.0)
	spLengthTransit, _, _, _, _ := crpQuery2.ShortestPathSearch(as, atTransit)

	var ans float64
	if spLength < spLengthTransit {
		ans = spLength
	} else {
		ans = spLengthTransit
	}

	// assert expected output dari test cases soal

	fOut, err = os.OpenFile(filepath+".out", os.O_RDONLY, 0644)
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

// please run the test using command: "cd tests/shortestpath && go test -run TestOSN2024KRL  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode,
func TestOSN2024KRL(t *testing.T) {

	dirPath := "./data/tests/shortestpath/osn2024_krl/"
	testDirs := []string{"tc"}

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
				SolveOSN2024KRL(t, testPath)

				runtime.GC()
				debug.FreeOSMemory()
			})

		}
	}
}

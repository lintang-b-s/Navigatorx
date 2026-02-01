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
taken from: https://icpcarchive.github.io/Europe%20Subcontests/German%20Collegiate%20Programming%20Contest%20(GCPC)/2013%20German%20Collegiate%20Programming%20Contest/problems.pdf
problem H: The King of the North

test data: https://archive.algo.is/icpc/nwerc/gcpc/2013/

my c++ solution (got AC on kattis: https://open.kattis.com/problems/thekingofthenorth?tab=metadata):
https://drive.google.com/file/d/1i6Kokh-IgvISSyCnGmGkKkWWLV9ZDdCz/view?usp=sharing

*/

const (
	INF = 1e12
)

func SolveTheKingOfTheNorth(t *testing.T, filepath string) {
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

	R, err := strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	C, err := strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	kingdom := make([][]int, R)
	for i := 0; i < len(kingdom); i++ {
		kingdom[i] = make([]int, C)
	}

	for i := 0; i < R; i++ {
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := fields(line)
		for j := 0; j < C; j++ {

			numBannerMen, err := strconv.Atoi(ff[j])
			if err != nil {
				t.Fatalf("err: %v", err)
			}

			kingdom[i][j] = numBannerMen
		}
	}

	// multi-sources nya ada di luar border kanan kiri atas (gabisa diagonal)
	// tinggal cari maxflow/mincut dari multisources(semua posisi lawan) ke castle out vertex(sink)
	// vertices id:
	// karena ini vertices with capacity kita harus splice jadi 2 vertices: (vertex-in, vertex-out)
	// for each cell (i,j) , verticeId = (i*m+j, r*c+i*m+j)
	// source=castle_i*m+ castle_j
	// for each sources:
	// sources atas, diatas border kingdom, ada C sources: R*C+R*C+k, for each 1<=k<=C
	// sources kiri, dikiri border kingdom, ada R sources: R*C+R*C+C+k, for each 1<=k<=R
	// sources kanan, dikanan border kingdom, ada R sources: R*C+R*C+C+R+k, for each 1<=k<=R
	// sources bawah, dibawah border kingdom, ada C sources: R*C+R*C+C+R+R+k, for each 1<=k<=C

	dg := da.NewPartitionGraph(R*C + R*C + C + R + R + C + 1)

	for i := 0; i < R; i++ {
		for j := 0; j < C; j++ {
			inVId := da.Index(i*C + j)
			outVId := da.Index(R*C + i*C + j)
			dg.AddVertex(da.NewPartitionVertex(inVId, inVId, 0, 0))
			dg.AddVertex(da.NewPartitionVertex(outVId, outVId, 0, 0))
		}
	}

	for i := 0; i < R; i++ {
		for j := 0; j < C; j++ {

			inVId := da.Index(i*C + j)
			outVId := da.Index(R*C + i*C + j)

			dg.AddEdgeW(inVId, outVId, kingdom[i][j], true)

			if i+1 <= R-1 {
				bawahInVid := da.Index((i+1)*C + j)
				dg.AddEdgeW(outVId, bawahInVid, kingdom[i+1][j], true)
			}

			if j+1 <= C-1 {
				kananVId := da.Index((i)*C + (j + 1))
				dg.AddEdgeW(outVId, kananVId, kingdom[i][(j+1)], true)
			}

			if i-1 >= 0 {
				atasVId := da.Index((i-1)*C + j)
				dg.AddEdgeW(outVId, atasVId, kingdom[i-1][j], true)
			}

			if j-1 >= 0 {
				kiriVId := da.Index(i*C + (j - 1))
				dg.AddEdgeW(outVId, kiriVId, kingdom[i][j-1], true)
			}
		}
	}

	// dari posisi enemies ke border army kerajaan

	// karena multi-sources kita harus tambah artificial source, dan tambahkan edges dari supersource ke semua sources degnan INF weight
	superSource := da.Index(R*C + R*C + C + R + R + C)

	dg.AddVertex(da.NewPartitionVertex(superSource, superSource, 0, 0))

	// atas
	for k := 1; k <= C; k++ {
		musuhVId := da.Index(R*C + R*C + k)
		dg.AddVertex(da.NewPartitionVertex(musuhVId, musuhVId, 0, 0))

		pasukanAtasInVId := da.Index(k)
		dg.AddEdgeW(musuhVId, pasukanAtasInVId, INF, true)
		dg.AddEdgeW(superSource, musuhVId, INF, true)
	}

	// bawah
	for k := 1; k <= C; k++ {
		musuhVId := da.Index(R*C + R*C + C + R + R + k)
		dg.AddVertex(da.NewPartitionVertex(musuhVId, musuhVId, 0, 0))

		pasukanBawahInVId := da.Index((R-1)*C + k)
		dg.AddEdgeW(musuhVId, pasukanBawahInVId, INF, true)
		dg.AddEdgeW(superSource, musuhVId, INF, true)
	}

	// kiri
	for k := 1; k <= R; k++ {
		musuhVId := da.Index(R*C + R*C + C + k)
		dg.AddVertex(da.NewPartitionVertex(musuhVId, musuhVId, 0, 0))

		pasukanKiriInVId := da.Index(k*C + 0)
		dg.AddEdgeW(musuhVId, pasukanKiriInVId, INF, true)
		dg.AddEdgeW(superSource, musuhVId, INF, true)
	}

	// kanan
	for k := 1; k <= R; k++ {
		musuhVId := da.Index(R*C + R*C + C + R + k)
		dg.AddVertex(da.NewPartitionVertex(musuhVId, musuhVId, 0, 0))

		pasukanKananInVId := da.Index(k*C + (C - 1))
		dg.AddEdgeW(musuhVId, pasukanKananInVId, INF, true)
		dg.AddEdgeW(superSource, musuhVId, INF, true)
	}

	line, err = readLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	ff = fields(line)

	castlei, err := strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	castlej, err := strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	castleOutVId := da.Index(R*C + (castlei*C + castlej))
	dinic := partitioner.NewDinicMaxFlow(dg, false)
	mf := dinic.ComputeMaxflowMinCut(superSource, castleOutVId)

	ans := mf.GetMinCut()

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
	expectedAns, err := strconv.Atoi(line)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	if ans != expectedAns {
		t.Fatalf("FAIL: Expected smallest possible army: %v, got: %v", expectedAns, ans)
	}
	t.Logf("solved test case: %v", filepath)
}

func TestTheKingOfTheNorth(t *testing.T) {
	dirPath := "./data/thekingofthenorth/"
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
				SolveTheKingOfTheNorth(t, testPath)

				runtime.GC()
				debug.FreeOSMemory()
			})

		}
	}
}

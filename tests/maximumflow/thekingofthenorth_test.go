package maximumflow

import (
	"bufio"
	"os"
	"path/filepath"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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

	f, err = os.OpenFile(filepath+".in", os.O_RDONLY, 0600)
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

	R, err := util.ParseTextInt(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	C, err := util.ParseTextInt(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	kingdom := make([][]int, R)
	for i := 0; i < len(kingdom); i++ {
		kingdom[i] = make([]int, C)
	}

	for i := 0; i < R; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := util.Fields(line)
		for j := 0; j < C; j++ {
			numBannerMen, err := util.ParseTextInt(ff[j])
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
	// for each cell (i,j), verticeId = (i*c+j, r*c+i*c+j)
	// sink=castle_i*c+ castle_j
	// for each sources:
	// sources atas, diatas border kingdom, ada C sources: R*C+R*C+k, for each 1<=k<=C
	// sources kiri, dikiri border kingdom, ada R sources: R*C+R*C+C+k, for each 1<=k<=R
	// sources kanan, dikanan border kingdom, ada R sources: R*C+R*C+C+R+k, for each 1<=k<=R
	// sources bawah, dibawah border kingdom, ada C sources: R*C+R*C+C+R+R+k, for each 1<=k<=C

	dn := partitioner.NewDinicMaxFlow[int64](R*C+R*C+C+R+R+C, true, false)
	for i := 0; i < R; i++ {
		for j := 0; j < C; j++ {

			inVId := da.Index(i*C + j)
			outVId := da.Index(R*C + i*C + j)

			dn.AddEdge(inVId, outVId, int64(kingdom[i][j]), true)

			if i+1 <= R-1 {
				bawahInVid := da.Index((i+1)*C + j)
				dn.AddEdge(outVId, bawahInVid, int64(kingdom[i+1][j]), true)
			}

			if j+1 <= C-1 {
				kananVId := da.Index((i)*C + (j + 1))
				dn.AddEdge(outVId, kananVId, int64(kingdom[i][(j+1)]), true)
			}

			if i-1 >= 0 {
				atasVId := da.Index((i-1)*C + j)
				dn.AddEdge(outVId, atasVId, int64(kingdom[i-1][j]), true)
			}

			if j-1 >= 0 {
				kiriVId := da.Index(i*C + (j - 1))
				dn.AddEdge(outVId, kiriVId, int64(kingdom[i][j-1]), true)
			}
		}
	}

	// dari posisi enemies ke border army kerajaan
	// atas
	for k := 1; k <= C; k++ {
		musuhVId := da.Index(R*C + R*C + k)
		pasukanAtasInVId := da.Index(k)
		dn.AddEdge(musuhVId, pasukanAtasInVId, INF, true)
	}

	// bawah
	for k := 1; k <= C; k++ {
		musuhVId := da.Index(R*C + R*C + C + R + R + k)
		pasukanBawahInVId := da.Index((R-1)*C + k)
		dn.AddEdge(musuhVId, pasukanBawahInVId, INF, true)
	}

	// kiri
	for k := 1; k <= R; k++ {
		musuhVId := da.Index(R*C + R*C + C + k)
		pasukanKiriInVId := da.Index(k * C)
		dn.AddEdge(musuhVId, pasukanKiriInVId, INF, true)
	}

	// kanan
	for k := 1; k <= R; k++ {
		musuhVId := da.Index(R*C + R*C + C + R + k)
		pasukanKananInVId := da.Index(k*C + (C - 1))
		dn.AddEdge(musuhVId, pasukanKananInVId, INF, true)
	}

	// karena multi-sources kita harus tambah artificial source, dan tambahkan edges dari supersource ke semua sources degnan INF weight
	superSource := da.Index(R*C + R*C + C + R + R + C)
	dn.AddArtificialVertex(superSource)

	// atas
	for k := 1; k <= C; k++ {
		musuhVId := da.Index(R*C + R*C + k)
		dn.AddEdge(superSource, musuhVId, INF, true)
	}

	// bawah
	for k := 1; k <= C; k++ {
		musuhVId := da.Index(R*C + R*C + C + R + R + k)
		dn.AddEdge(superSource, musuhVId, INF, true)
	}

	// kiri
	for k := 1; k <= R; k++ {
		musuhVId := da.Index(R*C + R*C + C + k)
		dn.AddEdge(superSource, musuhVId, INF, true)
	}

	// kanan
	for k := 1; k <= R; k++ {
		musuhVId := da.Index(R*C + R*C + C + R + k)
		dn.AddEdge(superSource, musuhVId, INF, true)
	}

	line, err = util.ReadLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	ff = util.Fields(line)

	castlei, err := util.ParseTextInt(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	castlej, err := util.ParseTextInt(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	castleOutVId := da.Index(R*C + (castlei*C + castlej))
	mf := dn.ComputeMaxflowMinCut(superSource, castleOutVId)

	ans := mf.GetMaxFlow()

	fOut, err = os.OpenFile(filepath+".out", os.O_RDONLY, 0600)
	if err != nil {
		t.Fatalf("could not open test file: %v", err)
	}
	defer fOut.Close()

	brOut := bufio.NewReader(fOut)
	line, err = util.ReadLine(brOut)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	expectedAns, err := util.ParseTextInt(line)
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

			})

		}
	}
}

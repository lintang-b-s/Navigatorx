package shortestpath

import (
	"bufio"
	"errors"
	"fmt"
	"io"
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

taken from: https://2022.nwerc.eu/main/problem-set.pdf
problem D: Delft Distance

contest page: https://2022.nwerc.eu/
test cases: https://chipcie.wisv.ch/archive/2022/nwerc/solutions.zip

skip test cases yang jumlah edges nya > 600k, biar cepet

 jangan submit solusi ini ke online judge, karen bakal TLE & MLE  wkwkwk
 pakai solusi c++ ku aja (got AC on kattis: https://open.kattis.com/problems/delftdistance):
 https://drive.google.com/file/d/1vNQ7GQH1JJJn-Z0u4EW_Nsk9AvFsSfdS/view?usp=sharing

This test will be completed in approximately 10-15 minutes.
*/

func readLine(br *bufio.Reader) (string, error) {
	line, err := br.ReadString('\n')
	if err != nil {
		if errors.Is(err, io.EOF) && len(line) > 0 {
		} else {
			return "", err
		}
	}
	return strings.TrimRight(line, "\r\n"), nil
}

func fields(s string) []string {

	return strings.Fields(s)
}

type pairEdge struct {
	to     int
	weight float64
}

func flattenEdges(es [][]pairEdge) []osmparser.Edge {
	flatten := make([]osmparser.Edge, 0, len(es))

	eid := 0

	for from, edges := range es {
		for _, e := range edges {
			flatten = append(flatten, osmparser.NewEdge(uint32(from), uint32(e.to), e.weight, 0, uint32(eid)))
			eid++
		}
	}

	return flatten
}

func solve(t *testing.T, filepath string) {
	var (
		err     error
		line    string
		h, w    int
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
	h, err = strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	w, err = strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	peta := make([][]string, h)
	for i := 0; i < h; i++ {
		peta[i] = make([]string, w)
	}

	for i := 0; i < h; i++ {
		line, err = readLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		for j := 0; j < w; j++ {
			peta[i][j] = fmt.Sprintf("%v", string(line[j]))
		}
	}

	adjList := make([][]pairEdge, h*w*4+2)

	cellToNId := func(i, j, nid int) int {
		return i*w + j + 3*(w*i+j) + nid
	}

	seperempatDiam := 2.5 * math.Pi

	for i := 0; i < h; i++ {
		for j := 0; j < w; j++ {
			cur := peta[i][j]
			if cur == "X" {
				i0 := cellToNId(i, j, 0)
				i1 := cellToNId(i, j, 1)
				i2 := cellToNId(i, j, 2)
				i3 := cellToNId(i, j, 3)

				// cur tower = tower kotak
				// buat edge ke 4 node dari x, atas kanan kiri bawah persegi
				adjList[i0] = append(adjList[i0], pairEdge{i1, 10})
				adjList[i1] = append(adjList[i1], pairEdge{i2, 10})
				adjList[i2] = append(adjList[i2], pairEdge{i3, 10})
				adjList[i3] = append(adjList[i3], pairEdge{i0, 10})

				adjList[i1] = append(adjList[i1], pairEdge{i0, 10})
				adjList[i2] = append(adjList[i2], pairEdge{i1, 10})
				adjList[i3] = append(adjList[i3], pairEdge{i2, 10})
				adjList[i0] = append(adjList[i0], pairEdge{i3, 10})

				if j+1 < w {

					j10 := cellToNId(i, j+1, 0)
					j12 := cellToNId(i, j+1, 2)
					j13 := cellToNId(i, j+1, 3)

					// buat edge ke tower kanan
					kanan := peta[i][j+1]
					if kanan == "O" {
						adjList[i0] = append(adjList[i0], pairEdge{j10, 10})
						adjList[i1] = append(adjList[i1], pairEdge{j13, 0})
						adjList[i2] = append(adjList[i2], pairEdge{j12, 10})
					} else {
						adjList[i0] = append(adjList[i0], pairEdge{j10, 10})
						adjList[i1] = append(adjList[i1], pairEdge{j13, 0})
						adjList[i2] = append(adjList[i2], pairEdge{j12, 10})
					}
				}

				if i+1 < h {
					i10 := cellToNId(i+1, j, 0)
					i11 := cellToNId(i+1, j, 1)
					i13 := cellToNId(i+1, j, 3)

					// buat edge ke tower bawah
					bawah := peta[i+1][j]
					if bawah == "O" {
						adjList[i2] = append(adjList[i2], pairEdge{i10, 0})
						adjList[i3] = append(adjList[i3], pairEdge{i13, 10})
						adjList[i1] = append(adjList[i1], pairEdge{i11, 10})
					} else {
						adjList[i2] = append(adjList[i2], pairEdge{i10, 0})
						adjList[i3] = append(adjList[i3], pairEdge{i13, 10})
						adjList[i1] = append(adjList[i1], pairEdge{i11, 10})
					}
				}
			} else {

				// cur tower = tower bundar
				// buat edge ke 4 node dari o, atas kanan kiri bawah pas lingkaran
				i0 := cellToNId(i, j, 0)
				i1 := cellToNId(i, j, 1)
				i2 := cellToNId(i, j, 2)
				i3 := cellToNId(i, j, 3)

				adjList[i0] = append(adjList[i0], pairEdge{i1, seperempatDiam})
				adjList[i1] = append(adjList[i1], pairEdge{i2, seperempatDiam})
				adjList[i2] = append(adjList[i2], pairEdge{i3, seperempatDiam})
				adjList[i3] = append(adjList[i3], pairEdge{i0, seperempatDiam})

				adjList[i1] = append(adjList[i1], pairEdge{i0, seperempatDiam})
				adjList[i2] = append(adjList[i2], pairEdge{i1, seperempatDiam})
				adjList[i3] = append(adjList[i3], pairEdge{i2, seperempatDiam})
				adjList[i0] = append(adjList[i0], pairEdge{i3, seperempatDiam})

				if j+1 < w {
					j10 := cellToNId(i, j+1, 0)
					j12 := cellToNId(i, j+1, 2)
					j13 := cellToNId(i, j+1, 3)

					// buat edge ke tower kanan
					kanan := peta[i][j+1]
					if kanan == "O" {
						adjList[i0] = append(adjList[i0], pairEdge{j10, 10})

						adjList[i1] = append(adjList[i1], pairEdge{j13, 0})

						adjList[i2] = append(adjList[i2], pairEdge{j12, 10})
					} else {
						adjList[i0] = append(adjList[i0], pairEdge{j10, 10})
						adjList[i1] = append(adjList[i1], pairEdge{j13, 0})
						adjList[i2] = append(adjList[i2], pairEdge{j12, 10})
					}
				}

				if i+1 < h {
					i10 := cellToNId(i+1, j, 0)
					i11 := cellToNId(i+1, j, 1)
					i13 := cellToNId(i+1, j, 3)
					// buat edge ke tower bawah
					bawah := peta[i+1][j]
					if bawah == "O" {
						adjList[i2] = append(adjList[i2], pairEdge{i10, 0})

						adjList[i3] = append(adjList[i3], pairEdge{i13, 10})

						adjList[i1] = append(adjList[i1], pairEdge{i11, 10})
					} else {
						adjList[i2] = append(adjList[i2], pairEdge{i10, 0})
						adjList[i3] = append(adjList[i3], pairEdge{i13, 10})
						adjList[i1] = append(adjList[i1], pairEdge{i11, 10})
					}
				}
			}
		}
	}

	source := h * w * 4
	target := h*w*4 + 1

	adjList[source] = append(adjList[source], pairEdge{cellToNId(0, 0, 0), 5.0})

	adjList[source] = append(adjList[source], pairEdge{cellToNId(0, 0, 3), 5.0})

	last1 := cellToNId(h-1, w-1, 1)
	last2 := cellToNId(h-1, w-1, 2)

	adjList[last1] = append(adjList[last1], pairEdge{target, 5.0})

	adjList[last2] = append(adjList[last2], pairEdge{target, 5.0})

	n := h*w*4 + 2
	re, g, oldToNewVIdMap, _ := buildCRP(t, adjList, n, 11, 15)

	crpQuery := routing.NewCRPBidirectionalSearch(re.GetRoutingEngine(), 1.0)

	sid := oldToNewVIdMap[datastructure.Index(source)]
	tid := oldToNewVIdMap[datastructure.Index(target)]

	as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1
	at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1

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

	t.Logf("solved test case: %v", filepath)
}

// please run the test using command: "cd tests && go test ./... -v -timeout=0"
// karena bakal timeout kalau pakai run test vscode
func TestCRPQueryDelftDistance(t *testing.T) {

	dirPath := "./data/tests/shortestpath/icpc_nwerc2022_delftdistance/"
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
				solve(t, testPath)

				runtime.GC()
				debug.FreeOSMemory()
			})

		}
	}
}

const (
	EPS = 1e-9
)

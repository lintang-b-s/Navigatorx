package stronglyconnectedcomponents

import (
	"bufio"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/Navigatorx/tests"
)

/*

taken from: https://cs.baylor.edu/~hamerly/icpc/qualifier_2015/problemset-naq-2015.pdf
Problem C: Cantina of Babel

test data: https://cs.baylor.edu/~hamerly/icpc/qualifier_2015/naq15-data.zip
*/

func SolveCantinaOfBabel(t *testing.T, filepath string) {
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

	line, err = util.ReadLine(br)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	ff := util.Fields(line)

	N, err := util.ParseTextInt(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	type bhs struct {
		bahasaSpeak      string
		bahasaUnderstand map[string]struct{}
	}

	type karakter struct {
		nama    string
		bahasak bhs
	}

	starwars := make([]karakter, 0)

	for i := 0; i < N; i++ {
		var (
			nama, bahasaSpeak string
		)
		bahasaUnderstand := make(map[string]struct{})
		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		ss := util.Fields(line)
		for j, word := range ss {
			switch j {
			case 0:
				nama = string(word)
			case 1:
				bahasaSpeak = string(word)
			default:
				bahasaUnderstand[string(word)] = struct{}{}
			}
		}

		starwars = append(starwars, karakter{nama, bhs{bahasaSpeak: bahasaSpeak,
			bahasaUnderstand: bahasaUnderstand}})
	}

	n := len(starwars)
	var (
		adjList  = make([][]tests.PairEdge, n)
		adjListT = make([][]tests.PairEdge, n)
	)

	bitpack := func(i, j int) int {
		return i | (j << 16)
	}

	edgesSet := make(map[int]struct{})

	for i := 0; i < n; i++ {
		swi := starwars[i]
		for j := 0; j < n; j++ {
			if i == j {
				continue
			}
			swj := starwars[j]

			if _, ok := edgesSet[bitpack(i, j)]; ok {
				continue
			}

			if swi.bahasak.bahasaSpeak == swj.bahasak.bahasaSpeak {
				adjList[i] = append(adjList[i], tests.NewPairEdge(j, 1))
				adjList[j] = append(adjList[j], tests.NewPairEdge(i, 1))

				edgesSet[bitpack(i, j)] = struct{}{}
				edgesSet[bitpack(j, i)] = struct{}{}

				adjListT[j] = append(adjListT[j], tests.NewPairEdge(i, 1))
				adjListT[i] = append(adjListT[i], tests.NewPairEdge(j, 1))
			} else if _, ok := swj.bahasak.bahasaUnderstand[swi.bahasak.bahasaSpeak]; ok {
				edgesSet[bitpack(i, j)] = struct{}{}

				adjList[i] = append(adjList[i], tests.NewPairEdge(j, 1))
				adjListT[j] = append(adjListT[j], tests.NewPairEdge(i, 1))
			}
		}
	}

	es := tests.FlattenEdges(adjList)

	op := osmparser.NewOSMParserV2()
	gs := datastructure.NewGraphStorageWithSize(len(es), n)
	g, _, _ := op.BuildGraphFloat64(es, gs, uint32(n), false)

	g.RunKosaraju()

	sccSizes := make(map[datastructure.Index]int)
	for _, scc := range g.GetSCCS() {
		sccSizes[scc]++
	}

	maxsccSize := 0
	for _, size := range sccSizes {
		if size > maxsccSize {
			maxsccSize = size
		}
	}

	ans := n - maxsccSize

	fOut, err = os.OpenFile(filepath+".ans", os.O_RDONLY, 0644)
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

	if expectedAns != ans {
		t.Fatalf("FAIL: Expected smallest possible army: %v, got: %v", expectedAns, ans)
	}
	t.Logf("solved test case: %v", filepath)
}

func TestCantinaOfBabelSCC(t *testing.T) {
	dirPath := "./data/cantinaofbabel/"
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
				SolveCantinaOfBabel(t, testPath)

			})

		}
	}
}

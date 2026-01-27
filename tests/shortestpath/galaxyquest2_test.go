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

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

/*
taken from: https://2023.nwerc.eu/main/problem-set.pdf



karena Customizable Route Planning only support point-to-point shortest path,
dan soal ini butuh compute shortest path distance from planet 1 to other planets..
test ini menjalankan query CRP dari source ke other vertices satu persatu

test data: https://chipcie.wisv.ch/archive/2023/nwerc/solutions.zip
exclude test cases that contains > 50k edges or > 30k vertices, to speed up the testings,
agar ukuran reponya ga kegedean juga heheh.
kalau mau test pakai semua test cases tinggal download dari link diatas and taruh di "tests/shortestpath/data/tests/shortestpath/icpc_nwerc2023_galaxyquest"

my c++ solution (got AC on kattis: https://open.kattis.com/problems/galaxyquest2):
https://drive.google.com/file/d/1ulMqGc3n89_jfIfiOAAedAhYfL4lo9Pj/view?usp=sharing

solution sketch:
ref: https://icpcarchive.github.io/Europe%20Contests/Northwestern%20Europe%20Regional%20Contest%20(NWERC)/2023%20Northwestern%20Europe%20Regional%20Contest/solution.pdf

let:
x_i = time for traverse the segment-i using acceleration/decceleration
d_i = segment-i length
decelerationTime_{i}=accelerationTime_{i}=x_i

total fuel time = \sum_{i} accelerationTime_{i} + deccelerationTime_{i} = 2 * \sum_{i} x_i

for each segment we accelerate, then constant velocity, then deccelerate, so:
constantSpeedTime_{i}= \frac{d_i - x_i^2}{x_i}  = \frac{d_i}{x_i} - x_i

total time take to traverse the segment-i =  accelerationTime_{i}+deccelerationTime_{i}+constantSpeedTime_{i}
                                          = x_i + x_i + (\frac{d_i}{x_i} - x_i)
                                          = \frac{d_i}{x_i} + x_i

the problem ask minimum fuel to take: \min 2 *\sum_{i} x_i , with constraint: time to reach the target < t
this is a constrained optimization problem, can be solved using langrange multiplier: https://tutorial.math.lamar.edu/classes/calciii/lagrangemultipliers.aspx

\min 2 * \sum_{i} x_i, with constraint: \sum_{i} \frac{d_i}{x_i} + x_i <= t

using langrange multiplier, we get:

1. grad(2 * \sum_{i} x_i) = \lambda * grad( \sum_{i} \frac{d_i}{x_i} + x_i ) , dimana grad() = gradient vector

2. \sum_{i} \frac{d_i}{x_i} + x_i  = t

solve this eq (1):
for each i we have:
2 = (1 - \frac{d_i}{x_i^2}) * lambda
with alg. manip., we get:

x_i = \frac{sqrt(d_i) }{ sqrt(1- \frac{2}{\lambda}) }
let c = \frac{1}{sqrt(1- \frac{2}{\lambda})}

x_i = c *sqrt(d_i)   (3)


subtituting eq (3) to eq (2) & doing some alg. manip., we get:
c + \frac{1}{c} = \frac{t}{\sum_{i} sqrt(d_i)} (4)

let q = \frac{t}{\sum_{i} sqrt(d_i)}

using abc formula, we get:
c = \frac{q +- \sqrt(q^2-4)}{2}

bcs we want to find \min 2 *\sum_{i} x_i = 2 * \min  * \sum_{i} c *sqrt(d_i)

we use the min c:
c = cLangrange (in this program) = \frac{q - \sqrt(q^2-4)}{2}

for calc \min \sum_{i} sqrt(d_i) , we can use dijkstra algorithm

if q^2 - 4 < 0, then return impossible (impossible to reach planet terget)
else:  just return the  2 * \min  * \sum_{i} cLangrange *sqrt(d_i)


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

	re, g, oldToNewVIdMap, _ := buildCRP(t, adjList, n, 11, 12)
	s := 0
	sid := oldToNewVIdMap[datastructure.Index(s)]
	as := g.GetExitOffset(sid) + g.GetOutDegree(sid) - 1

	dist := make(map[int]float64)
	dist[s] = 0

	t.Logf("calculating shortest paths from planet 1 to other planets.....")
	for v := 1; v < n; v++ {
		crpQuery := routing.NewCRPBidirectionalSearch(re.GetRoutingEngine(), 1.0)

		tid := oldToNewVIdMap[datastructure.Index(v)]
		at := g.GetEntryOffset(tid) + g.GetInDegree(tid) - 1

		spLength, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

		dist[v] = spLength
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

// please run the test using command: "cd tests && go test ./... -v -timeout=0"
// karena bakal timeout kalau pakai run test vscode
func TestCRPQueryGalaxyQuest(t *testing.T) {

	dirPath := "./data/tests/shortestpath/icpc_nwerc2023_galaxyquest/"
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

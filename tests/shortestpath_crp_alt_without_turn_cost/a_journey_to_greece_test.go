package shortestpath_crp_alt_without_turn_cost

import (
	"bufio"
	"math/bits"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	nvlog "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/Navigatorx/tests"
)

const (
	MAX_N                    = 16
	INF               int64  = 1 << 60
	greeceTestDataUrl string = "https://drive.google.com/uc?export=download&id=1m60bikUTTMVTm3ATja8AwETi3W0GfJqu"
	greeceZipFilePath        = "../shortestpath/data/tests/shortestpath/gcpc2015_a_journey_to_greece/greece.zip"
	greeceZipDirPath         = "../shortestpath/data/tests/shortestpath/gcpc2015_a_journey_to_greece"
)

/*
taken from problem A "A Journey to Greece": https://2015.gcpc.nwerc.eu/problemset_2015.pdf
test data: https://2015.gcpc.nwerc.eu/data_2015.zip

my c++ solution (AC https://codeforces.com/gym/100753/attachments): https://drive.google.com/file/d/1X-pOEvAfaFhKBJ1t96tu4pPuoZMoYrbg/view?usp=sharing

*/

/*
dp(S,j) minimum cost path start at athens coordinate 0, visit all places in the set S exactly once,
ends at place j. j and athens in S.

S is a subset of the set of all places + athens. there is 2^{n-1} possible subsets S that contain athens coordinate 0.
base case:
dp({0,j}, j) = w(0,j)

recurrence:
dp(S,j) = min(dp(S\{j}, i) + w(i,j)), for all i != j

solution:
min(dp(V,j)+w(j,0)), for all j.

dpTSP compute minimum cost at vertex u and have visited all vertices described by off bit in mask.
time: O(2^{n-1}*n^2), number of subsets 2^{n-1} * number of possible value of j * worst case O(n) works for each state
*/
func dpTSP(u, mask int, dp *[MAX_N][1 << (MAX_N - 1)]int64, dist [MAX_N][MAX_N]int64) int64 {
	if mask == 0 {
		return dist[u][0]
	}

	ans := dp[u][mask]
	if ans != -1 {
		return ans
	}

	dp[u][mask] = INF
	m := mask
	for m != 0 {
		twoPowV := tests.LSOne(m)                  // least significant bit
		v := bits.TrailingZeros(uint(twoPowV)) + 1 // 2^j=two_pow_v, get j+1
		dp[u][mask] = min(dp[u][mask], dist[u][v]+dpTSP(v, mask^twoPowV, dp, dist))
		m -= twoPowV
	}

	return dp[u][mask]
}

func dpTSPWithTaxi(u, mask int, dp *[MAX_N][1 << (MAX_N - 1)][2]int64, dist [MAX_N][MAX_N]int64, usedTaxi uint8, taxiTime int64) int64 {
	if mask == 0 {

		ans := dist[u][0]
		if usedTaxi == 0 {
			ans = min(ans, taxiTime)
		}

		return ans
	}

	ans := dp[u][mask][usedTaxi]
	if ans != -1 {
		return ans
	}

	dp[u][mask][usedTaxi] = INF
	m := mask
	for m != 0 {
		twoPowV := tests.LSOne(m)                  // least significant bit
		v := bits.TrailingZeros(uint(twoPowV)) + 1 // 2^j=two_pow_v, get j+1
		dp[u][mask][usedTaxi] = min(dp[u][mask][usedTaxi], dist[u][v]+dpTSPWithTaxi(v, mask^twoPowV, dp, dist, usedTaxi, taxiTime))
		if usedTaxi == 0 {
			dp[u][mask][usedTaxi] = min(dp[u][mask][usedTaxi], taxiTime+dpTSPWithTaxi(v, mask^twoPowV, dp, dist, 1, taxiTime))
		}
		m -= twoPowV
	}

	return dp[u][mask][usedTaxi]
}

func solveGreece(t *testing.T, filepath string) {
	var (
		err  error
		line string

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
	var (
		n, p, m, g, taxiTime int
	)

	n, err = strconv.Atoi(ff[0])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	p, err = strconv.Atoi(ff[1])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	m, err = strconv.Atoi(ff[2])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	g, err = strconv.Atoi(ff[3])
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	taxiTime, err = strconv.Atoi(ff[4])
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	type place struct {
		v, stayTime int
	}

	placeStay := make([]place, p)

	for i := 0; i < p; i++ {
		var (
			v, stayTime int
		)
		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := util.Fields(line)

		v, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		stayTime, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		placeStay[i] = place{v, stayTime}
	}

	adjList := make([][]tests.PairEdge, n)

	for i := 0; i < m; i++ {
		var (
			u, v, w int
		)
		line, err = util.ReadLine(br)
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		ff := util.Fields(line)

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

		adjList[u] = append(adjList[u], tests.NewPairEdge(v, float64(w)))
		adjList[v] = append(adjList[v], tests.NewPairEdge(u, float64(w)))
	}

	nodeCoords := tests.AssignNodeCoordinates(adjList)

	us := []int{4, 6}
	density := tests.Density(n, m, false)
	t.Logf("density: %v\n", density)
	if density >= 0.02 || m >= 20000 {
		// gak bikin multilevel partition, kalau graph nya dense
		// karena di graph dense,
		// misal n=100 dan avg degree dari setiap vertex 80
		// jumlah cut edges dari partition banyak -> shortcut edges banyak & gak make sense
		us = []int{25, 26}
	}
	re, _, oldToNewVIdMap, _, _ := buildCRP(t, nodeCoords, adjList, n, us, true)

	source := 0

	sid := oldToNewVIdMap[da.Index(source)]

	var dist [MAX_N][MAX_N]int64
	totalStayTime := int64(0)
	for i := 0; i < p; i++ {
		place := placeStay[i]

		vid := oldToNewVIdMap[da.Index(place.v)]
		crpQuery := routing.NewCRPALTBidirectionalSearchWithoutTurnCost(re.GetRoutingEngine())
		spLength, _, _ := crpQuery.ShortestPathSearch(sid, vid)
		dist[0][i+1] = int64(spLength)

		for j := 0; j < p; j++ {
			wplace := placeStay[j]
			crpQuery = routing.NewCRPALTBidirectionalSearchWithoutTurnCost(re.GetRoutingEngine())
			wid := oldToNewVIdMap[da.Index(wplace.v)]
			spLength, _, _ = crpQuery.ShortestPathSearch(vid, wid)

			dist[i+1][j+1] = int64(spLength)
		}

		crpQuery = routing.NewCRPALTBidirectionalSearchWithoutTurnCost(re.GetRoutingEngine())
		spLength, _, _ = crpQuery.ShortestPathSearch(vid, sid)
		dist[i+1][0] = int64(spLength)

		totalStayTime += int64(place.stayTime)
	}

	var dp [MAX_N][1 << (MAX_N - 1)]int64
	for i := 0; i < MAX_N; i++ {
		for j := 0; j < 1<<(MAX_N-1); j++ {
			dp[i][j] = -1
		}
	}

	withoutTaxi := dpTSP(0, (1<<p)-1, &dp, dist) + totalStayTime

	var dpWithTaxi [MAX_N][1 << (MAX_N - 1)][2]int64
	for i := 0; i < MAX_N; i++ {
		for j := 0; j < 1<<(MAX_N-1); j++ {
			for k := 0; k < 2; k++ {
				dpWithTaxi[i][j][k] = -1
			}
		}
	}

	withTaxi := dpTSPWithTaxi(0, (1<<p)-1, &dpWithTaxi, dist, 0, int64(taxiTime)) + totalStayTime

	ans := ""
	if withoutTaxi <= int64(g) {
		ans = "possible without taxi"
	} else if withTaxi <= int64(g) {
		ans = "possible with taxi"
	} else {
		ans = "impossible"
	}

	// assert expected output dari test cases soal
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

	expected := line

	if ans != expected {
		t.Fatalf("FAIL: expected answer: %v, got: %v", expected, ans)

	}

	t.Logf("solved a journey to greece test case: %v", filepath)
}

// please run the test using command: "go test ./tests/shortestpath_crp_alt_without_turn_cost -run TestCRPQueryAJourneyToGreeceMALT  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode
func TestCRPQueryAJourneyToGreeceMALT(t *testing.T) {
	logger, err := nvlog.New()
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	err = tests.Download(greeceZipFilePath, greeceTestDataUrl, logger, "greece")
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	err = tests.ExtractZip(greeceZipFilePath, greeceZipDirPath)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	testDirs := []string{"sample", "secret"}

	for _, dir := range testDirs {
		fullDir := filepath.Join(greeceZipDirPath, dir)

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
			t.Run("Multilevel-ALT without turn cost"+dir+"/"+baseName, func(t *testing.T) {
				solveGreece(t, testPath)
			})

		}
	}
}

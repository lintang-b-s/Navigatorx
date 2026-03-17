package query

import (
	"bufio"
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"math/rand"
	"net/http"
	"os"
	"strconv"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	mlpFile                 = "stress_test_yogyakarta"
	url                     = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile                = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
	landmarkFile     string = "./data/landmark.lm"
)

// there is also tests for crp query using test cases taken from
// programming contests problems (such as ICPC regionals & Indonesian NOI)
// which can be found in tests/shortestpath_crp_alt and tests/shortestpath
// cd tests/query &&  go test -v . --cover -coverpkg=../../pkg/... -coverprofile=query_coverage.out
// go tool cover -func=query_coverage.out
// go tool cover -html=query_coverage.out
func TestCRPQuerySimple(t *testing.T) {

	// https://visualgo.net/en/sssp

	bitpack := func(i, j da.Index) uint64 {
		return uint64(i) | (uint64(j) << 30)
	}

	testCases := []struct {
		name     string
		filepath string
		want     [][]map[uint64]float64 // level -> cellId -> bitpack(source,target) -> st-shortcutWeight

		apspFilePath string // all pairs shortest paths filepath.

	}{
		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> big
			name:     "visualgo example graph weighted big",
			filepath: "./data/samplegraph/big",

			apspFilePath: "./data/samplegraph/big.out",
		},
		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> cp4 4.10 d/w
			name:     "visualgo example graph weighted cp4 4.10 d/w",
			filepath: "./data/samplegraph/410",

			apspFilePath: "./data/samplegraph/410.out",
		},

		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> large
			name:         "visualgo example graph weighted large",
			filepath:     "./data/samplegraph/large",
			apspFilePath: "./data/samplegraph/large.out",
		},

		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> mrt
			name:         "visualgo example graph weighted mrt",
			filepath:     "./data/samplegraph/mrt",
			apspFilePath: "./data/samplegraph/mrt.out",
		},

		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> 414
			name:         "visualgo example graph weighted 414",
			filepath:     "./data/samplegraph/414",
			apspFilePath: "./data/samplegraph/414.out",
		},

		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> 416
			name:         "visualgo example graph weighted 416",
			filepath:     "./data/samplegraph/416",
			apspFilePath: "./data/samplegraph/416.out",
		},
		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> zigzag
			name:         "visualgo example graph weighted zigzag",
			filepath:     "./data/samplegraph/zigzag",
			apspFilePath: "./data/samplegraph/zigzag.out",
		},

		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> wheel
			name:         "visualgo example graph weighted wheel",
			filepath:     "./data/samplegraph/wheel",
			apspFilePath: "./data/samplegraph/wheel.out",
		},
	}

	buildGraph := func(filepath string) (*engine.Engine, *landmark.Landmark, map[da.Index]da.Index, error) {
		var (
			err  error
			line string
			n, m int
			f    *os.File
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
		n, err = strconv.Atoi(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		m, err = strconv.Atoi(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		var nodeCoords []osmparser.NodeCoord

		for i := 0; i < n; i++ {
			nodeCoords = append(nodeCoords, osmparser.NewNodeCoord(float64(i), float64(i)))
		}

		adjList := make([][]pairEdge, n)
		for i := 0; i < m; i++ {
			line, err = readLine(br)
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			ff := fields(line)
			u, err := strconv.Atoi(ff[0])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			v, err := strconv.Atoi(ff[1])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			w, err := strconv.Atoi(ff[2])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			adjList[u] = append(adjList[u], pairEdge{v, float64(w)})
		}
		es := flattenEdges(adjList)

		op := osmparser.NewOSMParserV2()
		acceptedNodeMap := make(map[int64]osmparser.NodeCoord, n)
		nodeToOsmId := make(map[da.Index]int64, n)
		for i := 0; i < n; i++ {
			acceptedNodeMap[int64(i)] = nodeCoords[i]
			nodeToOsmId[da.Index(i)] = int64(i)
		}

		op.SetAcceptedNodeMap(acceptedNodeMap)
		op.SetNodeToOsmId(nodeToOsmId)

		gs := da.NewGraphStorageWithSize(len(es), n)
		g := op.BuildGraph(es, gs, uint32(n), true)

		t.Logf("number of vertices: %v, number of edges: %v", uint32(n), len(es))

		g.SetGraphStorage(gs)

		logger, err := logger.New()
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		mp := partitioner.NewMultilevelPartitioner(
			[]int{int(math.Pow(2, 2)), int(math.Pow(2, 3))},
			2, 1,
			g, logger, true, false, true,
		)
		mp.RunMultilevelPartitioning()

		mlp := mp.BuildMLP()

		prep := preprocesser.NewPreprocessor(g, mlp, logger)
		err = prep.PreProcessing(false)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		t.Logf("Preprocessing completed successfully.")

		og := prep.GetOverlayGraph()
		custom := customizer.NewCustomizerDirect(g, og, logger)
		mt, err := custom.CustomizeDirect()
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		lm := landmark.NewLandmark()
		err = lm.PreprocessALT(util.MinInt(16, n), mt, custom, logger)
		if err != nil {
			t.Fatal(err)
		}
		err = lm.WriteLandmark(landmarkFile, custom)
		if err != nil {
			t.Fatal(err)
		}

		cf := costfunction.NewTimeCostFunctionEmpty()

		re, err := engine.NewEngineDirect(g, og, mt, logger, custom, cf)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		newToOldVidMap := prep.GetNewToOldVIdMap()
		return re, lm, newToOldVidMap, err
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			re, lm, newToOldVidMap, err := buildGraph(tc.filepath)
			graph := re.GetRoutingEngine().GetGraph()
			n := graph.NumberOfVertices()

			fOut, err := os.OpenFile(tc.filepath+".out", os.O_RDONLY, 0644)
			if err != nil {
				t.Fatalf("could not open test file: %v", err)
			}

			brOut := bufio.NewReader(fOut)

			apsp := make(map[uint64]float64, n*n) // all pairs shortest path. bitpack(source,target) -> shortest st-path cost
			for {
				line, err := readLine(brOut)
				if errors.Is(err, io.EOF) {
					break
				}
				ff := fields(line)
				ss, tt, stcosts := ff[0], ff[1], ff[2]
				source, err := strconv.Atoi(ss)
				if err != nil {
					t.Fatal(err)
				}
				target, err := strconv.Atoi(tt)
				if err != nil {
					t.Fatal(err)
				}
				stcost, err := strconv.ParseFloat(stcosts, 64)
				if err != nil {
					t.Fatal(err)
				}

				apsp[bitpack(da.Index(source), da.Index(target))] = stcost
			}
			fOut.Close()

			for source := da.Index(0); source < da.Index(n); source++ {
				for target := da.Index(0); target < da.Index(n); target++ {
					as := graph.GetDummyOutEdgeId(source)
					at := graph.GetDummyInEdgeId(target)
					crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)

					spCost, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

					oldS := newToOldVidMap[source]
					oldT := newToOldVidMap[target]

					query := bitpack(oldS, oldT)
					expectedSpCost := apsp[query]
					if !da.Eq(spCost, expectedSpCost) {
						t.Errorf("expected shortest path cost from %v to %v: %v, got: %v", oldS, oldT, spCost, expectedSpCost)
					}

				}
			}
		})
	}
}

type query struct {
	s, t da.Index
}

func setup(t *testing.T) (*engine.Engine, *landmark.Landmark) {
	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}
	workingDir, err := os.Getwd()
	err = util.ReadConfig(workingDir)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(osmfFile); os.IsNotExist(err) {
		output, err := os.Create(osmfFile)
		if err != nil {
			t.Fatal(err)
		}
		defer output.Close()

		t.Logf("downloading osm file......")
		response, err := http.Get(url)
		if err != nil {
			t.Fatal(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			t.Fatal(err)
		}
		t.Logf("download complete")
	}

	op := osmparser.NewOSMParserV2()

	graph, err := op.Parse(fmt.Sprintf("%s", osmfFile), logger, false)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger, true, false, true,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s", "crp_inertial_flow_"+mlpFile+".mlp"))
	if err != nil {
		t.Fatal(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	m, err := custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(16, m, custom, logger)
	if err != nil {
		t.Fatal(err)
	}
	err = lm.WriteLandmark(landmarkFile, custom)
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, lm
}

/*
stress test ini bertujuan untuk mencari apakah ada counterexample (output dari ALT CRP query bukan shortest path) dari implementasi  A*, landmarks, and triangle inequality (ALT) untuk query Customizable Route Planning (CRP) pada file
crp_bidir_astar_landmark.go

untuk plain dijkstra find single source shortest paths (sssp), from s to all other vertices
untuk alt query crp find point-to-point(p2p) shortest paths, untuk semua p2p sp queries diatas
note that Customizable Route Planning (CRP) query/multilevel dijkstra (MLD) [https://github.com/Project-OSRM/osrm-backend]/ multilevel A*, landmarks, and triangle inequality (ALT) hanya mempercepat p2p sp query bukan mempercepat sssp query

stress tests ini selesai dalam 5 menit
dan akan berhenti ketika ada counterexample
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphic #6 cpu cores #12 threads
ram: 16gb

please run the test using command: "cd tests/query && go test -run TestCRPQueryStressTest  -v -timeout=0  -count=1"
*/
func TestCRPQueryStressTest(t *testing.T) {
	re, lm := setup(t)
	g := re.GetRoutingEngine().GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	n := 2
	qset := make(map[da.Index]struct{})

	queries := make([]da.Index, 0, n)

	i := 0
	for i < n {
		s := da.Index(rd.Intn(V))

		if _, ok := qset[s]; ok {
			continue
		}

		qset[s] = struct{}{}
		queries = append(queries, da.Index(s))
		i++
	}
	numberOfVertices := g.NumberOfVertices()

	expectedSPTravelTimes := make([][]float64, n)

	t.Logf("start dijkstra query for all sources...")

	lock := sync.Mutex{}

	calcSpDijkstra := func(i int) any {
		s := queries[i]

		dijkstraQuery := routing.NewDijkstra(re.GetRoutingEngine(), false)

		sps, _ := dijkstraQuery.ShortestPath(s)

		lock.Lock()

		expectedSPTravelTimes[i] = sps

		if (i+1)%5 == 0 {
			t.Logf("done query from source number: %v\n", i+1)
		}
		lock.Unlock()

		return nil
	}

	workersDijkstra := concurrent.NewWorkerPool[int, any](7, n)

	for i := 0; i < n; i++ {
		workersDijkstra.AddJob(i)
	}

	workersDijkstra.Close()
	workersDijkstra.Start(calcSpDijkstra)
	workersDijkstra.WaitDirect()

	type query struct {
		i, s, t da.Index
		id      da.Index
	}

	newQuery := func(i, s, t, id da.Index) query {
		return query{i, s, t, id}
	}

	type counterExampleData struct {
		expectedSp, crpALTSP           float64
		expectedSpEdges, crpALTSPEdges []da.Index
		counterexample                 bool
	}

	newCounterExampleData := func(expectedSp, crpALTSP float64, expectedSpEdges, crpALTSPEdges []da.Index, cx bool) counterExampleData {
		return counterExampleData{expectedSp: expectedSp, crpALTSP: crpALTSP, expectedSpEdges: expectedSpEdges, crpALTSPEdges: crpALTSPEdges,
			counterexample: cx}
	}

	calcSp := func(q query) counterExampleData {
		i := q.i
		s := q.s
		target := q.t
		id := q.id
		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1
		at := g.GetEntryOffset(target) + g.GetInDegree(target) - 1
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)
		sp, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

		expectedSp := expectedSPTravelTimes[i][target]

		counterexample := false
		if !da.EqEps(expectedSp, sp, 1e-5) { // shortcuts weights (hasil dari Customization phase of CRP yang diwrite ke file & read lagi ) mungkin gak terlalu presisi
			counterexample = true
		}

		if (id+1)%5000 == 0 {
			t.Logf("done query id: %v\n", id+1)
		}
		if counterexample {
			return newCounterExampleData(expectedSp, sp, []da.Index{}, []da.Index{}, true)
		}

		return newCounterExampleData(0, 0, nil, nil, false)
	}

	workers := concurrent.NewWorkerPool[query, counterExampleData](100, n*numberOfVertices)

	for i := 0; i < n; i++ {
		s := queries[i]

		for t := da.Index(0); t < da.Index(numberOfVertices); t++ {
			workers.AddJob(newQuery(da.Index(i), s, t, da.Index(i)*da.Index(numberOfVertices)+t))
		}
	}
	t.Logf("start crp query...")

	ctx, cancel := context.WithCancel(context.Background())
	workers.Close()
	workers.StartWithContext(ctx, calcSp)
	workers.Wait()

	for res := range workers.CollectResults() {
		if res.counterexample {
			t.Logf("found counterExample!!\n")
			t.Errorf("found counter example!!, expected shortest path cost: %f, got: %f", res.crpALTSP, res.expectedSp)

			t.Logf("expected shortest path travel time: %f, got: %f\n", res.expectedSp, res.crpALTSP)
			cancel()

			t.Logf("\n")
		}
	}
}

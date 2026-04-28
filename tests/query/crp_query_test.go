package query

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"io"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

const (
	mlpFile                 = "./data/stress_test_yogyakarta.mlp"
	osmfFile                = "../../data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original_query_test.graph"
	overlayGraphFile string = "./data/overlay_graph_query_test.graph"
	metricsFile      string = "./data/metrics_query_test.txt"
	landmarkFile     string = "./data/landmark_query_test.lm"
	timeFunctionFile string = "./data/timefunction_query_test.txt"
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

	buildGraph := func(filepath string) (*routing.CRPRoutingEngine, *landmark.Landmark, map[da.Index]da.Index, error) {
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
		g, edgeInfoIds := op.BuildGraph(es, gs, uint32(n), false)

		t.Logf("number of vertices: %v, number of edges: %v", uint32(n), len(es))

		g.SetGraphStorage(gs)

		logger, err := logger.New()
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		mp := partitioner.NewMultilevelPartitioner(
			[]int{int(math.Pow(2, 2)), int(math.Pow(2, 3))},
			2, 1,
			g, logger, true, true,
		)

		mp.RunMultilevelPartitioning()

		mlp := mp.BuildMLP()

		prep := preprocesser.NewPreprocessor(g, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
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
		err = lm.PreprocessALT(util.MinInt(16, n), mt, g, logger)
		if err != nil {
			t.Fatal(err)
		}
		err = lm.WriteLandmark(landmarkFile, g.NumberOfVertices())
		if err != nil {
			t.Fatal(err)
		}

		cf := costfunction.NewTimeCostFunctionEmpty()

		eng, err := engine.NewEngineDirect(g, og, mt, logger, custom, cf, landmarkFile, timeFunctionFile)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		newToOldVidMap := prep.GetNewToOldVIdMap()
		return eng.GetRoutingEngine(), lm, newToOldVidMap, err
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			re, _, newToOldVidMap, err := buildGraph(tc.filepath)
			if err != nil {
				t.Fatal(err)
			}
			graph := re.GetGraph()
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
					crpQuery := routing.NewCRPALTBidirectionalSearch(re, 1.0)
					oldS := newToOldVidMap[source]
					oldT := newToOldVidMap[target]

					query := bitpack(oldS, oldT)
					expectedSpCost := apsp[query]

					sVertex := graph.GetVertex(source)
					tVertex := graph.GetVertex(target)
					emptyCoords := make([]da.Coordinate, 0)
					sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
					tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

					spCost, _, _, _, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)

					if !util.Eq(spCost, expectedSpCost) {
						t.Errorf("expected shortest path cost from %v to %v: %v, got: %v", oldS, oldT, spCost, expectedSpCost)
					}

				}
			}
		})
	}
}

func init() {
	util.InitConfig()
}

func setup(t *testing.T, turnCost bool) (*engine.Engine, *zap.Logger) {
	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	workingDir, err := util.FindProjectWorkingDir()
	if err != nil {
		t.Fatal(err)
	}
	err = util.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}

	if !turnCost {
		pkg.OffTurnCost()
	} else {
		pkg.OnTurnCost()
	}

	op := osmparser.NewOSMParserV2()
	graph, edgeInfoIds, err := op.Parse(fmt.Sprintf("%s", osmfFile), logger)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split("8,10,11,12,14", ",")
	// pss := strings.Split("30,31,32,33,34", ",") // tanpa partisi & tanpa bikin shortcuts
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^powosmfFile
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger, false, false,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpFile)
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)

	_, err = custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, logger
}

/*
stress test ini bertujuan untuk mencari apakah ada counterexample (output dari ALT CRP query bukan shortest path) dari implementasi  A*, landmarks, and triangle inequality (ALT) untuk query Customizable Route Planning (CRP) pada file
multilevel_astar_landmarks.go

untuk plain dijkstra find single source shortest paths (sssp), from s to all other vertices
untuk alt query crp find point-to-point(p2p) shortest paths, untuk semua p2p sp queries diatas
note that Customizable Route Planning (CRP) query/multilevel dijkstra (MLD) [https://github.com/Project-OSRM/osrm-backend]/ multilevel A*, landmarks, and triangle inequality (ALT) hanya mempercepat p2p sp query bukan mempercepat sssp query

stress tests ini selesai dalam 7 menit
dan akan berhenti ketika ada counterexample
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphic #6 cpu cores #12 threads
ram: 16gb

please run the test using command: "cd tests/query && go test -run TestCRPQueryStressNoTurnCostTest  -v -timeout=0  -count=1"
karena bakal time out kalau pakai vscode
*/

func TestCRPQueryStressNoTurnCostTest(t *testing.T) {
	eng, _ := setup(t, false)
	re := eng.GetRoutingEngine()
	g := re.GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	n := 20
	qset := make(map[da.Index]struct{})

	queries := make([]da.Index, 0, n)

	i := 0
	for i < n {
		s := da.Index(rd.Intn(V))
		if g.GetOutDegree(s) == 0 || g.GetInDegree(s) == 0 {
			continue
		}

		if _, ok := qset[s]; ok {
			continue
		}

		qset[s] = struct{}{}
		queries = append(queries, da.Index(s))
		i++
	}
	numberOfVertices := g.NumberOfVertices()

	expectedSPTravelTimes := make([][]float64, n)

	t.Logf("start dijkstra query for some sources to all other vertices...")

	lock := sync.Mutex{}

	calcSpDijkstra := func(i int) any {
		s := queries[i]

		dijkstraQuery := routing.NewDijkstra(re, false)

		sps, _ := dijkstraQuery.ShortestPath(s)

		lock.Lock()

		expectedSPTravelTimes[i] = sps

		if (i+1)%5 == 0 {
			t.Logf("done query from source number: %v\n", i+1)
		}
		lock.Unlock()

		return nil
	}

	workersDijkstra := concurrent.NewWorkerPool[int, any](70, 500)
	ctx, cancel := context.WithCancel(context.Background())
	workersDijkstra.StartWithContext(ctx, calcSpDijkstra)
	go func() {
		for range workersDijkstra.CollectResults() {
		}
	}()

	for i := 0; i < n; i++ {
		workersDijkstra.AddJob(i)
	}
	workersDijkstra.Close()
	workersDijkstra.Wait()

	cancel()

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
		inEdgeToS := g.GetEntryOffset(s) + g.GetInDegree(s) - 1
		_, as := g.GetHeadOfInedgeWithOutEdge(inEdgeToS)
		outEdgeFromTarget := g.GetExitOffset(target) + g.GetOutDegree(target) - 1
		_, at := g.GetTailOfOutedgeWithInEdge(outEdgeFromTarget)
		crpQuery := routing.NewCRPALTBidirectionalSearch(re, 1.0)
		// crpQuery := routing.NewCRPBidirectionalSearch(re, 1.0)

		sVertex := g.GetVertex(s)
		tVertex := g.GetVertex(target)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		sp, _, _, _, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		// sp, _, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		expectedSp := expectedSPTravelTimes[i][target]

		counterexample := false
		if !util.EqEps(expectedSp, sp, 1e-5) {
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

	workers := concurrent.NewWorkerPool[query, counterExampleData](70, 500)

	ctx, cancel = context.WithCancel(context.Background())
	workers.StartWithContext(ctx, calcSp)

	done := make(chan struct{}, 1)
	go func() {
		for i := 0; i < n; i++ {
			s := queries[i]

			for t := da.Index(0); t < da.Index(numberOfVertices); t++ {
				workers.AddJob(newQuery(da.Index(i), s, t, da.Index(i)*da.Index(numberOfVertices)+t))
			}
		}
		done <- struct{}{}
	}()
	t.Logf("start crp query...")

	go func() {
		<-done
		workers.Close()
		workers.Wait()
	}()

	t.Run("stress test crp query", func(t *testing.T) {
		for res := range workers.CollectResults() {
			if res.counterexample {
				t.Logf("found counterExample!!\n")
				t.Errorf("found counter example!!, expected shortest path cost: %f, got: %f", res.expectedSp, res.crpALTSP)

				cancel()

				t.Logf("\n")
			}
		}
	})
}

/*

please run the test using command: "cd tests/query && go test -run TestCRPQueryStressWithTurnCostTest  -v -timeout=0  -count=1"
karena bakal time out kalau pakai vscode
*/

func TestCRPQueryStressWithTurnCostTest(t *testing.T) {
	eng, _ := setup(t, true)
	re := eng.GetRoutingEngine()
	g := re.GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	n := 20
	qset := make(map[da.Index]struct{})

	queries := make([]da.Index, 0, n)

	i := 0
	for i < n {
		s := da.Index(rd.Intn(V))
		if g.GetOutDegree(s) == 0 || g.GetInDegree(s) == 0 {
			continue
		}

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

	expectedSpPaths := make([][]string, numberOfVertices)

	calcSpDijkstra := func(i int) any {
		s := queries[i]

		dijkstraQuery := routing.NewDijkstraWithTurnCost(re, false)

		sps, spPath := dijkstraQuery.ShortestPath(s)

		lock.Lock()

		expectedSpPaths[s] = make([]string, numberOfVertices)
		for target := 0; target < numberOfVertices; target++ {
			expectedpathCoords, _ := re.GetEdgePath(spPath[target])
			expectedPolyline := da.GooglePoylineFromCoords(*expectedpathCoords)
			expectedSpPaths[s][target] = expectedPolyline
		}
		expectedSPTravelTimes[i] = sps

		if (i+1)%5 == 0 {
			t.Logf("done query from source number: %v\n", i+1)
		}
		lock.Unlock()

		return nil
	}

	workersDijkstra := concurrent.NewWorkerPool[int, any](6, 5)
	ctx, cancel := context.WithCancel(context.Background())
	workersDijkstra.StartWithContext(ctx, calcSpDijkstra)
	go func() {
		for range workersDijkstra.CollectResults() {
		}
	}()

	for i := 0; i < n; i++ {
		workersDijkstra.AddJob(i)
	}
	workersDijkstra.Close()
	workersDijkstra.Wait()
	cancel()

	type query struct {
		i, s, t da.Index
		id      da.Index
	}

	newQuery := func(i, s, t, id da.Index) query {
		return query{i, s, t, id}
	}

	type counterExampleData struct {
		s, t                           da.Index
		expectedSp, crpALTSP           float64
		expectedSpEdges, crpALTSPEdges []da.Index
		counterexample                 bool
	}

	newCounterExampleData := func(s, t da.Index, expectedSp, crpALTSP float64, expectedSpEdges, crpALTSPEdges []da.Index, cx bool) counterExampleData {
		return counterExampleData{s: s, t: t, expectedSp: expectedSp, crpALTSP: crpALTSP, expectedSpEdges: expectedSpEdges, crpALTSPEdges: crpALTSPEdges,
			counterexample: cx}
	}

	// isSame := func(a []da.Index, b []da.Index) bool {
	// 	if len(a) != len(b) {
	// 		return false
	// 	}

	// 	for i := 0; i < len(a); i++ {
	// 		if a[i] != b[i] {
	// 			return false
	// 		}
	// 	}
	// 	return true
	// }

	calcSp := func(q query) counterExampleData {
		i := q.i
		s := q.s
		target := q.t

		id := q.id
		inEdgeToS := g.GetDummyInEdgeId(s)
		_, as := g.GetHeadOfInedgeWithOutEdge(inEdgeToS)
		outEdgeFromTarget := g.GetDummyOutEdgeId(target)
		tail, at := g.GetTailOfOutedgeWithInEdge(outEdgeFromTarget)
		crpQuery := routing.NewCRPALTBidirectionalSearch(re, 1.0) // salah kalau u-turn cost > 0

		util.AssertPanic(tail == target, "dummy target edge is invalid")
		sVertex := g.GetVertex(s)
		tVertex := g.GetVertex(target)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		sp, _, _, _, found := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)

		expectedSp := expectedSPTravelTimes[i][target]
		// expectedPolyline := expectedSpPaths[s][target]

		// gotPolyline := da.GooglePoylineFromCoords(*pathCoords)

		counterexample := false
		expectedFound := util.Lt(expectedSp, pkg.INF_WEIGHT)

		notValid := (!util.EqEps(expectedSp, sp, 1e-4) && found && expectedFound) || (found && !expectedFound) || (!found && expectedFound)
		if notValid {
			counterexample = true
			crpQuery2 := routing.NewCRPBidirectionalSearch(re, 1.0)
			sp2, spPath2, _ := crpQuery2.ShortestPathSearch(sPhantomNode, tPhantomNode)
			_, _ = sp2, spPath2
		}

		if (id+1)%5000 == 0 {
			t.Logf("done query id: %v\n", id+1)
		}
		if counterexample {
			return newCounterExampleData(s, target, expectedSp, sp, []da.Index{}, []da.Index{}, true)
		}

		return newCounterExampleData(0, 0, 0, 0, nil, nil, false)
	}

	workers := concurrent.NewWorkerPool[query, counterExampleData](12, 5)
	ctx, cancel = context.WithCancel(context.Background())
	workers.StartWithContext(ctx, calcSp)

	done := make(chan struct{}, 1)
	go func() {
		for i := 0; i < n; i++ {
			s := queries[i]

			for t := da.Index(0); t < da.Index(numberOfVertices); t++ {
				workers.AddJob(newQuery(da.Index(i), s, t, da.Index(i)*da.Index(numberOfVertices)+t))
			}
		}
		done <- struct{}{}
	}()
	t.Logf("start crp query...")

	go func() {
		<-done
		workers.Close()
		workers.Wait()
	}()

	t.Run("stress test crp query", func(t *testing.T) {
		for res := range workers.CollectResults() {
			if res.counterexample { // todo: aneh setelah tambahin multiple via-way turn restrictions jadi gak pass
				t.Logf("found counterExample!!\n")
				t.Errorf("found counter example!!, expected shortest path from %v to %v cost: %f, got: %f", res.s, res.t, res.expectedSp, res.crpALTSP)

				cancel()

				t.Logf("\n")
			}
		}
	})
}

// test ini bertujuan untuk memastikan rute shortest path dan rute alternatif yang direturn routing engine tidak melewati OSM turn restrictions.
/*
please run the test using command: "cd tests/query && go test -run TestCRPQueryTurnRestriction  -v -timeout=0  -count=1"
karena bakal time out kalau pakai vscode
*/
func TestCRPQueryTurnRestriction(t *testing.T) {
	eng, logger := setup(t, true)
	re := eng.GetRoutingEngine()
	g := re.GetGraph()
	altSearch := routing.NewAlternativeRouteSearch(re)

	type turnResType struct {
		restriction string
		from, to    int64
		viaWays     []int64
		viaNode     int64
		isViaWay    bool
	}
	testCases := []struct {
		name                          string
		turnRestriction               turnResType
		queryOrigin, queryDestination da.Coordinate
	}{
		{
			name:             "Simpang Pingit U-turn Restriction https://www.openstreetmap.org/relation/17842412 ,  example correct route:  https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.782881%2C110.361282%3B-7.782681%2C110.361341#map=19/-7.781811/110.361046",
			queryOrigin:      da.NewCoordinate(-7.782881, 110.361282),
			queryDestination: da.NewCoordinate(-7.782681, 110.361341),
			turnRestriction:  turnResType{restriction: "no_u_turn", from: 263612372, isViaWay: true, viaWays: []int64{590074069, 1490467372}, to: 898190693},
		},
		{
			name:             "Simpang Pingit U-turn Restriction 2 https://www.openstreetmap.org/relation/4763182 ,  example correct route: https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.781084%2C110.361035%3B-7.781015%2C110.360724#map=17/-7.784382/110.360112",
			queryOrigin:      da.NewCoordinate(-7.781084, 110.361035),
			queryDestination: da.NewCoordinate(-7.781015, 110.360724),
			turnRestriction:  turnResType{restriction: "no_u_turn", from: 898190692, isViaWay: true, viaWays: []int64{1459769996, 263612372}, to: 590074069},
		},

		{
			name:             "Cik di Tiro u-turn restriction https://www.openstreetmap.org/relation/13427535,  example correct route: https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.782253%2C110.374966%3B-7.782219%2C110.375243#map=17/-7.779046/110.375637",
			queryOrigin:      da.NewCoordinate(-7.782253, 110.374966),
			queryDestination: da.NewCoordinate(-7.782219, 110.375243),
			turnRestriction:  turnResType{restriction: "no_u_turn", from: 153821715, isViaWay: true, viaWays: []int64{1001303583}, to: 1001303581},
		},
		{
			name:             "Simpang tugu jogja u-turn restriction https://www.openstreetmap.org/relation/17670402,  example correct route: https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.783003%2C110.369079%3B-7.782729%2C110.367582#map=18/-7.782897/110.367665",
			queryOrigin:      da.NewCoordinate(-7.783003, 110.369079),
			queryDestination: da.NewCoordinate(-7.782729, 110.367582),
			turnRestriction:  turnResType{restriction: "no_u_turn", from: 357658481, isViaWay: true, viaWays: []int64{1110178248}, to: 1108475786},
		},

		{
			name:             "Simpang Gramedia no_right_turn restriction https://www.openstreetmap.org/relation/5710502,  example correct route: https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.783686%2C110.374746%3B-7.78296%2C110.377024#map=16/-7.78528/110.37525",
			queryOrigin:      da.NewCoordinate(-7.783686, 110.374746),
			queryDestination: da.NewCoordinate(-7.782960, 110.377024),
			turnRestriction:  turnResType{restriction: "no_right_turn", from: 1347054637, isViaWay: false, viaNode: 271845942, to: 179907371},
		},

		{
			name:             "Jalan Seturan Raya no_right_turn restriction https://www.openstreetmap.org/relation/18854138,  example correct route: openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.764298%2C110.41148%3B-7.761288%2C110.422211#map=19/-7.763936/110.411675",
			queryOrigin:      da.NewCoordinate(-7.764298, 110.411480),
			queryDestination: da.NewCoordinate(-7.761288, 110.422211),
			turnRestriction:  turnResType{restriction: "no_right_turn", from: 1124615933, isViaWay: false, viaNode: 1390908542, to: 586534196},
		},
	}

	isSame := func(a []int64, b []int64) bool {
		if len(a) != len(b) {
			return false
		}

		for i := 0; i < len(a); i++ {
			if a[i] != b[i] {
				return false
			}
		}
		return true
	}

	// path= list of edgeIds
	// cek palo path contain turn restriction (from-way,via-node/via-ways,to-way)
	isCorrect := func(path []da.Index, tr turnResType) ([]int64, bool) {
		restrictedPathLength := 0
		if tr.isViaWay {
			restrictedPathLength = len(tr.viaWays) + 2 // len(via-ways) + from + to. example: https://www.openstreetmap.org/relation/17842412
		} else {
			restrictedPathLength = 2 // from + to . (via Node ada di head nya from edge)
		}

		restrictedPath := make([]int64, restrictedPathLength)
		restrictedPath[0] = tr.from
		restrictedPath[restrictedPathLength-1] = tr.to
		if tr.isViaWay {
			for i := 1; i < restrictedPathLength-1; i++ {
				restrictedPath[i] = tr.viaWays[i-1]
			}
		} else {
			restrictedPath[1] = tr.viaNode
		}

		for i := 0; i < len(path)-(restrictedPathLength-1); i++ {
			subPath := make([]int64, 0, restrictedPathLength)

			if tr.isViaWay {
				cur := path[i]
				currOsmWayId := g.GetOsmWayId(cur)
				subPath = append(subPath, currOsmWayId)
				for j := 1; j <= restrictedPathLength-1; j++ {
					next := path[i+j]
					nextOsmWayId := g.GetOsmWayId(next)
					subPath = append(subPath, nextOsmWayId)
				}
			} else {
				from := path[i]
				viaNode := g.GetHeadOfOutEdge(from)
				to := path[i+1]

				fromWayId := g.GetOsmWayId(from)
				viaOsmNodeId := int64(g.GetVertexOsmId(viaNode))
				toWayId := g.GetOsmWayId(to)
				subPath = append(subPath, fromWayId)
				subPath = append(subPath, viaOsmNodeId)
				subPath = append(subPath, toWayId)
			}
			if isSame(subPath, restrictedPath) {
				return subPath, false
			}
		}

		return []int64{0}, true
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {

			rtree := spatialindex.NewRtree()

			rtree.Build(re.GetGraph(), logger)

			routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true)
			if err != nil {
				panic(err)
			}

			qOrigin := tc.queryOrigin
			qDestination := tc.queryDestination
			sp, tp := routingService.Snap(context.Background(), qOrigin.GetLat(), qOrigin.GetLon(), qDestination.GetLat(), qDestination.GetLon())
			crpQuery := routing.NewCRPALTBidirectionalSearch(re, 1.0)

			_, _, _, path, _ := crpQuery.ShortestPathSearch(sp, tp)
			if subPath, correct := isCorrect(path, tc.turnRestriction); !correct {
				t.Errorf("%s: expected not contain path %v, got contain the path", tc.name, subPath)
			}

			alts, _, _ := altSearch.FindAlternativeRoutes(sp, tp, 3, false, 0)

			for i := 0; i < len(alts); i++ {
				altPath := alts[i].GetEdgeIdPath()
				if subPath, correct := isCorrect(altPath, tc.turnRestriction); !correct {
					t.Errorf("%s: expected not contain path %v, got contain the path, restriction: %v", tc.name, subPath, tc.turnRestriction.restriction)
				}
			}
		})
	}
}

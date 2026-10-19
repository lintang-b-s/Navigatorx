package query

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"io"
	"math/rand"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/Navigatorx/tests"
	"go.uber.org/zap"
)

const (
	osmFileJogja = "./data/yogyakarta.osm.pbf"
	osmFileSolo  = "./data/solo.osm.pbf"
)

var (
	landmarkFile string = config.ProfilesRoot() + "/landmark_query_test.nlm"
)

// there is also tests for crp query using test cases taken from
// programming contests problems (such as ICPC regionals & Indonesian NOI)
// which can be found in tests/shortestpath_crp_alt and tests/shortestpath
// cd tests/query &&  go test -v . --cover -failfast -coverpkg=../../pkg/... -coverprofile=query_coverage.out  -v -timeout=0  -count=1
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
		want     [][]map[uint64]int32 // level -> cellId -> bitpack(source,target) -> st-shortcutWeight

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

	buildGraph := func(filepath string) (*routing.CRPRoutingEngine[float64], *landmark.Landmark[float64], map[da.Index]da.Index, error) {
		var (
			err  error
			line string
			n, m int
			f    *os.File
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
		n, err = util.ParseTextInt(ff[0])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		m, err = util.ParseTextInt(ff[1])
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		var nodeCoords []da.Coordinate

		for i := 0; i < n; i++ {
			nodeCoords = append(nodeCoords, da.NewCoordinate(float64(i), float64(i)))
		}

		adjList := make([][]tests.PairEdge, n)
		for i := 0; i < m; i++ {
			line, err = util.ReadLine(br)
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			ff := util.Fields(line)
			u, err := util.ParseTextInt(ff[0])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			v, err := util.ParseTextInt(ff[1])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			w, err := util.ParseTextInt(ff[2])
			if err != nil {
				t.Fatalf("err: %v", err)
			}
			adjList[u] = append(adjList[u], tests.NewPairEdge(v, float64(w)))
		}
		es := tests.FlattenEdges(adjList)

		op := extractor.NewExtractor[float64]()
		acceptedNodeMap := make(map[int64]da.Coordinate, n)
		nodeToOsmId := make(map[da.Index]int64, n)
		for i := 0; i < n; i++ {
			acceptedNodeMap[int64(i)] = nodeCoords[i]
			nodeToOsmId[da.Index(i)] = int64(i)
		}

		op.SetAcceptedNodeMap(acceptedNodeMap)
		op.SetNodeToOsmId(nodeToOsmId)

		rn := da.NewRoadNetworkDataContainerWithSize(len(es), n)
		g, timeFunction, _, _ := op.BuildGraph(es, rn, uint32(n), false)

		t.Logf("number of vertices: %v, number of edges: %v", uint32(n), len(es))

		logger, err := log.New()
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		mp := partitioner.NewMultilevelPartitioner(
			[]int{4, 8},
			2, 1,
			g, logger,
		)

		mp.RunMultilevelPartitioning()

		mlp := mp.BuildMLP()

		prep := preprocesser.NewPreprocessor(g, rn, timeFunction, mlp, logger)
		err = prep.PreProcessing(false)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		t.Logf("Preprocessing completed successfully.")

		og := prep.GetOverlayGraph()
		custom := customizer.NewCustomizerDirect(
			g, og, prep.GetTimeFunction(), logger,
		)
		mt, err := custom.CustomizeDirect()
		if err != nil {
			t.Fatalf("err: %v", err)
		}
		lm := landmark.NewLandmark[float64]()
		err = lm.PreprocessALT(min(16, n), mt.GetCostFunction(), g, logger)
		if err != nil {
			t.Fatal(err)
		}
		err = lm.WriteLandmark(landmarkFile, g.NumberOfVertices())
		if err != nil {
			t.Fatal(err)
		}

		eng, err := engine.NewEngineDirect(g, rn, og, mt, logger, landmarkFile)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		newToOldVidMap := prep.GetNewToOldVId()
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

			fOut, err := os.OpenFile(tc.filepath+".out", os.O_RDONLY, 0600)
			if err != nil {
				t.Fatalf("could not open test file: %v", err)
			}

			brOut := bufio.NewReader(fOut)

			apsp := make(map[uint64]float64, n*n) // all pairs shortest path. bitpack(source,target) -> shortest st-path cost
			for {
				line, err := util.ReadLine(brOut)
				if errors.Is(err, io.EOF) {
					break
				}
				ff := util.Fields(line)
				ss, tt, stcosts := ff[0], ff[1], ff[2]
				source, err := util.ParseTextInt(ss)
				if err != nil {
					t.Fatal(err)
				}
				target, err := util.ParseTextInt(tt)
				if err != nil {
					t.Fatal(err)
				}
				stcost, err := util.ParseTextFloat64(stcosts)
				if err != nil {
					t.Fatal(err)
				}

				apsp[bitpack(da.Index(source), da.Index(target))] = stcost
			}
			fOut.Close()

			for source := da.Index(0); source < da.Index(n); source++ {
				for target := da.Index(0); target < da.Index(n); target++ {

					crpQuery := routing.NewCRPALTQuery(re)
					ss := newToOldVidMap[source]
					tt := newToOldVidMap[target]

					query := bitpack(ss, tt)
					expectedSpCost := apsp[query]

					spCost, _, _ := crpQuery.ShortestPathSearch(source, target)

					if !util.Eq(spCost, expectedSpCost) {
						t.Errorf("expected shortest path cost from %v to %v: %v, got: %v", ss, tt, expectedSpCost, spCost)
					}

				}
			}
		})
	}
}

func init() {
	config.InitConfig()
	config.InitRegionName("query_test", pkg.TEST)
}

func setup(t *testing.T, osmFileInput string) (*engine.Engine[int32], *extractor.Extractor[int32], *zap.Logger) {
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	outputDir := filepath.Join(workingDir, "data")
	if err := util.EnsureDirExists(outputDir); err != nil {
		panic(err)
	}

	mapName := strings.Split(osmFileInput, "/")[2]
	mapName = strings.Split(mapName, ".")[0]
	config.InitRegionName(fmt.Sprintf("query_test_%s", mapName), pkg.TEST)

	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	err = config.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}

	op := extractor.NewExtractor[int32]()
	_, ebg, rn, _, timeFunction, err := op.Extract(filepath.Join(workingDir, osmFileInput), logger)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split("8,10,11,12,14", ",")
	// pss := strings.Split("30,31,32,33,34", ",") // tanpa partisi & tanpa bikin shortcuts
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := util.ParseTextInt(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^powosmFile
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		ebg, logger,
	)
	mp.RunMultilevelPartitioning()
	err = mp.SaveToFile()
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile()
	if err != nil {
		panic(err)
	}

	prep := preprocesser.NewPreprocessor(ebg, rn, timeFunction, mlp, logger)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	time.Sleep(1 * time.Second)
	custom := customizer.NewCustomizer[int32](logger)

	_, err = custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine[int32](logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, op, logger
}

/*

please run the test using command: "cd tests/query && go test -run TestCRPQueryStressTest  -v -timeout=0  -count=1"
karena bakal time out kalau pakai vscode
*/

func TestCRPQueryStressTest(t *testing.T) {
	eng, _, _ := setup(t, osmFileJogja)
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

	expectedSPCosts := make([][]int32, n)

	t.Logf("start dijkstra query for all sources...")

	lock := sync.Mutex{}

	expectedSpPaths := make([][]string, numberOfVertices)

	calcSpDijkstra := func(i int) any {
		s := queries[i]

		dijkstraQuery := routing.NewDijkstra(re, false)

		sps, spPath := dijkstraQuery.ShortestPath(s)

		lock.Lock()

		expectedSpPaths[s] = make([]string, numberOfVertices)
		for target := 0; target < numberOfVertices; target++ {
			expectedpathCoords, _ := re.GetSegmentPath(spPath[target])
			expectedPolyline := da.GooglePoylineFromCoords(*expectedpathCoords)
			expectedSpPaths[s][target] = expectedPolyline
		}
		expectedSPCosts[i] = sps

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
		expectedSp, crpALTSP           int32
		expectedSpEdges, crpALTSPEdges []da.Index
		counterexample                 bool
	}

	newCounterExampleData := func(s, t da.Index, expectedSp, crpALTSP int32, expectedSpEdges, crpALTSPEdges []da.Index, cx bool) counterExampleData {
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
		crpQuery := routing.NewCRPALTQuery(re) // salah kalau u-turn cost > 0
		sp, _, found := crpQuery.ShortestPathSearch(s, target)

		expectedSp := expectedSPCosts[i][target] // in nanoseconds
		// expectedPolyline := expectedSpPaths[s][target]

		// gotPolyline := da.GooglePoylineFromCoords(*pathCoords)

		counterexample := false
		expectedFound := util.Lt(expectedSp, util.Infinity[int32]())

		notValid := (!util.Eq(expectedSp, sp) && found && expectedFound) || (found && !expectedFound) || (!found && expectedFound)
		if notValid {
			counterexample = true
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
				t.Errorf("found counter example!!, expected shortest path from %v to %v cost: %d, got: %d", res.s, res.t, res.expectedSp, res.crpALTSP)

				cancel()

				t.Logf("\n")
			}
		}
	})
}

// test ini bertujuan untuk memastikan rute shortest path dan rute alternatif yang direturn routing engine tidak melewati OSM turn restrictions.
/*
please run the test using command: "go test ./tests/query  -run TestCRPQueryTurnRestriction  -v -timeout=0  -count=1"
karena bakal time out kalau pakai vscode
*/
func TestCRPQueryTurnRestriction(t *testing.T) {

	type turnResType struct {
		restriction string
		from, to    int64
		viaWays     []int64
		viaNode     int64
		isViaWay    bool
		id          int64
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
	isCorrect := func(path []da.Index, tr turnResType, rn *da.RoadNetworkDataContainer) ([]int64, bool) {
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
				currOsmWayId := int64(rn.GetOsmWayId(cur))
				subPath = append(subPath, currOsmWayId)
				for j := 1; j <= restrictedPathLength-1; j++ {
					next := path[i+j]
					nextOsmWayId := int64(rn.GetOsmWayId(next))
					subPath = append(subPath, nextOsmWayId)
				}
			} else {
				from := path[i]
				viaOsmId, _ := rn.GetTailHeadOsmNodeId(from)
				to := path[i+1]
				fromWayId := int64(rn.GetOsmWayId(from))
				toWayId := int64(rn.GetOsmWayId(to))
				subPath = append(subPath, fromWayId)
				subPath = append(subPath, int64(viaOsmId))
				subPath = append(subPath, toWayId)
			}
			if isSame(subPath, restrictedPath) {
				return subPath, false
			}
		}

		return []int64{0}, true
	}

	// tests all turn restrictions in this two jogja & solo osm.pbf file
	mapTestCases := []string{osmFileJogja, osmFileSolo}
	for _, mtc := range mapTestCases {

		eng, op, logger := setup(t, mtc)
		re := eng.GetRoutingEngine()
		rn := re.GetRoadNetworkContainer()
		rtree := spatialindex.NewRtree()
		rtree.Build(re.GetGraph(), rn, logger)
		routingService, err := usecases.NewRoutingService(logger, re, rn, rtree, 0.05, true)
		if err != nil {
			panic(err)
		}

		fromTurnRestrictions := op.GetTurnRestrictions()
		ways := op.GetWays()
		wayNodeMap := op.GetWayNodeMap()

		testCases := make([]struct {
			name             string
			turnRestriction  turnResType
			queryOrigin      da.Coordinate
			queryDestination da.Coordinate
		}, 0)

		valhallaVisualizerPolylineUrl := func(polyline string) string {
			return fmt.Sprintf("https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=false#%s", polyline)
		}

		for from, turnRestrictions := range fromTurnRestrictions {
			for _, tr := range turnRestrictions {
				fnodes := ways[from].GetOsmNodes()
				tnodes := ways[tr.GetTo()].GetOsmNodes()
				if len(fnodes) == 0 || len(tnodes) == 0 {
					continue
				}

				fc := wayNodeMap[fnodes[0]].GetCoord()
				tn := tnodes[len(tnodes)-1]

				if tr.IsWay() {
					viaways := tr.GetViaWays()
					vnodes := ways[viaways[len(viaways)-1]].GetOsmNodes()
					for _, vn := range vnodes {
						if vn == tn {
							tn = tnodes[0]
							break
						}
					}
				}

				tc := wayNodeMap[tn].GetCoord()
				trs := tr.GetTurnRestriction().String()
				testCases = append(testCases, struct {
					name             string
					turnRestriction  turnResType
					queryOrigin      da.Coordinate
					queryDestination da.Coordinate
				}{
					name: fmt.Sprintf(`%s Restriction https://www.openstreetmap.org/relation/%v ,  example correct route:  https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=%v%%2C%v%%3B%v%%2C%v`,
						trs, tr.GetId(), fc.GetLat(), fc.GetLon(), tc.GetLat(), tc.GetLon(),
					),
					turnRestriction: turnResType{
						restriction: trs,
						from:        from,
						to:          tr.GetTo(),
						viaWays:     tr.GetViaWays(),
						viaNode:     tr.GetVia(),
						isViaWay:    tr.GetIsWay(),
						id:          tr.GetId(),
					},
					queryOrigin:      fc,
					queryDestination: tc,
				})
			}
		}

		for _, tc := range testCases {
			t.Run(tc.name, func(t *testing.T) {

				qOrigin := tc.queryOrigin
				qDestination := tc.queryDestination
				sp, tp := routingService.Snap(context.Background(), qOrigin.GetLat(), qOrigin.GetLon(), qDestination.GetLat(), qDestination.GetLon())
				crpQuery := routing.NewCRPALTQuery(re)

				_, path, _ := crpQuery.ShortestPathSearch(sp.GetVId(), tp.GetVId())
				segmentPathCoords, _ := re.GetSegmentPath(path)
				spPolyline := da.GooglePoylineFromCoords(*segmentPathCoords)
				t.Logf("fastest route polyline for osm turn restriction https://www.openstreetmap.org/relation/%v: %s", tc.turnRestriction.id, valhallaVisualizerPolylineUrl(spPolyline))

				if subPath, correct := isCorrect(path, tc.turnRestriction, rn); !correct {
					isCorrect(path, tc.turnRestriction, rn) // buat debug
					t.Fatalf("%s: expected not contain path %v, got contain the path", tc.name, subPath)
				}

				source, target := sp.GetVId(), tp.GetVId()
				crpQuery2 := routing.NewCRPQuery(re) // better via-vertices
				ars := routing.NewAlternativeRouteSearch(re)
				param := ars.ParameterByRequest(source, target)
				crpQuery2.SetUpperBound(1.25)
				crpQuery2.ForAlternatives(true)
				cost, optPath, found := crpQuery2.ShortestPathSearch(source, target)
				if !found {
					return
				}
				optCost := util.WeightToSeconds(cost)
				alts, _ := ars.ComputeAlternativeRoutes(source, target, 4, false, 0, optCost, optPath, crpQuery2, param)
				crpQuery2.ForAlternatives(false)
				crpQuery2.Done()

				for i := 0; i < len(alts); i++ {
					altPath := alts[i].GetSegmentPath()
					if subPath, correct := isCorrect(altPath, tc.turnRestriction, rn); !correct {
						t.Errorf("%s: expected not contain path %v, got contain the path, restriction: %v", tc.name, subPath, tc.turnRestriction.restriction)
					}
				}
			})
		}
	}

}

package customizer

import (
	"bufio"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"net/http"
	"os"
	"strconv"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"

	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
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

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(url)
		if err != nil {
			t.Fatal(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			t.Fatal(err)
		}
		logger.Sugar().Infof("download complete")
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
		graph, logger, true, true,
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

	logger.Sugar().Infof("Preprocessing completed successfully.")

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

const (
	CUST_WORKERS = 10
	CELL_WORKERS = 5
)

// https://stackoverflow.com/questions/63842225/go-test-coverage-over-different-packages
// cd tests/customizer &&  go test -v . --cover -coverpkg=../../pkg/... -coverprofile=cust_coverage.out
// go tool cover -func=cust_coverage.out
// go tool cover -html=cust_coverage.out
func TestCRPCustomizerSimple(t *testing.T) {

	// https://visualgo.net/en/sssp

	bitpack := func(i, j da.Index) uint64 {
		return uint64(i) | (uint64(j) << 30)
	}

	testCases := []struct {
		name         string
		filepath     string
		want         [][]map[uint64]float64 // level -> cellId -> bitpack(source,target) -> st-shortcutWeight
		cellVertices [][][]da.Index

		apspFilePath    string // all pairs shortest paths filepath.
		minNumShortcuts int
		wantErr         bool
	}{
		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> big
			name:     "visualgo example graph weighted big",
			filepath: "./data/samplegraph/big",
			wantErr:  false,
			cellVertices: [][][]da.Index{
				{ // level 1
					{ // cell 1, level 1. // entry vertices: 1. exit vertices: 1,0
						0, 1,
					},
					{ // cell 2, level 1 // entry vertices: 3. exit vertices: 3,4
						3, 4,
					},
					{ // cell 3, level 1 // entry vertices: 5,7,8. exit vertices: gak ada
						5, 7, 8,
					},
					{ // cell 4, level 1 // entry vertices: 2,6 . exit vertices: 2,6
						2, 6,
					},
				},
				{ // level 2
					{ // cell 1, level 2.  // entry vertices: 3. exit vertices: 3,4,0,1
						0, 1, 3, 4,
					},
					{ // cell 2, level 2. // entry vertices: 2,5,6,7,8. exit vertices: 2
						5, 6, 7, 8, 2,
					},
				},
			},
			want: [][]map[uint64]float64{
				{
					{
						bitpack(1, 1): 0,
						bitpack(1, 0): pkg.INF_WEIGHT,
					},
					{
						bitpack(3, 3): 0.0,
						bitpack(3, 4): 20.0,
					},
					{},
					{
						bitpack(6, 2): pkg.INF_WEIGHT,
						bitpack(6, 6): 0,
						bitpack(2, 2): 0,
						bitpack(2, 6): 21,
					},
				},
				{
					{
						bitpack(3, 3): 0.0,
						bitpack(3, 4): 20.0,
						bitpack(3, 0): pkg.INF_WEIGHT,
						bitpack(3, 1): 29,
					},
					{
						bitpack(2, 2): 0.0,
						bitpack(5, 2): pkg.INF_WEIGHT, // gak ada path dari 5 ke 2 only pakai edges in cell 2 level 1. edges \in cell C, iff head and tail dari edge \in C.
						bitpack(6, 2): pkg.INF_WEIGHT,
						bitpack(7, 2): pkg.INF_WEIGHT,
						bitpack(8, 2): pkg.INF_WEIGHT,
					},
				},
			},
			minNumShortcuts: 17,
			apspFilePath:    "./data/samplegraph/big.out",
		},
		{
			// https://visualgo.net/en/sssp
			// example graph -> weighted -> cp4 4.10 d/w
			name:     "visualgo example graph weighted cp4 4.10 d/w",
			filepath: "./data/samplegraph/410",
			wantErr:  false,
			cellVertices: [][][]da.Index{
				{ // level 1
					{ // cell 1, level 1. entry vertices: 1, exit vertices: 1
						1,
					},
					{ // cell 2, level 1. entry vertices: 2, exit vertices: 2
						2,
					},
					{ // cell 3, level 1. entry vertices: 0, 3. exit vertices:  0,3
						0, 3,
					},
					{ // cell 4, level 1. entry vertices: 4. exit vertices: gak ada
						4,
					},
				},
				{ // level 2
					{ // cell 1, level 2. entry vertices: 1. exit vertices: 2
						1, 2,
					},
					{ // cell 2, level 2. entry vertices: 4, 0, 3. exit vertices: 0
						4, 0, 3,
					},
				},
			},
			want: [][]map[uint64]float64{
				{
					{
						bitpack(1, 1): 0,
					},
					{
						bitpack(2, 2): 0,
					},
					{
						bitpack(0, 0): 0,
						bitpack(3, 3): 0,
						bitpack(0, 3): 6,
						bitpack(3, 0): pkg.INF_WEIGHT,
					},
					{},
				},
				{
					{
						bitpack(1, 2): 2,
					},
					{
						bitpack(4, 0): pkg.INF_WEIGHT,
						bitpack(0, 0): 0,
						bitpack(3, 0): pkg.INF_WEIGHT,
					},
				},
			},
			minNumShortcuts: 9,
			apspFilePath:    "./data/samplegraph/410.out",
		},
	}

	buildGraph := func(filepath string, cellVertices [][][]da.Index) (*engine.Engine, *landmark.Landmark, map[da.Index]da.Index, error) {
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
			g, logger, true, false,
		)
		mp.SetCellVertices(cellVertices)

		mlp := mp.BuildMLP()

		prep := preprocesser.NewPreprocessor(g, mlp, logger)
		err = prep.PreProcessing(false)
		if err != nil {
			t.Fatalf("err: %v", err)
		}

		logger.Sugar().Infof("Preprocessing completed successfully.")

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
			re, lm, newToOldVidMap, err := buildGraph(tc.filepath, tc.cellVertices)
			if !tc.wantErr && (err != nil) {
				t.Errorf("expected error: %v, got error", tc.wantErr)
			} else if tc.wantErr && (err == nil) {
				t.Errorf("expected error: %v, got no error", tc.wantErr)
			}

			og := re.GetRoutingEngine().GetOverlayGraph()
			m := re.GetRoutingEngine().GetMetrics()
			cellMapInLevelOne := og.GetAllCellsInLevel(1)

			gotNumOfShortcuts := 0

			for cellId, cell := range cellMapInLevelOne {
				cellIdInLevel := og.OffUpperBit(cellId, 1)
				for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
					startOverlayVertexId := og.GetEntryId(cell, i)
					enOverlayVertex := og.GetVertex(startOverlayVertexId)
					enOriVId := newToOldVidMap[enOverlayVertex.GetOriginalVertex()]

					for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
						exitOverlayVId := og.GetExitId(cell, j)
						exOverlayVertex := og.GetVertex(exitOverlayVId)
						exOriVId := newToOldVidMap[exOverlayVertex.GetOriginalVertex()]

						shortcutId := da.Index(cell.GetShortcutWeightId(i, j))
						got := m.GetShortcutWeight(shortcutId)

						query := bitpack(enOriVId, exOriVId)

						expectedSw := tc.want[0][cellIdInLevel][query]
						if !da.Eq(got, expectedSw) {
							t.Errorf("expected shortcut weight: %v, got: %v", expectedSw, got)
						}
					}
				}

				gotNumOfShortcuts += int(cell.GetNumEntryPoints() * cell.GetNumExitPoints())
			}

			for level := 2; level <= og.GetLevelInfo().GetLevelCount(); level++ {
				// validating shortcut weights di level l
				cellMapInLevel := og.GetAllCellsInLevel(level)

				for cellId, cell := range cellMapInLevel {
					cellIdInLevel := og.OffUpperBit(cellId, uint8(level))

					for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
						startOverlayVertexId := og.GetEntryId(cell, i)
						enOverlayVertex := og.GetVertex(startOverlayVertexId)
						enOriVId := newToOldVidMap[enOverlayVertex.GetOriginalVertex()]

						for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
							exitOverlayVId := og.GetExitId(cell, j)
							exOverlayVertex := og.GetVertex(exitOverlayVId)
							exOriVId := newToOldVidMap[exOverlayVertex.GetOriginalVertex()]

							shortcutId := da.Index(cell.GetShortcutWeightId(i, j))
							got := m.GetShortcutWeight(shortcutId)
							query := bitpack(enOriVId, exOriVId)

							expectedSw := tc.want[level-1][cellIdInLevel][query]
							if !da.Eq(got, expectedSw) {
								t.Errorf("expected shortcut weight: %v, got: %v", expectedSw, got)
							}
						}
					}
					gotNumOfShortcuts += int(cell.GetNumEntryPoints() * cell.GetNumExitPoints())
				}
			}

			if gotNumOfShortcuts < tc.minNumShortcuts {
				t.Errorf("expected minimal number of shortcuts: %v, got: %v", tc.minNumShortcuts, gotNumOfShortcuts)
			}

			// validating precalculated landmark distances
			g := re.GetRoutingEngine().GetGraph()
			landmarks := lm.GetLandmarkVIds()
			lw := lm.GetLandmarkVWeights()
			vlw := lm.GetVerticesLandmarkWeights()
			n := g.NumberOfVertices()

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

			for i := 0; i < len(landmarks); i++ {
				landmarkId := landmarks[i]
				oldLandmarkId := newToOldVidMap[landmarkId]

				for v := 0; v < n; v++ {
					oldV := newToOldVidMap[da.Index(v)]
					gotLwDist := lw[i][v]
					expectedLwDist := apsp[bitpack(oldLandmarkId, da.Index(oldV))]
					if !da.Eq(gotLwDist, expectedLwDist) {
						t.Errorf("expected shortest path cost from landmarkId %v to vertex %v: %v, got: %v", landmarkId, v, gotLwDist, expectedLwDist)
					}
					gotVlwDist := vlw[v][i]
					expectedVlwDist := apsp[bitpack(da.Index(oldV), oldLandmarkId)]
					if !da.Eq(gotVlwDist, expectedVlwDist) {
						t.Errorf("expected shortest path cost from vertex %v to landmarkId %v: %v, got: %v", v, landmarkId, gotVlwDist, expectedVlwDist)
					}

				}
			}

			fOut.Close()
		})
	}
}

// todo: add test customizer & query pake file osm yang di gdrive
// done test customizer
// todo: add test preprocessor & query
// please run the test using command: "cd tests/customizer && go test -run TestCRPCustomizer  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode
func TestCRPCustomizer(t *testing.T) {
	re, lm := setup(t)

	og := re.GetRoutingEngine().GetOverlayGraph()
	m := re.GetRoutingEngine().GetMetrics()
	g := re.GetRoutingEngine().GetGraph()
	cellMapInLevelOne := og.GetAllCellsInLevel(1)

	// validating shortcut weights di level 1
	// shotcuts weights di level 1 didapat dari run dijkstra dari setiap entry vertices ke setiap exit vertices dari setiap cells di level 1
	// dengan hanya menggunakan edges & vertices dari cell yang memuat entry/exit vertex

	// ref1: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
	// dari ref1: Theorem 1. H is an overlay of G. or dist_H(u,v)=dist_G(u,v)
	// Note that the theorem holds even though some shortcuts added to H are not necessarily shortest path
	// in either G or H. Such shortcuts are redundant, but do not affect correctness

	// meskipun shortcuts weight bukan shortest path di graf G (kalau pakai semua vertices & edges dari original graf G), gak affect correctness dari CRP query

	expectedNumOfShortcuts := 0
	for _, cell := range cellMapInLevelOne {
		for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
			for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
				shortcutId := da.Index(cell.GetShortcutWeightId(i, j))
				sw := m.GetShortcutWeight(shortcutId)
				if da.Lt(sw, 0) {
					t.Errorf("shortcut weight must be positive")
				}
			}
		}
		expectedNumOfShortcuts += int(cell.GetNumEntryPoints() * cell.GetNumExitPoints())
	}

	for level := 2; level <= og.GetLevelInfo().GetLevelCount(); level++ {
		// validating shortcut weights di level l
		cellMapInLevel := og.GetAllCellsInLevel(level)

		for _, cell := range cellMapInLevel {
			for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					shortcutId := da.Index(cell.GetShortcutWeightId(i, j))
					sw := m.GetShortcutWeight(shortcutId)
					if da.Lt(sw, 0) {
						t.Errorf("shortcut weight must be positive")
					}
				}
			}
			expectedNumOfShortcuts += int(cell.GetNumEntryPoints() * cell.GetNumExitPoints())
		}
	}

	ow := m.GetWeights()
	gotNumOfShortcuts := ow.GetNumberOfShortcuts()
	if gotNumOfShortcuts != expectedNumOfShortcuts {
		t.Errorf("expected number of shortcuts: %v, got: %v", expectedNumOfShortcuts, gotNumOfShortcuts)
	}

	t.Logf("validating precalculated landmark distances...")

	// validating precalculated landmark distances
	landmarks := lm.GetLandmarkVIds()
	lw := lm.GetLandmarkVWeights()
	vlw := lm.GetVerticesLandmarkWeights()
	n := g.NumberOfVertices()
	for i := 0; i < len(landmarks); i++ {
		landmarkVId := landmarks[i]
		s := landmarkVId
		dijkstraQuery := routing.NewDijkstra(re.GetRoutingEngine(), false)
		sps, _ := dijkstraQuery.ShortestPath(s)
		for v := 0; v < n; v++ {
			spToV := sps[v] // sp dist dari landmark ke v
			expectedSpToV := lw[i][v]
			if !da.Eq(expectedSpToV, spToV) {
				t.Errorf("expected shortest path travel times: %v, got: %v", expectedSpToV, spToV)
			}
		}

		dijkstraQuery = routing.NewDijkstra(re.GetRoutingEngine(), true)
		sps, _ = dijkstraQuery.ShortestPath(s)
		for v := 0; v < n; v++ {
			got := sps[v] // sp dist dari v ke landmark
			expectedSPVToL := vlw[v][i]
			if !da.Eq(expectedSPVToL, got) {
				t.Errorf("expected shortest path travel times: %v, got: %v", expectedSPVToL, got)
			}
		}
	}

}

package customizer

import (
	"flag"
	"fmt"
	"io"
	"net/http"
	"os"
	"strconv"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"

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

func setup() (*engine.Engine, *landmark.Landmark) {
	if err := os.MkdirAll("./data", 0755); err != nil {
		panic(err)
	}
	logger, err := log.New()
	if err != nil {
		panic(err)
	}
	workingDir, err := os.Getwd()
	err = util.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}
	if _, err := os.Stat(osmfFile); os.IsNotExist(err) {
		output, err := os.Create(osmfFile)
		if err != nil {
			panic(err)
		}
		defer output.Close()

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(url)
		if err != nil {
			panic(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			panic(err)
		}
		logger.Sugar().Infof("download complete")
	}

	osmParser := osmparser.NewOSMParserV2()

	graph, err := osmParser.Parse(fmt.Sprintf("%s", osmfFile), logger, false)
	if err != nil {
		panic(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
		if err != nil {
			panic(err)
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
		panic(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s", "crp_inertial_flow_"+mlpFile+".mlp"))
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger)
	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}

	logger.Sugar().Infof("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	m, err := custom.Customize()
	if err != nil {
		panic(err)
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(16, m, custom, logger)
	if err != nil {
		panic(err)
	}
	err = lm.WriteLandmark(landmarkFile, custom)
	if err != nil {
		panic(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		panic(err)
	}

	return re, lm
}

const (
	CUST_WORKERS = 10
	CELL_WORKERS = 5
)

// todo: add test customizer & query pake file osm yang di gdrive
// done test customizer
// todo: add test preprocessor & query
// please run the test using command: "cd tests/customizer && go test -run TestCRPCustomizer  -v -timeout=0  -count=1"
// karena bakal timeout kalau pakai run test vscode
func TestCRPCustomizer(t *testing.T) {
	re, lm := setup()

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
		}
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

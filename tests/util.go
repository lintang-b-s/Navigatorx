package tests

import (
	"fmt"
	"io"
	"net/http"
	"os"
	"strconv"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	mlpFile                 = "stress_test_yogyakarta"
	url                     = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile                = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original_query_test.graph"
	overlayGraphFile string = "./data/overlay_graph_query_test.graph"
	metricsFile      string = "./data/metrics_query_test.txt"
	landmarkFile     string = "./data/landmark_query_test.lm"
)

func init() {
	util.InitConfig()
}

func Setup(t *testing.T) (*engine.Engine, *landmark.Landmark) {
	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}
	logger, err := log.New()
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

	graph, edgeInfoIds, err := op.Parse(fmt.Sprintf("%s", osmfFile), logger)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split("8,10,11,12,14", ",")
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
		graph, logger, false, false,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s.mlp", mlpFile))
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
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
	err = lm.PreprocessALT(16, m, graph, logger)
	if err != nil {
		t.Fatal(err)
	}
	err = lm.WriteLandmark(landmarkFile, graph)
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, lm
}

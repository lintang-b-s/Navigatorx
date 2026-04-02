package partitioner

import (
	"flag"
	"fmt"
	"io"
	"net/http"
	"os"
	"strconv"
	"strings"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"

	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
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

func setup() (*da.Graph, *partitioner.MultilevelPartitioner) {
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

	op := osmparser.NewOSMParserV2()

	graph, _, err := op.Parse(fmt.Sprintf("%s", osmfFile), logger, false)
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
		graph, logger, true, true, true,
	)

	return graph, mp
}

const (
	invalidCellId int = -1
)

// todo: add test customizer & query pake file osm yang di gdrive
// please run the test using command: "cd tests/partitioner &&  go test -v . --cover -coverpkg=../../pkg/... -coverprofile=part_coverage.out  -v -timeout=0  -count=1"
// go tool cover -func=part_coverage.out
// go tool cover -html=part_coverage.out
// karena bakal timeout kalau pakai run test vscode
func TestInertialFlowMLP(t *testing.T) {
	g, mp := setup()
	mp.RunMultilevelPartitioning()

	n := g.NumberOfVertices()
	validateMLP := func(cellVertices [][][]da.Index) (bool, int) {
		// v cellVertices: nodes in each cells in each level

		for l := 0; l < len(cellVertices); l++ {
			numVerticesInPartitionLevell := 0
			cellsInLevel := cellVertices[l]

			verticeCells := make([]int, n)
			for i := 0; i < n; i++ {
				verticeCells[i] = invalidCellId
			}

			for cellId, cell := range cellsInLevel {
				numVerticesInPartitionLevell += len(cell)
				for _, v := range cell {
					if verticeCells[v] != invalidCellId {
						// cek setiap sel di level l saling disjoint
						return false, l + 1
					}
					verticeCells[v] = cellId
				}
			}

			if numVerticesInPartitionLevell != n {
				// cek union semua sel sama dengan vertices dari graph
				return false, l + 1
			}
		}

		return true, 0
	}

	correct, wrongLevel := validateMLP(mp.GetCellVertices())
	if !correct {
		t.Errorf("jumlah cell vertices di level l: %v tidak sama dengan jumlah vertices pada graf", wrongLevel)
	}
}

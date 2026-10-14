package partitioner

import (
	"path/filepath"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"

	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
)

const (
	osmFile = "./data/yogyakarta.osm.pbf"
)

func init() {
	config.InitConfig()
	config.InitRegionName("partitioner_test", pkg.TEST)
}

func setup() (*da.Graph, *partitioner.MultilevelPartitioner, []uint64) {
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	outputDir := filepath.Join(workingDir, "data")
	if err := util.EnsureDirExists(outputDir); err != nil {
		panic(err)
	}
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	op := extractor.NewExtractor[int32]()

	nbg, ebg, _, ebgMapping, _, err := op.Extract(filepath.Join(pkg.WorkingDir, osmFile), logger)
	if err != nil {
		panic(err)
	}

	pss := strings.Split("8,10,11,12,14", ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := util.ParseTextInt(pss[i])
		if err != nil {
			panic(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		nbg, logger,
	)

	return ebg, mp, ebgMapping
}

const (
	invalidCellId int = -1
)

// todo: add test customizer & query pake file osm yang di gdrive (DONE)
// please run the test using command: "go test ./tests/partitioner -v . --cover -coverpkg=../../pkg/... -coverprofile=part_coverage.out  -v -timeout=0  -count=1"
// go tool cover -func=part_coverage.out
// go tool cover -html=part_coverage.out
// karena bakal timeout kalau pakai run test vscode
// this graph partitioning with minimized cut edges is np-hard, so we only check if each vertex lies inside one cell & union all cell vertices are the graph vertices .
func TestInertialFlowMLP(t *testing.T) {
	ebg, mp, ebgMapping := setup()
	mp.RunMultilevelPartitioning()
	mp.MapToEdgeBasedGraph(ebg, ebgMapping)

	n := ebg.NumberOfVertices()
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

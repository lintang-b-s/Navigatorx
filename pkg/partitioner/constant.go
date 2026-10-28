package partitioner

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

const (
	INVALID_LEVEL        da.Index = da.Index(math.MaxUint32)
	ARTIFICIAL_SOURCE_ID          = int32(2147483646)
	ARTIFICIAL_SINK_ID            = int32(2147483647)

	INVALID_CELL_ID                          = -1
	SOURCE_SINK_RATE                         = 0.2 // paling bagus: jumlah cut edges paling sedikit -> jumlah shortcut edges paling sedikit  &&  paling cepet selesai. dibanding ( 0.025, 0.05, 0.1, 0.15, 0.25, 0.3)
	INERTIAL_FLOW_ITERATION_LARGE_GRAPH      = 4
	LARGE_GRAPH_NUMBER_OF_VERTICES           = 1000000
	USE_RANDOMIZED_SELECT               bool = true
	InertialFlowChanSize                     = 16
	CellInOutChanSize                        = 50
)

//revive:disable:var-naming
var (
	// 	https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	INERTIAL_FLOW_WORKERS = max(pkg.NUM_CPU/4, 1)
	BISECTION_WORKERS     = max(pkg.NUM_CPU/6, 1)
	LEVEL_WORKERS         = max(pkg.NUM_CPU/6, 1)
)

//revive:enable:var-naming

type PartitionQuality string

const (
	BEST = "best"
	FAST = "fast"
)

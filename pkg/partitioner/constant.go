package partitioner

import "github.com/lintang-b-s/Navigatorx/pkg"

const (
	INVALID_LEVEL           = 9e9
	ARTIFICIAL_SOURCE_ID    = int32(2147483646)
	ARTIFICIAL_SINK_ID      = int32(2147483647)
	INERTIAL_FLOW_ITERATION = 25

	INVALID_PARTITION_ID                     = -1
	SOURCE_SINK_RATE                         = 0.2 // paling bagus: jumlah cut edges paling sedikit -> jumlah shortcuts paling sedikit  &&  paling cepet selesai. dibanding ( 0.025, 0.05, 0.1, 0.15, 0.25, 0.3)
	INERTIAL_FLOW_ITERATION_LARGE_GRAPH      = 1
	LARGE_GRAPH_NUMBER_OF_VERTICES           = 100000
	USE_RANDOMIZED_SELECT               bool = true
)

var (
	// 	https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	INERTIAL_FLOW_WORKERS = pkg.NUM_CPU / 4
	BISECTION_WORKERS     = pkg.NUM_CPU / 6
	LEVEL_WORKERS         = pkg.NUM_CPU / 6
)

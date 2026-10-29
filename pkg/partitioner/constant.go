package partitioner

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

const (
	// InvalidLevel is the invalid level value.
	InvalidLevel da.Index = da.Index(math.MaxUint32)
	// ArtificialSourceId is the artificial source id value.
	ArtificialSourceId = int32(2147483646)
	// ArtificialSinkId is the artificial sink id value.
	ArtificialSinkId = int32(2147483647)

	// InvalidCellId is the invalid cell id value.
	InvalidCellId = -1

	// SourceSinkRate is the source sink rate value.
	SourceSinkRate = 0.2 // paling bagus: jumlah cut edges paling sedikit -> jumlah shortcut edges paling sedikit  &&  paling cepet selesai. dibanding ( 0.025, 0.05, 0.1, 0.15, 0.25, 0.3)

	// InertialFlowIterationLargeGraph is the inertial flow iteration large graph value.
	InertialFlowIterationLargeGraph = 4

	// LargeGraphNumberOfVertices is the large graph number of vertices value.
	LargeGraphNumberOfVertices = 1000000

	// UseRandomizedSelect is the use randomized select value.
	UseRandomizedSelect bool = true

	// InertialFlowChanSize is the inertial flow chan size value.
	InertialFlowChanSize = 16

	// CellInOutChanSize is the cell in out chan size value.
	CellInOutChanSize = 50
)

var (
	// InertialFlowWorkers sets the number of workers used by inertial flow.
	// 	https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	InertialFlowWorkers = max(pkg.NumCPU/4, 1)
	// BisectionWorkers sets the number of workers used by bisection.
	// LevelWorkers sets the number of workers assigned to each level.
	BisectionWorkers = max(pkg.NumCPU/6, 1)
	// LevelWorkers sets the number of workers assigned to each level.
	LevelWorkers = max(pkg.NumCPU/6, 1)
)

// PartitionQuality represents partition quality.
type PartitionQuality string

const (
	// BEST is the best partition quality.
	BEST = "best"
	// FAST is the fast partition quality.
	FAST = "fast"
)

package customizer

import "github.com/lintang-b-s/Navigatorx/pkg"

var (
	/// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	CELL_WORKER       = pkg.NUM_CPU / 6
	CUSTOMIZER_WORKER = pkg.NUM_CPU / 3
)

const (
	CELL_ENTRIES_CHAN_SIZE         = 4096 * 4
	INVALID_LOOKUPTABLE_VAL_ID int = -1
	defaultMaxSpeedDummyEdge       = pkg.INF_WEIGHT
)

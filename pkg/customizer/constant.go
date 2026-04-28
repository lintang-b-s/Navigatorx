package customizer

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	/// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	CELL_WORKER       = util.ClampMin(pkg.NUM_CPU/6, 1)
	CUSTOMIZER_WORKER = util.ClampMin(pkg.NUM_CPU/3, 1)
)

const (
	CELL_ENTRIES_CHAN_SIZE         = 4096 * 4
	INVALID_LOOKUPTABLE_VAL_ID int = -1
)

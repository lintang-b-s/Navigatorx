package customizer

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
)

var (
	/// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	CELL_WORKER       = max(pkg.NUM_CPU/6, 1)
	CUSTOMIZER_WORKER = max(pkg.NUM_CPU/3, 1)
)

const (
	CELL_ENTRIES_CHAN_SIZE = 1024
	dijkstraResChanSize    = 1024
	cellCliqueOutChanSize  = 512
)

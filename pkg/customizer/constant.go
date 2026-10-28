package customizer

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
)

var (
	// CellWorker sets the number of workers assigned to cells.
	/// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	CellWorker = max(pkg.NumCPU/6, 1)
	// CustomizerWorker sets the number of customizer workers.
	CustomizerWorker = max(pkg.NumCPU/3, 1)
)

const (
	// CellEntriesChanSize is the cell entries chan size value.
	CellEntriesChanSize   = 1024
	dijkstraResChanSize   = 1024
	cellCliqueOutChanSize = 512
)

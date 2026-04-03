package customizer

import "github.com/lintang-b-s/Navigatorx/pkg"

var (
	/// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	CELL_WORKER       = pkg.NUM_CPU / 6
	CUSTOMIZER_WORKER = pkg.NUM_CPU / 3
)

package landmark

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
)

var (
	// WORKERS handles workers.
	// 	https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	WORKERS          = max(pkg.NumCPU/4, 1)
	landmarkChanSize = 4
)

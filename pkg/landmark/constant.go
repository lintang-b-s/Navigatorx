package landmark

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
)

var (
	// 	https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	WORKERS          = max(pkg.NUM_CPU/4, 1)
	landmarkChanSize = 4
)

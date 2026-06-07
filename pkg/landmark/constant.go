package landmark

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	// 	https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	WORKERS          = util.ClampMin(pkg.NUM_CPU/3, 1)
	landmarkChanSize = 6
)

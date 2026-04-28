package landmark

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	// 	https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	WORKERS int32 = int32(util.ClampMin(pkg.NUM_CPU/3, 1))
)

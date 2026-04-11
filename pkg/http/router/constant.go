package router

import (
	"sync/atomic"
	"time"
)

var isShuttingDown atomic.Bool

const ( // https://victoriametrics.com/blog/go-graceful-shutdown/
	shutdownHardPeriod = 3 *time.Second
	readinessDrainDelay = 1*time.Second
)
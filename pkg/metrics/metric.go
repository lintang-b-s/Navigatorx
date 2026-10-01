// Package metrics provides utilities for managing and calculating graph metrics (edge & turn costs) and stalling tables.
package metrics

import (
	"fmt"
	"sync"
	"sync/atomic"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Metric[W util.RoutingNumber] struct {
	// https://go101.org/article/concurrent-atomic-operation.html   https://pkg.go.dev/sync/atomic#Pointer.Load   https://go.dev/ref/mem#atomic
	// https://goperf.dev/01-common-patterns/atomic-ops/
	// https://github.com/cockroachdb/cockroach/blob/d30c905fff79ef825adc96bcc647f1872a90f2ff/pkg/util/syncutil/map.go#L59
	shortcutWeights                             atomic.Pointer[da.OverlayWeights[W]]
	costFunction                                atomic.Pointer[TimeFunction[W]]
	lastSegmentSpeedFiles, lastTurnPenaltyFiles atomic.Pointer[[]string]
	metricFilepath, timeFunctionFilePath        string
	mu                                          sync.Mutex
}

func NewMetric[W util.RoutingNumber](
	numOfVertices int,
	timeFunctionFilePath string,
	overlayWeights *da.OverlayWeights[W],
	metricFilepath string,
) *Metric[W] {
	m := &Metric[W]{
		metricFilepath:       metricFilepath,
		timeFunctionFilePath: timeFunctionFilePath,
		mu:                   sync.Mutex{},
	}
	m.shortcutWeights.Store(overlayWeights)
	m.lastSegmentSpeedFiles.Store(&[]string{})
	m.lastTurnPenaltyFiles.Store(&[]string{})

	return m
}

func (met *Metric[W]) GetShortcutWeights() *da.OverlayWeights[W] {
	return met.shortcutWeights.Load()
}

func (met *Metric[W]) SetTimeFunction(tf *TimeFunction[W]) {
	met.costFunction.Store(tf)
}

func (met *Metric[W]) GetCostFunction() *TimeFunction[W] {
	return met.costFunction.Load()
}

// GetWeight. get weight dari outEdge dengan id eId
// eId adalah id/index dari outEdge yang ingin didapat weightnya
func (met *Metric[W]) GetWeight(eId da.Index) W {
	cf := met.costFunction.Load()
	return cf.GetWeight(eId)
}

func (met *Metric[W]) GetDurationFromLength(segId da.Index, length uint32) W {
	cf := met.costFunction.Load()
	r := float64(length) / float64(cf.segmentLengths[segId])
	return W(float64(cf.segmentDurations[segId]) * r)
}

func (met *Metric[W]) GetDuration(segId da.Index) W {
	cf := met.costFunction.Load()
	return W(cf.segmentDurations[segId])
}

func (met *Metric[W]) GetSegmentLength(segId da.Index) uint32 {
	cf := met.costFunction.Load()
	return cf.GetSegmentLength(segId)
}

func (met *Metric[W]) GetSegmentSpeed(segId da.Index) float64 {
	cf := met.costFunction.Load()
	if util.Ge(int32(cf.segmentDurations[segId]), util.INF_WEIGHT_FIXED) {
		return 0
	}
	return float64(cf.segmentLengths[segId]) / float64(cf.segmentDurations[segId])
}

func (met *Metric[W]) GetShortcutWeight(offset da.Index) W {
	cf := met.shortcutWeights.Load()
	return cf.GetWeight(offset)
}

func (met *Metric[W]) GetFilePath() string {
	return met.metricFilepath
}

func (met *Metric[W]) SetLastSegmentSpeedFiles(filepaths []string) {

	met.lastSegmentSpeedFiles.Store(&filepaths)
}

func (met *Metric[W]) SetLastTurnPenaltyFiles(filepaths []string) {
	met.lastTurnPenaltyFiles.Store(&filepaths)
}

func (met *Metric[W]) GetLastSegmentSpeedFiles() []string {
	return *met.lastSegmentSpeedFiles.Load()
}

func (met *Metric[W]) GetLastTurnPenaltyFiles() []string {
	return *met.lastTurnPenaltyFiles.Load()
}

func (met *Metric[W]) UpdateMetrics() error {
	newMet, err := ReadFromFile[W](met.metricFilepath, met.timeFunctionFilePath)
	if err != nil {
		return fmt.Errorf("UpdateMetrics: failed to read new metrics, filepath: %s: %w", met.metricFilepath, err)
	}

	met.mu.Lock()
	defer met.mu.Unlock()
	nmw := newMet.shortcutWeights.Load()
	nmcf := newMet.costFunction.Load()
	met.shortcutWeights.Store(nmw)
	met.costFunction.Store(nmcf)
	return nil
}

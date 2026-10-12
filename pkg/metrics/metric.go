// Package metrics provides utilities for managing and calculating graph metrics (edge & turn costs).
package metrics

import (
	"fmt"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Metric[W util.RoutingNumber] struct {
	shortcutWeights                             *da.OverlayWeights[W]
	costFunction                                *TimeFunction[W]
	lastSegmentSpeedFiles, lastTurnPenaltyFiles []string
	metricFilepath, timeFunctionFilePath        string
}

func NewMetric[W util.RoutingNumber](
	numOfVertices int,
	timeFunctionFilePath string,
	overlayWeights *da.OverlayWeights[W],
	metricFilepath string,
) *Metric[W] {
	m := &Metric[W]{
		shortcutWeights:       overlayWeights,
		metricFilepath:        metricFilepath,
		timeFunctionFilePath:  timeFunctionFilePath,
		lastSegmentSpeedFiles: make([]string, 0),
		lastTurnPenaltyFiles:  make([]string, 0),
	}

	return m
}

func (met *Metric[W]) GetShortcutWeights() *da.OverlayWeights[W] {
	return met.shortcutWeights
}

func (met *Metric[W]) SetTimeFunction(tf *TimeFunction[W]) {
	met.costFunction = tf
}

func (met *Metric[W]) GetCostFunction() *TimeFunction[W] {
	return met.costFunction
}

// GetWeight. get weight dari outEdge dengan id eId
// eId adalah id/index dari outEdge yang ingin didapat weightnya
func (met *Metric[W]) GetWeight(eId da.Index) W {
	cf := met.costFunction
	return cf.GetWeight(eId)
}

func (met *Metric[W]) GetDurationFromLength(segId da.Index, length float64) float64 {
	cf := met.costFunction
	r := float64(length) / float64(cf.segmentLengths[segId])
	return float64(cf.segmentDurations[segId]) * r
}

func (met *Metric[W]) GetDurationSeconds(segId da.Index) float64 {
	cf := met.costFunction
	dur := W(cf.segmentDurations[segId])
	return util.WeightToSeconds(dur)
}

func (met *Metric[W]) GetSegmentLength(segId da.Index) float64 {
	cf := met.costFunction
	l := cf.GetSegmentLength(segId)
	return util.DistanceToMeters(l)
}

func (met *Metric[W]) GetSegmentSpeed(segId da.Index) float64 {
	cf := met.costFunction
	if util.Ge(int32(cf.segmentDurations[segId]), util.INF_WEIGHT_FIXED) {
		return 0
	}
	return float64(cf.segmentLengths[segId]) / float64(cf.segmentDurations[segId])
}

func (met *Metric[W]) GetShortcutWeight(offset da.Index) W {
	sw := met.shortcutWeights
	return sw.GetWeight(offset)
}

func (met *Metric[W]) GetFilePath() string {
	return met.metricFilepath
}

func (met *Metric[W]) SetLastSegmentSpeedFiles(filepaths []string) {

	met.lastSegmentSpeedFiles = filepaths
}

func (met *Metric[W]) SetLastTurnPenaltyFiles(filepaths []string) {
	met.lastTurnPenaltyFiles = filepaths
}

func (met *Metric[W]) GetLastSegmentSpeedFiles() []string {
	return met.lastSegmentSpeedFiles
}

func (met *Metric[W]) GetLastTurnPenaltyFiles() []string {
	return met.lastTurnPenaltyFiles
}

func (met *Metric[W]) UpdateMetrics() error {
	newMet, err := ReadFromFile[W](met.metricFilepath, met.timeFunctionFilePath)
	if err != nil {
		return fmt.Errorf("UpdateMetrics: failed to read new metrics, filepath: %s: %w", met.metricFilepath, err)
	}

	nmw := newMet.shortcutWeights
	nmcf := newMet.costFunction
	met.shortcutWeights = nmw
	met.costFunction = nmcf
	return nil
}

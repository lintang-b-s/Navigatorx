// Package metrics provides utilities for managing and calculating graph metrics (edge & turn costs).
package metrics

import (
	"fmt"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// Metric represents the edge and shortcut metrics for a graph.
type Metric[W util.Numeric] struct {
	shortcutWeights                             *da.OverlayWeights[W]
	costFunction                                *TimeFunction[W]
	lastSegmentSpeedFiles, lastTurnPenaltyFiles []string
	metricFilepath, timeFunctionFilePath        string
}

// NewMetric creates a new Metric instance.
func NewMetric[W util.Numeric](
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

// GetShortcutWeights get shortcut weights struct
func (met *Metric[W]) GetShortcutWeights() *da.OverlayWeights[W] {
	return met.shortcutWeights
}

// SetTimeFunction set time function
func (met *Metric[W]) SetTimeFunction(tf *TimeFunction[W]) {
	met.costFunction = tf
}

// GetCostFunction gets current cost function
func (met *Metric[W]) GetCostFunction() *TimeFunction[W] {
	return met.costFunction
}

// GetWeight gets weight dari outEdge dengan id eId
// eId adalah id/index dari outEdge yang ingin didapat weightnya
//
//go:inline
func (met *Metric[W]) GetWeight(eId da.Index) W {
	cf := met.costFunction
	return cf.GetWeight(eId)
}

// GetDurationFromLength get duration of a road segment at specified length
func (met *Metric[W]) GetDurationFromLength(segId da.Index, length float64) float64 {
	cf := met.costFunction
	r := float64(length) / float64(cf.segmentLengths[segId])
	return float64(cf.segmentDurations[segId]) * r
}

// GetDurationSeconds get duration of a road segment in seconds
func (met *Metric[W]) GetDurationSeconds(segId da.Index) float64 {
	cf := met.costFunction
	dur := W(cf.segmentDurations[segId])
	return util.WeightToSeconds(dur)
}

// GetSegmentLength get road segment length
func (met *Metric[W]) GetSegmentLength(segId da.Index) float64 {
	cf := met.costFunction
	l := cf.GetSegmentLength(segId)
	return util.DistanceToMeters(l)
}

// GetSegmentSpeed get road segment speed
func (met *Metric[W]) GetSegmentSpeed(segId da.Index) float64 {
	cf := met.costFunction
	if util.Ge(int32(cf.segmentDurations[segId]), util.InfWeightFixed) {
		return 0
	}
	return float64(cf.segmentLengths[segId]) / float64(cf.segmentDurations[segId])
}

// GetShortcutWeight get shortcut edge weight
func (met *Metric[W]) GetShortcutWeight(offset da.Index) W {

	return met.shortcutWeights.GetWeight(offset)
}

// GetFilePath returns the file path of the metric.
func (met *Metric[W]) GetFilePath() string {
	return met.metricFilepath
}

// SetLastSegmentSpeedFiles sets the file paths of the last segment speed files.
func (met *Metric[W]) SetLastSegmentSpeedFiles(filepaths []string) {

	met.lastSegmentSpeedFiles = filepaths
}

// SetLastTurnPenaltyFiles sets the file paths of the last turn penalty files.
func (met *Metric[W]) SetLastTurnPenaltyFiles(filepaths []string) {
	met.lastTurnPenaltyFiles = filepaths
}

// GetLastSegmentSpeedFiles returns the file paths of the last segment speed files.
func (met *Metric[W]) GetLastSegmentSpeedFiles() []string {
	return met.lastSegmentSpeedFiles
}

// GetLastTurnPenaltyFiles returns the file paths of the last turn penalty files.
func (met *Metric[W]) GetLastTurnPenaltyFiles() []string {
	return met.lastTurnPenaltyFiles
}

// UpdateMetrics reloads metrics from files.
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

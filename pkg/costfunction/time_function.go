// Package costfunction provides typed edge and turn cost calculations.
package costfunction

import (
	"math"
	"path/filepath"
	"strings"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func PreprocessingTimeFunctionPath(graphFilename string) string {
	base := strings.TrimSuffix(filepath.Base(graphFilename), filepath.Ext(graphFilename))
	base = strings.TrimSuffix(base, "_original")
	return filepath.Join(filepath.Dir(graphFilename), base+"_preprocessing_timefunction.ntf")
}

type TimeFunction[W util.RoutingNumber] struct {
	defaultWeights []W      // centisecond (if the input is openstreetmap file)
	segmentLengths []uint32 // centimeter
	edgeMaxSpeeds  []uint32 // centimeter / centisecond  (1/100 second)
	turnTable      []uint16 // centisecond // map from turnTableId  to turn cost in centiseconds
	// turnMatrices[v][i][j] -> turnCosts = flattened 1-D indexed array index of i-th incoming edge dari v and j-th outgoing edge dari v  = i*outDegree + j
	// = turnCost buat keluar dari v dari i-th incoming edge ke j-th outgoing edge dari v
	// salah satu buat support turn restrictions di openstreetmap. cara lain, osrm pakai edge-expanded graph representation: https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/understanding_osrm_graph_representation.md
	// cara support turn restrictions yang pakai turn table/compact graph representation di propose di: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
	isRoadNetwork bool
}

func NewTimeCostFunction[W util.RoutingNumber](
	roadNetwork bool,
	defaultWeights []W,
	segmentLengths, edgeMaxSpeeds []uint32,
	turnTable []uint16,
) *TimeFunction[W] {
	return &TimeFunction[W]{
		defaultWeights: defaultWeights,
		segmentLengths: segmentLengths,
		edgeMaxSpeeds:  edgeMaxSpeeds,
		turnTable:      turnTable,
		isRoadNetwork:  roadNetwork,
	}
}

func NewTimeCostFunctionEmpty[W util.RoutingNumber]() *TimeFunction[W] {
	return &TimeFunction[W]{}
}

func NewPreprocessingTimeFunction[W util.RoutingNumber](
	roadNetwork bool,
	defaultWeights []W,
	segmentLengths []uint32,
) *TimeFunction[W] {
	edgeMaxSpeeds := make([]uint32, len(defaultWeights))
	if roadNetwork {
		for i, weight := range defaultWeights {
			if weight == 0 {
				continue
			}
			edgeMaxSpeeds[i] = speedFromWeight(segmentLengths[i], weight)
		}
	}
	return NewTimeCostFunction(
		roadNetwork, defaultWeights, segmentLengths, edgeMaxSpeeds, nil,
	)
}

// speedFromWeight returns centimeters per centisecond, numerically equal to meters per second.
func speedFromWeight[W util.RoutingNumber](length uint32, weight W) uint32 {
	return uint32(math.Round(float64(length) / float64(weight)))
}

func (tf *TimeFunction[W]) GetWeight(eID da.Index) W {
	return tf.GetWeightFromLength(eID, tf.segmentLengths[eID])
}

func (tf *TimeFunction[W]) GetWeightFromLength(eID da.Index, length uint32) W {
	defaultWeight := tf.defaultWeights[eID]
	if !tf.isRoadNetwork || defaultWeight == 0 || defaultWeight >= util.Infinity[W]() {
		return defaultWeight
	}
	speed := tf.edgeMaxSpeeds[eID]
	if speed == 0 {
		return util.Infinity[W]()
	}
	weight := int32(length / speed)
	if weight >= util.Infinity[int32]() {
		return util.Infinity[W]()
	}
	return W(weight)
}

func (tf *TimeFunction[W]) GetDefaultWeight(eID da.Index) W {
	return tf.defaultWeights[eID]
}

// GetSegmentLength. get road segment lengths in centimeter
func (tf *TimeFunction[W]) GetSegmentLength(eID da.Index) uint32 {
	return tf.segmentLengths[eID]
}

func (tf *TimeFunction[W]) GetDefaultWeights() []W {
	return tf.defaultWeights
}

func (tf *TimeFunction[W]) GetSegmentLengths() []uint32 {
	return tf.segmentLengths
}

func (tf *TimeFunction[W]) GetEdgeMaxSpeeds() []uint32 {
	return tf.edgeMaxSpeeds
}

func (tf *TimeFunction[W]) ReorderEdges(oldEdgeIDs []da.Index) {
	defaultWeights := make([]W, len(oldEdgeIDs))
	segmentLengths := make([]uint32, len(oldEdgeIDs))
	edgeMaxSpeeds := make([]uint32, len(oldEdgeIDs))
	for newID, oldID := range oldEdgeIDs {
		defaultWeights[newID] = tf.defaultWeights[oldID]
		segmentLengths[newID] = tf.segmentLengths[oldID]
		edgeMaxSpeeds[newID] = tf.edgeMaxSpeeds[oldID]
	}
	tf.defaultWeights = defaultWeights
	tf.segmentLengths = segmentLengths
	tf.edgeMaxSpeeds = edgeMaxSpeeds
}

func (tf *TimeFunction[W]) WithCustomization(
	edgeMaxSpeeds []uint32,
	turnTable []uint16,
) *TimeFunction[W] {
	return NewTimeCostFunction(
		tf.isRoadNetwork, tf.defaultWeights, tf.segmentLengths, edgeMaxSpeeds, turnTable,
	)
}

func (tf *TimeFunction[W]) GetSegmentSpeed(eID da.Index) uint32 {
	if !tf.isRoadNetwork {
		return 0
	}
	return tf.edgeMaxSpeeds[eID]
}

func (tf *TimeFunction[W]) GetTurnCost(turnTableID da.Index) W {
	if !pkg.WITH_TURN_COSTS || !tf.isRoadNetwork || tf.turnTable == nil {
		return 0
	}
	value := tf.turnTable[turnTableID]
	if value == util.TurnCostForbidden {
		return util.Infinity[W]()
	}

	return W(value)
}

func (tf *TimeFunction[W]) WeightToSeconds(value W) float64 {
	if tf.isRoadNetwork {
		return float64(value) / util.CentiScale
	}
	return float64(value)
}

func (tf *TimeFunction[W]) WeightFromSeconds(value float64) W {
	return W(util.RoundCentiseconds(value))
}

func (tf *TimeFunction[W]) DistanceToMeters(value uint32) float64 {
	return float64(value) / util.CentiScale
}

func (tf *TimeFunction[W]) DistanceFromMeters(value float64) uint32 {
	return uint32(math.Round(value * util.CentiScale))
}

func (tf *TimeFunction[W]) SpeedToMetersPerSecond(value uint32) float64 {
	return float64(value)
}

// SpeedFromKilometerPerHour converts CSV speed input to centimeters per centisecond.
func (tf *TimeFunction[W]) SpeedFromKilometerPerHour(value float64) uint32 {
	speed := math.Round(value / 3.6)
	return uint32(speed)
}

func (tf *TimeFunction[W]) IsRoadNetwork() bool {
	return tf.isRoadNetwork
}

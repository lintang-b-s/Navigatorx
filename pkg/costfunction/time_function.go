// Package costfunction provides interfaces and implementations for edge and turn cost calculations.
package costfunction

import (
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

type TimeFunction struct {
	turnTable []float64 // map from turnTableId  to turn cost in seconds
	// turnMatrices[v][i][j] -> turnCosts = flattened 1-D indexed array index of i-th incoming edge dari v and j-th outgoing edge dari v  = i*outDegree + j
	// = turnCost buat keluar dari v dari i-th incoming edge ke j-th outgoing edge dari v

	defaultWeights []float64
	segmentLengths []float64
	edgeMaxSpeeds  []float64 // map from outEdge id to its maxspeed in meters per second (m/s)
	isRoadNetwork  bool
}

func NewTimeCostFunction(roadNetwork bool, defaultWeights, segmentLengths, edgeMaxSpeeds, turnTable []float64) *TimeFunction {
	tf := &TimeFunction{
		defaultWeights: defaultWeights,
		segmentLengths: segmentLengths,
		edgeMaxSpeeds:  edgeMaxSpeeds,
		isRoadNetwork:  roadNetwork,
	}
	if pkg.WITH_TURN_COSTS {
		tf.turnTable = turnTable
	}
	return tf
}

func NewTimeCostFunctionEmpty() *TimeFunction {
	return &TimeFunction{}
}

func NewPreprocessingTimeFunction(roadNetwork bool, defaultWeights, segmentLengths []float64) *TimeFunction {
	edgeMaxSpeeds := make([]float64, len(defaultWeights))
	if roadNetwork {
		for i := range edgeMaxSpeeds {
			if !util.Eq(defaultWeights[i], 0) {
				edgeMaxSpeeds[i] = segmentLengths[i] / defaultWeights[i]
			}
		}
	}
	return NewTimeCostFunction(roadNetwork, defaultWeights, segmentLengths, edgeMaxSpeeds, nil)
}

// GetWeight. Get travel time dari outEdge (in minutes).
// eId adalah id dari outEdge yang ingin didapat weightnya
func (tf *TimeFunction) GetWeight(eId da.Index) float64 {
	return tf.GetWeightFromLength(eId, tf.segmentLengths[eId])
}

func (tf *TimeFunction) GetWeightFromLength(eId da.Index, eLength float64) float64 {
	eDefaultWeight := tf.defaultWeights[eId]
	if tf.isRoadNetwork {
		maxspeed := tf.edgeMaxSpeeds[eId] // m/s
		if util.Eq(eDefaultWeight, pkg.INF_WEIGHT) {
			return eDefaultWeight
		} else if util.Eq(maxspeed, 0) {
			// blokade jalan. kasih inf travel time
			return pkg.INF_WEIGHT
		} else if eDefaultWeight == 0 {
			return 0
		}

		return eLength / maxspeed
	}
	// ini buat correctness test
	return eDefaultWeight
}

func (tf *TimeFunction) GetDefaultWeight(eId da.Index) float64 {
	return tf.defaultWeights[eId]
}

func (tf *TimeFunction) GetSegmentLength(eId da.Index) float64 {
	return tf.segmentLengths[eId]
}

func (tf *TimeFunction) GetDefaultWeights() []float64 {
	return tf.defaultWeights
}

func (tf *TimeFunction) GetSegmentLengths() []float64 {
	return tf.segmentLengths
}

func (tf *TimeFunction) GetEdgeMaxSpeeds() []float64 {
	return tf.edgeMaxSpeeds
}

func (tf *TimeFunction) ReorderEdges(oldEdgeIDs []da.Index) {
	defaultWeights := make([]float64, len(oldEdgeIDs))
	segmentLengths := make([]float64, len(oldEdgeIDs))
	edgeMaxSpeeds := make([]float64, len(oldEdgeIDs))
	for newID, oldID := range oldEdgeIDs {
		defaultWeights[newID] = tf.defaultWeights[oldID]
		segmentLengths[newID] = tf.segmentLengths[oldID]
		edgeMaxSpeeds[newID] = tf.edgeMaxSpeeds[oldID]
	}
	tf.defaultWeights = defaultWeights
	tf.segmentLengths = segmentLengths
	tf.edgeMaxSpeeds = edgeMaxSpeeds
}

func (tf *TimeFunction) WithCustomization(edgeMaxSpeeds, turnTable []float64) *TimeFunction {
	return NewTimeCostFunction(tf.isRoadNetwork, tf.defaultWeights, tf.segmentLengths, edgeMaxSpeeds, turnTable)
}

func (tf *TimeFunction) GetSegmentSpeed(eId da.Index) float64 {
	if tf.isRoadNetwork {
		maxspeed := tf.edgeMaxSpeeds[eId] // m/s
		return maxspeed
	}
	// ini buat correctness test
	return 0
}

func (tf *TimeFunction) GetTurnCost(turnTableId da.Index) float64 {
	if tf.turnTable == nil {
		return 0
	}

	return tf.turnTable[turnTableId]
}

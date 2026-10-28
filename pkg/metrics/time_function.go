package metrics

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// PrepTimeFunctionPath returns the preprocessed time function file path.
func PrepTimeFunctionPath() string {
	root := config.ProfilesRoot()
	fpath := fmt.Sprintf("%s/%s/%s_prep.ntf", root, pkg.ProfileName, pkg.RegionName)
	return fpath
}

// TimeFunction represents edge travel times and turn costs for a graph.
type TimeFunction[W util.Numeric] struct {
	weights          []W      // centisecond (if the input is openstreetmap file). weight of each graph edge indexed by its edge id.
	segmentLengths   []uint32 // centimeter. length of each road segment.
	segmentDurations []uint32 // centisecond. duration of each road segments.
	turnCosts        []uint16 // centisecond. duration of turn cost of each edge-based graph edge indexed by its edge id.
	isRoadNetwork    bool
}

// NewTimeCostFunction creates a new TimeFunction instance.
func NewTimeCostFunction[W util.Numeric](
	roadNetwork bool,
	weights []W,
	segmentLengths []uint32,
	segmentDurations []uint32,
) *TimeFunction[W] {
	return &TimeFunction[W]{
		weights:          weights,
		segmentLengths:   segmentLengths,
		isRoadNetwork:    roadNetwork,
		segmentDurations: segmentDurations,
		turnCosts:        make([]uint16, 0),
	}
}

// NewTimeCostFunctionEmpty creates an empty TimeFunction instance.
func NewTimeCostFunctionEmpty[W util.Numeric]() *TimeFunction[W] {
	return &TimeFunction[W]{}
}

// SetTurnCosts sets turn costs.
func (tf *TimeFunction[W]) SetTurnCosts(turnCosts []uint16) {
	tf.turnCosts = turnCosts
}

// ForWeights iterates over all weights and applies the given handler function.
func (tf *TimeFunction[W]) ForWeights(handle func(eId da.Index, weight W, length uint32)) {
	for eId, w := range tf.weights {
		handle(da.Index(eId), w, tf.segmentLengths[eId])
	}
}

// GetWeight returns the weight of the given edge ID.
func (tf *TimeFunction[W]) GetWeight(eId da.Index) W {
	return tf.weights[eId]
}

// Update updates segment durations and turn costs and returns an updated TimeFunction.
func (tf *TimeFunction[W]) Update(
	upEbgNodeIds, upEbgEdgeIds []da.Index, upSpLimits []uint32, upTurnEdgeIds []da.Index, upTurnSegmentIds []da.Index, upTurnPenalties []uint16,
) *TimeFunction[W] {

	// update segments duration
	for i, eId := range upEbgEdgeIds {
		segmentId := upEbgNodeIds[i]
		dur := tf.weightFromSpeed(upSpLimits[i], tf.segmentLengths[segmentId])
		tc := tf.turnCosts[eId]
		tf.weights[eId] = dur + W(tc)
		tf.segmentDurations[segmentId] = uint32(dur)
	}

	// update turn costs
	for i, eId := range upTurnEdgeIds {
		segId := upTurnSegmentIds[i]
		dur := tf.segmentDurations[segId]
		tf.weights[eId] = W(dur) + W(upTurnPenalties[i])
		tf.turnCosts[eId] = upTurnPenalties[i]
	}

	return NewTimeCostFunction(
		tf.isRoadNetwork, tf.weights, tf.segmentLengths, tf.segmentDurations,
	)
}

func (tf *TimeFunction[W]) weightFromSpeed(speed uint32, length uint32) W {
	if speed == 0 {
		return util.Infinity[W]()
	}
	return W(length / speed)
}

// GetSegmentLength returns road segment lengths in centimeters.
func (tf *TimeFunction[W]) GetSegmentLength(segId da.Index) uint32 {
	return tf.segmentLengths[segId]
}

// GetSegmentSpeed return road segment speed in meter/second
func (tf *TimeFunction[W]) GetSegmentSpeed(segId da.Index) float64 {
	dur := float64(tf.segmentDurations[segId])
	length := float64(tf.segmentLengths[segId])
	speed := length / dur
	return speed
}

// Getweights returns the edge weights slice.
func (tf *TimeFunction[W]) Getweights() []W {
	return tf.weights
}

// GetSegmentLengths returns the segment lengths slice.
func (tf *TimeFunction[W]) GetSegmentLengths() []uint32 {
	return tf.segmentLengths
}

// GetSegmentDurations returns the segment durations slice.
func (tf *TimeFunction[W]) GetSegmentDurations() []uint32 {
	return tf.segmentDurations
}

// ApplySegmentsPermutation applies edge and node permutations to weights and segment metrics.
func (tf *TimeFunction[W]) ApplySegmentsPermutation(perm, nPerm []int, isRn bool) {
	tf.weights = util.ApplyPermutation(tf.weights, perm)
	tf.segmentLengths = util.ApplyPermutation(tf.segmentLengths, nPerm)
	if isRn {
		tf.segmentDurations = util.ApplyPermutation(tf.segmentDurations, nPerm)
	}
}

// IsRoadNetwork returns whether the time function is for a road network.
func (tf *TimeFunction[W]) IsRoadNetwork() bool {
	return tf.isRoadNetwork
}

package metrics

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func PrepTimeFunctionPath() string {
	root := config.ProfilesRoot()
	fpath := fmt.Sprintf("%s/%s_%s_prep.ntf", root, pkg.ProfileName, pkg.RegionName)
	return fpath
}

type TimeFunction[W util.RoutingNumber] struct {
	weights          []W      // centisecond (if the input is openstreetmap file)
	segmentLengths   []uint32 // centimeter
	segmentDurations []uint32 // centisecond. duration of traveling road segments
	isRoadNetwork    bool
}

func NewTimeCostFunction[W util.RoutingNumber](
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
	}
}

func NewTimeCostFunctionEmpty[W util.RoutingNumber]() *TimeFunction[W] {
	return &TimeFunction[W]{}
}

func (tf *TimeFunction[W]) ForWeights(handle func(eId da.Index, weight W, length uint32)) {
	for eId, w := range tf.weights {
		handle(da.Index(eId), w, tf.segmentLengths[eId])
	}
}

func (tf *TimeFunction[W]) GetWeight(eId da.Index) W {
	return tf.weights[eId]
}

func (tf *TimeFunction[W]) Update(
	upEbgNodeIds, upEbgEdgeIds []da.Index, upSpLimits []uint32, upTurnEdgeIds []da.Index, upTurnPenalties []uint16,
) *TimeFunction[W] {

	for i, eId := range upEbgEdgeIds {
		segmentId := upEbgNodeIds[i]
		dur := tf.weightFromSpeed(upSpLimits[i], tf.segmentLengths[segmentId])

		tf.weights[eId] = dur
		tf.segmentDurations[segmentId] = uint32(dur)
	}

	for i, eId := range upTurnEdgeIds {
		tf.weights[eId] += W(upTurnPenalties[i])
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

// GetSegmentLength. get road segment lengths in centimeter
func (tf *TimeFunction[W]) GetSegmentLength(segId da.Index) uint32 {
	return tf.segmentLengths[segId]
}

func (tf *TimeFunction[W]) Getweights() []W {
	return tf.weights
}

func (tf *TimeFunction[W]) GetSegmentLengths() []uint32 {
	return tf.segmentLengths
}

func (tf *TimeFunction[W]) GetSegmentDurations() []uint32 {
	return tf.segmentDurations
}

func (tf *TimeFunction[W]) ApplySegmentsPermutation(perm, nPerm []int, isRn bool) {
	tf.weights = util.ApplyPermutation(tf.weights, perm)
	tf.segmentLengths = util.ApplyPermutation(tf.segmentLengths, nPerm)
	if isRn {
		tf.segmentDurations = util.ApplyPermutation(tf.segmentDurations, nPerm)
	}
}

func (tf *TimeFunction[W]) IsRoadNetwork() bool {
	return tf.isRoadNetwork
}

package metrics

import (
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	maxTimeFunctionItems = uint32(math.MaxInt32)
	int32Marker          = uint8(1)
	float64Marker        = uint8(2)
	int64Marker          = uint8(3)
)

// NumericMarker identifies the concrete generic number stored in an artifact.
func NumericMarker[W util.RoutingNumber]() uint8 {
	var zero W
	switch any(zero).(type) {
	case int32:
		return int32Marker
	case float64:
		return float64Marker
	case int64:
		return int64Marker
	default:
		panic("unsupported routing number")
	}
}

// WriteRoutingNumbers writes a typed numeric slice without per-element boxing.
func WriteRoutingNumbers[W util.RoutingNumber](w *util.BinaryWriter, values []W) error {
	switch typed := any(values).(type) {
	case []int32:
		return w.WriteInt32s(typed)
	case []float64:
		return w.WriteFloat64s(typed)
	case []int64:
		return w.WriteInt64s(typed)
	default:
		panic("unsupported routing number slice")
	}
}

// ReadRoutingNumbers reads a typed numeric slice without per-element boxing.
func ReadRoutingNumbers[W util.RoutingNumber](r *util.BinaryReader) ([]W, error) {
	var zero W

	var vals any
	var err error
	switch any(zero).(type) {
	case int32:
		vals, err = r.ReadInt32s()
	case int64:
		vals, err = r.ReadInt64s()
	case float64:
		vals, err = r.ReadFloat64s()
	default:
		return nil, fmt.Errorf("unsupported routing number type %T", zero)
	}
	if err != nil {
		return nil, err
	}

	return vals.([]W), nil
}

func (tf *TimeFunction[W]) WriteToFile(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {

		if err := w.Uint8(NumericMarker[W]()); err != nil {
			return err
		}
		if err := w.Bool(tf.isRoadNetwork); err != nil {
			return err
		}
		if err := WriteRoutingNumbers[W](w, tf.weights); err != nil {
			return err
		}
		if err := w.WriteUint32s(tf.segmentLengths); err != nil {
			return err
		}
		if err := w.WriteUint32s(tf.segmentDurations); err != nil {
			return err
		}
		return nil
	})
}

func ReadCostFunctionFromFile[W util.RoutingNumber](
	filename string,
) (*TimeFunction[W], error) {
	file, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	marker, err := r.Uint8()
	if err != nil {
		return nil, err
	}
	expectedMarker := NumericMarker[W]()
	if marker != expectedMarker {
		return nil, fmt.Errorf(
			"time-function numeric representation %d does not match expected %d",
			marker, expectedMarker,
		)
	}
	roadNetwork, err := r.Bool()
	if err != nil {
		return nil, fmt.Errorf("read time-function road-network flag: %w", err)
	}
	weights, err := ReadRoutingNumbers[W](r)
	if err != nil {
		return nil, fmt.Errorf("read time-function default weights: %w", err)
	}
	segmentLengths, err := r.ReadUint32s()
	if err != nil {
		return nil, err
	}
	segmentDurations, err := r.ReadUint32s()
	if err != nil {
		return nil, err
	}
	return NewTimeCostFunction(
		roadNetwork, weights, segmentLengths, segmentDurations,
	), nil
}

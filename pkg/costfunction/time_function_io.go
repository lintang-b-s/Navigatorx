package costfunction

import (
	"bufio"
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	maxTimeFunctionItems = uint32(math.MaxInt32)
	int32Marker          = uint8(1)
	float64Marker        = uint8(2)
)

// NumericMarker identifies the concrete generic number stored in an artifact.
func NumericMarker[W util.RoutingNumber]() uint8 {
	var zero W
	switch any(zero).(type) {
	case int32:
		return int32Marker
	case float64:
		return float64Marker
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
	default:
		panic("unsupported routing number slice")
	}
}

// ReadRoutingNumbers reads a typed numeric slice without per-element boxing.
func ReadRoutingNumbers[W util.RoutingNumber](r *util.BinaryReader, values []W) error {
	switch typed := any(values).(type) {
	case []int32:
		return r.ReadInt32s(typed)
	case []float64:
		return r.ReadFloat64s(typed)
	default:
		panic("unsupported routing number slice")
	}
}

func writeOptional[T any](
	w *util.BinaryWriter,
	values []T,
	write func(*util.BinaryWriter, []T) error,
) error {
	if values == nil {
		return w.Bool(false)
	}
	if err := w.Bool(true); err != nil {
		return err
	}
	return write(w, values)
}

func readOptional[T any](
	r *util.BinaryReader,
	read func(*util.BinaryReader, []T) error,
) ([]T, error) {
	present, err := r.Bool()
	if err != nil || !present {
		return nil, err
	}
	length, err := r.Length(maxTimeFunctionItems)
	if err != nil {
		return nil, err
	}
	values := make([]T, length)
	if err := read(r, values); err != nil {
		return nil, err
	}
	return values, nil
}

func (tf *TimeFunction[W]) WriteToFile(filename string) error {
	return tf.writeToFile(filename)
}

func (tf *TimeFunction[W]) WritePreprocessingToFile(filename string) error {
	return tf.writeToFile(filename)
}

func (tf *TimeFunction[W]) writeToFile(filename string) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {

		if err := w.Uint8(NumericMarker[W]()); err != nil {
			return err
		}
		if err := w.Bool(tf.isRoadNetwork); err != nil {
			return err
		}
		if err := writeOptional(w, tf.defaultWeights, WriteRoutingNumbers[W]); err != nil {
			return err
		}
		if err := writeOptional(w, tf.segmentLengths, func(w *util.BinaryWriter, values []uint32) error {
			return w.WriteUint32s(values)
		}); err != nil {
			return err
		}
		if err := writeOptional(w, tf.edgeMaxSpeeds, func(w *util.BinaryWriter, values []uint32) error {
			return w.WriteUint32s(values)
		}); err != nil {
			return err
		}
		return writeOptional(w, tf.turnTable, func(w *util.BinaryWriter, values []uint16) error {
			return w.WriteUint16s(values)
		})
	})
}

func ReadFromFile[W util.RoutingNumber](
	filename string,
	_ *bufio.Reader,
) (*TimeFunction[W], error) {
	return readFromFile[W](filename)
}

func ReadPreprocessingFromFile[W util.RoutingNumber](filename string) (*TimeFunction[W], error) {
	return readFromFile[W](filename)
}

func readFromFile[W util.RoutingNumber](filename string) (*TimeFunction[W], error) {
	file, r, err := util.OpenCompressedArtifact(filename)
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
	defaultWeights, err := readOptional(r, ReadRoutingNumbers[W])
	if err != nil {
		return nil, fmt.Errorf("read time-function default weights: %w", err)
	}
	segmentLengths, err := readOptional(r, func(r *util.BinaryReader, values []uint32) error {
		return r.ReadUint32s(values)
	})
	if err != nil {
		return nil, fmt.Errorf("read time-function segment lengths: %w", err)
	}
	edgeMaxSpeeds, err := readOptional(r, func(r *util.BinaryReader, values []uint32) error {
		return r.ReadUint32s(values)
	})
	if err != nil {
		return nil, fmt.Errorf("read time-function edge speeds: %w", err)
	}
	turnTable, err := readOptional(r, func(r *util.BinaryReader, values []uint16) error {
		return r.ReadUint16s(values)
	})
	if err != nil {
		return nil, fmt.Errorf("read time-function turn costs: %w", err)
	}
	if len(defaultWeights) != len(segmentLengths) ||
		(len(edgeMaxSpeeds) != 0 && len(defaultWeights) != len(edgeMaxSpeeds)) {
		return nil, fmt.Errorf("time-function edge array lengths do not match")
	}
	return NewTimeCostFunction(
		roadNetwork, defaultWeights, segmentLengths, edgeMaxSpeeds, turnTable,
	), nil
}

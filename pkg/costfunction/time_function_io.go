package costfunction

import (
	"bufio"
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const maxTimeFunctionItems = uint32(math.MaxInt32)

func writeOptionalFloat64s(w *util.BinaryWriter, values []float64) error {
	if values == nil {
		return w.Bool(false)
	}
	if err := w.Bool(true); err != nil {
		return err
	}
	return w.WriteFloat64s(values)
}

func readOptionalFloat64s(r *util.BinaryReader) ([]float64, error) {
	present, err := r.Bool()
	if err != nil || !present {
		return nil, err
	}
	length, err := r.Length(maxTimeFunctionItems)
	if err != nil {
		return nil, err
	}
	values := make([]float64, length)
	if err := r.ReadFloat64s(values); err != nil {
		return nil, err
	}
	return values, nil
}

func (tf *TimeFunction) WriteToFile(filename string) error {
	return tf.writeToFile(filename)
}

func (tf *TimeFunction) WritePreprocessingToFile(filename string) error {
	return tf.writeToFile(filename)
}

func (tf *TimeFunction) writeToFile(filename string) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {
		if err := w.Bool(tf.isRoadNetwork); err != nil {
			return err
		}
		if err := writeOptionalFloat64s(w, tf.defaultWeights); err != nil {
			return err
		}
		if err := writeOptionalFloat64s(w, tf.segmentLengths); err != nil {
			return err
		}
		if err := writeOptionalFloat64s(w, tf.edgeMaxSpeeds); err != nil {
			return err
		}
		return writeOptionalFloat64s(w, tf.turnTable)
	})
}

func ReadFromFile(filename string, _ *bufio.Reader) (*TimeFunction, error) {
	return readFromFile(filename)
}

func ReadPreprocessingFromFile(filename string) (*TimeFunction, error) {
	return readFromFile(filename)
}

func readFromFile(filename string) (*TimeFunction, error) {

	file, r, err := util.OpenCompressedArtifact(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	roadNetwork, err := r.Bool()
	if err != nil {
		return nil, fmt.Errorf("read time-function road-network flag: %w", err)
	}

	defaultWeights, err := readOptionalFloat64s(r)
	if err != nil {
		return nil, fmt.Errorf("read time-function default weights: %w", err)
	}

	segmentLengths, err := readOptionalFloat64s(r)
	if err != nil {
		return nil, fmt.Errorf("read time-function segment lengths: %w", err)
	}
	edgeMaxSpeeds, err := readOptionalFloat64s(r)
	if err != nil {
		return nil, fmt.Errorf("read time-function edge speeds: %w", err)
	}

	turnTable, err := readOptionalFloat64s(r)
	if err != nil {
		return nil, fmt.Errorf("read time-function turn costs: %w", err)
	}
	if len(defaultWeights) != len(segmentLengths) || (len(edgeMaxSpeeds) != 0 && len(defaultWeights) != len(edgeMaxSpeeds)) {
		return nil, fmt.Errorf("time-function edge array lengths do not match")
	}
	return &TimeFunction{
		turnTable:      turnTable,
		defaultWeights: defaultWeights,
		segmentLengths: segmentLengths,
		edgeMaxSpeeds:  edgeMaxSpeeds,
		isRoadNetwork:  roadNetwork,
	}, nil
}

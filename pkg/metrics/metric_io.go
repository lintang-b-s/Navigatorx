package metrics

import (
	"bufio"
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const maxMetricItems = uint32(math.MaxInt32)

func readFloat64s(r *util.BinaryReader) ([]float64, error) {
	length, err := r.Length(maxMetricItems)
	if err != nil {
		return nil, err
	}
	values := make([]float64, length)
	if err := r.ReadFloat64s(values); err != nil {
		return nil, err
	}
	return values, nil
}

func writeFloat64Table(w *util.BinaryWriter, table *flatFloat64Table) error {
	rowCount := len(table.offsets) - 1
	if err := w.Length(rowCount); err != nil {
		return err
	}
	if err := w.Length(len(table.values)); err != nil {
		return err
	}
	for _, offset := range table.offsets {
		if err := w.Uint32(offset); err != nil {
			return err
		}
	}
	for _, value := range table.values {
		if err := w.Float64(value); err != nil {
			return err
		}
	}
	return nil
}

func readFloat64Table(r *util.BinaryReader) (*flatFloat64Table, error) {
	rowCount, err := r.Length(maxMetricItems)
	if err != nil {
		return nil, err
	}
	valueCount, err := r.Length(maxMetricItems)
	if err != nil {
		return nil, err
	}
	table := &flatFloat64Table{
		values:  make([]float64, valueCount),
		offsets: make([]uint32, rowCount+1),
	}
	for i := range table.offsets {
		offset, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		if offset > valueCount {
			return nil, fmt.Errorf("stalling offset %d exceeds value count %d", offset, valueCount)
		}
		if i == 0 && offset != 0 {
			return nil, fmt.Errorf("first stalling offset is %d, expected 0", offset)
		}
		if i > 0 && offset < table.offsets[i-1] {
			return nil, fmt.Errorf("stalling offsets are not monotonic at index %d", i)
		}
		table.offsets[i] = offset
	}
	if table.offsets[len(table.offsets)-1] != valueCount {
		return nil, fmt.Errorf("final stalling offset is %d, expected %d", table.offsets[len(table.offsets)-1], valueCount)
	}
	if err := r.ReadFloat64s(table.values); err != nil {
		return nil, err
	}
	return table, nil
}

func (met *Metric) WriteToFile(filename string) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {
		if err := w.WriteFloat64s(met.weights.Load().GetWeights()); err != nil {
			return err
		}
		if err := writeFloat64Table(w, met.entryStallingTables.Load().(*flatFloat64Table)); err != nil {
			return err
		}
		return writeFloat64Table(w, met.exitStallingTables.Load().(*flatFloat64Table))
	})
}

func ReadFromFile(filename, timeFunctionFilePath string, readBuf *bufio.Reader) (*Metric, error) {
	file, r, err := util.OpenCompressedArtifact(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	weights, err := readFloat64s(r)
	if err != nil {
		return nil, fmt.Errorf("read metric weights: %w", err)
	}

	entryTables, err := readFloat64Table(r)
	if err != nil {
		return nil, fmt.Errorf("read entry stalling tables: %w", err)
	}
	exitTables, err := readFloat64Table(r)
	if err != nil {
		return nil, fmt.Errorf("read exit stalling tables: %w", err)
	}

	if len(entryTables.offsets) != len(exitTables.offsets) {
		return nil, fmt.Errorf("entry and exit stalling table counts differ")
	}

	timeFunction, err := costfunction.ReadFromFile(timeFunctionFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("read time function: %w", err)
	}
	overlayWeights := da.NewOverlayWeights(uint32(len(weights)))
	overlayWeights.SetWeights(weights)
	metric := &Metric{
		metricFilepath:       filename,
		timeFunctionFilePath: timeFunctionFilePath,
	}
	metric.weights.Store(overlayWeights)
	metric.entryStallingTables.Store(entryTables)
	metric.exitStallingTables.Store(exitTables)
	metric.costFunction.Store(timeFunction)
	metric.lastSegmentSpeedFiles.Store([]string{})
	metric.lastTurnPenaltyFiles.Store([]string{})
	return metric, nil
}

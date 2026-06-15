package metrics

import (
	"bufio"
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	maxMetricItems = uint32(math.MaxInt32)
	metricMagic    = uint32(0x32544d4e)
)

func readValues[T any](r *util.BinaryReader, read func(*util.BinaryReader, []T) error) ([]T, error) {
	length, err := r.Length(maxMetricItems)
	if err != nil {
		return nil, err
	}
	values := make([]T, length)
	if err := read(r, values); err != nil {
		return nil, err
	}
	return values, nil
}

func writeTable[W util.RoutingNumber](
	w *util.BinaryWriter,
	table *flatTable[W],
	write func(*util.BinaryWriter, []W) error,
) error {
	if err := w.Length(len(table.offsets) - 1); err != nil {
		return err
	}
	for _, offset := range table.offsets {
		if err := w.Uint32(offset); err != nil {
			return err
		}
	}
	return write(w, table.values)
}

func readTable[W util.RoutingNumber](
	r *util.BinaryReader,
	read func(*util.BinaryReader, []W) error,
) (*flatTable[W], error) {
	rowCount, err := r.Length(maxMetricItems)
	if err != nil {
		return nil, err
	}
	table := &flatTable[W]{offsets: make([]uint32, rowCount+1)}
	for i := range table.offsets {
		offset, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		if i == 0 && offset != 0 {
			return nil, fmt.Errorf("first stalling offset is %d, expected 0", offset)
		}
		if i > 0 && offset < table.offsets[i-1] {
			return nil, fmt.Errorf("stalling offsets are not monotonic at index %d", i)
		}
		table.offsets[i] = offset
	}
	table.values, err = readValues(r, read)
	if err != nil {
		return nil, err
	}
	if table.offsets[len(table.offsets)-1] != uint32(len(table.values)) {
		return nil, fmt.Errorf("final stalling offset is %d, expected %d", table.offsets[len(table.offsets)-1], len(table.values))
	}
	return table, nil
}

func (met *Metric[W]) WriteToFile(filename string) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {
		if err := w.Uint32(metricMagic); err != nil {
			return err
		}
		if err := w.Uint8(costfunction.NumericMarker[W]()); err != nil {
			return err
		}
		if err := costfunction.WriteRoutingNumbers(w, met.weights.Load().GetWeights()); err != nil {
			return err
		}
		if err := writeTable(w, met.entryStallingTables.Load(), costfunction.WriteRoutingNumbers[W]); err != nil {
			return err
		}
		return writeTable(w, met.exitStallingTables.Load(), costfunction.WriteRoutingNumbers[W])
	})
}

func ReadFromFile[W util.RoutingNumber](
	filename, timeFunctionFilePath string,
	readBuf *bufio.Reader,
) (*Metric[W], error) {
	file, r, err := util.OpenCompressedArtifact(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	magic, err := r.Uint32()
	if err != nil || magic != metricMagic {
		return nil, fmt.Errorf("unsupported legacy metric artifact; regenerate it with cmd/customizer")
	}
	marker, err := r.Uint8()
	if err != nil {
		return nil, err
	}
	expectedMarker := costfunction.NumericMarker[W]()
	if marker != expectedMarker {
		return nil, fmt.Errorf("metric numeric representation %d does not match expected %d", marker, expectedMarker)
	}
	weights, err := readValues(r, costfunction.ReadRoutingNumbers[W])
	if err != nil {
		return nil, fmt.Errorf("read metric weights: %w", err)
	}
	entryTables, err := readTable(r, costfunction.ReadRoutingNumbers[W])
	if err != nil {
		return nil, fmt.Errorf("read entry stalling tables: %w", err)
	}
	exitTables, err := readTable(r, costfunction.ReadRoutingNumbers[W])
	if err != nil {
		return nil, fmt.Errorf("read exit stalling tables: %w", err)
	}
	if len(entryTables.offsets) != len(exitTables.offsets) {
		return nil, fmt.Errorf("entry and exit stalling table counts differ")
	}
	timeFunction, err := costfunction.ReadFromFile[W](timeFunctionFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("read time function: %w", err)
	}
	overlayWeights := da.NewOverlayWeights[W](uint32(len(weights)))
	overlayWeights.SetWeights(weights)
	metric := &Metric[W]{
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

package metrics

import (
	"fmt"
	"sync"
	"sync/atomic"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (met *Metric[W]) WriteToFile(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {
		if err := w.Uint8(NumericMarker[W]()); err != nil {
			return err
		}
		weights := met.weights.Load()
		return WriteRoutingNumbers(w, weights.GetWeights())
	})
}

func ReadFromFile[W util.RoutingNumber](
	filename, timeFunctionFilePath string,
) (*Metric[W], error) {
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
		return nil, fmt.Errorf("metric numeric representation %d does not match expected %d", marker, expectedMarker)
	}
	weights, err := ReadRoutingNumbers[W](r)
	if err != nil {
		return nil, fmt.Errorf("read metric weights: %w", err)
	}

	// cost function
	timeFunction, err := ReadCostFunctionFromFile[W](timeFunctionFilePath)
	if err != nil {
		return nil, fmt.Errorf("read time function: %w", err)
	}

	overlayWeights := da.NewOverlayWeights[W](uint32(len(weights)))
	overlayWeights.SetWeights(weights)
	cl := &atomic.Bool{}
	cl.Store(false)
	metric := &Metric[W]{
		metricFilepath:       filename,
		timeFunctionFilePath: timeFunctionFilePath,
		mu:                   sync.Mutex{},
	}
	metric.weights.Store(overlayWeights)
	metric.costFunction.Store(timeFunction)
	metric.lastSegmentSpeedFiles.Store(&[]string{})
	metric.lastTurnPenaltyFiles.Store(&[]string{})
	return metric, nil
}

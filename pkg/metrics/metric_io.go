package metrics

import (
	"fmt"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// WriteToFile writes the metric shortcut weights to a compressed binary file.
func (met *Metric[W]) WriteToFile(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {
		nm, err := NumericMarker[W]()
		if err != nil {
			return err
		}
		if err := w.Uint8(nm); err != nil {
			return err
		}
		weights := met.shortcutWeights
		return WriteNumerics(w, weights.GetWeights())
	})
}

// ReadFromFile reads a Metric instance from a compressed binary file.
func ReadFromFile[W util.Numeric](
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
	expectedMarker, err := NumericMarker[W]()
	if err != nil {
		return nil, err
	}
	if marker != expectedMarker {
		return nil, fmt.Errorf("metric numeric representation %d does not match expected %d", marker, expectedMarker)
	}
	weights, err := ReadNumerics[W](r)
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

	metric := &Metric[W]{
		metricFilepath:        filename,
		timeFunctionFilePath:  timeFunctionFilePath,
		shortcutWeights:       overlayWeights,
		costFunction:          timeFunction,
		lastSegmentSpeedFiles: make([]string, 0),
		lastTurnPenaltyFiles:  make([]string, 0),
	}

	return metric, nil
}

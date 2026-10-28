package datastructure

import (
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// OverlayWeights stores shortcut weights in the overlay graph.
/*
for each cell C, we have:
qC = number of exit points (vertex that have at least one outgoing edge that point to vertex in other cell)
pC = number of entry points (vertex that have at least one incoming edge that point to vertex in other cell)
fC = poisition in OverlayWeights.weights where the first entry of cell C is represented
shortcut between i-th entry point and the j-th exit point of cell C will be stored in W [fC + iqC + j].
*/
type OverlayWeights[W util.Numeric] struct {
	weights []W
}

// GetWeight returns shortcut edge weight.
func (ow *OverlayWeights[W]) GetWeight(id Index) W {
	shortcutWeight := ow.weights[id]
	return shortcutWeight
}

// GetWeights returns weights.
func (ow *OverlayWeights[W]) GetWeights() []W {
	return ow.weights
}

// SetWeights sets weights.
func (ow *OverlayWeights[W]) SetWeights(weights []W) {
	copy(ow.weights, weights)
}

// GetNumberOfShortcuts returns number of shortcuts.
func (ow *OverlayWeights[W]) GetNumberOfShortcuts() int {
	return len(ow.weights)
}

// SetWeight sets weight.
func (ow *OverlayWeights[W]) SetWeight(index int, weight W) {
	ow.weights[index] = weight
}

// NewOverlayWeights creates and returns overlay weights.
func NewOverlayWeights[W util.Numeric](weightVectorSize uint32) *OverlayWeights[W] {
	weights := make([]W, weightVectorSize)
	for i := range weights {
		weights[i] = util.Infinity[W]()
	}
	return &OverlayWeights[W]{weights: weights}
}

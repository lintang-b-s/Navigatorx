package datastructure

import (
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// OverlayWeights stores shortcut weights in the overlay graph.
/*
for each cell C, we have:
qC = number of exit points (vertex that have at least one out edge that point to vertex in other cell)
pC = number of entry points (vertex that have at least one in edge that point to vertex in other cell)
fC = poisition in OverlayWeights.weights where the first entry of C is represented
shortcut betweeb i-th entry point and the j-th exit point of C will be stored in W [fC + iqC + j].
*/
type OverlayWeights[W util.RoutingNumber] struct {
	weights []W
}

func (ow *OverlayWeights[W]) GetWeight(i Index) W {
	shortcutWeight := ow.weights[i]
	return shortcutWeight
}

func (ow *OverlayWeights[W]) GetWeights() []W {
	return ow.weights
}

func (ow *OverlayWeights[W]) SetWeights(weights []W) {
	copy(ow.weights, weights)
}

func (ow *OverlayWeights[W]) GetNumberOfShortcuts() int {
	return len(ow.weights)
}

func (ow *OverlayWeights[W]) SetWeight(index int, weight W) {
	ow.weights[index] = weight
}

func NewOverlayWeights[W util.RoutingNumber](weightVectorSize uint32) *OverlayWeights[W] {
	weights := make([]W, weightVectorSize)
	for i := range weights {
		weights[i] = util.Infinity[W]()
	}
	return &OverlayWeights[W]{weights: weights}
}

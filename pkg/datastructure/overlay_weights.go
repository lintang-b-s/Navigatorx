package datastructure

import (
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

/*
we store shortcut weights in OverlayWeights.weights
for each cell C, we have:
qC = number of exit points (vertex that have at least one out edge that point to vertex in other cell)
pC = number of entry points (vertex that have at least one in edge that point to vertex in other cell)
fC = poisition in OverlayWeights.weights where the first entry of C is represented
shortcut betweeb i-th entry point and the j-th exit point of C will be stored in W [fC + iqC + j].
*/
type OverlayWeights struct {
	weights []float64
	lock    *sync.RWMutex
}

func (ow *OverlayWeights) GetWeight(i Index) float64 {
	shortcutWeight := ow.weights[i]
	return shortcutWeight
}

func (ow *OverlayWeights) GetWeights() []float64 {
	return ow.weights
}

func (ow *OverlayWeights) SetWeights(weights []float64) {

	copy(ow.weights, weights)
}

func (ow *OverlayWeights) GetNumberOfShortcuts() int {
	return len(ow.weights)
}

func (ow *OverlayWeights) SetWeight(index int, weight float64) {
	ow.lock.Lock()
	ow.weights[index] = weight
	ow.lock.Unlock()
}

func (ow *OverlayWeights) Lock() {
	ow.lock.Lock()
}

func (ow *OverlayWeights) Unlock() {
	ow.lock.Unlock()
}

func NewOverlayWeights(weightVectorSize uint32) *OverlayWeights {
	weights := make([]float64, weightVectorSize)
	for i := range weights {
		weights[i] = pkg.INF_WEIGHT
	}
	return &OverlayWeights{weights: weights, lock: &sync.RWMutex{}}
}

package datastructure

import (
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

/*
Customizable Route Planning In Road Networks, Delling et al., Page 11:
First, for each cell C in the overlay graph, we keep three integers: pC (the number of entry points), qC
(the number of exit points), and fC (the position in W where the first entry of C’s matrix is represented).
During customization and queries, the cost of the shortcut between the i-th entry point and the j-th exit
point of C will be stored in W [fC + iqC + j].
*/
type OverlayWeights struct {
	weights []float64
	lock    *sync.RWMutex
}

func (ow *OverlayWeights) GetWeight(i Index) float64 {
	ow.lock.RLock()
	shortcutWeight := ow.weights[i]
	ow.lock.RUnlock()
	return shortcutWeight
}

func (ow *OverlayWeights) GetWeights() []float64 {
	return ow.weights
}

func (ow *OverlayWeights) SetWeights(weights []float64) {

	copy(ow.weights, weights)
}

func (ow *OverlayWeights) SetWeight(index int, weight float64) {
	ow.weights[index] = weight
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

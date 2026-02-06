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
type OverlayWeightsTD struct {
	weights []*PWL
	lock    *sync.RWMutex
}

func (ow *OverlayWeightsTD) GetWeightAtTime(i Index, time float64) float64 {
	ow.lock.RLock()
	shortcutWeight := ow.weights[i].Eval(time)  // in seconds
	ow.lock.RUnlock()
	return shortcutWeight / 60.0
}

func (ow *OverlayWeightsTD) GetWeightPWL(i Index) *PWL {
	ow.lock.RLock()
	shortcutWeight := ow.weights[i]
	shortcutWeight.CheckIsFIFO()
	ow.lock.RUnlock()
	return shortcutWeight
}

func (ow *OverlayWeightsTD) GetWeights() []*PWL {
	return ow.weights
}

func (ow *OverlayWeightsTD) SetWeights(weights []*PWL) {
	copy(ow.weights, weights)
}

func (ow *OverlayWeightsTD) SetWeight(index int, weight *PWL) {
	ow.lock.Lock()
	ow.weights[index] = weight
	ow.lock.Unlock()
}

func (ow *OverlayWeightsTD) Lock() {
	ow.lock.Lock()
}

func (ow *OverlayWeightsTD) Unlock() {
	ow.lock.Unlock()
}

func NewOverlayWeightsTD(weightVectorSize uint32) *OverlayWeightsTD {
	weights := make([]*PWL, weightVectorSize)
	for i := range weights {
		ps := make([]*Point, 1)
		ps[0] = NewPoint(0, pkg.INF_WEIGHT)
		weights[i] = NewPWL(ps)
	}
	return &OverlayWeightsTD{weights: weights, lock: &sync.RWMutex{}}
}

func NewEmptyOverlayWeightsTD() *OverlayWeightsTD {
	weights := make([]*PWL, 0)

	return &OverlayWeightsTD{weights: weights, lock: &sync.RWMutex{}}
}

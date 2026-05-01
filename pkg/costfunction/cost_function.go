// Package costfunction provides interfaces and implementations for edge and turn cost calculations.
package costfunction

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type CostFunction interface {
	GetWeight(eId da.Index, eDefaultWeight, eLength float64) float64

	GetTurnCost(turnTableId da.Index) float64
}

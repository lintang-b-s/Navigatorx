package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type CostFunction interface {
	GetWeight(eId da.Index, eDefaultWeight, eLength float64) float64

	GetTurnCost(turnType pkg.TurnType) float64
}

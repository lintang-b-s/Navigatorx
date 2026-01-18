package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type TimeDependentFunction struct {
	osmWaySpeed map[int64]*da.PWL
	graph       *da.Graph
}

func NewTimeDependentCostFunction(graph *da.Graph) *TimeDependentFunction {
	return &TimeDependentFunction{
		osmWaySpeed: make(map[int64]*da.PWL),
		graph:       graph,
	}
}

func (tf *TimeDependentFunction) GetWeight(e EdgeAttributes) float64 {
	return 0
}

// harusnya ambil dari csv (TODO)
func (tf *TimeDependentFunction) GetWeightPWL(e EdgeAttributes) *da.PWL {

	eOsmWayId := tf.graph.GetOsmWayId(e.GetEdgeId())
	pwl, ok := tf.osmWaySpeed[eOsmWayId]
	if !ok {
		panic("PWL for eOsmWayId not found")
	}
	return pwl
}

func (tf *TimeDependentFunction) GetWeightAtTime(e EdgeAttributes, time float64) float64 {

	eOsmWayId := tf.graph.GetOsmWayId(e.GetEdgeId())

	pwl, ok := tf.osmWaySpeed[eOsmWayId]
	if !ok {
		panic("PWL for eOsmWayId not found")
	}
	return pwl.Eval(time)
}

func (tf *TimeDependentFunction) GetTurnCost(turnType pkg.TurnType) float64 {
	switch turnType {
	case pkg.U_TURN:
		return pkg.INF_WEIGHT
	case pkg.NO_ENTRY:
		return pkg.INF_WEIGHT
	default:
		return 0
	}

}

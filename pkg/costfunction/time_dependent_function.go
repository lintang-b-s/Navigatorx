package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type TimeDependentFunction struct {
	osmWaySpeed map[int64]*da.PWL
	graph       *da.Graph
}

func NewTimeDependentCostFunction(graph *da.Graph, osmWaySpeed map[int64]*da.PWL) *TimeDependentFunction {
	tdf := &TimeDependentFunction{
		osmWaySpeed: osmWaySpeed,
		graph:       graph,
	}

	return tdf
}

func (tf *TimeDependentFunction) GetWeight(e EdgeAttributes) float64 {
	speed := e.GetEdgeSpeed()
	if speed == 0 {
		return e.GetLength() / (defaultSpeed * 1000 / 60)
	}
	return e.GetLength() / speed
}

func (tf *TimeDependentFunction) GetWeightPWL(e EdgeAttributes) *da.PWL {

	eOsmWayId := tf.graph.GetOsmWayId(e.GetEdgeId())
	osmSpeedPwl, ok := tf.osmWaySpeed[eOsmWayId]
	if !ok {
		defTravelTime := tf.GetWeight(e)
		ps := make([]*da.Point, 1)
		ps[0] = da.NewPoint(0, defTravelTime)
		constPWL := da.NewPWL(ps)
		return constPWL
	}

	ttfPs := make([]*da.Point, osmSpeedPwl.Size())
	for i, sp := range osmSpeedPwl.GetPoints() {
		// O(n), n = jumlah breakpoints osmSpeedPwl
		eLength := e.GetLength() // m
		eTravelTime := eLength / (sp.GetY() * 1000 / 60)
		ttfPs[i] = da.NewPoint(sp.GetX(), eTravelTime)
	}

	return da.NewPWL(ttfPs)
}

func (tf *TimeDependentFunction) GetWeightAtTime(e EdgeAttributes, time float64) float64 {

	eOsmWayId := tf.graph.GetOsmWayId(e.GetEdgeId())

	pwl, ok := tf.osmWaySpeed[eOsmWayId]
	if !ok {
		defTravelTime := tf.GetWeight(e)
		return defTravelTime
	}
	speed := pwl.Eval(time)

	eLength := e.GetLength() // m
	eTravelTime := eLength / (speed * 1000 / 60)
	return eTravelTime
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

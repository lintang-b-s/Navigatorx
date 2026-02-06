package costfunction

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type TimeDependentFunction struct {
	osmWayTTF map[int64]*da.PWL
	graph     *da.Graph
}

func NewTimeDependentCostFunction(graph *da.Graph, osmWayTTF map[int64]*da.PWL) *TimeDependentFunction {
	tdf := &TimeDependentFunction{
		osmWayTTF: osmWayTTF,
		graph:     graph,
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
	ttfPWL, ok := tf.osmWayTTF[eOsmWayId]
	if !ok {
		defTravelTime := tf.GetWeight(e)
		ps := make([]*da.Point, 1)
		ps[0] = da.NewPoint(0, defTravelTime)
		constPWL := da.NewPWL(ps)
		return constPWL
	}

	if ttfPWL.Size() > 2 {
		return da.ImaiIriApprox(ttfPWL, pkg.EPSILON_IMAI_IRI_APPROX_PWL)
	}

	return ttfPWL
}

func (tf *TimeDependentFunction) GetWeightAtTime(e EdgeAttributes, time float64) float64 {

	eOsmWayId := tf.graph.GetOsmWayId(e.GetEdgeId())

	pwl, ok := tf.osmWayTTF[eOsmWayId]
	if !ok {
		defTravelTime := tf.GetWeight(e)
		return defTravelTime
	}
	travelTimeSeconds := pwl.Eval(time) // in seconds
	return travelTimeSeconds / 60.0
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

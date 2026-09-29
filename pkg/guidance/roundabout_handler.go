package guidance

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

func (db *DirectionBuilder) handleRoundabout(prevPoint, tail, head da.Coordinate, streetName string, u da.Index) {
	if !db.prevInRoundabout {
		sign := da.USE_ROUNDABOUT
		point := da.NewCoordinate(tail.GetLat(), tail.GetLon())
		roundaboutInstruction := da.NewRoundaboutInstruction()

		db.doublePrevInitialBearing = db.prevInitialBearing
		if db.prevInstruction >= 0 {
			db.prevInitialBearing = geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())
		} else {
			// start point dari shortetest path & dan bundaran (roundabout)
			db.prevInitialBearing = geo.ComputeInitialBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
		}

		turnBearing := geo.ComputeFinalBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(), tail.GetLon())
		ann := db.annotation()
		prevIns := da.NewInstructionWithRoundabout(sign, streetName, point, true, roundaboutInstruction, db.cumulativeDistance,
			db.cumulativeCost, db.segmentIds, ann, turnBearing)
		db.instructions = append(db.instructions, prevIns)
		db.prevInstruction = len(db.instructions) - 1

		// reset edgeIDs and points
		db.reset()
	}

	db.graph.ForOutEdgesOf(u, func(_, v da.Index, _ da.Index) {

		eIsRoundabout := db.rn.IsRoundabout(v)
		if !eIsRoundabout {
			db.instructions[db.prevInstruction].IncrementExitNumber()
		}
	})
}

package guidance

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
checkUTurn. check if current edge is U-turn. Example:

A --doublePrevEdge-->B
				    |
					|
				PrevEdge
					|
					|
					|
D <--currentEdge---C

If from A->B turn right, and from B->C turn right, and the relative bearing between A->B and C->D is close to 180 degrees, then it can be considered a U-turn.
*/ // nolint: gofmt
func (db *DirectionBuilder) checkUTurn(sign da.TurnType, name string, edgeId da.Index) (bool, da.TurnType) {
	isUTurn := false
	uTurnType := da.U_TURN_UNKNOWN

	head := db.graph.GetHeadOfOutEdge(edgeId)

	if db.doublePrevInitialBearing != 0 && (sign > 0) == (db.prevInstruction.GetTurnSign() > 0) &&
		((db.lefthand && (sign == da.TURN_SLIGHT_RIGHT || sign == da.TURN_RIGHT || sign == da.TURN_SHARP_RIGHT)) ||
			(!db.lefthand && (sign == da.TURN_SLIGHT_LEFT || sign == da.TURN_LEFT || sign == da.TURN_SHARP_LEFT))) &&
		((db.lefthand && db.prevInstruction.GetTurnSign() == da.TURN_SLIGHT_RIGHT || db.prevInstruction.GetTurnSign() == da.TURN_RIGHT || db.prevInstruction.GetTurnSign() == da.TURN_SHARP_RIGHT) ||
			(!db.lefthand && db.prevInstruction.GetTurnSign() == da.TURN_SLIGHT_LEFT || db.prevInstruction.GetTurnSign() == da.TURN_LEFT || db.prevInstruction.GetTurnSign() == da.TURN_SHARP_LEFT)) &&
		isSamePrimaryName(name, db.doublePrevStreetName) {
		head := db.graph.GetVertex(head)
		headLat, headLon := head.GetLat(), head.GetLon()
		tail := db.graph.GetVertex(db.graph.GetTailOfOutedge(edgeId))
		currentInitialBearing := geo.ComputeInitialBearing(tail.GetLat(), tail.GetLon(), headLat, headLon)
		relativeBearing := math.Abs(db.doublePrevInitialBearing - currentInitialBearing)
		relativeBearingDeg := util.RadiansToDegree(relativeBearing)
		if relativeBearingDeg > U_TURN_RELATIVE_BEARING_MIN && relativeBearingDeg < U_TURN_RELATIVE_BEARING_MAX {
			isUTurn = true
			if sign < 0 {
				uTurnType = da.U_TURN_LEFT
			} else {
				uTurnType = da.U_TURN_RIGHT
			}
		}
	}
	return isUTurn, uTurnType
}

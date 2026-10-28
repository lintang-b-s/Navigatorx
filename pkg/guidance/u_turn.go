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
func (db *DirectionBuilder) checkUTurn(sign da.TurnType, name string, segmentId da.Index) (bool, da.TurnType) {
	isUTurn := false
	uTurnType := da.UTurnUnknown

	if db.doublePrevInitialBearing != 0 && (db.prevSign != da.IGNORE) &&
		db.isSameConsecutiveTurn(db.prevSign, sign) &&
		isSameName(db.doublePrevStreetName, name) {

		tail := db.rn.GetSegmentTailCoord(segmentId)
		head := db.GetHeadPoint(segmentId, tail, 10)

		currentInitialBearing := geo.ComputeInitialBearing(tail.GetLat(), tail.GetLon(), head.GetLat(), head.GetLon())
		relativeBearing := math.Abs(db.doublePrevInitialBearing - currentInitialBearing)
		relativeBearingDeg := util.RadiansToDegree(relativeBearing)
		if relativeBearingDeg > UTurnRelativeBearingMin && relativeBearingDeg < UTurnRelativeBearingMax {
			isUTurn = true
			if sign == da.TurnLeft || sign == da.TurnSlightLeft || sign == da.TurnSharpLeft {
				uTurnType = da.UTurnLeft
			} else {
				uTurnType = da.UTurnRight
			}
		}
	}
	return isUTurn, uTurnType
}

func (db *DirectionBuilder) isSameConsecutiveTurn(prevSign, sign da.TurnType) bool {
	if ((db.lefthand && (sign == da.TurnSlightRight || sign == da.TurnRight || sign == da.TurnSharpRight)) ||
		(!db.lefthand && (sign == da.TurnSlightLeft || sign == da.TurnLeft || sign == da.TurnSharpLeft))) &&
		((db.lefthand && prevSign == da.TurnSlightRight || prevSign == da.TurnRight || prevSign == da.TurnSharpRight) ||
			(!db.lefthand && prevSign == da.TurnSlightLeft || prevSign == da.TurnLeft || prevSign == da.TurnSharpLeft)) {
		return true
	}
	return false
}

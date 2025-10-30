package guidance

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://www.movable-type.co.uk/scripts/latlong.html
// initial bearing (baering from a to b with meridian line crossing a)
func computeInitialBearing(lat1, lon1, lat2, lon2 float64) float64 {
	bearing := geo.BearingTo(lat1, lon1, lat2, lon2)
	bearing = util.DegreeToRadians(bearing)
	return bearing
}

// final bearing (baering from a to b with meridian line crossing b)
func computeFinalBearing(lat1, lon1, lat2, lon2 float64) float64 {
	bearing := geo.BearingTo(lat2, lon2, lat1, lon1)
	bearing = math.Mod(bearing+180, 360)
	bearing = util.DegreeToRadians(bearing)
	return bearing
}

// computeDeltaBearing. compute \Delta (current edge initial bearing - prev edge initial bearing) . output in radians
func computeDeltaBearing(prevLat, prevLon, lat, lon, prevInitialBearing float64) float64 {
	initialBearing := computeInitialBearing(prevLat, prevLon, lat, lon)
	prevInitialBearing, initialBearing = alignInitialBearing(prevInitialBearing, initialBearing)
	return initialBearing - prevInitialBearing
}

/*
alignInitialBearing. handle case ketika initialBearing-prevInitialBearing > 180° atau  initialBearing-prevInitialBearing < -180°.

misal:

	          \
			   \ initialBearing (350°)
				\
				/
			   /		prevInitialBearing (20°)
			  /

harusnya belok kiri, tapi karena > 180° jadi belok kanan.
how to fix: prevInitialBearing + 360°.

misal:

		 /	initialBearing (10°)
		/
	   /
	   \
		\
		 \		prevInitialBearing (340°)
		  \

dif = initialBearing - prevInitialBearing = 10° - 340° = -330°.
harusnya belok kanan, tapi karena < -180° jadi belok kiri.
how to fix: initialBearing + 360°.
*/
func alignInitialBearing(prevInitialBearing, initialBearing float64) (float64, float64) {
	dif := util.RadiansToDegree(initialBearing) - util.RadiansToDegree(prevInitialBearing)
	if dif > 180 {
		prevInitialBearing += 2 * math.Pi
	} else if dif < -180 {
		initialBearing += 2 * math.Pi
	}
	return prevInitialBearing, initialBearing
}

func getTurnDirection(prevLat, prevLon, lat, long, prevInitialBearing float64) int {
	delta := computeDeltaBearing(prevLat, prevLon, lat, long, prevInitialBearing)
	absDelta := math.Abs(delta)
	deltaDegree := util.RadiansToDegree(absDelta)
	if deltaDegree < 12 {
		// 12°
		return datastructure.CONTINUE_ON_STREET
	} else if deltaDegree < 40 {

		if delta < 0 {
			return (datastructure.TURN_SLIGHT_LEFT)
		} else {
			return (datastructure.TURN_SLIGHT_RIGHT)
		}
	} else if deltaDegree < 105 {

		if delta < 0 {
			return (datastructure.TURN_LEFT)
		} else {
			return (datastructure.TURN_RIGHT)
		}

	} else if delta < 0 {
		return (datastructure.TURN_SHARP_LEFT)

	} else {
		return (datastructure.TURN_SHARP_RIGHT)

	}
}

package guidance

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://www.movable-type.co.uk/scripts/latlong.html
// initial bearing (bearing from a to b with meridian line crossing a)
// return in radian
func computeInitialBearing(lat1, lon1, lat2, lon2 float64) float64 {
	bearing := geo.BearingTo(lat1, lon1, lat2, lon2)
	bearing = util.DegreeToRadians(bearing)
	return bearing
}

// final bearing (bearing from a to b with meridian line crossing b)
func computeFinalBearing(lat1, lon1, lat2, lon2 float64) float64 {
	bearing := geo.BearingTo(lat2, lon2, lat1, lon1)
	bearing = math.Mod(bearing+180, 360)
	bearing = util.DegreeToRadians(bearing)
	return bearing
}

// computeDeltaBearing. compute  (current edge initial bearing - prev edge initial bearing) . output in radians.
// current edge initial bearing = initialBearing dari titik prev ke titik current (curLat, curLon)
// prevInitialBearing = prev edge initial bearing dari titik doublePrev ke titik prev.
// prevInitialBearing in radians.
func computeDeltaBearing(prevLat, prevLon, curLat, curLon, prevInitialBearing float64) float64 {
	initialBearing := computeInitialBearing(prevLat, prevLon, curLat, curLon)
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

harusnya belok kiri, tapi karena initialBearing-prevInitialBearing = 330° > 180° jadi belok kanan.
how to fix: prevInitialBearing + 360°.
setelah fix:  karena initialBearing-prevInitialBearing = -30° turn slight left

misal:

		 /	initialBearing (10°)
		/
	   /
	   \
		\
		 \		prevInitialBearing (340°)
		  \

dif = initialBearing - prevInitialBearing = 10° - 340° = -330° < -180°
harusnya belok kanan, tapi karena < -180° jadi belok kiri.
how to fix: initialBearing + 360°.
setelah fix:  karena initialBearing-prevInitialBearing = 370° - 340° = 30°  turn slight right

prevInitialBearing, initialBearing  in radians
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

/*
contoh1:

		initial bearing 90°
	---------
	|
	|
	|
	| prevInitialBearing 0°

deltaBearing = 90°
turn right

contoh2:

prevInitialBearing 90°
--------

	\
	 \
	  \
	   \ initialBearing 120°

deltaBearing = 30°
turn slight right

contoh3:

	   / initialBearing 75°
	  /
	 /
	/

--------

prevInitialBearing 90°

deltaBearing = -15°
turn slight left

contoh4:

				prevInitialBearing 270°
			----------
		   /
		  /
		 /
		/
	   /	initialBearing 250°

deltaBearing = -20°
turn slight left
*/
func getTurnDirection(prevLat, prevLon, lat, long, prevInitialBearing float64) da.TurnType {
	delta := computeDeltaBearing(prevLat, prevLon, lat, long, prevInitialBearing)
	absDelta := math.Abs(delta)
	deltaDegree := util.RadiansToDegree(absDelta)
	if deltaDegree < 12 {
		// 12°
		return da.CONTINUE_ON_STREET
	} else if deltaDegree < 40 {

		if delta < 0 {
			return (da.TURN_SLIGHT_LEFT)
		} else {
			return (da.TURN_SLIGHT_RIGHT)
		}
	} else if deltaDegree < 105 {

		if delta < 0 {
			return (da.TURN_LEFT)
		} else {
			return (da.TURN_RIGHT)
		}

	} else if delta < 0 {
		return (da.TURN_SHARP_LEFT)

	} else {
		return (da.TURN_SHARP_RIGHT)

	}
}

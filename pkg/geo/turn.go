package geo

import (
	"math"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://www.movable-type.co.uk/scripts/latlong.html
// initial bearing (bearing from a to b with meridian line crossing a).
// return in radian.
func ComputeInitialBearing(lat1, lon1, lat2, lon2 float64) float64 {
	bearing := BearingTo(lat1, lon1, lat2, lon2)
	bearing = util.DegreeToRadians(bearing)
	return bearing
}

// final bearing (bearing from a to b with meridian line crossing b)
func ComputeFinalBearing(lat1, lon1, lat2, lon2 float64) float64 {
	bearing := BearingTo(lat2, lon2, lat1, lon1)
	bearing = math.Mod(bearing+180, 360)
	bearing = util.DegreeToRadians(bearing)
	return bearing
}

// ComputeRelativeBearing. compute  relative bearing = (current edge initial bearing - prev edge initial bearing) . output in radians.
// current edge initial bearing = initialBearing dari titik prev ke titik current (curLat, curLon)
// prevInitialBearing = prev edge initial bearing dari titik doublePrev ke titik prev.
// prevInitialBearing in radians.
// https://en.wikipedia.org/wiki/Bearing_(navigation)#Relative
// https://i0.wp.com/blog.mytimezero.com/wp-content/uploads/2017/12/art-blog-course-heading-fr.png?resize=636%2C471&ssl=1
func ComputeRelativeBearing(prevLat, prevLon, curLat, curLon, prevInitialBearing float64) float64 {
	initialBearing := ComputeInitialBearing(prevLat, prevLon, curLat, curLon)
	relativeBearing := alignRelativeBearing(initialBearing - prevInitialBearing)
	return relativeBearing
}

/*
alignRelativeBearing. handle case ketika initialBearing-prevInitialBearing > 180° atau  initialBearing-prevInitialBearing < -180°.

misal:

	          \
			   \ initialBearing (350°)
				\
				/
			   /		prevInitialBearing (20°)
			  /

harusnya belok kiri, tapi relativeBearing = 330° > 180° jadi belok kanan.

setelah fix:  relativeBearing = -30° turn slight left

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

setelah fix:  relativeBearing  = 30°  turn slight right

relativeBearing in radians
*/
func alignRelativeBearing(relativeBearing float64) float64 {
	if util.RadiansToDegree(relativeBearing) > 180 {
		relativeBearing -= 2 * math.Pi
	} else if util.RadiansToDegree(relativeBearing) < -180 {
		relativeBearing += 2 * math.Pi
	}
	return relativeBearing
}

/*
contoh1:

		initial bearing 90°
	---------
	|
	|
	|
	| prevInitialBearing 0°

relativeBearing = 90°
turn right

contoh2:

prevInitialBearing 90°
--------
		\
		 \
	  	  \
	  	   \ initialBearing 120°

relativeBearing = 30°
turn slight right

contoh3:

		   / initialBearing 75°
		  /
		 /
		/
--------

prevInitialBearing 90°

relativeBearing = -15°
turn slight left

contoh4:

				prevInitialBearing 270°
			----------
		   /
		  /
		 /
		/
	   /	initialBearing 250°

relativeBearing = -20°
turn slight left
*/ // nolint: gofmt
func GetTurnDirection(prevLat, prevLon, lat, long, prevInitialBearing float64) da.TurnType {
	delta := ComputeRelativeBearing(prevLat, prevLon, lat, long, prevInitialBearing)
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

package osmparser

import (
	"strings"
	"time"

	"github.com/bytedance/gopkg/lang/stringx"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/paulmach/osm"
)

func (o *OsmParser) roadTypeSpeed(roadType string) float64 {
	return o.maxspeeds[pkg.GetHighwayType(roadType)]
}

func findKeyNotEmpty(way *osm.Way, keys ...string) string {
	for _, key := range keys {
		if val := way.Tags.Find(key); val != "" {
			return val
		}
	}
	return ""
}

// see: https://wiki.openstreetmap.org/wiki/Template:Map_Features:name
// see https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav , directions attributes
// see https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla , general tags (ways)
// contoh: https://www.openstreetmap.org/way/750686303 :
// https://wiki.openstreetmap.org/wiki/Exit_Info
func getName(way *osm.Way, tempMap map[string]string) {
	name := findKeyNotEmpty(way,
		"name",
		"official_name",
		"reg_name",
		"nat_name",
		"loc_name",
		"name_alt",
		"name_int",
		"old_name",
	)

	tempMap[STREET_NAME] = name

	// taken from: https://wiki.openstreetmap.org/wiki/Key:ref
	// When a road has no number, leave this key unused; this makes sure that routing programs do not try to describe the road by a number.
	// tag ref itu buat: https://wiki.openstreetmap.org/wiki/Exit_Info
	// ref buat driving instruction ketika kita di junction
	// contoh: https://www.openstreetmap.org/directions?engine=fossgis_osrm_car&route=-7.523894%2C110.810037%3B-7.525958%2C110.800896#map=17/-7.524809/110.805455
	// take exit 504 on ...
	// continue onto jl ... (6)
	refName := findKeyNotEmpty(way,
		"ref",
		"junction:ref",
		"destination:ref",
		"destination:ref:to",
		"int_ref",
		"nat_ref",
		"reg_ref",
		"loc_ref",
		"old_ref",
	)

	tempMap[STREET_REF] = refName

	destinationRefName := findKeyNotEmpty(way,
		"destination:ref",
		"destination:ref:to",
	)

	tempMap[DESTINATION_REF] = destinationRefName

	destination := findKeyNotEmpty(way,
		"destination",
		"destination:street",
		"destination:ref:street:to",
	)
	tempMap[DESTINATION] = destination
	// todo: add ref di driving direction
}

//7. https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way

func isDirectionProhibited(value string) bool {
	if value == "no" {
		return true
	}
	return false
}

// https://wiki.openstreetmap.org/wiki/Key:oneway
//
//	https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way
func getReversedOneWay(way *osm.Way, tempMap map[string]string) (bool, bool) {
	restrictedForward := false
	restrictedBackward := false
	if pkg.IsVehicle {
		// see Land-based transportation: https://wiki.openstreetmap.org/wiki/Key:access
		// vehicle is all type of vehicle (bicycle, car, bus, motorcycle, etc)
		/*
			taken from: https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way
				On a street, such as highway=unclassified, highway=residential or highway=pedestrian, oneway=* can safely be interpreted as applying only to vehicles. It should not affect pedestrian routing.
				, and is effectively a synonym for vehicle:backward=no.
		*/
		vehicleForward := way.Tags.Find("vehicle:forward")
		motorVehicleForward := way.Tags.Find("motor_vehicle:forward")
		restrictedForward = isDirectionProhibited(vehicleForward) || isDirectionProhibited(motorVehicleForward)

		vehicleBackward := way.Tags.Find("vehicle:backward")
		motorVehicleBackward := way.Tags.Find("motor_vehicle:backward")
		restrictedBackward = isDirectionProhibited(vehicleBackward) || isDirectionProhibited(motorVehicleBackward)

		if tagPrefix, ok := pkg.VehicleTypeTag[pkg.VehicleType]; ok {
			if !restrictedForward {
				restrictedForward = isDirectionProhibited(way.Tags.Find(tagPrefix + ":forward"))
			}
			if !restrictedBackward {
				restrictedBackward = isDirectionProhibited(way.Tags.Find(tagPrefix + ":backward"))
			}
		}

		// https://taginfo.openstreetmap.org/projects/valhalla#tags ,  public service vehicle (https://wiki.openstreetmap.org/wiki/Key:access)
		if _, isPublicService := pkg.PublicServiceVehicles[pkg.VehicleType]; isPublicService {

			// cek restricted forward
			if psvForward := way.Tags.Find("lanes:psv:forward"); isDirectionProhibited(psvForward) {
				restrictedForward = true
			} else if psvForward := way.Tags.Find("bus:psv:forward"); isDirectionProhibited(psvForward) && pkg.VehicleType == pkg.BUS {
				// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, bus/psv/lanes:psv:forward
				restrictedForward = true
			} else if psvForward := way.Tags.Find("busway:psv:forward"); isDirectionProhibited(psvForward) && pkg.VehicleType == pkg.BUS {
				//  https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, busway/lanes:psv:backward
				restrictedForward = true
			}

			// cek restricted backward
			if psBackward := way.Tags.Find("lanes:psv:backward"); isDirectionProhibited(psBackward) {
				restrictedBackward = true
			} else if psBackward := way.Tags.Find("bus:psv:backward"); isDirectionProhibited(psBackward) && pkg.VehicleType == pkg.BUS {
				// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, bus/psv/lanes:psv:forward
				restrictedBackward = true
			} else if psBackward := way.Tags.Find("busway:psv:backward"); isDirectionProhibited(psBackward) && pkg.VehicleType == pkg.BUS {
				//  https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, busway/lanes:psv:backward
				restrictedBackward = true
			}
		}

		// cek bicycle sepeda
		// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla , cycleway:both or cycleway:right and cycleway:left
		bicycleBoth := way.Tags.Find("cycleway:both") != "" || way.Tags.Find("cycleway:right") != "" || way.Tags.Find("cycleway:left") != ""
		if bicycleBoth && pkg.VehicleType == pkg.BICYCLE {
			restrictedForward = false
			restrictedBackward = false
		}
		// kayake ada yang kurang

	} else {
		// pedestrian
		/*
			taken from: https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way
			On highway=steps and highway=via_ferrata, the tag oneway=* can safely be interpreted as applying to pedestrians.
		*/

		footForward := way.Tags.Find("foot:forward")
		restrictedForward = isDirectionProhibited(footForward)

		footBackward := way.Tags.Find("foot:backward")
		restrictedBackward = isDirectionProhibited(footBackward)
	}

	return restrictedForward, restrictedBackward
}

var dayMap = map[string]time.Weekday{
	"Mo": time.Monday,
	"Tu": time.Tuesday,
	"We": time.Wednesday,
	"Th": time.Thursday,
	"Fr": time.Friday,
	"Sa": time.Saturday,
	"Su": time.Sunday,
	// ada PH: public holiday (https://wiki.openstreetmap.org/wiki/Conditional_restrictions)
	// tapi kita gak tahu cara cek today is public holiday, jadi gak usah
}

const (
	DIRECTION_FORWARD_REVERSIBLE uint8 = iota
	DIRECTION_BACKWARD_REVERSIBLE
	NO_ROUTE_REVERSIBLE
)

// https://wiki.openstreetmap.org/wiki/Conditional_restrictions
// parse "Mo-Fr 07:00-09:00" or "23:00-05:00" or "23:00-05:00, 10:00-15:00" or "Mo-Fr,Su 22:00-06:00" or etc
func isConditionalAccessInTimerange(condTime string, currentTime time.Time) (inTimeRange bool, err error) {

	// condTime = Mo-Fr 07:00-09:00 or 23:00-05:00 or "23:00-05:00, 10:00-15:00" or "Mo-Fr,Su 22:00-06:00" or
	// "Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00" or "Mo-Fr 07:00-09:00,16:00-18:00" or etc

	condTimeVals := strings.Split(condTime, ";")

	for _, condTimeVal := range condTimeVals {
		var (
			fromH, fromMin, toH, toMin                 int
			fromHours, fromMinutes, toHours, toMinutes []int
			days                                       []time.Weekday
		)
		tokens := strings.Fields(condTimeVal)

		switch len(tokens) {
		case 1:

			// timeRange = 17:01-8:59
			timeRangeString := tokens[0]
			timeRanges := strings.Split(timeRangeString, ",")

			for _, timeRange := range timeRanges {
				timeParts := strings.Split(timeRange, "-")
				// timeParts = [17:01, 8:59]
				var fromTime, toTime []string
				if stringx.ContainsAnySubstrings(timeParts[0], []string{":"}) {
					fromTime = strings.Split(timeParts[0], ":")

				} else if stringx.ContainsAnySubstrings(timeParts[0], []string{"."}) {
					fromTime = strings.Split(timeParts[0], ".")

				} else {
					fromTime = strings.Split(timeParts[0], ",")
				}

				if stringx.ContainsAnySubstrings(timeParts[1], []string{":"}) {

					toTime = strings.Split(timeParts[1], ":")

				} else if stringx.ContainsAnySubstrings(timeParts[1], []string{"."}) {

					toTime = strings.Split(timeParts[1], ".")
				} else {

					toTime = strings.Split(timeParts[1], ",")
				}

				fromH, err = util.ParseInt(fromTime[0])
				fromHours = append(fromHours, fromH)
				fromMin, err = util.ParseInt(fromTime[1])
				fromMinutes = append(fromMinutes, fromMin)

				toH, err = util.ParseInt(toTime[0])
				toHours = append(toHours, toH)
				toMin, err = util.ParseInt(toTime[1])
				toMinutes = append(toMinutes, toMin)
			}

		case 2:
			//  dayString: Mo-Fr
			dayString := tokens[0]

			for _, group := range strings.Split(dayString, ",") {
				group = strings.TrimSpace(group)
				if parts := strings.Split(group, "-"); len(parts) == 2 {
					// "Mo-Fr"
					start, okS := dayMap[parts[0]]
					end, okE := dayMap[parts[1]]
					if !okS || !okE {
						continue
					}
					if end == time.Sunday {
						end = time.Saturday
						days = append(days, time.Sunday)
					}
					for d := start; d <= end; d++ {
						days = append(days, d)
					}
				} else {
					// "Sa"
					if d, ok := dayMap[group]; ok {
						days = append(days, d)
					}
				}
			}

			// timeRange = 17:01-8:59
			timeRangeString := tokens[1]
			timeRanges := strings.Split(timeRangeString, ",")

			for _, timeRange := range timeRanges {
				timeParts := strings.Split(timeRange, "-")
				// timeParts = [17:01, 8:59]
				var fromTime, toTime []string
				if stringx.ContainsAnySubstrings(timeParts[0], []string{":"}) {
					fromTime = strings.Split(timeParts[0], ":")

				} else if stringx.ContainsAnySubstrings(timeParts[0], []string{"."}) {
					fromTime = strings.Split(timeParts[0], ".")

				} else {
					fromTime = strings.Split(timeParts[0], ",")
				}

				if stringx.ContainsAnySubstrings(timeParts[1], []string{":"}) {

					toTime = strings.Split(timeParts[1], ":")

				} else if stringx.ContainsAnySubstrings(timeParts[1], []string{"."}) {

					toTime = strings.Split(timeParts[1], ".")
				} else {

					toTime = strings.Split(timeParts[1], ",")
				}

				fromH, err = util.ParseInt(fromTime[0])
				fromHours = append(fromHours, fromH)
				fromMin, err = util.ParseInt(fromTime[1])
				fromMinutes = append(fromMinutes, fromMin)

				toH, err = util.ParseInt(toTime[0])
				toHours = append(toHours, toH)
				toMin, err = util.ParseInt(toTime[1])
				toMinutes = append(toMinutes, toMin)
			}
		}

		if len(days) > 0 {
			currentWeekday := currentTime.Weekday()
			dayMatch := false
			for _, d := range days {
				if d == currentWeekday {
					dayMatch = true
					break
				}
			}
			if !dayMatch {
				inTimeRange = false

				continue
			}
		}

		for i := 0; i < len(fromHours); i++ {
			fromH = fromHours[i]
			fromMin = fromMinutes[i]
			toH = toHours[i]
			toMin = toMinutes[i]

			currentMins := currentTime.Hour()*60 + currentTime.Minute()
			fromMins := fromH*60 + fromMin
			toMins := toH*60 + toMin
			if isInTimeRange(currentMins, fromMins, toMins) {
				inTimeRange = true
				return
			}
		}

	}

	return
}

// https://wiki.openstreetmap.org/wiki/Tag:oneway%3Dreversible
func getReversibleOneWay(val string, currentTime time.Time) (direction uint8, err error) {

	isSplit := false

	// isSplit=true iff ';' di luar kurung buka kurung tutup
	isInParentheses := false
	for _, ch := range val {
		if ch == '(' {
			isInParentheses = true
		} else if ch == ')' {
			isInParentheses = false
		} else if ch == ';' && !isInParentheses {
			isSplit = true
			break
		}
	}

	var ranges []string
	if isSplit {
		ranges = strings.Split(val, ";")
		// ranges = [ "yes @ (09:00-17:00)", "no @ (17:01-8:59)"]
	} else {
		ranges = []string{val}
	}

	for _, condRange := range ranges {
		// condRange: yes @ (09:00-17:00) atau -1 @ (17:01-8:59)
		// atau yes @ (Mo-Fr 17:00-21:00) atau -1 @ (Mo-Fr 07:30-10:00)
		condRange = strings.TrimSpace(condRange)
		// condRange: yes @ (09:00-17:00) atau -1 @ (17:01-8:59)
		// atau yes @ (Mo-Fr 17:00-21:00) atau -1 @ (Mo-Fr 07:30-10:00)
		parts := strings.Split(condRange, "@")

		direction := strings.TrimSpace(parts[0])
		var forwardDir bool
		switch direction {
		case "yes":
			forwardDir = true
		case "-1":
			forwardDir = false
		}

		isInTimeRange := true

		if len(parts) > 1 {
			condTime := strings.Trim(strings.TrimSpace(parts[1]), "()")

			isInTimeRange, err = isConditionalAccessInTimerange(condTime, currentTime)
		}

		if !isInTimeRange {
			continue
		} else {
			switch forwardDir {
			case true:
				return DIRECTION_FORWARD_REVERSIBLE, nil
			case false:
				return DIRECTION_BACKWARD_REVERSIBLE, nil
			}
		}
	}

	return NO_ROUTE_REVERSIBLE, nil
}

func isInTimeRange(currentMins, fromMins, toMins int) bool {
	if fromMins <= toMins {
		// hours: 10:00-17:00 atau minutes: 600-1020
		return currentMins >= fromMins && currentMins < toMins
	}
	// hours: 20:00-09:00 atau minutes: 1200-540
	// misal curentHours: 22:00 atau currentMins: 1320
	return currentMins >= fromMins || currentMins < toMins
}

// isBarrierNodeAccessable. return true if barrier node accessable
func (p *OsmParser) isBarrierNodeAccessable(node *osm.Node) (bool, error) {
	accessVal := node.Tags.Find("access")
	barrierTypeString := node.Tags.Find("barrier")
	var err error

	// https://wiki.openstreetmap.org/wiki/Key:access#Transport_mode_restrictions

	// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, valhalla barrier=bollard/block/removable -> automotive (car,motorcycle,etc) access=no

	//https://wiki.openstreetmap.org/wiki/Key:barrier
	// for splitting street segment to 2 disconnected graph edge
	// if the access tag of the barrier node is != "no" , we dont split the segment
	// for example, at the barrier at the entrance to FMIPA UGM, where entry is only allowed after 16.00 WIB or before 8.00 wib. (https://www.openstreetmap.org/node/8837559091)
	// contoh 2 pogung swing_gate: https://www.openstreetmap.org/node/10303116750
	// contoh access=no + barrier=gate: https://www.openstreetmap.org/node/12180882980
	// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla
	//
	barrierType := getBarrierType(barrierTypeString)
	switch barrierType {
	case BOLLARD, BLOCK, REMOVABLE:
		if pkg.VehicleType == pkg.MOTORCAR || pkg.VehicleType == pkg.MOTORCYCLE || pkg.VehicleType == pkg.BUS ||
			pkg.VehicleType == pkg.GOODS || pkg.VehicleType == pkg.TAXI || pkg.VehicleType == pkg.MINIBUS {
			return false, nil
		}
		return true, nil
	default:
		// barrierType == NO_BARRIER, or selain yang diatas

		if accessVal == "no" {
			// see https://wiki.openstreetmap.org/wiki/Key:access , section Transport mode restrictions
			// see https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla , access tags for routing (nodes)

			if pkg.VehicleType == pkg.BUS {
				busAllowed := node.Tags.Find("bus") != "no"
				if busAllowed {
					return true, nil
				}

				psvAllowed := node.Tags.Find("psv") != "no"
				return psvAllowed, nil

			} else if pkg.VehicleType == pkg.FOOT {

				pedestrianAccessVal := node.Tags.Find("foot")
				pedestrianAllowed := pedestrianAccessVal != "no" && pedestrianAccessVal != "discouraged"
				return pedestrianAllowed, nil
			} else if pkg.VehicleType == pkg.BICYCLE {
				bicycleAccessVal := node.Tags.Find("bicycle")
				bicycleAllowed := bicycleAccessVal != "no" && bicycleAccessVal != "none"
				if bicycleAllowed {
					return true, nil
				}
			}

			return false, nil
		} else if accessCondType := node.Tags.Find("access:conditional"); accessCondType != "" {
			// https://www.openstreetmap.org/node/10303116750#map=19/-7.756394/110.381924&layers=N
			// contoh accessCondType: "no @ (Mo-Fr,Su 22:00-06:00; Sa 22:30-06:00)"
			// atau  "yes @ (09:00-17:00); no @ (17:01-8:59)"]

			isSplit := false

			// isSplit=true iff ';' di luar kurung buka kurung tutup
			isInParentheses := false
			for _, ch := range accessCondType {
				if ch == '(' {
					isInParentheses = true
				} else if ch == ')' {
					isInParentheses = false
				} else if ch == ';' && !isInParentheses {
					isSplit = true
					break
				}
			}

			var ranges []string
			if isSplit {
				ranges = strings.Split(accessCondType, ";")
				// ranges = [ "yes @ (09:00-17:00)", "no @ (17:01-8:59)"]
			} else {
				ranges = []string{accessCondType}
			}

			for _, condRange := range ranges {
				// condRange: yes @ (09:00-17:00) atau -1 @ (17:01-8:59)
				// atau yes @ (Mo-Fr 17:00-21:00) atau -1 @ (Mo-Fr 07:30-10:00)
				condRange = strings.TrimSpace(condRange)
				parts := strings.Split(condRange, "@")

				condAccessVal := strings.TrimSpace(parts[0])
				var no bool
				if condAccessVal == "no" || condAccessVal == "agricultural" ||
					condAccessVal == "forestry" || condAccessVal == "discouraged" {
					no = true
				}

				isInTimeRange := true

				if len(parts) > 1 {
					condTime := strings.Trim(strings.TrimSpace(parts[1]), "()")

					isInTimeRange, err = isConditionalAccessInTimerange(condTime, p.currentTime)
					if err != nil {
						return false, err
					}
				}

				if !isInTimeRange {
					continue
				} else {
					// isInTimeRange=true
					// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, OSM tags for routing (nodes):
					// kita cuma prohibite node for routing, ketika access val = no ,discouraged, agricultural, dan forestry
					if no == true {
						return false, nil
					}

					return true, nil
				}
			}
		}

		// access=yes atau yang lainnya
		if isAccessTagProhibited(accessVal) {
			return false, nil
		}

		// vehicle type accesss
		vehicleTagVal := node.Tags.Find("vehicle")
		vehicleProbhibited := isAccessTagProhibited(vehicleTagVal)
		if pkg.IsVehicle && vehicleProbhibited {
			return false, nil
		}

		// motor vehicle type accesss
		motorVehicleTagVal := node.Tags.Find("motor_vehicle")
		motorizedVehicleProhibited := isAccessTagProhibited(motorVehicleTagVal)
		if pkg.MotorizedVehicle && motorizedVehicleProhibited {
			return false, nil
		}

		// specific vehicle type accesss
		specificVehicleTypeTagVal := node.Tags.Find(pkg.VehicleTypeTag[pkg.VehicleType])
		vehicleTypeProhibited := isAccessTagProhibited(specificVehicleTypeTagVal)
		if vehicleTypeProhibited {
			return false, nil
		}

		return true, nil
	}
}

func isAccessTagProhibited(accessVal string) bool {
	if accessVal == "no" || accessVal == "discouraged" || accessVal == "agricultural" ||
		accessVal == "forestry" {
		return true
	}

	return false
}

func (p *OsmParser) acceptOsmWay(way *osm.Way) bool {

	// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla , access tags (ways)

	accessVal := way.Tags.Find("access")
	// access=yes atau yang lainnya
	if isAccessTagProhibited(accessVal) {
		return false
	}

	// vehicle type accesss
	vehicleTagVal := way.Tags.Find("vehicle")
	vehicleProbhibited := isAccessTagProhibited(vehicleTagVal)
	if pkg.IsVehicle && vehicleProbhibited {
		return false
	}

	// motor vehicle type accesss
	motorVehicleTagVal := way.Tags.Find("motor_vehicle")
	motorizedVehicleProhibited := isAccessTagProhibited(motorVehicleTagVal)
	if pkg.MotorizedVehicle && motorizedVehicleProhibited {
		return false
	}

	// specific vehicle type accesss
	specificVehicleTypeTagVal := way.Tags.Find(pkg.VehicleTypeTag[pkg.VehicleType])
	vehicleTypeProhibited := isAccessTagProhibited(specificVehicleTypeTagVal)
	if vehicleTypeProhibited {
		return false
	}

	busPsvAccess := way.Tags.Find("bus:psv:forward")

	busProhibited := isAccessTagProhibited(busPsvAccess)
	if pkg.VehicleType == pkg.BUS && busProhibited {
		return false
	}

	psvAccess := way.Tags.Find("psv:psv:forward")
	lanePsvAccess := way.Tags.Find("lanes:psv:forward")

	busProhibited = isAccessTagProhibited(psvAccess) || isAccessTagProhibited(lanePsvAccess)
	if pkg.VehicleType == pkg.BUS && busProhibited {
		return false
	}

	// pedestrian type accesss
	footTagVal := way.Tags.Find("foot")
	pedestrianProhibited := isAccessTagProhibited(footTagVal)
	if pkg.VehicleType == pkg.FOOT && pedestrianProhibited {
		return false
	}

	// bicycle type accesss
	bicycleTagVal := way.Tags.Find("bicycle")
	cyclewayTagVal := way.Tags.Find("cycleway")

	bicycleProhibited := isAccessTagProhibited(bicycleTagVal) || isAccessTagProhibited(cyclewayTagVal)
	if pkg.VehicleType == pkg.BICYCLE && bicycleProhibited {
		return false
	}

	highway := way.Tags.Find("highway")
	junction := way.Tags.Find("junction")

	if highway != "" {
		if _, ok := p.highwayWhitelist[highway]; ok {
			return true
		}
	} else if junction != "" {
		return true
	}

	return false
}

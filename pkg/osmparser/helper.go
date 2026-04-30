package osmparser

import (
	"fmt"
	"strconv"
	"strings"
	"time"

	"github.com/bytedance/gopkg/lang/stringx"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/paulmach/osm"
)

func (p *OsmParser) roadTypeSpeed(roadType string) float64 {
	return p.maxspeeds[pkg.GetHighwayType(roadType)]
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
	return value == "no"
}

// https://wiki.openstreetmap.org/wiki/Key:oneway
//
//	https://wiki.openstreetmap.org/wiki/Forward_%26_backward,_left_%26_right#Identifying_the_direction_of_a_way
func getReversedOneWay(way *osm.Way) (bool, bool) {
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

const (
	DIRECTION_FORWARD_REVERSIBLE uint8 = iota
	DIRECTION_BACKWARD_REVERSIBLE
	NO_ROUTE_REVERSIBLE
)

const (
	SUNRISE_DEFAULT = 6 * 60
	SUNSET_DEFAULT  = 18 * 60
	DAWN_DEFAULT    = 5*60 + 30
	DUSK_DEFAULT    = 18*60 + 30
)

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

func parseOsmTime(timeStr string) (int, error) {
	timeStr = strings.Trim(strings.TrimSpace(timeStr), "()")
	if timeStr == "" {
		return 0, fmt.Errorf("empty time string")
	}

	// Handle offsets like sunrise+01:00 or sunset-00:30
	if idx := strings.IndexAny(timeStr, "+-"); idx != -1 {
		base := strings.TrimSpace(timeStr[:idx])
		offset := strings.TrimSpace(timeStr[idx+1:])
		sign := timeStr[idx]

		if isSolarEvent(base) {
			baseMins, _ := parseSolarEvent(base)
			offsetMins, err := parseOsmTime(offset)
			if err != nil {
				return 0, err
			}
			if sign == '+' {
				return baseMins + offsetMins, nil
			}
			return baseMins - offsetMins, nil
		}
	}

	if isSolarEvent(timeStr) {
		mins, _ := parseSolarEvent(timeStr)
		return mins, nil
	}

	// Handle HH:MM, HH.MM, HH,MM
	var parts []string
	if strings.Contains(timeStr, ":") {
		parts = strings.Split(timeStr, ":")
	} else if strings.Contains(timeStr, ".") {
		parts = strings.Split(timeStr, ".")
	} else if strings.Contains(timeStr, ",") {
		parts = strings.Split(timeStr, ",")
	} else {
		// Just a number?
		val, err := strconv.Atoi(timeStr)
		if err != nil {
			return 0, fmt.Errorf("invalid time format: %s", timeStr)
		}
		return val * 60, nil // Assume hours (OSM default for single number)
	}

	if len(parts) != 2 {
		return 0, fmt.Errorf("invalid time format: %s", timeStr)
	}

	h, err := strconv.Atoi(parts[0])
	if err != nil {
		return 0, err
	}
	m, err := strconv.Atoi(parts[1])
	if err != nil {
		return 0, err
	}

	return h*60 + m, nil
}

func isSolarEvent(s string) bool {
	s = strings.ToLower(strings.TrimSpace(s))
	return s == "sunrise" || s == "sunset" || s == "dawn" || s == "dusk"
}

func parseSolarEvent(s string) (int, error) {
	s = strings.ToLower(strings.TrimSpace(s))
	switch s {
	case "sunrise":
		return SUNRISE_DEFAULT, nil
	case "sunset":
		return SUNSET_DEFAULT, nil
	case "dawn":
		return DAWN_DEFAULT, nil
	case "dusk":
		return DUSK_DEFAULT, nil
	}
	return 0, fmt.Errorf("not a solar event: %s", s)
}

func splitTimeRange(timeRange string) []string {
	var parts []string
	var current strings.Builder
	depth := 0
	for _, r := range timeRange {
		switch r {
		case '(':
			depth++
		case ')':
			depth--
		}
		if r == '-' && depth == 0 {
			parts = append(parts, current.String())
			current.Reset()
		} else {
			current.WriteRune(r)
		}
	}
	parts = append(parts, current.String())
	return parts
}

// https://wiki.openstreetmap.org/wiki/Conditional_restrictions
// parse "Mo-Fr 07:00-09:00" or "23:00-05:00" or "23:00-05:00, 10:00-15:00" or "Mo-Fr,Su 22:00-06:00" or etc
func isConditionalAccessInTimerange(condTime string, currentTime time.Time) (inTimeRange bool, err error) {

	// condTime = Mo-Fr 07:00-09:00 or 23:00-05:00 or "23:00-05:00, 10:00-15:00" or "Mo-Fr,Su 22:00-06:00" or
	// "Mo-Fr 14:00-21:00; Sa-Su,PH 07:00-10:00" or "Mo-Fr 07:00-09:00,16:00-18:00" or etc
	// ada yang baru lagi awoakow: "dusk-dawn"

	condTimeVals := strings.Split(condTime, ";")

	for _, condTimeVal := range condTimeVals {
		var (
			fromMinsSlice, toMinsSlice []int
			days                       []time.Weekday
		)
		tokens := strings.Fields(condTimeVal)
		if len(tokens) == 0 {
			continue
		}

		if tokens[0] == "24/7" {
			inTimeRange = true
			return
		}

		switch len(tokens) {
		case 1:

			// timeRange = 17:01-8:59 or sunset-sunrise
			timeRangeString := tokens[0]
			timeRanges := strings.Split(timeRangeString, ",")

			for _, timeRange := range timeRanges {
				timeParts := splitTimeRange(timeRange)
				if len(timeParts) != 2 {
					// Might be just a date? Or invalid.
					continue
				}

				fm, err := parseOsmTime(timeParts[0])
				if err != nil {
					return false, fmt.Errorf("isConditionalAccessInTimerange: %w", err)
				}
				fromMinsSlice = append(fromMinsSlice, fm)

				tm, err := parseOsmTime(timeParts[1])
				if err != nil {
					return false, fmt.Errorf("isConditionalAccessInTimerange: %w", err)
				}
				toMinsSlice = append(toMinsSlice, tm)
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

					if end < start {
						// Handle wrap around like Sa-Tu
						for d := start; d <= 6; d++ {
							days = append(days, d)
						}
						for d := 0; d <= int(end); d++ {
							days = append(days, time.Weekday(d))
						}
					} else {
						for d := start; d <= end; d++ {
							days = append(days, d)
						}
					}
				} else {
					// "Sa"
					if d, ok := dayMap[group]; ok {
						days = append(days, d)
					}
				}
			}

			// timeRange = 17:01-8:59 or sunset-sunrise
			timeRangeString := tokens[1]
			timeRanges := strings.Split(timeRangeString, ",")

			for _, timeRange := range timeRanges {
				timeParts := splitTimeRange(timeRange)
				if len(timeParts) != 2 {
					continue
				}

				fm, err := parseOsmTime(timeParts[0])
				if err != nil {
					return false, fmt.Errorf("isConditionalAccessInTimerange: %w", err)
				}
				fromMinsSlice = append(fromMinsSlice, fm)

				tm, err := parseOsmTime(timeParts[1])
				if err != nil {
					return false, fmt.Errorf("isConditionalAccessInTimerange: %w", err)
				}
				toMinsSlice = append(toMinsSlice, tm)
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

		currentMins := currentTime.Hour()*60 + currentTime.Minute()
		for i := 0; i < len(fromMinsSlice); i++ {
			if isInTimeRange(currentMins, fromMinsSlice[i], toMinsSlice[i]) {
				inTimeRange = true
				return
			}
		}

	}

	return
}

// https://wiki.openstreetmap.org/wiki/Tag:oneway%3Dreversible
func GetReversibleOneWay(val string, currentTime time.Time) (direction uint8, err error) {

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
			if err != nil {
				return NO_ROUTE_REVERSIBLE, fmt.Errorf("getReversibleOneWay: %w", err)
			}
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

func IsConditionalRestrictionCurrentlyProhibited(currentTime time.Time, accessCondType string) (bool, error) {
	// https://www.openstreetmap.org/node/10303116750#map=19/-7.756394/110.381924&layers=N
	// contoh accessCondType: "no @ (Mo-Fr,Su 22:00-06:00; Sa 22:30-06:00)"
	// atau  "yes @ (09:00-17:00); no @ (17:01-8:59)"]
	var err error
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

			isInTimeRange, err = isConditionalAccessInTimerange(condTime, currentTime)
			if err != nil {
				return false, fmt.Errorf("isBarrierNodeAccessible: %w", err)
			}
		}

		if !isInTimeRange {
			continue
		} else {
			// isInTimeRange=true
			// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, OSM tags for routing (nodes):
			// kita cuma prohibite node for routing, ketika access val = no ,discouraged, agricultural, dan forestry
			if no {
				return false, nil
			}

			return true, nil
		}
	}

	return false, nil
}

// isBarrierNodeAccessible. return true if barrier node accessible
func (p *OsmParser) isBarrierNodeAccessible(node *osm.Node) (bool, error) {

	barrierTypeString := node.Tags.Find("barrier")

	// https://wiki.openstreetmap.org/wiki/Key:access#Transport_mode_restrictions

	// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla, valhalla barrier=bollard/block/removable -> automotive (car,motorcycle,etc) access=no

	//https://wiki.openstreetmap.org/wiki/Key:barrier
	// for splitting street segment to 2 disconnected graph edge
	// if the access tag of the barrier node is != "no" , we dont split the segment
	// for example, at the barrier at the entrance to FMIPA UGM, where entry is only allowed after 16.00 WIB or before 8.00 wib. (https://www.openstreetmap.org/node/8837559091)
	// contoh 2 pogung swing_gate: https://www.openstreetmap.org/node/10303116750
	// contoh access=no + barrier=gate: https://www.openstreetmap.org/node/12180882980
	// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla

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

		if accessCondVal := node.Tags.Find("access:conditional"); accessCondVal != "" {
			p.conditionalBarrierNodes = append(p.conditionalBarrierNodes, da.NewConditionalBarrierNode(int64(node.ID), accessCondVal))
		}

		// see https://wiki.openstreetmap.org/wiki/Key:access , section Transport mode restrictions
		// see https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla , access tags for routing (nodes)
		findTag := func(key string) string {
			return node.Tags.Find(key)
		}
		allowed := isAccessByVehicleModeAllowed(findTag)
		if !allowed {
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

/*
https://wiki.openstreetmap.org/wiki/Key:access

Transport mode restrictions are arraigned into a hierarchy such that parent keys like access=*, vehicle=*, motor_vehicle=*,etc may be used to set the default values for keys below them in the hierarchy. The access=* key is the highest level of the access restriction hierarchy and is used to describe a general access restriction that applies to all transport modes. Adding access=* is equivalent to adding all 2nd level access tags with the same value e.g.access=no is equivalent to adding foot=no, dog=no, horse=no, vehicle=no, etc. Specific access keys may be added to override a default values specified by a parent key.

For example:

access=no foot=yes – access is only allow by foot and via no other means.
access=no, bus=yes – only buses are allowed to enter (for example a road only for buses and forbidden also to pedestrians)
access=yes, motor_vehicle=no – all transport modes except motor vehicles can use the element
access=forestry, foot=permissive - forestry vehicles can use the route legally and that pedestrians can use it currently but that permission may later be withdrawn. Note that this tagging at the same time prohibits the way for all not explicitly tagged modes of travel, e.g. cyclists and horse riders (often, in the case of forestry access, a mistake). It is often better to use the specific tags and not the general access=* for limitations.
*/

func isAccessByVehicleModeAllowed(findTag func(key string) string) bool {
	// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla , access tags (ways)

	accessVal := findTag("access")
	// access=yes atau yang lainnya
	prohibited := isAccessTagProhibited(accessVal)

	// kalau access=no (prohibited=true), vehicle=yes && pkg.IsVehicle=true, berarti allowed

	// disini kita return true kalau tipe kendaraan profile gak prohibited, meskipun access=no

	// vehicle type access
	vehicleTagVal := findTag("vehicle")
	vehicleProbhibited := isAccessTagProhibited(vehicleTagVal)
	if pkg.IsVehicle && !vehicleProbhibited {
		// kalau access=no (prohibited=true), vehicle=yes, berarti allowed
		return true
	}

	// motor vehicle type access
	motorVehicleTagVal := findTag("motor_vehicle")
	motorizedVehicleProhibited := isAccessTagProhibited(motorVehicleTagVal)
	if pkg.MotorizedVehicle && !motorizedVehicleProhibited {
		return true
	}

	// specific vehicle type access
	specificVehicleTypeTagVal := findTag(pkg.VehicleTypeTag[pkg.VehicleType])
	vehicleTypeProhibited := isAccessTagProhibited(specificVehicleTypeTagVal)
	if !vehicleTypeProhibited {
		return true
	}

	busPsvAccess := findTag("bus:psv:forward")

	busProhibited := isAccessTagProhibited(busPsvAccess)
	if pkg.VehicleType == pkg.BUS && !busProhibited {
		return true
	}

	psvAccess := findTag("psv:psv:forward")
	lanePsvAccess := findTag("lanes:psv:forward")

	busProhibited = isAccessTagProhibited(psvAccess) || isAccessTagProhibited(lanePsvAccess)
	if pkg.VehicleType == pkg.BUS && !busProhibited {
		return true
	}

	// pedestrian type access
	footTagVal := findTag("foot")
	pedestrianProhibited := isAccessTagProhibited(footTagVal)
	if pkg.VehicleType == pkg.FOOT && !pedestrianProhibited {
		return true
	}

	// bicycle type access
	bicycleTagVal := findTag("bicycle")
	cyclewayTagVal := findTag("cycleway")

	bicycleProhibited := isAccessTagProhibited(bicycleTagVal) || isAccessTagProhibited(cyclewayTagVal)
	if pkg.VehicleType == pkg.BICYCLE && !bicycleProhibited {
		return true
	}

	// karena kendaraan profile prohibited
	// berarti return allowed as !prohibited

	return !prohibited
}

func GetAccessValConditionalRestriction(val string) string {
	ss := strings.Split(val, "@")
	return strings.TrimSpace(ss[0])
}

func GetTimeRangeValConditionalRestriction(val string) string {
	ss := strings.Split(val, "@") // (....)
	return strings.TrimSpace(ss[1])
}

func IsAccessByVehicleModeConditionallyAllowed(findTag func(key string) string) bool {
	// see: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla , access tags (ways)

	accessVal := findTag("access:conditional")
	// access=yes atau yang lainnya
	prohibited := isAccessTagProhibited(accessVal)

	// kalau access=no (prohibited=true), vehicle=yes && pkg.IsVehicle=true, berarti allowed

	// disini kita return true kalau tipe kendaraan profile gak prohibited, meskipun access=no

	// vehicle type access
	vehicleTagVal := findTag("vehicle:conditional")
	vehicleProbhibited := isAccessTagProhibited(vehicleTagVal)
	if pkg.IsVehicle && !vehicleProbhibited {
		// kalau access=no (prohibited=true), vehicle=yes, berarti allowed
		return true
	}

	// motor vehicle type access
	motorVehicleTagVal := findTag("motor_vehicle:conditional")
	motorizedVehicleProhibited := isAccessTagProhibited(motorVehicleTagVal)
	if pkg.MotorizedVehicle && !motorizedVehicleProhibited {
		return true
	}

	// specific vehicle type access
	specificVehicleTypeTagVal := findTag(pkg.VehicleTypeTag[pkg.VehicleType] + ":conditional")
	vehicleTypeProhibited := isAccessTagProhibited(specificVehicleTypeTagVal)
	if !vehicleTypeProhibited {
		return true
	}

	busPsvAccess := findTag("bus:psv:forward:conditional")

	busProhibited := isAccessTagProhibited(busPsvAccess)
	if pkg.VehicleType == pkg.BUS && !busProhibited {
		return true
	}

	psvAccess := findTag("psv:psv:forward:conditional")
	lanePsvAccess := findTag("lanes:psv:forward:conditional")

	busProhibited = isAccessTagProhibited(psvAccess) || isAccessTagProhibited(lanePsvAccess)
	if pkg.VehicleType == pkg.BUS && !busProhibited {
		return true
	}

	// pedestrian type access
	footTagVal := findTag("foot:conditional")
	pedestrianProhibited := isAccessTagProhibited(footTagVal)
	if pkg.VehicleType == pkg.FOOT && !pedestrianProhibited {
		return true
	}

	// bicycle type access
	bicycleTagVal := findTag("bicycle:conditional")
	cyclewayTagVal := findTag("cycleway:conditional")

	bicycleProhibited := isAccessTagProhibited(bicycleTagVal) || isAccessTagProhibited(cyclewayTagVal)
	if pkg.VehicleType == pkg.BICYCLE && !bicycleProhibited {
		return true
	}

	// karena kendaraan profile prohibited
	// berarti return allowed as !prohibited

	return !prohibited
}

func (p *OsmParser) acceptOsmWay(way *osm.Way) bool {
	findTag := func(key string) string {
		return way.Tags.Find(key)
	}
	allowed := isAccessByVehicleModeAllowed(findTag)
	if !allowed {
		return false
	}

	conditionalTrafficModeVal := findKeyNotEmpty(way, "access:conditional",
		"vehicle:conditional",
		"motor_vehicle:conditional",
		pkg.VehicleTypeTag[pkg.VehicleType]+":conditional",
		"bus:psv:forward:conditional",
		"psv:psv:forward:conditional",
		"lanes:psv:forward:conditional",
		"foot:conditional",
		"bicycle:conditional",
		"cycleway:conditional",
	)

	if conditionalTrafficModeVal != "" {
		p.conditionalTrafficModesVal[int64(way.ID)] = conditionalTrafficModeVal
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

// isRoundaboutByName. cek osm way is roundabout or not by the name.
// example: https://www.openstreetmap.org/way/1350609078 , namane bundaran tapi gak ada tag junction=roundabout.
func (p *OsmParser) isRoundaboutByName(name string) bool {
	nameLower := strings.ToLower(name)

	return stringx.ContainsAnySubstrings(nameLower, roundaboutSubName)
}

func parseOsmWayMaxSpeedVal(maxSpeedVal string) (float64, error) {
	var maxSpeed float64
	if strings.Contains(maxSpeedVal, "mph") {
		currSpeed, err := strconv.ParseFloat(strings.ReplaceAll(maxSpeedVal, " mph", ""), 64)
		if err != nil {
			return 0, err
		}
		maxSpeed = currSpeed * 1.60934
	} else if strings.Contains(maxSpeedVal, "km/h") {
		currSpeed, err := strconv.ParseFloat(strings.ReplaceAll(maxSpeedVal, " km/h", ""), 64)
		if err != nil {
			return 0, err
		}
		maxSpeed = currSpeed
	} else if strings.Contains(maxSpeedVal, "knots") {
		currSpeed, err := strconv.ParseFloat(strings.ReplaceAll(maxSpeedVal, " knots", ""), 64)
		if err != nil {
			return 0, err
		}
		maxSpeed = currSpeed * 1.852
	} else {
		// without unit, anggap km/h
		// taken from: https://wiki.openstreetmap.org/wiki/Key:maxspeed
		// The maximum fixed numeric speed limit,
		// followed by the appropriate unit,
		// if not measured in km/h. When the value is in km/h then no unit should be included. For example, maxspeed=60 for 60 km/h and maxspeed=50 mph for 50 mph (note the space between the value and the unit).
		currSpeed, err := strconv.ParseFloat(maxSpeedVal, 64)
		if err != nil {
			return 0, err
		}
		maxSpeed = currSpeed
	}
	return maxSpeed, nil
}

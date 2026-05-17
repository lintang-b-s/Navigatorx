// Package pkg provides common constants and types used across the Navigatorx project.
package pkg

import (
	"math"
	"runtime"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	maxCentAccel       = 1.6671305  // m/s²  (avg 86 vehicles peak lateral acceleration  https://www.jsheld.com/uploads/PDFs/Lateral-and-Tangential-Accelerations-of-Left-Turning-Vehicles-from-Naturalistic-Observations.pdf)
	avgAccelAfterTurn  = 0.980665   // m/s² (0.10g avg acceleration phase 1 row 2  https://www.jsheld.com/insights/articles/a-naturalistic-study-of-vehicle-acceleration-and-deceleration-at-an-intersection)
	avgDecelBeforeTurn = -1.6671305 // m/s²  (0.17g avg decceleration phase 1 row 1  https://www.jsheld.com/insights/articles/a-naturalistic-study-of-vehicle-acceleration-and-deceleration-at-an-intersection)
)

func CalcResolution(l, lPrime, minResolution float64) float64 {
	delta := min(l, lPrime)
	delta = min(delta, minResolution)
	return delta
}

// CalcTurningSpeed computes max turning speed in m/s.
// l, lPrime: edge lengths in meters.
// turnAngleDeg: the turn angle in DEGREES (converted internally).
// https://ae.iti.kit.edu/download/turn_ch.pdf
func CalcTurningSpeed(l, lPrime, minResolution, turnAngleDeg float64) float64 {
	angleBetweenEdgesDeg := 180 - turnAngleDeg
	turnAngleRad := util.DegreeToRadians(angleBetweenEdgesDeg)
	delta := CalcResolution(l, lPrime, minResolution)
	radius := math.Tan(turnAngleRad/2) * delta / 2
	turningSpeed := math.Sqrt(maxCentAccel * radius)
	return turningSpeed
}

/* CalcTurningCost computes turn cost in seconds.
// maxV: turning speed (in m/s)
// vlimitTo, vlimitFrom: speed limit road segment from e, speed limit road segment to e'(in m/s).
vlimitFrom
// -----e---->
//           |
//           |
//			 e'	vlimitTo
// 			 |
// 			 |
// 			\/
// https://ae.iti.kit.edu/download/turn_ch.pdf
*/ // nolint: gofmt
func CalcTurningCost(turningSpeed, vlimitFrom, vlimitTo float64) float64 {

	distTraveledWhileDecel := (turningSpeed*turningSpeed - vlimitFrom*vlimitFrom) / (2 * avgDecelBeforeTurn)
	distTraveledWhileAccel := (vlimitTo*vlimitTo - turningSpeed*turningSpeed) / (2 * avgAccelAfterTurn)

	timeFullSpeedFrom := distTraveledWhileDecel / vlimitFrom
	timeFullSpeedTo := distTraveledWhileAccel / vlimitTo

	timeWhileDecel := (turningSpeed - vlimitFrom) / avgDecelBeforeTurn
	timeWhileAccel := (vlimitTo - turningSpeed) / avgAccelAfterTurn

	decCost := timeWhileDecel - timeFullSpeedFrom
	accCost := timeWhileAccel - timeFullSpeedTo
	// turning cost (travel time while accelerating with turn - travel time cost without any turns/full speed)?

	return decCost + accCost
}

// TurnType represents the type of turn at a junction.
type TurnType uint8

type VehicleTypeT uint8

const (
	BICYCLE    VehicleTypeT = iota
	FOOT       VehicleTypeT = iota
	MOTORCAR   VehicleTypeT = iota
	MOTORCYCLE VehicleTypeT = iota
	BUS        VehicleTypeT = iota
	HGV        VehicleTypeT = iota
	GOODS      VehicleTypeT = iota
	TAXI       VehicleTypeT = iota
	MINIBUS    VehicleTypeT = iota
)

var (
	NUM_CPU     = runtime.NumCPU()
	ProfileName = "car"
	RegionName  = ""

	// supported: bicycle, foot, motorcycle, bus, hgv, goods, taxi, minibus, motorcar
	VehicleType VehicleTypeT = MOTORCAR

	// lebih dari roda dua   // see land-based transportation https://wiki.openstreetmap.org/wiki/Key:access
	DoubleTrackedVehicleEnabled = true
	MotorizedVehicleEnabled     = true

	// vehicle or pedestrians. for oneway interperation for routing: https://wiki.openstreetmap.org/wiki/Key:oneway
	// see Land-based transportation: https://wiki.openstreetmap.org/wiki/Key:access
	// vehicle is all type of vehicle (bicycle, motorcar, bus, motorcycle, hgv, goods,taxi, minibus)
	IsVehicleEnabled bool = true

	VehicleTypeTag = map[VehicleTypeT]string{
		BICYCLE:    "bicycle",
		FOOT:       "foot",
		MOTORCAR:   "motorcar",
		MOTORCYCLE: "motorcycle",
		BUS:        "bus",
		HGV:        "hgv",
		GOODS:      "goods",
		TAXI:       "taxi",
		MINIBUS:    "minibus",
	}
	PublicServiceVehicles = map[VehicleTypeT]struct{}{
		BUS:  {},
		TAXI: {},
	}
)

func GetVehicleType(vehicleType string) VehicleTypeT {
	switch vehicleType {
	case "bicycle":
		return BICYCLE
	case "foot":
		return FOOT
	case "motorcar":
		return MOTORCAR
	case "motorcycle":
		return MOTORCYCLE
	case "bus":
		return BUS
	case "hgv":
		return HGV
	case "goods":
		return GOODS
	case "taxi":
		return TAXI
	case "minibus":
		return MINIBUS
	default:
		return MOTORCAR
	}
}

func GetIsDoubleTrackedVehicle() bool {
	switch VehicleType {
	case MOTORCYCLE, BICYCLE, FOOT:
		return false
	default:
		// bus, hgv, goods, taxi, minibus, motorcar
		return true
	}
}

func GetIsMotorizedVehicle() bool {
	switch VehicleType {
	case BICYCLE, FOOT:
		return false
	default:
		// motorcycle, bus, hgv, goods, taxi, minibus, motorcar
		return true
	}
}

func GetIsVehicle() bool {
	switch VehicleType {
	case FOOT:
		return false
	default:
		return true
	}

}

const (
	NONE        TurnType = iota // turn cost  = 0
	LEFT_TURN   TurnType = iota
	RIGHT_TURN  TurnType = iota
	STRAIGHT_ON TurnType = iota
	U_TURN      TurnType = iota
	NO_ENTRY    TurnType = iota // turn cost = INF_WEIGHT
)

const (
	INF_WEIGHT        float64 = 1e15
	INF_WEIGHT_INT    int64   = 1e15
	DUMMY_EDGE_LENGTH float64 = 1.0

	TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND float64 = 0.0
	ALTERNATIVE_ROUTE_SIMILARITY_THRESHOLD float64 = 92.0

	INVALID_LAT float64 = 91
	INVALID_LON float64 = 181
)

const (
	DEBUG = false
)

type OsmHighwayType uint8

// enum buat osm highway buat routing: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
// https://wiki.openstreetmap.org/wiki/Routing
// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla
const (
	MOTORWAY         OsmHighwayType = 0
	TRUNK            OsmHighwayType = 1
	PRIMARY          OsmHighwayType = 2
	SECONDARY        OsmHighwayType = 3
	TERTIARY         OsmHighwayType = 4
	RESIDENTIAL      OsmHighwayType = 5
	SERVICE          OsmHighwayType = 6
	UNCLASSIFIED     OsmHighwayType = 7
	MOTORWAY_LINK    OsmHighwayType = 8
	TRUNK_LINK       OsmHighwayType = 9
	PRIMARY_LINK     OsmHighwayType = 10
	SECONDARY_LINK   OsmHighwayType = 11
	TERTIARY_LINK    OsmHighwayType = 12
	LIVING_STREET    OsmHighwayType = 13
	ROAD             OsmHighwayType = 14
	TRACK            OsmHighwayType = 15
	MOTORROAD        OsmHighwayType = 16
	UNKNOWN          OsmHighwayType = 17
	RESIDENTIAL_LINK OsmHighwayType = 18
	PRIVATE          OsmHighwayType = 19
	FOOTWAY          OsmHighwayType = 20
	PEDESTRIAN       OsmHighwayType = 21
	STEPS            OsmHighwayType = 22
	FERRY            OsmHighwayType = 23
	CYCLEWAY         OsmHighwayType = 24 // https://wiki.openstreetmap.org/wiki/Tag:highway%3Dcycleway
	PATH             OsmHighwayType = 25
	BUS_GUIDEWAY     OsmHighwayType = 26

	INVALID_HIGHWAY OsmHighwayType = 255
)

// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
func GetHighwayType(roadType string) OsmHighwayType {
	switch roadType {
	case "motorway":
		return MOTORWAY
	case "trunk":
		return TRUNK
	case "primary":
		return PRIMARY
	case "secondary":
		return SECONDARY
	case "tertiary":
		return TERTIARY
	case "unclassified":
		return UNCLASSIFIED
	case "residential":
		return RESIDENTIAL
	case "residential_link":
		return RESIDENTIAL_LINK
	case "service":
		return SERVICE
	case "motorway_link":
		return MOTORWAY_LINK
	case "trunk_link":
		return TRUNK_LINK
	case "primary_link":
		return PRIMARY_LINK
	case "secondary_link":
		return SECONDARY_LINK
	case "tertiary_link":
		return TERTIARY_LINK
	case "living_street":
		return LIVING_STREET
	case "road":
		return ROAD
	case "track":
		return TRACK
	case "motorroad":
		return MOTORROAD
	case "private":
		return PRIVATE
	case "footway":
		return FOOTWAY
	case "pedestrian":
		return PEDESTRIAN
	case "steps":
		return STEPS
	case "ferry":
		return FERRY
	case "cycleway":
		return CYCLEWAY
	default:
		return UNKNOWN
	}
}

func GetHighwayTypeString(highwayType OsmHighwayType) string {
	switch highwayType {
	case MOTORWAY:
		return "motorway"
	case TRUNK:
		return "trunk"
	case PRIMARY:
		return "primary"
	case SECONDARY:
		return "secondary"
	case TERTIARY:
		return "tertiary"
	case UNCLASSIFIED:
		return "unclassified"
	case RESIDENTIAL:
		return "residential"
	case RESIDENTIAL_LINK:
		return "residential_link"
	case SERVICE:
		return "service"
	case MOTORWAY_LINK:
		return "motorway_link"
	case TRUNK_LINK:
		return "trunk_link"
	case PRIMARY_LINK:
		return "primary_link"
	case SECONDARY_LINK:
		return "secondary_link"
	case TERTIARY_LINK:
		return "tertiary_link"
	case LIVING_STREET:
		return "living_street"
	case ROAD:
		return "road"
	case TRACK:
		return "track"
	case MOTORROAD:
		return "motorroad"
	case PRIVATE:
		return "private"
	case FOOTWAY:
		return "footway"
	case PEDESTRIAN:
		return "pedestrian"
	case STEPS:
		return "steps"
	case FERRY:
		return "ferry"
	case CYCLEWAY:
		return "cycleway"
	default:
		return "unknown"
	}
}

var (
	WITH_TURN_COSTS bool = true
)

func OffTurnCost() {
	WITH_TURN_COSTS = false
}

func OnTurnCost() {
	WITH_TURN_COSTS = true
}

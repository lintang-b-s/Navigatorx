// Package pkg provides common constants and types used across the Navigatorx project.
package pkg

import (
	"math"
	"runtime"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	maxCentAccel       = 1.6671305  // m/s²  (0.17 g avg 86 vehicles peak lateral acceleration  https://www.jsheld.com/uploads/PDFs/Lateral-and-Tangential-Accelerations-of-Left-Turning-Vehicles-from-Naturalistic-Observations.pdf)
	avgAccelAfterTurn  = 2.353596   // m/s² (0.24g avg acceleration phase 2 row 2  https://www.jsheld.com/insights/articles/a-naturalistic-study-of-vehicle-acceleration-and-deceleration-at-an-intersection)
	avgDecelBeforeTurn = -1.6671305 // m/s²  (0.17g avg decceleration phase 1 row 2  https://www.jsheld.com/insights/articles/a-naturalistic-study-of-vehicle-acceleration-and-deceleration-at-an-intersection)
)

// CalcResolution calculates the minimum resolution delta between edge lengths.
func CalcResolution(l, lPrime, minResolution float64) float64 {
	delta := min(l, lPrime)
	delta = min(delta, minResolution)
	return delta
}

// CalcTurningSpeed computes max turning speed in m/s.
// l, lPrime: edge lengths in meters.
// turnAngleDeg: the turn angle in DEGREES (converted internally).
// limit on the turning speed section 3.2 Turning Speed: https://ae.iti.kit.edu/download/turn_ch.pdf
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
// Modification of Turn costs section 3.2 Turning Speed: https://ae.iti.kit.edu/download/turn_ch.pdf
*/ // nolint: gofmt
func CalcTurningCost(turningSpeed, vlimitFrom, vlimitTo float64) float64 {

	distTraveledWhileDecel := (turningSpeed*turningSpeed - vlimitFrom*vlimitFrom) / (2 * avgDecelBeforeTurn)
	distTraveledWhileAccel := (vlimitTo*vlimitTo - turningSpeed*turningSpeed) / (2 * avgAccelAfterTurn)

	timeFullSpeedFrom := distTraveledWhileDecel / vlimitFrom
	timeFullSpeedTo := distTraveledWhileAccel / vlimitTo

	timeWhileDecel := (turningSpeed - vlimitFrom) / avgDecelBeforeTurn
	decCost := timeWhileDecel - timeFullSpeedFrom

	timeWhileAccel := (vlimitTo - turningSpeed) / avgAccelAfterTurn
	accCost := timeWhileAccel - timeFullSpeedTo

	// turning cost (travel time while accelerating & decelerating with turn cost - travel time cost at full speed)?
	tc := max(0, decCost+accCost) // turn costs must be non negative. ini kemarin turn costs negative ternyata yang bikin rute nya kadang ada cycle
	return tc
}

// VehicleTypeT represents the type of vehicle used for routing.
type VehicleTypeT uint8

// Vehicle types supported for routing.
const (
	BICYCLE VehicleTypeT = iota
	FOOT
	MOTORCAR
	MOTORCYCLE
	BUS
	HGV
	GOODS
	TAXI
	MINIBUS
)

var (
	// NumCPU is the number of logical CPUs usable by the current process.
	NumCPU = runtime.NumCPU()
	// ProfileName specifies the routing profile name (e.g. "car").
	ProfileName = "car"
	// RegionName specifies the active map region name.
	RegionName = ""
	// WorkingDir specifies the working directory for file storage.
	WorkingDir = ""
	// TIPE specifies the execution mode.
	TIPE = ROUTER

	// VehicleType indicates the active vehicle type for routing (supported: bicycle, foot, motorcycle, bus, hgv, goods, taxi, minibus, motorcar).
	VehicleType = MOTORCAR

	// DoubleTrackedVehicleEnabled indicates whether the vehicle is double-tracked (more than two wheels).
	// see land-based transportation https://wiki.openstreetmap.org/wiki/Key:access
	DoubleTrackedVehicleEnabled = true
	// MotorizedVehicleEnabled indicates whether the vehicle is motorized.
	MotorizedVehicleEnabled = true

	// IsVehicleEnabled indicates whether the traveler is a vehicle or pedestrian for oneway interpretation.
	// for oneway interperation for routing: https://wiki.openstreetmap.org/wiki/Key:oneway
	// see Land-based transportation: https://wiki.openstreetmap.org/wiki/Key:access
	// vehicle is all type of vehicle (bicycle, motorcar, bus, motorcycle, hgv, goods, taxi, minibus)
	IsVehicleEnabled = true

	// VehicleTypeTag maps vehicle types to OSM tag strings.
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
	// PublicServiceVehicles identifies public service vehicle types.
	PublicServiceVehicles = map[VehicleTypeT]struct{}{
		BUS:  {},
		TAXI: {},
	}
)

// GetVehicleType returns the VehicleTypeT corresponding to a vehicle string.
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

// GetIsDoubleTrackedVehicle returns true if the current vehicle is double-tracked.
func GetIsDoubleTrackedVehicle() bool {
	switch VehicleType {
	case MOTORCYCLE, BICYCLE, FOOT:
		return false
	default:
		// bus, hgv, goods, taxi, minibus, motorcar
		return true
	}
}

// GetIsMotorizedVehicle returns true if the current vehicle is motorized.
func GetIsMotorizedVehicle() bool {
	switch VehicleType {
	case BICYCLE, FOOT:
		return false
	default:
		// motorcycle, bus, hgv, goods, taxi, minibus, motorcar
		return true
	}
}

// GetIsVehicle returns true if the current entity is a vehicle (not foot).
func GetIsVehicle() bool {
	switch VehicleType {
	case FOOT:
		return false
	default:
		return true
	}

}

// TurnType represents the type of turn at a junction.
type TurnType uint8

// TurnType constants representing turn directions.
const (
	NONE TurnType = iota // turn cost  = 0
	LeftTurn
	RightTurn
	StraightOn
	UTurn
	NoEntry // turn cost = INF_WEIGHT
)

// Weight and distance threshold constants.
const (
	InfWeight       float64 = 1e15
	InfWeightInt    int64   = 1e15
	DummyEdgeLength float64 = 1.0

	TrafficLightAdditionalWeightSecond  float64 = 0.0
	AlternativeRouteSimilarityThreshold float64 = 92.0

	InvalidLat float64 = 91
	InvalidLon float64 = 181
)

// DEBUG flag enables debug logging and assertions when true.
const (
	DEBUG = false
)

// OsmHighwayType represents OpenStreetMap highway tag classifications.
type OsmHighwayType uint8

// OSM highway type enum for routing: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
// https://wiki.openstreetmap.org/wiki/Routing
// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla
const (
	MOTORWAY        OsmHighwayType = 0
	TRUNK           OsmHighwayType = 1
	PRIMARY         OsmHighwayType = 2
	SECONDARY       OsmHighwayType = 3
	TERTIARY        OsmHighwayType = 4
	RESIDENTIAL     OsmHighwayType = 5
	SERVICE         OsmHighwayType = 6
	UNCLASSIFIED    OsmHighwayType = 7
	MotorwayLink    OsmHighwayType = 8
	TrunkLink       OsmHighwayType = 9
	PrimaryLink     OsmHighwayType = 10
	SecondaryLink   OsmHighwayType = 11
	TertiaryLink    OsmHighwayType = 12
	LivingStreet    OsmHighwayType = 13
	ROAD            OsmHighwayType = 14
	TRACK           OsmHighwayType = 15
	MOTORROAD       OsmHighwayType = 16
	UNKNOWN         OsmHighwayType = 17
	ResidentialLink OsmHighwayType = 18
	PRIVATE         OsmHighwayType = 19
	FOOTWAY         OsmHighwayType = 20
	PEDESTRIAN      OsmHighwayType = 21
	STEPS           OsmHighwayType = 22
	FERRY           OsmHighwayType = 23
	CYCLEWAY        OsmHighwayType = 24 // https://wiki.openstreetmap.org/wiki/Tag:highway%3Dcycleway
	PATH            OsmHighwayType = 25
	BusGuideway     OsmHighwayType = 26

	InvalidHighway OsmHighwayType = 255
)

// GetHighwayType maps an OSM road type string to an OsmHighwayType.
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
		return ResidentialLink
	case "service":
		return SERVICE
	case "motorway_link":
		return MotorwayLink
	case "trunk_link":
		return TrunkLink
	case "primary_link":
		return PrimaryLink
	case "secondary_link":
		return SecondaryLink
	case "tertiary_link":
		return TertiaryLink
	case "living_street":
		return LivingStreet
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

// GetHighwayTypeString returns the string representation of an OsmHighwayType.
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
	case ResidentialLink:
		return "residential_link"
	case SERVICE:
		return "service"
	case MotorwayLink:
		return "motorway_link"
	case TrunkLink:
		return "trunk_link"
	case PrimaryLink:
		return "primary_link"
	case SecondaryLink:
		return "secondary_link"
	case TertiaryLink:
		return "tertiary_link"
	case LivingStreet:
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

/*
turn lanes constants. menyimpan semua tipe tag turn:lanes values
https://wiki.openstreetmap.org/wiki/Key:turn#Values
*/

// TurnLaneType represents turn lane bit flags.
type TurnLaneType uint16

// Turn lane bit flag constants.
const (
	LaneNoEntry              = 0
	LaneNone    TurnLaneType = 1 << iota // none or ""
	LaneStraight
	LaneSharpLeft
	LaneLeft
	LaneSlightLeft
	LaneSlightRight
	LaneRight
	LaneSharpRight
	LaneUturn
	LaneMergeToLeft
	LaneMergeToRight
)

// FileType represents the operational mode for file processing.
type FileType int

// Application mode constants.
const (
	ROUTER FileType = iota
	EVAL
	TEST
)

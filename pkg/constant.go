package pkg

import (
	"runtime"
)

// enum of turn_type
type TurnType uint8

type VehicleTypeT uint8

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
	NUM_CPU     = runtime.NumCPU()
	ProfileName = "car"
	// supported: bicycle, foot, motorcycle, bus, hgv, goods, taxi, minibus, motorcar
	VehicleType VehicleTypeT = MOTORCAR

	// lebih dari roda dua   // see land-based transportation https://wiki.openstreetmap.org/wiki/Key:access
	DoubleTrackedVehicle = true
	MotorizedVehicle     = true

	// vehicle or pedestrians. for oneway interperation for routing: https://wiki.openstreetmap.org/wiki/Key:onewayf
	// see Land-based transportation: https://wiki.openstreetmap.org/wiki/Key:access
	// vehicle is all type of vehicle (bicycle, motorcar, bus, motorcycle, hgv, goods,taxi, minibus)
	IsVehicle bool = true

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
		BUS:  struct{}{},
		TAXI: struct{}{},
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
	LEFT_TURN TurnType = iota
	RIGHT_TURN
	STRAIGHT_ON
	U_TURN
	NO_ENTRY
	NONE
)

const (
	INF_WEIGHT     float64 = 1e15
	INF_WEIGHT_INT         = 1e15

	TRAFFIC_LIGHT_ADDITIONAL_WEIGHT_SECOND  = 0.0
	ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD = 92.0

	INVALID_LAT = 91
	INVALID_LON = 181
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

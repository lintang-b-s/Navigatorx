package pkg

// enum of turn_type
type TurnType uint8

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
	TRAFFIC_LIGHT_PENALTY_SP_SECOND         = 0.0
	ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD = 90.0
	NERF_MAXSPEED_OSM                       = 0.9
	EPSILON_IMAI_IRI_APPROX_PWL             = 0.01 // 1.0%
	EPSILON_IMAI_IRI_APPROX_PWL_SHORTCUTS   = 0.01 // 1.0%
)

const (
	DEBUG = false
)

type OsmHighwayType uint8

// enum buat osm highway buat routing: https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Telenav
const (
	MOTORWAY       OsmHighwayType = 0
	TRUNK          OsmHighwayType = 1
	PRIMARY        OsmHighwayType = 2
	SECONDARY      OsmHighwayType = 3
	TERTIARY       OsmHighwayType = 4
	RESIDENTIAL    OsmHighwayType = 5
	SERVICE        OsmHighwayType = 6
	UNCLASSIFIED   OsmHighwayType = 7
	MOTORWAY_LINK  OsmHighwayType = 8
	TRUNK_LINK     OsmHighwayType = 9
	PRIMARY_LINK   OsmHighwayType = 10
	SECONDARY_LINK OsmHighwayType = 11
	TERTIARY_LINK  OsmHighwayType = 12
	LIVING_STREET  OsmHighwayType = 13
	ROAD           OsmHighwayType = 14
	TRACK          OsmHighwayType = 15
	MOTORROAD      OsmHighwayType = 16
	UNKNOWN        OsmHighwayType = 17
)


func GetHighwayType(roadType string) OsmHighwayType {
	switch roadType {
	case "motorway":
		return  MOTORWAY
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
	default:
		return UNKNOWN
	}
}
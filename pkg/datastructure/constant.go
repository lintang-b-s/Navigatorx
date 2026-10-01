package datastructure

import "math"

const (
	INVALID_VERTEX_ID        Index = 10e8
	INVALID_SEGMENT_ID       Index = 10e8 + 1
	INVALID_PARALLEL_EDGE_ID Index = 10e8 + 3
	INVALID_ENTRY_POINT      Index = 10e8 + 4
	INVALID_EXIT_POINT       Index = 10e8 + 5

	BASE_VERTICES_SIZE                      = 2000
	OVERLAY_VERTICES_SIZE                   = 32 // 2 ini kecil aja, biar gak consume memory banyak pas load test
	OVERLAY_CELL_SIZE                       = 16
	INVALID_OSM_WAY_ID               int64  = 1<<34 - 1
	INITIAL_BIT_VECTOR_SIZE                 = 1000
	DEFAULT_BIT_SIZE_OSM_WAY_ID             = 34
	BIT_SIZE_OSM_NODE_ID                    = 34
	INITIAL_APPROX_SEGMENT_SIZE             = 1000
	INVALID_STREET_NAME_ID           uint32 = math.MaxUint32
	INITIAL_REACHIBILITY_BITSET_SIZE        = 10
	GeohashBits                             = 6 * 5
)

type IndexStorageType int

const (
	TWO_LEVEL_STORAGE IndexStorageType = iota
	ARRAY_STORAGE
	MAP_STORAGE
)

type SegmentFlagType uint8

const (
	FlagParallel             SegmentFlagType = 1 << iota //  flag yang nandain kalau this road segment adalah parallel via-way yang termasuk dalam via-ways turn restrictions
	FlagJunctionHead                                     // flag yang nandain kalo head dari road segment adalah junction node
	FlagJunctionTail                                     // flag yang nandain kalo tail dari road segment adalah junction node
	FlagContainsTrafficLight                             // flag yang nandain di road segment ini terdapat traffic light/bangjo
	FlagIsRoundabout                                     // flag yang nandain road segment ini adalah bundaran
	FlagIsCurved                                         // flag yang nandain road segment ini curved/gak lurus straight line in mercator projected 2d coordinate
	FlagIsForward                                        // flag yang nandain road segment ini arahnya forward (dari list of nodes data osm way). by default true kalau one-way road.
	FlagIsBackward                                       //flag yang nandain road segment ini arahnya backward (dari list of nodes data osm way)
)

type NodeFlagType uint8

const (
	FlagNodeTrafficLight NodeFlagType = 1 << iota
)

const (
	INVALID_LK_TABLE_ID Index = Index(math.MaxUint32)
)

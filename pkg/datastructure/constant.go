package datastructure

import "math"

const (
	workersNum                     = 5
	INVALID_VERTEX_ID        Index = 10e8
	INVALID_EDGE_ID          Index = 10e8 + 1
	INVALID_EDGE_INFO_ID     Index = 10e8 + 2
	INVALID_PARALLEL_EDGE_ID Index = 10e8 + 3
	INVALID_ENTRY_POINT      Index = 10e8 + 4
	INVALID_EXIT_POINT       Index = 10e8 + 5

	OVERLAY_INFO_SIZE                  = 32 // 2 ini kecil aja, biar gak consume memory banyak pas load test
	OVERLAY_CELL_INFO_SIZE             = 16
	INVALID_OSM_WAY_ID          int64  = 1<<34 - 1
	INITIAL_BIT_VECTOR_SIZE            = 1000
	DEFAULT_BIT_SIZE_OSM_WAY_ID        = 34
	BIT_SIZE_OSM_NODE_ID               = 34
	INITIAL_APPROX_EDGE_SIZE           = 1000
	INVALID_STREET_NAME_ID      uint32 = math.MaxUint32
)

type QueryInfoStorageType int

const (
	TWO_LEVEL_STORAGE QueryInfoStorageType = iota
	ARRAY_STORAGE
	MAP_STORAGE
)

const (
	FlagDummy uint8 = 1 << iota
	FlagParallel
	FlagJunctionHead         // flag yang nandain kalo head dari edge adalah junction node
	FlagJunctionTail         // flag yang nandain kalo tail dari edge adalah junction node
	FlagContainsTrafficLight // flag yang nandain di edge/road segment ini terdapat traffic light/bangjo
)

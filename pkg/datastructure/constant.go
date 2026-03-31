package datastructure

const (
	workersNum                       = 5
	INVALID_VERTEX_ID          Index = 10e8
	INVALID_EDGE_ID            Index = 10e8 + 1
	INVALID_ENTRY_POINT        Index = 10e8 + 2
	OVERLAY_INFO_SIZE                = 128 // 2 ini kecil aja, biar gak consume memory banyak pas load test
	OVERLAY_CELL_INFO_SIZE           = 64
	INVALID_OSM_WAY_ID               = -1
	DEFAULT_EDGE_GEOMETRY_SIZE       = 4
)

type QueryInfoStorageType int

const (
	TWO_LEVEL_STORAGE QueryInfoStorageType = iota
	ARRAY_STORAGE
	MAP_STORAGE
)

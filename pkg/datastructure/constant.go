package datastructure

const (
	workersNum                   = 5
	INVALID_VERTEX_ID      Index = 10e8
	INVALID_EDGE_ID        Index = 10e8 + 1
	INVALID_ENTRY_POINT    int = 10e8 + 2
	OVERLAY_INFO_SIZE            = 1024
	OVERLAY_CELL_INFO_SIZE       = 256
)

type QueryInfoStorageType int

const (
	TWO_LEVEL_STORAGE QueryInfoStorageType = iota
	ARRAY_STORAGE
	MAP_STORAGE
)

package spatialindex

const (
	MAX_CANDIDATES                   = 35
	MAX_CANDIDATES_MAP_MATCHING      = 70
	FILTERED_CANDIDATES_MAP_MATCHING = 25
	JUNCTION_HEAD_FLAG               = uint64(1) << 63
	JUNCTION_TAIL_FLAG               = uint64(1) << 62
)

package spatialindex

const (
	MAX_CANDIDATES                   = 35
	MAX_CANDIDATES_MAP_MATCHING      = 70
	FILTERED_CANDIDATES_MAP_MATCHING = 25
)

const (
	JUNCTION_FORWARD_HEAD_FLAG uint8 = 1 << iota
	JUNCTION_FORWARD_TAIL_FLAG
	JUNCTION_BACKWARD_HEAD_FLAG 
	JUNCTION_BACKWARD_TAIL_FLAG
	BIDIRECTIONAL // this road-segment contained in bidirectional (two-way or oneway=false) osm-way
)

package routing

const (
	EDGE_OFFSET                    = 1000
	UPPERBOUND_SHORTEST_PATH       = 1.0
	UNPACK_OVERLAY_OFFSET      int = 31 // set 32-th bit on
	PATH_UNPACKER_WORKERS      int = 3
	ALTERNATIVE_ROUTES_WORKERS     = 4
	USE_CRP_FOR_ALTERNATIVE_ROUTES bool = true
)

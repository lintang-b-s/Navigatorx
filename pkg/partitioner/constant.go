package partitioner

const (
	INVALID_LEVEL                            = 9e9
	ARTIFICIAL_SOURCE_ID                     = int32(2147483646)
	ARTIFICIAL_SINK_ID                       = int32(2147483647)
	INERTIAL_FLOW_ITERATION                  = 30
	INERTIAL_FLOW_WORKERS                    = INERTIAL_FLOW_ITERATION
	BISECTION_WORKERS                        = 40
	LEVEL_WORKERS                            = 30
	INVALID_PARTITION_ID                     = -1
	SOURCE_SINK_RATE                         = 0.2 // paling bagus :jumlah cut edges paling sedikit -> jumlah shortcuts paling sedikit  &&  paling cepet selesai. dibanding ( 0.025, 0.05, 0.1, 0.15, 0.25, 0.3)
	INERTIAL_FLOW_ITERATION_LARGE_GRAPH      = 2
	USE_RANDOMIZED_SELECT               bool = true
)

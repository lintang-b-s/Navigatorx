package routing

const (
	UNPACK_OVERLAY_OFFSET int = 31 // set 32-th bit on
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	PATH_UNPACKER_ALTERNATIVE_WORKERS int = 2
	PATH_UNPACKER_WORKERS             int = 2

	ALTERNATIVE_ROUTES_WORKERS = 2
	// 8 goroutines per alternative routes query
	// 2 goroutines per sp route  query
	// todo: worker pool size sesuain logical cpu cores pc yang run routing engine
	MAX_FILTERED_ALTERNATIVE_ROUTE_CANDIDATES = 8
)

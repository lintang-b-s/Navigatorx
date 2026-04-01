package routing

const (
	UNPACK_OVERLAY_OFFSET int = 31 // set 32-th bit on
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	PACKED_PATH_SIZE = 64

	UNPACKED_PATH_SIZE = 1024

	UNPACKER_EDGE_PATH_SIZE    = 32
	UNPACKER_OVERLAY_PATH_SIZE = 32
)

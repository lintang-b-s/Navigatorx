package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

const (
	UNPACK_OVERLAY_OFFSET int = 31 // set 32-th bit on
	// https://goperf.dev/01-common-patterns/worker-pool/#worker-count-and-cpu-cores
	PACKED_PATH_SIZE = 64

	UNPACKED_PATH_SIZE = 64

	UNPACKER_OVERLAY_PATH_SIZE    = 8
	VIA_VERTICES_INITIAL_CAPACITY = 16

	CUSTOMIZER_UPDATER_TIMER_SECONDS      = 1 * time.Second
	CONDITIONAL_RESTRICTION_TIMER_SECONDS = 180 * time.Second
	MAX_RETRY                             = 5
	TRAVERSE_FROM_VIA_VERTEX_COUNT        = 200
)

var (
	EmptyCoords            = da.NewCoordinatesWithCap(0)
	EmptyIndexSet          = []da.Index{}
	EmptyCost              = []float64{}
	EmptyCoordsRoutes      = []*da.Coordinates{da.NewCoordinatesWithCap(0)}
	EmptyIndexSetRoutes    = [][]da.Index{}
	EmptyPolylines         = []string{}
	EmptyDrivingDirections = [][]da.DrivingDirection{}
)

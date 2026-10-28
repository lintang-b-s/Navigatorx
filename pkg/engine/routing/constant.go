package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

const (
	// UnpackOverlayOffset is the unpack overlay offset value.
	UnpackOverlayOffset int = 31 // set 32-th bit on
	// PackedPathSize is the packed path size value.
	PackedPathSize int = 64

	// UnpackedPathSize is the unpacked path size value.
	UnpackedPathSize int = 64

	// UnpackerOverlayPathSize is the unpacker overlay path size value.
	UnpackerOverlayPathSize int = 8

	// ViaVerticesInitialCapacity is the via vertices initial capacity value.
	ViaVerticesInitialCapacity int = 16

	// CustomizerUpdaterTimer is the customizer updater timer seconds value.
	CustomizerUpdaterTimer = 1 * time.Second

	// ConditionalRestrictionTimer is the conditional restriction timer seconds value.
	ConditionalRestrictionTimer = 180 * time.Second

	// MaxRetry is the max retry value.
	MaxRetry = 5

	// TraverseFromViaVertexCount is the traverse from via vertex count value.
	TraverseFromViaVertexCount = 100

	// RelaxedUBSEpsilonStart is the relaxed ubsepsilon start value.
	RelaxedUBSEpsilonStart = 1.05

	// RelaxedUBSEpsilonMax is the relaxed ubsepsilon max value.
	RelaxedUBSEpsilonMax = 2.0
)

var (
	// EmptyCoords is the empty coords value.
	// EmptyIndexSet is the empty index set value.
	EmptyCoords = da.NewCoordinatesWithCap(0)
	// EmptyIndexSet is the empty index set value.
	EmptyIndexSet = []da.Index{}

	// EmptyCost is the empty cost value.
	EmptyCost = []float64{}

	// EmptyCoordsRoutes is the empty coords routes value.
	EmptyCoordsRoutes = []*da.Coordinates{da.NewCoordinatesWithCap(0)}

	// EmptyIndexSetRoutes is the empty index set routes value.
	EmptyIndexSetRoutes = [][]da.Index{}

	// EmptyPolylines is the empty polylines value.
	EmptyPolylines = []string{}

	// EmptyDrivingDirections is the empty driving directions value.
	EmptyDrivingDirections = [][]da.DrivingDirection{}
)

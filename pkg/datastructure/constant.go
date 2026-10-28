package datastructure

import "math"

const (
	// InvalidVertexId is the invalid vertex id value.
	InvalidVertexId Index = 2e9
	// InvalidSegmentId is the invalid segment id value.
	InvalidSegmentId Index = 2e9 + 1
	// InvalidParallelEdgeId is the invalid parallel edge id value.
	InvalidParallelEdgeId Index = 2e9 + 3

	// InvalidEntryPoint is the invalid entry point value.
	InvalidEntryPoint Index = 2e9 + 4

	// InvalidExitPoint is the invalid exit point value.
	InvalidExitPoint Index = 2e9 + 5

	// BaseVerticesSize is the base vertices size value.
	BaseVerticesSize = 2000

	// OverlayVerticesSize is the overlay vertices size value.
	OverlayVerticesSize = 32 // 2 ini kecil aja, biar gak consume memory banyak pas load test

	// TouchedVerticesSize is the touched vertices size value.
	TouchedVerticesSize = 1024

	// OverlayCellSize is the overlay cell size value.
	OverlayCellSize = 16

	// InvalidOSMWayId is the invalid osmway id value.
	InvalidOSMWayId int64 = 1<<34 - 1

	// InitialBitVectorSize is the initial bit vector size value.
	InitialBitVectorSize = 1000

	// DefaultBitSizeOSMWayId is the default bit size osmway id value.
	DefaultBitSizeOSMWayId = 34

	// BitSizeOSMNodeId is the bit size osmnode id value.
	BitSizeOSMNodeId = 34

	// InitialApproxSegmentSize is the initial approx segment size value.
	InitialApproxSegmentSize = 1000

	// InvalidStreetNameId is the invalid street name id value.
	InvalidStreetNameId uint32 = math.MaxUint32

	// InitialReachibilityBitsetSize is the initial reachibility bitset size value.
	InitialReachibilityBitsetSize = 10

	// GeohashBits is the geohash bits value.
	GeohashBits = 6 * 5
)

// IndexStorageType represents index storage type.
type IndexStorageType int

const (
	// ArrayStorageMode is the array storage value.
	// MapStorageMode is the map storage value.
	ArrayStorageMode IndexStorageType = iota
	// MapStorageMode is the map storage value.
	MapStorageMode
)

// SegmentFlagType represents segment flag type.
type SegmentFlagType uint8

const (

	// FlagParallel is the flag parallel value.
	FlagParallel SegmentFlagType = 1 << iota //  flag yang nandain kalau this road segment adalah parallel via-way yang termasuk dalam via-ways turn restrictions
	// FlagJunctionHead is the flag junction head value.
	FlagJunctionHead // flag yang nandain kalo head dari road segment adalah junction node

	// FlagJunctionTail is the flag junction tail value.
	FlagJunctionTail // flag yang nandain kalo tail dari road segment adalah junction node

	// FlagContainsTrafficLight is the flag contains traffic light value.
	FlagContainsTrafficLight // flag yang nandain di road segment ini terdapat traffic light/bangjo

	// FlagIsRoundabout is the flag is roundabout value.
	FlagIsRoundabout // flag yang nandain road segment ini adalah bundaran

	// FlagIsCurved is the flag is curved value.
	FlagIsCurved // flag yang nandain road segment ini curved/gak lurus straight line in mercator projected 2d coordinate

	// FlagIsForward is the flag is forward value.
	FlagIsForward // flag yang nandain road segment ini arahnya forward (dari list of nodes data osm way). by default true kalau one-way road.

	// FlagIsBackward is the flag is backward value.
	FlagIsBackward // flag yang nandain road segment ini arahnya backward (dari list of nodes data osm way)
)

// NodeFlagType represents node flag type.
type NodeFlagType uint8

const (
	// FlagNodeTrafficLight is the flag node traffic light value.
	FlagNodeTrafficLight NodeFlagType = 1 << iota
)

const (
	// InvalidLkTableId is the invalid lk table id value.
	InvalidLkTableId Index = Index(math.MaxUint32)
)

package datastructure

import (
	"math"
	"sync"
)

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

	// CRPMaxExploredSize is the approximation of maximum number of explored vertices in CRP p2p shortest path query.
	CRPMaxExploredSize Index = 2000

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
	// ArrayCRPStorageMode is the array CRP storage value.
	// tailored to a Customizable Route Planning query (or its variants) that tends to explored far fewer vertices than the total number of vertices in the graph.
	ArrayCRPStorageMode IndexStorageType = iota
	// ArrayStorageMode is array storage value.
	ArrayStorageMode
	// MapStorageMode is the map storage value.
	MapStorageMode
	// ArrayStorageUniMode is the array storage value for Combination of CRP and unidirectional ALT.
	ArrayStorageUniMode
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
	indexesPoolLen         = 11
)

var (
	// inspired by http/2 sync.pool example in: https://github.com/golang/go/blob/7e394a2/src/net/http/h2_bundle.go#L998-L1043
	indexSizeClasses = [indexesPoolLen]int{
		1 << 5,
		1 << 6,
		1 << 7,
		1 << 8,
		1 << 9,
		1 << 10,
		1 << 11,
		1 << 12,
		1 << 13,
		1 << 14,
		1 << 15,
	}

	indexesPools [indexesPoolLen]sync.Pool = [indexesPoolLen]sync.Pool{
		{New: func() interface{} {
			s := make([]Index, 0, 1<<5)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<6)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<7)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<8)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<9)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<10)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<11)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<12)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<13)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<14)
			return s
		}},
		{New: func() interface{} {
			s := make([]Index, 0, 1<<15)
			return s
		}},
	}
)

// GetIndexes get slice of index from the pool
func GetIndexes(capacity int) []Index {
	for i := range indexesPoolLen {
		if capacity <= indexSizeClasses[i] {
			s := indexesPools[i].Get().([]Index)
			s = s[:0]
			return s
		}
	}
	s := make([]Index, 0, capacity)
	return s
}

// PutIndexes put slice of index to the pool
func PutIndexes(s []Index) {
	c := cap(s)
	for i := range indexesPoolLen {
		if c == indexSizeClasses[i] {
			indexesPools[i].Put(s) // nolint:staticcheck
			return
		}
	}
}

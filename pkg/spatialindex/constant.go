package spatialindex

const (
	// MaxCandidates is the max candidates value.
	MaxCandidates = 35
	// MaxCandidatesMapMatching is the max candidates map matching value.
	MaxCandidatesMapMatching = 70
	// FilteredCandidatesMapMatching is the filtered candidates map matching value.
	FilteredCandidatesMapMatching = 25
	outputEvery                   = 5
	maxEntries                    = 64
	leafBytesSizes                = 9
)

const (
	// JunctionForwardHeadFlag is the junction forward head flag value.
	JunctionForwardHeadFlag uint8 = 1 << iota
	// JunctionForwardTailFlag is the junction forward tail flag value.
	JunctionForwardTailFlag

	// JunctionBackwardHeadFlag is the junction backward head flag value.
	JunctionBackwardHeadFlag

	// JunctionBackwardTailFlag is the junction backward tail flag value.
	JunctionBackwardTailFlag

	// BIDIRECTIONAL is the bidirectional value.
	BIDIRECTIONAL // this road-segment contained in bidirectional (two-way or oneway=false) osm-way
)

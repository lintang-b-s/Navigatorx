package guidance

import da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

const (
	// RelativeBearingUTurn is the relative bearing uturn value.
	RelativeBearingUTurn float64 = 100.0
	// MergeRelativeBearing is the merge relative bearing value.
	MergeRelativeBearing float64 = 40.0

	// KeepLeftRightALTTurnRelativeBearing is the keep left right altturn relative bearing value.
	KeepLeftRightALTTurnRelativeBearing float64 = 20.0

	// ContinueALTTurnRelativeBearing is the continue altturn relative bearing value.
	ContinueALTTurnRelativeBearing float64 = 8.6

	// ContinueALTCurrentRelativeBearing is the continue  altcurrent relative bearing value.
	ContinueALTCurrentRelativeBearing float64 = 7

	// UTurnRelativeBearingMin is the uturn relative bearing min value.
	UTurnRelativeBearingMin float64 = 160

	// UTurnRelativeBearingMax is the uturn relative bearing max value.
	UTurnRelativeBearingMax float64 = 185

	ignoreTurnSignCacheVal = uint64(da.IGNORE) | uint64(da.InvalidStreetNameId)<<8
)

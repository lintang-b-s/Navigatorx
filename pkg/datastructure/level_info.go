package datastructure

import "github.com/lintang-b-s/navigatorx-crp/pkg/util"

type LevelInfo struct {
	offset []uint8 // offset of each level in the bitpacked cell numbers
}

func NewLevelInfo(offset []uint8) *LevelInfo {
	return &LevelInfo{offset: offset}
}

func (li *LevelInfo) GetCellNumberOnLevel(l uint8, cellNumber Pv) Index {
	return Index((cellNumber & ^(^Pv(0) << Pv(li.offset[l]))) >> li.offset[l-1])
}

// GetHighestDifferingLevel. get the highest level(1-indexed) where two cell numbers differ
func (li *LevelInfo) GetHighestDifferingLevel(c1, c2 Pv) uint8 {
	diff := c1 ^ c2
	if diff == 0 {
		return 0
	}

	for l := len(li.offset) - 1; l > 0; l-- {
		if diff>>Pv(li.offset[l-1]) > 0 {
			return uint8(l)
		}
	}
	return 0
}

/*
GetQueryLevel. Customizable Route Planning in Road Networks, Daniel Delling, et al. Page 14

define its query level lst (v) as the highest level such that v is not
at the same cell as s or t. Equivalently, lst(v) is the maximum i such that ci(v) ∩ {s, t} = ∅.
To compute lst (v), we first determine the most significant differing bit of pv(s) and pv(v). (Recall that
pv(v) encodes the cell number of v on each level.) This bit indicates the topmost level ls(v) in which they
differ. We do the same for pv(t) and pv(v) to determine lt(v). The minimum of ls (v) and lt (v) is lst (v).
*/
func (li *LevelInfo) GetQueryLevel(sCellNumber, tCellNumber, vCellNumber Pv) uint8 {
	l_sv := li.GetHighestDifferingLevel(sCellNumber, vCellNumber)
	l_tv := li.GetHighestDifferingLevel(tCellNumber, vCellNumber)

	return uint8(util.MinInt(int(l_sv), int(l_tv)))
}

// get cell number.
func (li *LevelInfo) TruncateToLevel(cellNumber Pv, level uint8) Pv {
	// shift right to remove bits below the given level (but still contains bits above the level)
	return cellNumber >> Pv(li.offset[level-1])
}

func (li *LevelInfo) GetLevelCount() int {
	return len(li.offset) - 1
}

func (li *LevelInfo) GetOffsets() []uint8 {
	return li.offset
}

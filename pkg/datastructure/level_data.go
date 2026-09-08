package datastructure

import (
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type LevelData struct {
	offset []uint8 // offset of each level in the bitpacked cell numbers
}

func NewLevelData(offset []uint8) *LevelData {
	return &LevelData{offset: offset}
}

// off bits above the given level
func (li *LevelData) OffUpperBit(l uint8, cellNumber Pv) Pv {
	return (cellNumber & ^(^Pv(0) << Pv(li.offset[l])))
}

func (li *LevelData) GetCellNumberOnLevel(l uint8, cellNumber Pv) Pv {
	withoutUpperBit := li.OffUpperBit(l, cellNumber)
	return withoutUpperBit >> li.offset[l-1]
}

// GetHighestDifferingLevel. get the highest level(1-indexed) where two cell numbers differ
func (li *LevelData) GetHighestDifferingLevel(c1, c2 Pv) uint8 {
	diff := c1 ^ c2
	if diff == 0 {
		return 0
	}

	for l := len(li.offset) - 1; l > 0; l-- {
		diffInLevel := diff >> Pv(li.offset[l-1])
		if diffInLevel > 0 {

			return uint8(l)
		}
	}
	return 0
}

/*
highest level s.t. vertex v is not at the same cell as s or t.
*/
func (li *LevelData) GetQueryLevel(sCellNumber, tCellNumber, vCellNumber Pv) uint8 {
	l_sv := li.GetHighestDifferingLevel(sCellNumber, vCellNumber)
	l_tv := li.GetHighestDifferingLevel(tCellNumber, vCellNumber)

	return uint8(util.MinInt(int(l_sv), int(l_tv)))
}

// get cell number. level is 1-indexed
func (li *LevelData) TruncateToLevel(cellNumber Pv, level uint8) Pv {
	// shift right to remove bits below the given level (but still contains bits above the level)
	return cellNumber >> Pv(li.offset[level-1])
}

func (li *LevelData) GetLevelCount() int {
	return len(li.offset) - 1
}

func (li *LevelData) GetOffsets() []uint8 {
	return li.offset
}

// l is 1-indexed level
func (li *LevelData) GetOffsetInLevel(l int) uint8 {
	return li.offset[l-1]
}

package datastructure

type LevelInfo struct {
	offset []uint8 // offset of each level in the bitpacked cell numbers
}

func NewLevelInfo(offset []uint8) *LevelInfo {
	return &LevelInfo{offset: offset}
}

func (li *LevelInfo) GetCellNumberOnLevel(l uint8, cellNumber Pv) Index {
	return Index((cellNumber & ^(^Pv(0) << Pv(li.offset[l]))) >> li.offset[l-1])
}

// GetHighestDifferingLevel. get the highest level where two cell numbers differ
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

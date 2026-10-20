package datastructure

// adapted from: https://github.com/michaelwegner/CRP/blob/master/datastructures/LevelInfo.h

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

reference: 1. Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable
Route Planning in Road Networks. Transportation Science, No. 2, Volume 51,
pages 566-591. Available at: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

the explanation below taken from Delling et al. (2015) ref [1]:

"Given any vertex v, define its query level lst(v) as the highest level such that v is not
at the same cell as s or t. Equivalently, lst(v) is the maximum i such that ci(v) ∩ {s, t} = ∅." (Delling et al. (2015) )

"To compute lst(v), we first determine the most significant differing bit of pv(s) and pv(v). (Recall that pv(v) encodes the cell number of v on each level.) This bit indicates the topmost level ls(v) in which they
differ. We do the same for pv(t) and pv(v) to determine lt(v). The minimum of ls(v) and lt(v) is lst(v)." (Delling et al. (2015) )
*/
func (li *LevelData) GetQueryLevel(sCellNumber, tCellNumber, vCellNumber Pv) uint8 {
	l_sv := li.GetHighestDifferingLevel(sCellNumber, vCellNumber)
	l_tv := li.GetHighestDifferingLevel(tCellNumber, vCellNumber)

	return min(l_sv, l_tv)
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

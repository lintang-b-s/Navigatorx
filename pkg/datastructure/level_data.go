package datastructure

// adapted from: https://github.com/michaelwegner/CRP/blob/master/datastructures/LevelInfo.h

import "math/bits"

// LevelData stores the bit offsets of each level in the bitpacked cell numbers and fucntions to computing query level lst(v) (see the comment on QueryLevel()).
type LevelData struct {
	offset     []uint8   // offset of each level in the bitpacked cell numbers (see buildMLP() in partitioner_io.go)
	bitToLevel [65]uint8 // maps from most significant differing bit of two number (bits.Len64(a^b)) to highest differing level
}

// NewLevelData creates a new LevelData with the given level offsets.
func NewLevelData(offset []uint8) *LevelData {
	ld := &LevelData{offset: offset}
	ld.initBitToLevel()
	return ld
}

// initBitToLevel precomputes maps from most significant differing bit of two number (bits.Len64(a^b)) to highest differing level.
func (li *LevelData) initBitToLevel() {
	li.bitToLevel[0] = 0
	for bit := 1; bit <= 64; bit++ {
		var lvl uint8
		for l := len(li.offset) - 1; l > 0; l-- {
			if uint8(bit-1) >= li.offset[l-1] {
				lvl = uint8(l)
				break
			}
		}
		li.bitToLevel[bit] = lvl
	}
}

// OffUpperBit clears bits above the given level-l (1-indexed).
func (li *LevelData) OffUpperBit(l uint8, cellNumber Pv) Pv {
	return (cellNumber & ^(^Pv(0) << Pv(li.offset[l])))
}

// GetCellNumberOnLevel returns cell number on level-l (1-indexed).
func (li *LevelData) GetCellNumberOnLevel(cellNumber Pv, l uint8) Pv {
	withoutUpperBit := li.OffUpperBit(l, cellNumber)
	return li.TruncateToLevel(withoutUpperBit, l)
}

// HighestDifferingLevel gets the highest level (1-indexed) where two cell numbers differ.
// first, by computing the most significant differing bit of cell numbers c1 and c2. this done by bits.Len64(uint64(c1^c2)).
// then use the precomputed map li.bitToLevel[bits.Len64(uint64(c1^c2))], which maps to highest differing level of cellnumbers c1 and c2.
func (li *LevelData) HighestDifferingLevel(c1, c2 Pv) uint8 {
	return li.bitToLevel[bits.Len64(uint64(c1^c2))]
}

// QueryLevel returns the highest level (1-indexed) s.t. vertex v is not at the same cell as s or t.
//
// Reference: Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable
// Route Planning in Road Networks. Transportation Science, No. 2, Volume 51,
// pages 566-591. Available at: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
//
// The explanation below taken from Delling et al. (2015):
//
// "Since the number of levels and the number of cells per supercell are small,
// all identifiers for a vertex v can be packed in a single 64-bit integer pv(v), with the lower bits representing lower levels." (Delling et al. (2015))
//
// "Given any vertex v, define its query level lst(v) as the highest level such that v is not
// at the same cell as s or t. Equivalently, lst(v) is the maximum i such that ci(v) ∩ {s, t} = ∅." ( Delling et al. (2015))
//
// "To compute lst(v), we first determine the most significant differing bit of pv(s) and pv(v).
// (Recall that pv(v) encodes the cell number of v on each level.) This bit indicates the topmost level
// ls(v) in which they differ. We do the same for pv(t) and pv(v) to determine lt(v).
// The minimum of ls(v) and lt(v) is lst(v)." ( Delling et al. (2015))
//
// in this implementation for computing query level lst(v), we already precomputes maps from most significant differing bit of two number (bits.Len64(a^b)) to highest differing level
// vCellNumber is the bit packed cell numbers of v in each levels pv(v). with lower bits of pv(v) represent cell number containing vertex v in lower level.
func (li *LevelData) QueryLevel(sCellNumber, tCellNumber, vCellNumber Pv) uint8 {
	lsv := li.HighestDifferingLevel(sCellNumber, vCellNumber)
	ltv := li.HighestDifferingLevel(tCellNumber, vCellNumber)
	return min(lsv, ltv)
}

// TruncateToLevel gets cell number by shifting right. Level is 1-indexed.
func (li *LevelData) TruncateToLevel(cellNumber Pv, level uint8) Pv {
	// shift right to remove bits below the given level (but still contains bits above the level)
	return cellNumber >> Pv(li.offset[level-1])
}

// GetLevelCount returns the total number of partition levels.
func (li *LevelData) GetLevelCount() int {
	return len(li.offset) - 1
}

// GetOffsets returns the slice of level offsets.
func (li *LevelData) GetOffsets() []uint8 {
	return li.offset
}

// GetOffsetInLevel returns the bit offset at 1-indexed level l.
func (li *LevelData) GetOffsetInLevel(l int) uint8 {
	return li.offset[l-1]
}

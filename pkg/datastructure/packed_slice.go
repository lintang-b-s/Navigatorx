package datastructure

/*
PackedSlice. untuk store slice of unsigned integers dengan ukuran 33-51 bit efficiently (dari sisi space).
misal buat simpan osmNodeId dari setiap graph node:
Untuk osm node: https://wiki.openstreetmap.org/wiki/Stats 2025 ada sekitar 10 billion osm nodeids, jadi 34 bit dengan max value unsigned integer 34bit: 2^34-1 = 17179869183 biar aman untuk beberapa tahun kedepan

adapted/inspired by:
https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/packed_vector.md


How to set value
Use 33 bits as an example, let's say input is {1597322404, 1432114613, 1939964443, 2112255763}
The binary for 1597322404 is 001011111001101010011010010100100 in 33 bits, if we use inner array's 64bit value to record which then result
is 0000000000000000000000000000000001011111001101010011010010100100.

The binary for 1432114613 is 001010101010111000101010110110101 in 33 bits, for previous 64bit value there are still 31 left,
so 1010101010111000101010110110101 will be recorded in the first 64 bit value and 00 will be recorded in the second 64bit value. The final result for inner
array's first 64bit value is 1010101010111000101010110110101001011111001101010011010010100100 and second 64bit is still all 0.

The binary for 1939964443 is 001110011101000011000001000011011 in 33 bits, we could merge the value as a whole part into second 64bit value,
so 35 bits(2 bits from first input, 33 bits from second input) has taken and result is 0000000000000000000000000000000111001110100001100000100001101100


The binary for 2112255763 is 001111101111001100111011100010011 in 33 bits, we could merge the lower 29 bits into the free space of
second 64bit value and merge remaining part into third 64 bit vlaue. The result of inner array's second 64bit value is 1110111100110011101110001001100111001110100001100000100001101100
and third 64bit value is 0000000000000000000000000000000000000000000000000000000000000011.


For information record in 1110111100110011101110001001100111001110100001100000100001101100,
00 is from 1432114613, 001110011101000011000001000011011 is from 1939964443, 11101111001100111011100010011 is from 2112255763.
*/

// PackedSlice represents packed slice.
type PackedSlice struct {
	data         []uint64
	numberOfBits uint8 // number of bits untuk max value dari values yang disimpan di packed slice.
	// Untuk osm node: https://wiki.openstreetmap.org/wiki/Stats 2025 ada sekitar 10 billion osm nodeids, jadi 34 bit dengan max value unsigned integer 34bit: 2^34-1 =  17179869183 biar aman untuk beberapa tahun kedepan
	numOfItems uint64
}

// NewPackedSlice creates and returns packed slice.
func NewPackedSlice(numberOfBits uint8, numberOfItems uint64) *PackedSlice {
	ps := &PackedSlice{data: make([]uint64, 1), numberOfBits: numberOfBits, numOfItems: 0}
	capacity := ps.getLowerIndex(numberOfItems) + 1
	ps.data = make([]uint64, 1, capacity)
	return ps
}

// getLowerIndex get dataIndex of lower 64bit for index
func (ps *PackedSlice) getLowerIndex(index uint64) uint64 {
	return (index * uint64(ps.numberOfBits)) / 64
}

// getLowerOffset get offset (starting bit) of lower 64bit for index
func (ps *PackedSlice) getLowerOffset(index uint64) uint8 {
	return uint8(index*uint64(ps.numberOfBits)) % 64
}

// getUpperIndex get dataIndex of upper 64bit for index
func (ps *PackedSlice) getUpperIndex(index uint64) uint64 {
	lowerOffset := ps.getLowerOffset(index)
	if lowerOffset+ps.numberOfBits > 64 {
		// upper 64bit for index is in the new 64bit
		return ps.getLowerIndex(index) + 1
	}
	// upper 64bit for index is same as lower 64bit
	return ps.getLowerIndex(index)
}

// Append appends value to PackedSlice.
func (ps *PackedSlice) Append(val uint64) {
	index := ps.numOfItems
	lowerIndex := ps.getLowerIndex(index)
	lowerOffset := ps.getLowerOffset(index)

	if uint64(len(ps.data)) <= lowerIndex {
		ps.data = append(ps.data, 0)
	}

	if ps.getUpperIndex(index) == lowerIndex {
		// kalau upper 64bit dan lower 64bit dari val sama
		ps.data[lowerIndex] |= (val << lowerOffset)
	} else {
		// kalau upper 64bit dan lower 64bit dari val b3eq
		bitsInLower := 64 - lowerOffset
		lowerBitMask := (uint64(1) << bitsInLower) - 1
		// lower bits value
		lowerBitsVal := val & lowerBitMask
		// place the lower bits value starting at its lower offset position
		ps.data[lowerIndex] |= (lowerBitsVal << lowerOffset)

		// get upper bits value and place it in next 64 bit ps.data
		upperBitsVal := val >> bitsInLower
		if uint64(len(ps.data)) <= lowerIndex+1 {
			ps.data = append(ps.data, upperBitsVal)
		} else {
			ps.data[lowerIndex+1] |= upperBitsVal
		}
	}

	ps.numOfItems++
}

// Get gets the value at index from PackedSlice.
func (ps *PackedSlice) Get(index uint64) uint64 {
	lowerIndex := ps.getLowerIndex(index)
	lowerOffset := ps.getLowerOffset(index)

	if ps.getUpperIndex(index) == lowerIndex {
		// upper 64bit dan lower 64bit dari index adalah sama
		valMask := (uint64(1) << ps.numberOfBits) - 1
		return (ps.data[lowerIndex] >> lowerOffset) & valMask
	}

	// upper 64bit dan lower 64bit dari index adalah beda
	bitsInLower := 64 - lowerOffset
	// lower bits bitmask that starting at lowerOffset in ps.data[lowerIndex]
	lowerBitMask := (^uint64(0)) << uint64(lowerOffset)
	lowerBitsVal := (ps.data[lowerIndex] & lowerBitMask) >> lowerOffset

	upperNumOfBits := ps.numberOfBits - bitsInLower
	upperMask := (uint64(1) << upperNumOfBits) - 1
	upperBitsVal := (ps.data[lowerIndex+1] & upperMask) << bitsInLower

	return upperBitsVal | lowerBitsVal
}

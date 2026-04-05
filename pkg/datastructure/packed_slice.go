package datastructure

/*
PackedSlice. untuk store slice of unsigned integers dengan ukuran 33-63 bit efficiently (dari sisi space).
misal buat simpan osmNodeId dari setiap graph node:
Untuk osm node: https://wiki.openstreetmap.org/wiki/Stats 2025 ada sekitar 10 billion osm nodeids, jadi 34 bit dengan max value unsigned integer 34bit: 2^34-1 = 17179869183 biar aman untuk beberapa tahun kedepan

adapted/inspired from:
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




untuk 64bit 1110111100110011101110001001100111001110100001100000100001101100:

values yang diinsert: {1597322404, 1432114613, 1939964443, 2112255763}

1432114613 adalah item kedua (atau index ke 1)
upperNumOfBits[1]=3 (jumlah bit 1432114613 di 1110111100110011101110001001100111001110100001100000100001101100 )
lowerOffset[1]=33 (index dari first bit 1432114613 di 1010101010111000101010110110101001011111001101010011010010100100)




*/

type PackedSlice struct {
	data         []uint64
	numberOfBits uint8 // number of bits untuk max value dari values yang disimpan di packed slice.
	// Untuk osm node: https://wiki.openstreetmap.org/wiki/Stats 2025 ada sekitar 10 billion osm nodeids, jadi 34 bit dengan max value unsigned integer 34bit: 2^34-1 =  17179869183 biar aman untuk beberapa tahun kedepan
	lowerOffset    []uint8 // map dari index dari value -> offset dari bit pertama value di lower 64bit
	upperNumOfBits []uint8 // map dari index dari value -> jumlah bit di upper 64bit
	numOfItems     uint64
}

func NewPackedSlice(numberOfBits uint8, numberOfItems uint64) *PackedSlice {
	ps := &PackedSlice{data: make([]uint64, 1), numberOfBits: numberOfBits, lowerOffset: make([]uint8, 0), upperNumOfBits: make([]uint8, 0), numOfItems: 0}
	capacity := ps.getLowerIndex(numberOfItems) + 1
	ps.data = make([]uint64, 1, capacity)
	return ps
}

/*
untuk ps.Append(val): val adalah value yang ingin diinsert ke PackedSlice
misal:
numberOfBits = 33

contoh kasus pertama, kita ingin insert 1432114613 ke PackedSlice:
first 64 bit value (sebelum isi 1432114613):
0000000000000000000000000000000001011111001101010011010010100100

untuk kasus diatas, second 64 bit value (sebelum isi 1432114613):
0000000000000000000000000000000000000000000000000000000000000000

kita tau lastIndex=0, karena first 64 bit val masih ada yang kosong

misal kita mau isi 1432114613 / 001010101010111000101010110110101:
lastIndex=0

1. get ps.data[lastIndex]
2. hitung jumlah bit tersisa di ps.data[lastIndex] dengan cara:
63 - ps.upperNumOfBits[lastIndex]+1
misal di kasus pertama:

ps.upperNumOfBits[lastIndex]=33 (karena 33 bit dari value 1597322404 diisi di first 64bit)
jumlah bit tersisa=63-33+1=31 bits left

kasus kedua: second 64bit setelah 1432114613 diisi: 0000000000000000000000000000000000000000000000000000000000000000
jumlah bit tersisa:
63-2+1=62 (karena 2 bit dari 1432114613 diisi di second 64bit)

3. isi bit dari value yang ingin diinsert di lower 64bit dan upper 64bit:
misal:
untuk kasus pertama:
ada 31 bits left di first 64bit, ya kita isi 31 bit dari 1432114613 di 31 bits left dari first 64bit
dan 2 bits sisanya di second 64 bit

update ps.upperNumOfBits dan ps.lowerOffset dan lastIndex

untuk kasus kedua: kita ingin insert 1939964443:
ada 62 bit tersisa di second 64bit, jadi ya tinggal isi 33 bit dari 1939964443 ke second 64 bit
update ps.upperNumOfBits dan ps.lowerOffset dan lastIndex

udah gitu doang.
kayaknya ada yang miss?


untuk ps.Get(index): index adalah index dari value yang sudah pernah diinsert ke PackedSlice
contohnya: karena kita mau map dari graph nodeId -> osm nodeId
index nya dalah graph nodeId, val nya adalah osm nodeId

kita bisa ps.lowerOffset[graphNodeId] untuk mendapatkan offset dari first bit osmNodeId dari graphNodeId di suatu lower 64bit
masalahnya diindex ps.data manakah lower 64bit value tsb?

misal untuk kasus pertama:
kita get index 1 (atau kita pengen dapetin value 1432114613):
kita tau 1432114613 disimpan dengan cara:
31 bit dari 1432114613 di first 64bit dan 2 last bit dari 1432114613 di second 64 bit.

patternnya untuk numberOfBits=33:
first 64bit: 33 bit 1st val + 31 bit 2th val.
second 64bit: 2 bit 2th val + 33 bit 3th val + 29 bit dari 4th val
third 64bit: 4 bit dari 4th val + 33 bit dari 5th val + 27 bit dari 6th val

dari item ke dua (dengan 33 bit) ke first 64bit (index ke 0 dari ps.data) atau its lower 64bit:
ps.Get(index): indexnya 0-based indexing,

buat dapetin lower 64bit atau ps.getLowerIndex(index):
floor((index*ps.numberOfBits)/64)
index: 0 = floor(0*33/64)=0   (1st val)
index: 1 = floor(1*33/64)=0   (2th val)
index: 2 = floor(2*33/64)=1
index: 3 = floor(3*33/64)=1
index: 4 = floor(4*33/64)=2  (5th val)
index: 5 = floor(5*33/64)=2

gitu gan

buat get value dari index (sama kaya yang dijelasin https://github.com/Telenav/open-source-spec/blob/master/osrm/doc/packed_vector.md):
contoh pertama (case ps.getLowerIndex(index) != ps.getUpperIndex(index)):
misal 31 bit dari value 1432114613 ada di first 64bit:
(1010101010111000101010110110101001011111001101010011010010100100 & 1111111111111111111111111111111000000000000000000000000000000000) >> 33

atau
index = index dari value 1432114613
karena ps.getLowerIndex(index) != ps.getUpperIndex(index)
lowerBitMask =  (^0) <<  ps.lowerOffset[index]
(ps.data[ps.getLowerIndex(index)] & lowerBitMask ) >> ps.lowerOffset[index]

untuk dapetin last 2 bit dari 1432114613 di second 64bit:
(1010101010111000101010110110101001011111001101010011010010100100 & 0000000000000000000000000000000000000000000000000000000000000011) << 31
kenapa 31? karena numberOfBits=33, dan kita mau ambil last 2 bit dari 33 bit tsb.

atau
(ps.data[ps.getUpperIndex(index)] & upperMask) << (ps.numberOfBits - ps.upperNumOfBits[index])


upperMask = (uint64(1) << ps.upperNumOfBits[index] )-1
misal ps.upperNumOfBits[index] = 2:
(1<<2)-1
upperMask = 100 - 001 = 011


contoh kedua (case ps.getLowerIndex(index) == ps.getUpperIndex(index)):
untuk dapetin lower bits dari value 1939964443 (yaitu 001110011101000011000001000011011) di 1110111100110011101110001001100111001110100001100000100001101100:
index = index dari vaue 1939964443
karena ps.getLowerIndex(index) == ps.getUpperIndex(index)
lowerBitMask = ((^0) << ps.lowerOffset[index]) & ^((^0) <<  (ps.lowerOffset[index] + ps.upperNumOfBits[index]) )


udah gitu doang. kayaknya gak ada yang miss?

logic untuk ps.getUpperIndex(index):
if  (ps.lowerOffset[index] + ps.numberOfBits) > 64 then:
return ps.getLowerIndex(index)+1
else:
return ps.getLowerIndex(index)
*/

// getLowerIndex. get dataIndex of lower 64bit for index
func (ps *PackedSlice) getLowerIndex(index uint64) uint64 {
	return (index * uint64(ps.numberOfBits)) / 64
}

// getUpperIndex. get dataIndex of upper 64bit for index
func (ps *PackedSlice) getUpperIndex(index uint64) uint64 {
	if ps.lowerOffset[index]+ps.numberOfBits > 64 {
		return ps.getLowerIndex(index) + 1
	}
	return ps.getLowerIndex(index)
}

// Append. append value ke PackedSlice“
func (ps *PackedSlice) Append(val uint64) {
	var prevIndex uint64

	if ps.numOfItems == 0 {
		prevIndex = 0
	} else {
		prevIndex = ps.numOfItems - 1
	}

	index := ps.numOfItems
	if uint64(len(ps.lowerOffset)) <= index {
		ps.lowerOffset = append(ps.lowerOffset, 0)
		ps.upperNumOfBits = append(ps.upperNumOfBits, 0)
	}

	lastIndex := ps.getLowerIndex(index)

	var lastOffset uint8
	if ps.getLowerIndex(prevIndex) == ps.getUpperIndex(prevIndex) &&
		ps.getLowerIndex(index) == ps.getUpperIndex(prevIndex) {
		// misal second 64bit: 2 bit 2th val + 33 bit 3th val + 29 bit dari 4th val
		// 1110111100110011101110001001100111001110100001100000100001101100
		// untuk dapetin lowerOfset dari 4th val: 2 + 33 = 35

		lastOffset = (ps.upperNumOfBits[prevIndex] + ps.lowerOffset[prevIndex])
	} else if ps.getLowerIndex(index) != ps.getUpperIndex(prevIndex) {
		lastOffset = 0
	} else {
		lastOffset = ps.upperNumOfBits[prevIndex]
	}

	// range lastOffset [0,63]
	bitsLeft := 63 - lastOffset + 1

	// 64 bit uint64
	// kita pengen dapetin lowerBitMask dari val, misal bitsLeft=31:
	// 111.....111111 (64 bit)
	// ilangin lower 33 bit dari uint64 diatas
	// 111.....111111 >> 33
	lowerBitMask := (^uint64(0)) >> (64 - bitsLeft)
	lowerBitsVal := val & lowerBitMask

	ps.lowerOffset[index] = lastOffset

	// isi lower bit value dari val dulu di lower 64 bit:

	// kasus pertama: 0000000000000000000000000000000001011111001101010011010010100100
	// 001011111001101010011010010100100 diisi oleh 1st value
	// ada 31 left
	if uint64(len(ps.data)) <= lastIndex {
		ps.data = append(ps.data, 0)
	}

	ps.data[lastIndex] |= (lowerBitsVal << ps.lowerOffset[index])

	if ps.getLowerIndex(index) != ps.getUpperIndex(index) {
		// kalau upper 64bit dan lower 64bit dari val beda:
		ps.upperNumOfBits[index] = ps.numberOfBits - bitsLeft

		upperBitsVal := val >> bitsLeft
		ps.data = append(ps.data, upperBitsVal)
	} else {
		ps.upperNumOfBits[index] = ps.numberOfBits
	}

	ps.numOfItems++
}

// Get. get value dari index
// contoh: index bisa berupa graphNodeId, value dari index adlh its corresponding osmNodeId
func (ps *PackedSlice) Get(index uint64) uint64 {
	if ps.getLowerIndex(index) == ps.getUpperIndex(index) {
		// upper 64bit dan lower 64bit dari index adalah sama
		dataIndex := ps.getLowerIndex(index)
		bitMask := ((^uint64(0)) << ps.lowerOffset[index]) & ^((^uint64(0)) << (ps.lowerOffset[index] + ps.upperNumOfBits[index]))
		return (ps.data[dataIndex] & bitMask) >> uint64(ps.lowerOffset[index])
	}

	// upper 64bit dan lower 64bit dari index adalah beda

	lowerBitMask := (^uint64(0)) << ps.lowerOffset[index]

	lower := (ps.data[ps.getLowerIndex(index)] & lowerBitMask) >> ps.lowerOffset[index]

	upperMask := (uint64(1) << ps.upperNumOfBits[index]) - 1
	upper := (ps.data[ps.getUpperIndex(index)] & upperMask) << (ps.numberOfBits - ps.upperNumOfBits[index])

	return upper | lower
}

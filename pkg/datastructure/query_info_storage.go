package datastructure

import (
	"maps"
	"math"
)

/*
design decision:
commit 83a333d36ca77b910b3e6197eb0910192be0c868:
dari heap profile: https://drive.google.com/file/d/1b7IdOpduPOu1oKJH94UUzbacUv8fBgnw/view?usp=sharing
https://drive.google.com/file/d/1LICWTdl-_pIXd5hXh4kEQjorp-5NXGU8/view?usp=sharing

sebelumnya di (di commit diatas) crp query phase, buat simpen predecessor, estimate sp, dan heap node cost dari setiap lablled & scanned vertices pakai slice/array
setiap kali query kita preallocate VertexInfo sebanyak NumberOfOvelayVertices, buat osm map yang include diy,solo, semarang sekitar 100k
VertexInfo total 48 byte
pas di load test pake k6 100 vu, makan memory banyak wkwk
*/
type TwoLevelStorage struct {
	overlay        map[Index]int
	base           []int
	maxEdgesInCell int
}

func NewTwoLevelStorage(baseSize, maxEdgesInCell int) *TwoLevelStorage {
	base := make([]int, baseSize)
	for i := 0; i < baseSize; i++ {
		base[i] = math.MaxInt
	}

	return &TwoLevelStorage{
		overlay:        make(map[Index]int, OVERLAY_INFO_SIZE),
		base:           base,
		maxEdgesInCell: maxEdgesInCell,
	}
}

func isOverlay(u Index, maxEdgesInCell int) bool {
	return u >= Index(maxEdgesInCell)*2
}

func (s *TwoLevelStorage) Get(id Index) int {
	if isOverlay(id, s.maxEdgesInCell) {
		// https://go.dev/blog/swisstable
		// go1.24 use swiss table for its hash table (open adressing)
		// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
		// open adressing avg case: unsuccesful search & insert in O(1/(1-a)) or O(1)
		val, ok := s.overlay[id]
		if !ok {
			return math.MaxInt
		}
		return val
	}

	return s.base[id]
}

func (s *TwoLevelStorage) Set(id Index, queryInfoId int) {
	if isOverlay(id, s.maxEdgesInCell) {
		s.overlay[id] = queryInfoId
		return
	}
	s.base[id] = queryInfoId
}

func (s *TwoLevelStorage) Clear() {
	for i := 0; i < len(s.base); i++ {
		s.base[i] = math.MaxInt
	}

	// https://go101.org/optimizations/6-map.html
	for key := range s.overlay {
		delete(s.overlay, key)
	}
}

func (s *TwoLevelStorage) Clone() QueryInfoStorage {
	overlayClone := make(map[Index]int, len(s.overlay))
	maps.Copy(overlayClone, s.overlay)
	base := make([]int, len(s.base))
	copy(base, s.base)

	return &TwoLevelStorage{overlay: overlayClone,
		base: base, maxEdgesInCell: s.maxEdgesInCell}
}

func (s *TwoLevelStorage) ForAllItems(handle func(offsetedVId Index, queryInfoId int)) {
	for offsetedEdgeId, queryInfoId := range s.base {
		handle(Index(offsetedEdgeId), queryInfoId)
	}

	for overlayVId, queryInfoId := range s.overlay {
		handle(overlayVId, queryInfoId)
	}
}

type ArrayStorage struct {
	base []int
}

func NewArrayStorage(size int) *ArrayStorage {
	base := make([]int, size)
	for i := 0; i < size; i++ {
		base[i] = math.MaxInt
	}
	return &ArrayStorage{
		base: base,
	}
}

func (s *ArrayStorage) Get(id Index) int {
	return s.base[id]
}

func (s *ArrayStorage) Set(id Index, info int) {
	s.base[id] = info
}

func (s *ArrayStorage) Clear() {
	for i := 0; i < len(s.base); i++ {
		s.base[i] = math.MaxInt
	}
}

func (s *ArrayStorage) Clone() QueryInfoStorage {

	base := make([]int, len(s.base))
	copy(base, s.base)

	return &TwoLevelStorage{
		base: base}
}

func (s *ArrayStorage) ForAllItems(handle func(offsetedVId Index, queryInfoId int)) {
	for offsetedEdgeId, queryInfoId := range s.base {
		handle(Index(offsetedEdgeId), queryInfoId)
	}

}

type MapStorage struct {
	overlay map[Index]int
}

func NewMapStorage(baseSize int) *MapStorage {

	return &MapStorage{
		overlay: make(map[Index]int, OVERLAY_INFO_SIZE),
	}
}

func (s *MapStorage) Get(id Index) int {
	// https://go.dev/blog/swisstable
	// go1.24 use swiss table for its hash table (open adressing)
	// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
	// open adressing avg case: unsuccesful search & insert in O(1/(1-a)) or O(1)
	val, ok := s.overlay[id]
	if !ok {
		return math.MaxInt
	}
	return val
}

func (s *MapStorage) Set(id Index, queryInfoId int) {
	s.overlay[id] = queryInfoId
}

func (s *MapStorage) Clear() {
	// https://go101.org/optimizations/6-map.html
	for key := range s.overlay {
		delete(s.overlay, key)
	}
}

func (s *MapStorage) Clone() QueryInfoStorage {
	overlayClone := make(map[Index]int, len(s.overlay))
	maps.Copy(overlayClone, s.overlay)

	return &TwoLevelStorage{overlay: overlayClone}
}

func (s *MapStorage) ForAllItems(handle func(offsetedVId Index, queryInfoId int)) {

	for overlayVId, queryInfoId := range s.overlay {
		handle(overlayVId, queryInfoId)
	}
}

package datastructure

import (
	"maps"
	"math"
)

type TwoLevelStorage struct {
	overlay        map[Index]uint32
	base           []uint32
	maxEdgesInCell uint32
}

func NewTwoLevelStorage(baseSize, maxEdgesInCell uint32) *TwoLevelStorage {
	base := make([]uint32, baseSize)
	for i := uint32(0); i < baseSize; i++ {
		base[i] = math.MaxUint32
	}

	return &TwoLevelStorage{
		overlay:        make(map[Index]uint32, OVERLAY_INFO_SIZE),
		base:           base,
		maxEdgesInCell: maxEdgesInCell,
	}
}

func isOverlay(u Index, maxEdgesInCell uint32) bool {
	return u >= Index(maxEdgesInCell)*2
}

func (s *TwoLevelStorage) Get(id Index) uint32 {
	if isOverlay(id, s.maxEdgesInCell) {
		// https://go.dev/blog/swisstable
		// go1.24 use swiss table for its hash table (open adressing)
		// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
		// open adressing avg case: unsuccesful search & insert in O(1/(1-a)) or O(1)
		val, ok := s.overlay[id]
		if !ok {
			return math.MaxUint32
		}
		return val
	}

	return s.base[id]
}

func (s *TwoLevelStorage) Set(id Index, queryInfoId uint32) {
	if isOverlay(id, s.maxEdgesInCell) {
		s.overlay[id] = queryInfoId
		return
	}
	s.base[id] = queryInfoId
}

func (s *TwoLevelStorage) Clear() {
	for i := 0; i < len(s.base); i++ {
		s.base[i] = math.MaxUint32
	}

	// https://go101.org/optimizations/6-map.html
	for key := range s.overlay {
		delete(s.overlay, key)
	}
}

func (s *TwoLevelStorage) Clone() QueryInfoStorage {
	overlayClone := make(map[Index]uint32, len(s.overlay))
	maps.Copy(overlayClone, s.overlay)
	base := make([]uint32, len(s.base))
	copy(base, s.base)

	return &TwoLevelStorage{overlay: overlayClone,
		base: base, maxEdgesInCell: s.maxEdgesInCell}
}

func (s *TwoLevelStorage) ForAllItems(handle func(offsetedVId Index, queryInfoId uint32)) {
	for offsetedEdgeId, queryInfoId := range s.base {
		handle(Index(offsetedEdgeId), queryInfoId)
	}

	for overlayVId, queryInfoId := range s.overlay {
		handle(overlayVId, queryInfoId)
	}
}

type ArrayStorage struct {
	base []uint32
}

func NewArrayStorage(size uint32) *ArrayStorage {
	base := make([]uint32, size)
	for i := uint32(0); i < size; i++ {
		base[i] = math.MaxUint32
	}
	return &ArrayStorage{
		base: base,
	}
}

func (s *ArrayStorage) Get(id Index) uint32 {
	return s.base[id]
}

func (s *ArrayStorage) Set(id Index, info uint32) {
	s.base[id] = info
}

func (s *ArrayStorage) Clear() {
	for i := 0; i < len(s.base); i++ {
		s.base[i] = math.MaxUint32
	}
}

func (s *ArrayStorage) Clone() QueryInfoStorage {
	base := make([]uint32, len(s.base))
	copy(base, s.base)

	return &ArrayStorage{
		base: base}
}

func (s *ArrayStorage) ForAllItems(handle func(offsetedVId Index, queryInfoId uint32)) {
	for offsetedEdgeId, queryInfoId := range s.base {
		handle(Index(offsetedEdgeId), queryInfoId)
	}

}

type MapStorage struct {
	overlay map[Index]uint32
}

func NewMapStorage(baseSize uint32) *MapStorage {

	return &MapStorage{
		overlay: make(map[Index]uint32, OVERLAY_INFO_SIZE),
	}
}

func (s *MapStorage) Get(id Index) uint32 {
	// https://go.dev/blog/swisstable
	// go1.24 use swiss table for its hash table (open adressing)
	// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
	// open adressing avg case: unsuccesful search & insert in O(1/(1-a)) or O(1)
	val, ok := s.overlay[id]
	if !ok {
		return math.MaxUint32
	}
	return val
}

func (s *MapStorage) Set(id Index, queryInfoId uint32) {
	s.overlay[id] = queryInfoId
}

func (s *MapStorage) Clear() {
	// https://go101.org/optimizations/6-map.html
	for key := range s.overlay {
		delete(s.overlay, key)
	}
}

func (s *MapStorage) Clone() QueryInfoStorage {
	overlayClone := make(map[Index]uint32, len(s.overlay))
	maps.Copy(overlayClone, s.overlay)

	return &MapStorage{overlay: overlayClone}
}

func (s *MapStorage) ForAllItems(handle func(offsetedVId Index, queryInfoId uint32)) {

	for overlayVId, queryInfoId := range s.overlay {
		handle(overlayVId, queryInfoId)
	}
}

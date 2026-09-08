package datastructure

import (
	"maps"
	"math"

	"github.com/bits-and-blooms/bitset"
	"github.com/bytedance/gopkg/collection/hashset"
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
		overlay:        make(map[Index]uint32, OVERLAY_VERTICES_SIZE),
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
		// go1.24 use swiss table for its hash table (open addressing)
		// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
		// open addressing avg case: unsuccessful search & insert in O(1/(1-a)) or O(1)
		val, ok := s.overlay[id]
		if !ok {
			return math.MaxUint32
		}
		return val
	}

	return s.base[id]
}

func (s *TwoLevelStorage) Set(id Index, vertexIndex uint32) {
	if isOverlay(id, s.maxEdgesInCell) {
		s.overlay[id] = vertexIndex
		return
	}
	s.base[id] = vertexIndex
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

func (s *TwoLevelStorage) Clone() IndexStorage {
	overlayClone := make(map[Index]uint32, len(s.overlay))
	maps.Copy(overlayClone, s.overlay)
	base := make([]uint32, len(s.base))
	copy(base, s.base)

	return &TwoLevelStorage{overlay: overlayClone,
		base: base, maxEdgesInCell: s.maxEdgesInCell}
}

func (s *TwoLevelStorage) ForAllItems(handle func(offsetedVId Index, vertexIndex uint32)) {
	for offsetedEdgeId, vertexIndex := range s.base {
		handle(Index(offsetedEdgeId), vertexIndex)
	}

	for overlayVId, vertexIndex := range s.overlay {
		handle(overlayVId, vertexIndex)
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

func (s *ArrayStorage) Clone() IndexStorage {
	base := make([]uint32, len(s.base))
	copy(base, s.base)

	return &ArrayStorage{
		base: base}
}

func (s *ArrayStorage) ForAllItems(handle func(offsetedVId Index, vertexIndex uint32)) {
	for offsetedEdgeId, vertexIndex := range s.base {
		handle(Index(offsetedEdgeId), vertexIndex)
	}

}

type MapStorage struct {
	overlay map[Index]uint32
}

func NewMapStorage(baseSize uint32) *MapStorage {

	return &MapStorage{
		overlay: make(map[Index]uint32, OVERLAY_VERTICES_SIZE),
	}
}

func (s *MapStorage) Get(id Index) uint32 {
	// https://go.dev/blog/swisstable
	// go1.24 use swiss table for its hash table (open addressing)
	// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
	// open addressing avg case: unsuccessful search & insert in O(1/(1-a)) or O(1)
	val, ok := s.overlay[id]
	if !ok {
		return math.MaxUint32
	}
	return val
}

func (s *MapStorage) Set(id Index, vertexIndex uint32) {
	s.overlay[id] = vertexIndex
}

func (s *MapStorage) Clear() {
	// https://go101.org/optimizations/6-map.html
	for key := range s.overlay {
		delete(s.overlay, key)
	}
}

func (s *MapStorage) Clone() IndexStorage {
	overlayClone := make(map[Index]uint32, len(s.overlay))
	maps.Copy(overlayClone, s.overlay)

	return &MapStorage{overlay: overlayClone}
}

func (s *MapStorage) ForAllItems(handle func(offsetedVId Index, vertexIndex uint32)) {

	for overlayVId, vertexIndex := range s.overlay {
		handle(overlayVId, vertexIndex)
	}
}

type ExploredBitsetStorage struct {
	scanned *bitset.BitSet // https://abseil.io/fast/hints.html#bit-vectors-instead-of-sets
}

func NewExploredBitsetStorage(approxMaxSearchSize uint32) *ExploredBitsetStorage {
	return &ExploredBitsetStorage{bitset.New(uint(approxMaxSearchSize))}
}

func (sc *ExploredBitsetStorage) Test(vertexIndex uint32) bool {
	return sc.scanned.Test(uint(vertexIndex))
}

func (sc *ExploredBitsetStorage) Set(vertexIndex uint32) {
	sc.scanned.Set(uint(vertexIndex))
}

func (sc *ExploredBitsetStorage) Clear(maxEdgesInCell uint32) {
	sc.scanned.ClearAll()
}

type ExploredSettorage struct {
	scanned hashset.Uint32Set
}

func NewExploredSettorage(approxMaxSearchSize uint32) *ExploredSettorage {
	return &ExploredSettorage{hashset.NewUint32WithSize(int(approxMaxSearchSize))}
}

func (sc *ExploredSettorage) Test(vertexIndex uint32) bool {
	return sc.scanned.Contains(vertexIndex)
}

func (sc *ExploredSettorage) Set(vertexIndex uint32) {
	sc.scanned.Add(vertexIndex)
}

func (sc *ExploredSettorage) Clear(maxEdgesInCell uint32) {
	sc.scanned.Range(func(value uint32) bool {
		sc.scanned.Remove(value)
		return true
	})
}

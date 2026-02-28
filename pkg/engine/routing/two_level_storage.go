package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

/*
inspiration (TwoLevelStorage OSRM): https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp

design decision:
commit 83a333d36ca77b910b3e6197eb0910192be0c868:
dari heap profile: https://drive.google.com/file/d/1b7IdOpduPOu1oKJH94UUzbacUv8fBgnw/view?usp=sharing
https://drive.google.com/file/d/1LICWTdl-_pIXd5hXh4kEQjorp-5NXGU8/view?usp=sharing

sebelumnya di (di commit diatas) crp query phase, buat simpen predecessor, estimate sp, dan heap node cost dari setiap lablled & scanned vertices pakai slice/array
setiap kali query kita preallocate VertexInfo sebanyak NumberOfOvelayVertices, buat osm map yang include diy,solo, semarang sekitar 100k
VertexInfo total 48 byte
pas di load test pake k6 100 vu, makan memory banyak wkwk
*/
type TwoLevelStorage[T comparable] struct {
	overlay        map[da.Index]*VertexInfo[T]
	base           []*VertexInfo[T]
	maxEdgesInCell int
}

func NewTwoLevelStorage[T comparable](baseSize, maxEdgesInCell int) *TwoLevelStorage[T] {
	base := make([]*VertexInfo[T], baseSize)
	for i := 0; i < baseSize; i++ {
		base[i] = NewVertexInfo[T](pkg.INF_WEIGHT, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), nil)
	}
	return &TwoLevelStorage[T]{
		overlay:        make(map[da.Index]*VertexInfo[T], OVERLAY_INFO_SIZE),
		base:           base,
		maxEdgesInCell: maxEdgesInCell,
	}
}

func isOverlay(u da.Index, maxEdgesInCell int) bool {
	return u >= da.Index(maxEdgesInCell)*2
}

func (s *TwoLevelStorage[T]) Get(id da.Index) *VertexInfo[T] {
	if isOverlay(id, s.maxEdgesInCell) {
		val, ok := s.overlay[id]
		if !ok {
			return NewVertexInfo[T](pkg.INF_WEIGHT, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), nil)
		}
		return val
	}

	return s.base[id]
}

func (s *TwoLevelStorage[T]) Set(id da.Index, info *VertexInfo[T]) {
	if isOverlay(id, s.maxEdgesInCell) {
		s.overlay[id] = info
		return
	}
	s.base[id] = info
}

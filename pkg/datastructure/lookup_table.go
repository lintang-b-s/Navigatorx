package datastructure

import (
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// inspired by https://github.com/Project-OSRM/osrm-backend/blob/master/include/updater/source.hpp

type KeyVal struct {
	keys []uint64
	val  Index
}

func NewKeyVal(keys []uint64, val Index) KeyVal {
	return KeyVal{keys: keys, val: val}
}

// compare. lexicographic order compare.
func (kv KeyVal) compare(kvb KeyVal) bool {
	for i := 0; i < len(kv.keys); i++ {
		if kv.keys[i] != kvb.keys[i] {
			return kv.keys[i] < kvb.keys[i]
		}
	}

	return false
}

func (kv KeyVal) eq(kvb KeyVal) bool {
	for i := 0; i < len(kv.keys); i++ {
		if kv.keys[i] != kvb.keys[i] {
			return false
		}
	}

	return true
}

type SegmentKV struct {
	kv KeyVal // map from osm node id pair (u,v) to edge-based graph node id
}

func NewSegmentKV(u, v uint64, ebgnId Index) *SegmentKV {
	keys := []uint64{u, v}
	kv := NewKeyVal(keys, ebgnId)
	return &SegmentKV{kv: kv}
}

func (s *SegmentKV) Compare(b *SegmentKV) bool {
	return s.kv.compare(b.kv)
}

func (s *SegmentKV) Eq(b *SegmentKV) bool {
	return s.kv.eq(b.kv)
}

func (s *SegmentKV) Val() Index {
	return s.kv.val
}

type TurnKV struct {
	kv KeyVal // map from osm node id triple (u,v,w) to edge-based graph edge id
}

func NewTurnKV(u, v, w uint64, ebgeId Index) *TurnKV {
	keys := []uint64{u, v, w}
	kv := NewKeyVal(keys, ebgeId)
	return &TurnKV{kv: kv}
}

func (t *TurnKV) Compare(b *TurnKV) bool {
	return t.kv.compare(b.kv)
}

func (t *TurnKV) Val() Index {
	return t.kv.val
}

func (t *TurnKV) Eq(b *TurnKV) bool {
	return t.kv.eq(b.kv)
}

type LookupKV[T any] interface {
	Compare(b T) bool
	Eq(b T) bool
	Val() Index
	WriteToFile(w *util.BinaryWriter) error
}

type LookupTable[T LookupKV[T]] struct {
	data []T
}

// NewNewLookupTable. bikin LookupTable dengan tipe generic T.
func NewLookupTable[T LookupKV[T]](data []T) *LookupTable[T] {

	sort.Slice(data, func(i, j int) bool {
		return data[i].Compare(data[j])
	})

	return &LookupTable[T]{data: data}
}

// Get. get edge-based graph (ebg) node id or ebg edge id
// O(logn) binary search
func (lt *LookupTable[T]) Get(key T) Index {
	n := len(lt.data)

	l := 0
	r := n - 1

	for l <= r {
		mid := l + (r-l)/2
		if lt.data[mid].Eq(key) {
			return lt.data[mid].Val()
		} else if lt.data[mid].Compare(key) {
			l = mid + 1
		} else {
			r = mid - 1
		}
	}

	return INVALID_LK_TABLE_ID
}

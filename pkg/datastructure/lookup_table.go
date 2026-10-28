package datastructure

import (
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// inspired by https://github.com/Project-OSRM/osrm-backend/blob/master/include/updater/source.hpp

// KeyVal represents a key-value pair where keys is a slice of uint64 and val is an Index.
type KeyVal struct {
	keys []uint64
	val  Index
}

// NewKeyVal creates a new KeyVal.
func NewKeyVal(keys []uint64, val Index) KeyVal {
	return KeyVal{keys: keys, val: val}
}

// compare. lexicographic order less comparation.
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

// SegmentKV maps an OSM node ID pair (u, v) to an edge-based graph node ID.
type SegmentKV struct {
	kv KeyVal // map from osm node id pair (u,v) to edge-based graph node id
}

// NewSegmentKV creates a new SegmentKV for node pair (u, v) and edge-based graph node ID ebgnId.
func NewSegmentKV(u, v uint64, ebgnId Index) *SegmentKV {
	keys := []uint64{u, v}
	kv := NewKeyVal(keys, ebgnId)
	return &SegmentKV{kv: kv}
}

// Compare reports whether s is lexicographically less than b.
func (s *SegmentKV) Compare(b *SegmentKV) bool {
	return s.kv.compare(b.kv)
}

// Eq reports whether s is equal to b.
func (s *SegmentKV) Eq(b *SegmentKV) bool {
	return s.kv.eq(b.kv)
}

// Val returns the stored edge-based graph node ID.
func (s *SegmentKV) Val() Index {
	return s.kv.val
}

// TurnKV maps an OSM node ID triple (u, v, w) to an edge-based graph edge ID.
type TurnKV struct {
	kv KeyVal // map from osm node id triple (u,v,w) to edge-based graph edge id
}

// NewTurnKV creates a new TurnKV for node triple (u, v, w) and edge-based graph edge ID ebgeId.
func NewTurnKV(u, v, w uint64, ebgeId Index) *TurnKV {
	keys := []uint64{u, v, w}
	kv := NewKeyVal(keys, ebgeId)
	return &TurnKV{kv: kv}
}

// Compare reports whether t is lexicographically less than b.
func (t *TurnKV) Compare(b *TurnKV) bool {
	return t.kv.compare(b.kv)
}

// Val returns the stored edge-based graph edge ID.
func (t *TurnKV) Val() Index {
	return t.kv.val
}

// Eq reports whether t is equal to b.
func (t *TurnKV) Eq(b *TurnKV) bool {
	return t.kv.eq(b.kv)
}

// LookupKV is an interface for key-value entries stored in a LookupTable.
type LookupKV[T any] interface {
	Compare(b T) bool
	Eq(b T) bool
	Val() Index
	WriteToFile(w *util.BinaryWriter) error
}

// LookupTable provides binary search lookup for sorted key-value pairs.
type LookupTable[T LookupKV[T]] struct {
	data []T
}

// NewLookupTable creates a new sorted LookupTable from data.
func NewLookupTable[T LookupKV[T]](data []T) *LookupTable[T] {

	sort.Slice(data, func(i, j int) bool {
		return data[i].Compare(data[j])
	})

	return &LookupTable[T]{data: data}
}

// Get returns the value associated with key, or INVALID_LK_TABLE_ID if not found.
// O(logn) binary search
func (lt *LookupTable[T]) Get(key T) Index {
	n := len(lt.data)

	l := 0
	r := n - 1

	for l <= r {
		mid := l + (r-l)/2
		switch {
		case lt.data[mid].Eq(key):
			return lt.data[mid].Val()
		case lt.data[mid].Compare(key):
			l = mid + 1
		default:
			r = mid - 1
		}
	}

	return INVALID_LK_TABLE_ID
}

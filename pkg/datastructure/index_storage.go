// Package datastructure provides core graph and query data structures.
//
// Inspired by OSRM QueryHeap: https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp
//
// Reference:
// 1. Delling, D., Goldberg, A. V., Pajor, T., and Werneck, R. F. (2015). Customizable
// Route Planning in Road Networks. Transportation Science, 51(2), 566-591.
// https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
package datastructure

import (
	"math"

	"github.com/bits-and-blooms/bitset"
)

// ArrayStorage stores heap indices for vertices using a array and touched tracking.
// Unset positions are marked with [math.MaxUint32]. Touched indices are recorded in a touched list,
// enabling fast O(|touched|) resets between queries instead of O(|V|).
//
// From Delling et al. (2015) ref [1]:
//
//	"Finally, we keep track of all vertices touched by one quick, allowing for a quick initialization of the next one."
//
// remark: i think the word "quick" in this paper sentence is typo, the correct one is "query".
type ArrayStorage struct {
	base    []Index
	touched []Index
}

// NewArrayStorage creates and initializes an [ArrayStorage] with the given size.
// All entries are initialized to [math.MaxUint32] to indicate an unset state.
func NewArrayStorage(size Index) *ArrayStorage {
	base := make([]Index, size)
	for i := Index(0); i < size; i++ {
		base[i] = math.MaxUint32
	}
	return &ArrayStorage{
		base:    base,
		touched: make([]Index, 0, TOUCHED_VERTICES_SIZE),
	}
}

// Get returns the heap index associated with the given vertex id.
// If the vertex is not in the heap, it returns [math.MaxUint32].
//
//go:inline
func (s *ArrayStorage) Get(id Index) Index {
	return s.base[id]
}

// Set assigns the heap index info to the given vertex id.
// If the vertex was previously unset, its id is appended to the touched list.
//
//go:inline
func (s *ArrayStorage) Set(id Index, info Index) {
	if s.base[id] == math.MaxUint32 {
		s.touched = append(s.touched, id)
	}
	s.base[id] = info
}

// Clear resets all modified entries back to [math.MaxUint32] and empties the touched list.
//
//go:inline
func (s *ArrayStorage) Clear() {
	for _, id := range s.touched {
		s.base[id] = math.MaxUint32
	}
	s.touched = s.touched[:0]
}

// MapStorage stores heap indices for overlay/boundary vertices using a hash map.
// It is intended for sparse vertex sets such as overlay vertices where allocating
// a full flat array is unnecessary.
type MapStorage struct {
	overlay map[Index]Index
}

// NewMapStorage creates a new [MapStorage] preallocated with OVERLAY_VERTICES_SIZE capacity.
func NewMapStorage(baseSize Index) *MapStorage {
	return &MapStorage{
		overlay: make(map[Index]Index, OVERLAY_VERTICES_SIZE),
	}
}

// Get returns the heap index associated with the given vertex id.
// If the vertex does not exist in the map, it returns [math.MaxUint32].
//
//go:inline
func (s *MapStorage) Get(id Index) Index {
	// https://go.dev/blog/swisstable
	// go1.24 use swiss table for its hash table (open addressing)
	// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
	// open addressing avg case: unsuccessful search & insert in O(1/(1-a)) or O(1), assuming uniform hashing.
	val, ok := s.overlay[id]
	if !ok {
		return math.MaxUint32
	}
	return val
}

// Set maps the given vertex id to its heap index vertexIndex.
//
//go:inline
func (s *MapStorage) Set(id Index, vertexIndex Index) {
	s.overlay[id] = vertexIndex
}

// Clear removes all entries from the map.
//
//go:inline
func (s *MapStorage) Clear() {
	// https://go101.org/optimizations/6-map.html
	for key := range s.overlay {
		delete(s.overlay, key)
	}
}

// ExploredBitsetStorage tracks explored (exracted from priority queue) vertices during search using a bitset.
//
// Reference:
// https://abseil.io/fast/hints.html#bit-vectors-instead-of-sets
type ExploredBitsetStorage struct {
	explored *bitset.BitSet
}

// NewExploredBitsetStorage creates an [ExploredBitsetStorage] with the given capacity.
func NewExploredBitsetStorage(approxMaxSearchSize Index) *ExploredBitsetStorage {
	return &ExploredBitsetStorage{bitset.New(uint(approxMaxSearchSize))}
}

// Test reports whether the vertex at vertexIndex has been marked as explored.
func (sc *ExploredBitsetStorage) Test(vertexIndex Index) bool {
	return sc.explored.Test(uint(vertexIndex))
}

// Set marks the vertex at vertexIndex as explored.
func (sc *ExploredBitsetStorage) Set(vertexIndex Index) {
	sc.explored.Set(uint(vertexIndex))
}

// Clear unsets all bits in the bitset, resetting the explored state.
func (sc *ExploredBitsetStorage) Clear() {
	sc.explored.ClearAll()
}

package datastructure

import (
	"errors"
	"math"
)

type CRPQueryKey struct {
	node           Index
	entryExitPoint Index
	outEdgeId      Index
}

func (qk *CRPQueryKey) GetNode() Index {
	return qk.node
}

func (qk *CRPQueryKey) GetEntryExitPoint() Index {
	return qk.entryExitPoint
}

func (qk *CRPQueryKey) GetOutEdgeId() Index {
	return qk.outEdgeId
}

func NewCRPQueryKey(node Index, entryExitPoint Index) CRPQueryKey {
	return CRPQueryKey{node: node, entryExitPoint: entryExitPoint}
}

func NewCRPQueryKeyWithOutEdgeId(node, entryExitPoint, outEdgeId Index) CRPQueryKey {
	return CRPQueryKey{node: node, entryExitPoint: entryExitPoint, outEdgeId: outEdgeId}
}

type PriorityQueueNode[T comparable] struct {
	rank float64
	item T
}

func (p *PriorityQueueNode[T]) GetItem() T {
	return p.item
}

func (p *PriorityQueueNode[T]) GetRank() float64 {
	return p.rank
}

func NewPriorityQueueNode[T comparable](rank float64, item T) PriorityQueueNode[T] {
	return PriorityQueueNode[T]{rank: rank, item: item}
}

// MinHeap binary heap priorityqueue
type MinHeap[T comparable] struct {
	heap []PriorityQueueNode[T]
	d    int
	pos  map[T]int
}

func NewBinaryHeap[T comparable]() *MinHeap[T] {
	return NewdAryHeap[T](2)
}

func NewFourAryHeap[T comparable]() *MinHeap[T] {
	return NewdAryHeap[T](4)
}

func NewdAryHeap[T comparable](d int) *MinHeap[T] {
	return &MinHeap[T]{
		heap: make([]PriorityQueueNode[T], 0),
		d:    d,
		pos:  make(map[T]int),
	}
}

// parent get index dari parent
func (h *MinHeap[T]) parent(index int) int {
	return (index - 1) / h.d
}

// heapifyUp mempertahankan heap property. check apakah parent dari index lebih besar kalau iya swap, then recursive ke parent.  O(logN) tree height.
func (h *MinHeap[T]) heapifyUp(index int) {
	for index != 0 && h.heap[index].rank < h.heap[h.parent(index)].rank {
		h.Swap(index, h.parent(index))
		index = h.parent(index)
	}
}

// heapifyDown mempertahankan heap property. check apakah nilai salah satu children dari index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) tree height.
func (h *MinHeap[T]) heapifyDown(index int) {

	leftMostChild := index*h.d + 1
	if leftMostChild >= len(h.heap) {
		return
	}

	sentinel := leftMostChild + h.d
	if sentinel > len(h.heap) {
		sentinel = len(h.heap)
	}

	smallest := leftMostChild
	for i := leftMostChild + 1; i < sentinel; i++ {
		if h.heap[i].rank < h.heap[smallest].rank {
			smallest = i
		}
	}

	if h.heap[smallest].rank < h.heap[index].rank {
		h.Swap(index, smallest)

		h.heapifyDown(smallest)
	}
}

func (h *MinHeap[T]) Swap(i, j int) {
	h.heap[i], h.heap[j] = h.heap[j], h.heap[i]
	h.pos[h.heap[i].item] = i
	h.pos[h.heap[j].item] = j

}

// isEmpty check apakah heap kosong
func (h *MinHeap[T]) isEmpty() bool {
	return len(h.heap) == 0
}

// isEmpty check apakah heap kosong
func (h *MinHeap[T]) IsEmpty() bool {
	return len(h.heap) == 0
}

// size ukuran heap
func (h *MinHeap[T]) Size() int {
	return len(h.heap)
}

func (h *MinHeap[T]) Clear() {
	h.heap = make([]PriorityQueueNode[T], 0)
	h.pos = make(map[T]int)
}

// getMin mendapatkan nilai minimum dari min-heap (index 0)
func (h *MinHeap[T]) GetMin() (PriorityQueueNode[T], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T]{}, errors.New("heap is empty")
	}
	return h.heap[0], nil
}

func (h *MinHeap[T]) GetMinrank() float64 {
	if h.isEmpty() {
		return math.MaxFloat64
	}
	return h.heap[0].rank
}

// insert item baru
func (h *MinHeap[T]) Insert(key PriorityQueueNode[T]) {
	h.heap = append(h.heap, key)
	index := h.Size() - 1
	h.pos[key.item] = index
	h.heapifyUp(index)
}

// extractMin ambil nilai minimum dari min-heap (index 0) & pop dari heap. O(logN), heapifyDown(0) O(logN)
func (h *MinHeap[T]) ExtractMin() (PriorityQueueNode[T], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T]{}, errors.New("heap is empty")
	}
	root := h.heap[0]

	h.Swap(0, h.Size()-1)

	h.heap = h.heap[:h.Size()-1]
	h.pos[root.item] = -1
	if len(h.heap) > 0 {
		h.heapifyDown(0)
	}

	return root, nil
}

func (h *MinHeap[T]) getItemPos(item PriorityQueueNode[T]) int {
	return h.pos[item.item]
}

// decreaseKey update rank dari item min-heap.   O(logN) heapify.
func (h *MinHeap[T]) DecreaseKey(item PriorityQueueNode[T]) error {
	itemPos := h.getItemPos(item)
	if itemPos < 0 || itemPos >= h.Size() {
		return errors.New("invalid index or new value")
	}

	h.heap[itemPos] = item

	h.heapifyUp(itemPos)
	return nil
}

type AltQueryKey struct {
	vertex  Index
	
}

func NewAltQueryKey(vertex Index, ) AltQueryKey {
	return AltQueryKey{vertex: vertex,}
}

func (a *AltQueryKey) GetVertex() Index {
	return a.vertex
}

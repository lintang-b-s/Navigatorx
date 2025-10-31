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
	return CRPQueryKey{node: node, entryExitPoint: entryExitPoint, outEdgeId:outEdgeId}
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
	pos  map[T]int
}

func NewMinHeap[T comparable]() *MinHeap[T] {
	return &MinHeap[T]{
		heap: make([]PriorityQueueNode[T], 0),
		pos:  make(map[T]int),
	}
}

// parent get index dari parent
func (h *MinHeap[T]) parent(index int) int {
	return (index - 1) / 2
}

// leftChild get index dari left child
func (h *MinHeap[T]) leftChild(index int) int {
	return 2*index + 1
}

// rightChild get index dari right child
func (h *MinHeap[T]) rightChild(index int) int {
	return 2*index + 2
}

// heapifyUp mempertahankan heap property. check apakah parent dari index lebih besar kalau iya swap, then recursive ke parent.  O(logN) tree height.
func (h *MinHeap[T]) heapifyUp(index int) {
	for index != 0 && h.heap[index].rank < h.heap[h.parent(index)].rank {
		h.heap[index], h.heap[h.parent(index)] = h.heap[h.parent(index)], h.heap[index]

		h.pos[h.heap[index].item] = index
		h.pos[h.heap[h.parent(index)].item] = h.parent(index)
		index = h.parent(index)
	}
}

// heapifyDown mempertahankan heap property. check apakah nilai salah satu children dari index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) tree height.
func (h *MinHeap[T]) heapifyDown(index int) {
	smallest := index
	left := h.leftChild(index)
	right := h.rightChild(index)

	if left < len(h.heap) && h.heap[left].rank < h.heap[smallest].rank {
		smallest = left
	}
	if right < len(h.heap) && h.heap[right].rank < h.heap[smallest].rank {
		smallest = right
	}
	if smallest != index {
		h.heap[index], h.heap[smallest] = h.heap[smallest], h.heap[index]
		h.pos[h.heap[index].item] = index
		h.pos[h.heap[smallest].item] = smallest

		h.heapifyDown(smallest)
	}
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
	h.heap[0] = h.heap[h.Size()-1]
	h.heap = h.heap[:h.Size()-1]
	h.pos[root.item] = -1
	h.heapifyDown(0)
	return root, nil
}

// deleteNode delete node specific. O(N) linear search.
func (h *MinHeap[T]) DeleteNode(item PriorityQueueNode[T]) error {
	index := -1
	// Find the index of the node to delete
	for i := 0; i < h.Size(); i++ {
		if i == h.pos[item.item] {
			index = i
			break
		}
	}
	if index == -1 {
		return errors.New("key not found in the heap")
	}
	// Replace the node with the last element
	h.heap[index] = h.heap[h.Size()-1]
	h.heap = h.heap[:h.Size()-1]
	h.pos[item.item] = -1
	// Restore heap property
	h.heapifyUp(index)
	h.heapifyDown(index)
	return nil
}

// decreaseKey update rank dari item min-heap.   O(logN) heapify.
func (h *MinHeap[T]) DecreaseKey(item PriorityQueueNode[T]) error {
	if h.pos[item.item] < 0 || h.pos[item.item] >= h.Size() || item.rank > h.heap[h.pos[item.item]].rank {
		return errors.New("invalid index or new value")
	}
	h.heap[h.pos[item.item]] = item
	h.heapifyUp(h.pos[item.item])
	return nil
}

func (h *MinHeap[T]) Getitem(item T) PriorityQueueNode[T] {
	return h.heap[h.pos[item]]
}

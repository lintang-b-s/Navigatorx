package datastructure

import (
	"errors"

	"github.com/lintang-b-s/Navigatorx/pkg"
)

type CRPQueryKey struct {
	node           Index // nodeId or boundary/overlay nodeId
	entryExitPoint Index // edgeId or node query level (if node is an overlay vertex)
	outInEdgeId    Index //
	overlay        bool  // is node a boundary/overlay vertex
}

func (qk *CRPQueryKey) GetNode() Index {
	return qk.node
}

func (qk *CRPQueryKey) GetEntryExitPoint() Index {
	return qk.entryExitPoint
}

func (qk *CRPQueryKey) GetOutInEdgeId() Index {
	return qk.outInEdgeId
}

func (qk *CRPQueryKey) IsOverlay() bool {
	return qk.overlay
}

func NewDijkstraKey(node Index, entryExitPoint Index) CRPQueryKey {
	return CRPQueryKey{node: node, entryExitPoint: entryExitPoint}
}

func NewCRPQueryKey(node Index, entryExitPoint Index, overlay bool) CRPQueryKey {
	return CRPQueryKey{node: node, entryExitPoint: entryExitPoint, overlay: overlay}
}

func NewCRPQueryKeyWithOutInEdgeId(node, entryExitPoint, outInEdgeId Index) CRPQueryKey {
	return CRPQueryKey{node: node, entryExitPoint: entryExitPoint, outInEdgeId: outInEdgeId}
}

type PriorityQueueNode[T comparable] struct {
	item        T
	rank        float64
	queryInfoId uint32
}

func (p *PriorityQueueNode[T]) GetItem() T {
	return p.item
}

func (p *PriorityQueueNode[T]) GetRank() float64 {
	return p.rank
}

func (p *PriorityQueueNode[T]) SetRank(rank float64) {
	p.rank = rank
}

func (p *PriorityQueueNode[T]) getQueryInfoId() uint32 {
	return p.queryInfoId
}

func NewPriorityQueueNode[T comparable](rank float64, item T, queryInfoId uint32) PriorityQueueNode[T] {
	return PriorityQueueNode[T]{rank: rank, item: item, queryInfoId: queryInfoId}
}

// DAryHeap d-ary heap priorityqueue
type DAryHeap[T comparable] struct {
	heap []PriorityQueueNode[T]
	d    uint32
}

func NewBinaryHeap[T comparable]() *DAryHeap[T] {
	return NewdAryHeap[T](2)
}

func NewFourAryHeap[T comparable]() *DAryHeap[T] {
	return NewdAryHeap[T](4)
}

func NewdAryHeap[T comparable](d int) *DAryHeap[T] {
	return &DAryHeap[T]{
		heap: make([]PriorityQueueNode[T], 0),
		d:    uint32(d),
	}
}

func (h *DAryHeap[T]) Preallocate(maxSearchSize uint32) {
	h.heap = make([]PriorityQueueNode[T], 0, maxSearchSize)
}

// parent get index dari parent
func (h *DAryHeap[T]) parent(index uint32) uint32 {
	return (index - 1) / h.d
}

// heapifyUp mempertahankan heap property. check apakah parent dari index lebih besar kalau iya swap, then recursive ke parent.  O(logN) tree height.
func (h *DAryHeap[T]) heapifyUp(index uint32, updatePos func(queryInfoId, newHeapNodeId uint32)) {
	for index != 0 && h.heap[index].rank < h.heap[h.parent(index)].rank {
		h.Swap(index, h.parent(index), updatePos)
		index = h.parent(index)
	}
}

// heapifyDown mempertahankan heap property. check apakah nilai salah satu children dari index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) tree height.
func (h *DAryHeap[T]) heapifyDown(index uint32, updatePos func(queryInfoId, newHeapNodeId uint32)) {

	leftMostChild := index*h.d + 1
	if leftMostChild >= uint32(len(h.heap)) {
		return
	}

	sentinel := leftMostChild + h.d
	if sentinel > uint32(len(h.heap)) {
		sentinel = uint32(len(h.heap))
	}

	smallest := leftMostChild
	for i := leftMostChild + 1; i < sentinel; i++ {
		if h.heap[i].rank < h.heap[smallest].rank {
			smallest = i
		}
	}

	if h.heap[smallest].rank < h.heap[index].rank {
		h.Swap(index, smallest, updatePos)

		h.heapifyDown(smallest, updatePos)
	}
}

func (h *DAryHeap[T]) Swap(i, j uint32, updatePos func(queryInfoId, newHeapNodeId uint32)) {
	h.heap[i], h.heap[j] = h.heap[j], h.heap[i]

	updatePos(h.heap[i].getQueryInfoId(), i)
	updatePos(h.heap[j].getQueryInfoId(), j)
}

// isEmpty check apakah heap kosong
func (h *DAryHeap[T]) isEmpty() bool {
	return len(h.heap) == 0
}

// isEmpty check apakah heap kosong
func (h *DAryHeap[T]) IsEmpty() bool {
	return len(h.heap) == 0
}

// size ukuran heap
func (h *DAryHeap[T]) Size() uint32 {
	return uint32(len(h.heap))
}

func (h *DAryHeap[T]) Clear() {
	h.heap = h.heap[:0] //  buat slice length jadi 0, tapi capacity tetep sama
}

// getMin mendapatkan nilai minimum dari min-heap (index 0)
func (h *DAryHeap[T]) GetMin() (PriorityQueueNode[T], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T]{}, errors.New("heap is empty")
	}
	return h.heap[0], nil
}

func (h *DAryHeap[T]) GetMinrank() float64 {
	if h.isEmpty() {
		return pkg.INF_WEIGHT
	}
	return h.heap[0].rank
}

// insert item baru
func (h *DAryHeap[T]) Insert(key PriorityQueueNode[T], queryInfoId uint32, updatePos func(queryInfoId, newHeapNodeId uint32)) {
	h.heap = append(h.heap, key)
	index := uint32(h.Size() - 1)
	updatePos(queryInfoId, uint32(index))

	h.heapifyUp(index, updatePos)
}

// extractMin ambil node dg nilai minimum dari min-heap (index 0) & pop dari heap. O(logN), heapifyDown(0) O(logN)
func (h *DAryHeap[T]) ExtractMin(updatePos func(queryInfoId, newHeapNodeId uint32)) (PriorityQueueNode[T], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T]{}, errors.New("heap is empty")
	}
	root := h.heap[0]

	h.Swap(0, h.Size()-1, updatePos)

	h.heap = h.heap[:h.Size()-1]

	if len(h.heap) > 0 {
		h.heapifyDown(0, updatePos)
	}

	return root, nil
}

// decreaseKey update rank dari item min-heap.   O(logN) heapify.
func (h *DAryHeap[T]) DecreaseKey(itemPos uint32, rank float64, updatePos func(queryInfoId, newHeapNodeId uint32)) error {
	if itemPos < 0 || itemPos >= h.Size() || h.heap[itemPos].GetRank() < rank {
		return errors.New("invalid index or new value")
	}

	h.heap[itemPos].SetRank(rank)
	h.heapifyUp(itemPos, updatePos)
	return nil
}

type AltQueryKey struct {
	vertex Index
}

func NewAltQueryKey(vertex Index) AltQueryKey {
	return AltQueryKey{vertex: vertex}
}

func (a *AltQueryKey) GetVertex() Index {
	return a.vertex
}

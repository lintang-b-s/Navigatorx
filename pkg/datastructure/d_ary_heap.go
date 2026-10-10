package datastructure

import (
	"errors"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var ErrHeapEmpty = errors.New("heap is empty")

func NewDijkstraKey(node Index) QueryKey {
	return QueryKey{node: node}
}

type QueryKey struct {
	node       Index // nodeId or boundary/overlay nodeId
	queryLevel uint8
	overlay    bool // is node a boundary/overlay vertex
}

func (qk *QueryKey) GetNode() Index {
	return qk.node
}

func (qk *QueryKey) IsOverlay() bool {
	return qk.overlay
}

func (qk *QueryKey) GetQueryLevel() int {
	return int(qk.queryLevel)
}

func NewQKey(node Index, queryLevel uint8, overlay bool) QueryKey {
	return QueryKey{node: node, queryLevel: queryLevel, overlay: overlay}
}

type PriorityQueueNode[T comparable, W util.RoutingNumber] struct {
	item        T
	rank        W
	vertexIndex uint32
}

func (p *PriorityQueueNode[T, W]) GetItem() T {
	return p.item
}

func (p *PriorityQueueNode[T, W]) GetRank() W {
	return p.rank
}

func (p *PriorityQueueNode[T, W]) SetRank(rank W) {
	p.rank = rank
}

func NewPriorityQueueNode[T comparable, W util.RoutingNumber](rank W, item T, vertexIndex uint32) PriorityQueueNode[T, W] {
	return PriorityQueueNode[T, W]{rank: rank, item: item, vertexIndex: vertexIndex}
}

// DAryHeap d-ary heap priorityqueue
type DAryHeap[T comparable, W util.RoutingNumber] struct {
	heap []PriorityQueueNode[T, W]
	d    uint32
}

func NewBinaryHeap[T comparable, W util.RoutingNumber]() *DAryHeap[T, W] {
	return NewdAryHeap[T, W](2)
}

// inspired from implicit 4-ary min-heap code implementation of this paper: https://sidsen.azurewebsites.net//papers/heaps-alenex14.pdf
// code: http://code.google.com/p/priority-queue-testing/
//
// NewFourAryHeap create min 4-ary heap
func NewFourAryHeap[T comparable, W util.RoutingNumber]() *DAryHeap[T, W] {
	return NewdAryHeap[T, W](4)
}

func NewdAryHeap[T comparable, W util.RoutingNumber](d int) *DAryHeap[T, W] {
	return &DAryHeap[T, W]{
		heap: make([]PriorityQueueNode[T, W], 0),
		d:    uint32(d),
	}
}

func (h *DAryHeap[T, W]) Preallocate(maxSearchSize uint32) {
	h.heap = make([]PriorityQueueNode[T, W], 0, maxSearchSize)
}

// parent get index dari parent
func (h *DAryHeap[T, W]) parent(index uint32) uint32 {
	return (index - 1) / h.d
}

// heapifyUp mempertahankan heap property. check apakah parent dari index lebih besar kalau iya swap, then recursive ke parent.  O(logN) complete 2/4-ary tree height.
func (h *DAryHeap[T, W]) heapifyUp(index uint32, updatePos func(vertexIndex, newHeapNodeId uint32)) {
	for index != 0 && util.Lt(h.heap[index].rank, h.heap[h.parent(index)].rank) {
		h.Swap(index, h.parent(index), updatePos)
		index = h.parent(index)
	}
}

// heapifyDown mempertahankan heap property. check apakah nilai salah satu children dari index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) complete 2/4-ary tree height.
func (h *DAryHeap[T, W]) heapifyDown(index uint32, updatePos func(vertexIndex, newHeapNodeId uint32)) {

	leftMostChild := index*h.d + 1
	if leftMostChild >= uint32(len(h.heap)) { // this node (index) dont have any children nodes
		return
	}

	sentinel := leftMostChild + h.d
	if sentinel > uint32(len(h.heap)) {
		sentinel = uint32(len(h.heap))
	}
	smallest := leftMostChild
	for i := leftMostChild + 1; i < sentinel; i++ {
		if util.Lt(h.heap[i].rank, h.heap[smallest].rank) {
			smallest = i
		}
	}

	if util.Lt(h.heap[smallest].rank, h.heap[index].rank) {
		h.Swap(index, smallest, updatePos)
		h.heapifyDown(smallest, updatePos)
	}
}

func (h *DAryHeap[T, W]) Swap(i, j uint32, updatePos func(vertexIndex, newHeapNodeId uint32)) {
	h.heap[i], h.heap[j] = h.heap[j], h.heap[i]

	updatePos(h.heap[i].vertexIndex, i)
	updatePos(h.heap[j].vertexIndex, j)
}

// isEmpty check apakah heap kosong
func (h *DAryHeap[T, W]) isEmpty() bool {
	return len(h.heap) == 0
}

// isEmpty check apakah heap kosong
func (h *DAryHeap[T, W]) IsEmpty() bool {
	return len(h.heap) == 0
}

// size ukuran heap
func (h *DAryHeap[T, W]) Size() uint32 {
	return uint32(len(h.heap))
}

func (h *DAryHeap[T, W]) Clear() {
	h.heap = h.heap[:0] //  buat slice length jadi 0, tapi capacity tetep sama
}

// getMin mendapatkan nilai minimum dari min-heap (index 0)
func (h *DAryHeap[T, W]) GetMin() (PriorityQueueNode[T, W], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T, W]{}, ErrHeapEmpty
	}
	return h.heap[0], nil
}

func (h *DAryHeap[T, W]) GetMinrank() W {
	if h.isEmpty() {
		return util.Infinity[W]()
	}
	return h.heap[0].rank
}

// insert item baru
func (h *DAryHeap[T, W]) Insert(node PriorityQueueNode[T, W], vertexIndex uint32, updatePos func(vertexIndex, newHeapNodeId uint32)) {
	h.heap = append(h.heap, node)
	index := uint32(h.Size() - 1)
	updatePos(vertexIndex, uint32(index))
	h.heapifyUp(index, updatePos)
}

// extractMin ambil node dg nilai minimum dari min-heap (index 0) & pop dari heap. O(logN), heapifyDown(0) O(logN)
func (h *DAryHeap[T, W]) ExtractMin(updatePos func(vertexIndex, newHeapNodeId uint32)) (PriorityQueueNode[T, W], error) {
	if h.isEmpty() {
		return PriorityQueueNode[T, W]{}, ErrHeapEmpty
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
func (h *DAryHeap[T, W]) DecreaseKey(itemPos uint32, rank W, updatePos func(vertexIndex, newHeapNodeId uint32)) {
	h.heap[itemPos].SetRank(rank)
	h.heapifyUp(itemPos, updatePos)
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

package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// inspired by OSRM QueryHeap https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp

type QueryHeap[T comparable, W util.RoutingNumber] struct {
	heap         *DAryHeap[T, W] // 4-ary minheap
	verticesData []VertexData[W] // berisi cost, parent, heapNodeId (vertexIndex dari heapNode di 4-ary minheap array)

	verticesIndex     IndexStorage // map dari nodeId/edgeId/overlayVertexId dari graph & overlay graph ke vertexIndex dari verticesData
	maxEdgesInCell    uint32
	verticesIndexType IndexStorageType
	explored          ExploredSetStorage
}

func NewQueryHeap[T comparable, W util.RoutingNumber](baseSize, maxEdgesInCell uint32, tipe IndexStorageType, preallocateMinHeap bool) *QueryHeap[T, W] {
	minHeap := NewFourAryHeap[T, W]()
	approxMaxSearchSize := maxEdgesInCell*2 + OVERLAY_VERTICES_SIZE

	if preallocateMinHeap {
		// buat clone queryHeap dari crpQuery di alternativeRoutes gak perlu preallocate heap
		minHeap.Preallocate(approxMaxSearchSize)
	}

	switch tipe {
	case TWO_LEVEL_STORAGE:
		explored := NewExploredBitsetStorage(approxMaxSearchSize)
		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      make([]VertexData[W], 0, approxMaxSearchSize),
			verticesIndex:     NewTwoLevelStorage(baseSize, maxEdgesInCell),
			maxEdgesInCell:    maxEdgesInCell,
			verticesIndexType: tipe,
			explored:          explored,
		}
	case ARRAY_STORAGE:
		explored := NewExploredBitsetStorage(approxMaxSearchSize)

		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      make([]VertexData[W], 0, approxMaxSearchSize),
			verticesIndex:     NewArrayStorage(baseSize),
			maxEdgesInCell:    maxEdgesInCell,
			verticesIndexType: tipe,
			explored:          explored,
		}
	case MAP_STORAGE:
		explored := NewExploredSettorage(approxMaxSearchSize)

		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      make([]VertexData[W], 0, approxMaxSearchSize),
			verticesIndex:     NewMapStorage(baseSize),
			maxEdgesInCell:    maxEdgesInCell,
			verticesIndexType: tipe,
			explored:          explored,
		}
	default:
		explored := NewExploredBitsetStorage(approxMaxSearchSize)
		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      make([]VertexData[W], 0, approxMaxSearchSize),
			verticesIndex:     NewTwoLevelStorage(baseSize, maxEdgesInCell),
			maxEdgesInCell:    maxEdgesInCell,
			verticesIndexType: tipe,
			explored:          explored,
		}
	}
}

// updatePosition. buat update heapNodeId dari vertexIndex (dipake pas heapifyUp dan heapifyDown)
func (qh *QueryHeap[T, W]) updatePosition(nodeIndex uint32, newHeapNodeId uint32) {
	qh.verticesData[nodeIndex].SetHeapNodeId(newHeapNodeId)
}

// Insert. insert node ke priority queue
// node/id bisa berupa nodeId/edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) Insert(id Index, priority W, vData VertexData[W], queryKey T) {
	newVertexIndex := uint32(len(qh.verticesData))

	qh.verticesData = append(qh.verticesData, vData)

	qh.verticesIndex.Set(id, newVertexIndex)

	heapNode := NewPriorityQueueNode[T, W](priority,
		queryKey, newVertexIndex)
	qh.heap.Insert(heapNode, newVertexIndex, qh.updatePosition)
}

// ExtractMin. remove and extract heapNode dari heap dengan lowest priority
func (qh *QueryHeap[T, W]) ExtractMin() PriorityQueueNode[T, W] {
	topNode, _ := qh.heap.ExtractMin(qh.updatePosition)
	return topNode
}

// DecreaseKey. decreaseKey() operation dari min heap. decrease priority dari node ke newPriority
// node/id bisa berupa nodeId/edgeId/overlayVertexId dari graph & overlay graph
// newPriority adlh priority dari vertex di 4-ary min heap, kalau ALT priority dari pq beda sama estimated sp cost/vCost
// vCost adalah estimate sp cost dari vertex
func (qh *QueryHeap[T, W]) DecreaseKey(id Index, newPriority, vCost W, newPar VertexEdgePair) {
	vertexIndex := qh.verticesIndex.Get(id)
	heapNodeId := qh.verticesData[vertexIndex].GetHeapNodeId()
	qh.verticesData[vertexIndex].UpdateParent(newPar)
	qh.verticesData[vertexIndex].UpdateCost(vCost)
	qh.heap.DecreaseKey(heapNodeId, newPriority, qh.updatePosition)
}

// Get. Get sp cost dari node
// node/id bisa berupa nodeId/edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) GetCost(id Index) W {
	vertexIndex := qh.verticesIndex.Get(id)
	if vertexIndex == math.MaxUint32 {
		return util.Infinity[W]()
	}
	return qh.verticesData[vertexIndex].GetCost()
}

// Clear. ya clear
// dipake karena queryheap reussable objects (routing engine pakai sync.Pool)
func (qh *QueryHeap[T, W]) Clear() {
	qh.verticesIndex.Clear()
	qh.verticesData = qh.verticesData[:0] //  buat slice length jadi 0, tapi capacity tetep sama, buat prevent array doubling dari dynamic array (slice)
	// ingat: reslicing slice gak bakal bikin slice baru/resliced slices tetep refer ke original slice (https://go.dev/blog/slices-intro)
	qh.heap.Clear()
	qh.explored.Clear(qh.maxEdgesInCell)
}

// Get. get vertexIndex dari node
// node/id bisa berupa nodeId/edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) Get(id Index) VertexData[W] {
	vertexIndex := qh.verticesIndex.Get(id)
	return qh.verticesData[vertexIndex]
}

func (qh *QueryHeap[T, W]) PreallocateHeap(initHeapSize int) {
	qh.heap.Preallocate(uint32(initHeapSize))
}

// Size. return heap size
func (qh *QueryHeap[T, W]) Size() uint32 {
	return qh.heap.Size()
}

// GetMinRank. get priority/rank of top heapNode
func (qh *QueryHeap[T, W]) GetMinrank() W {
	return qh.heap.GetMinrank()
}

// Explore. mark node as explored
// node/id bisa berupa nodeId/edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) Explore(id Index) {
	vertexIndex := qh.verticesIndex.Get(id)
	qh.explored.Set(vertexIndex)
}

func (qh *QueryHeap[T, W]) Set(id Index, vData VertexData[W], queryKey T) {
	vertexIndex := qh.verticesIndex.Get(id)
	if vertexIndex == math.MaxUint32 {
		newVertexIndex := uint32(len(qh.verticesData))
		qh.verticesData = append(qh.verticesData, vData)
		qh.verticesIndex.Set(id, newVertexIndex)
		return
	}

	qh.verticesData[vertexIndex].UpdateParent(vData.GetParent())
	qh.verticesData[vertexIndex].UpdateCost(vData.GetCost())
}

func (qh *QueryHeap[T, W]) IsEmpty() bool {
	return qh.heap.isEmpty()
}

func (qh *QueryHeap[T, W]) SetQueryLevel(id Index, qLevel uint8) {
	vertexIndex := qh.verticesIndex.Get(id)
	qh.verticesData[vertexIndex].parent.SetQueryLevel(qLevel)
}

func (qh *QueryHeap[T, W]) IsExplored(id Index) bool {
	vertexIndex := qh.verticesIndex.Get(id)
	if vertexIndex == math.MaxUint32 { // belum ke label & ke explored
		return false
	}
	return qh.explored.Test(vertexIndex)
}

func (qh *QueryHeap[T, W]) IsLabelled(id Index) bool {
	vertexIndex := qh.verticesIndex.Get(id)
	if vertexIndex == math.MaxUint32 { // belum ke label & ke explored
		return false
	}
	return true
}

// ForLabelledItems. get all items inserted to pq.
// karena kita support turn costs:
// offsetedVId bisa berupa edgeId atau overlay vertex id.
func (qh *QueryHeap[T, W]) ForLabelledItems(handle func(offsetedVId Index, vData VertexData[W])) {
	qh.verticesIndex.ForAllItems(func(offsetedVId Index, nodeIndex uint32) {
		if nodeIndex == math.MaxUint32 {
			return //  belum ke label & ke explored
		}
		handle(offsetedVId, qh.verticesData[nodeIndex])
	})
}

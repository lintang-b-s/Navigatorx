package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type QueryHeap[T comparable] struct {
	heap           *DAryHeap[T]     // 4-ary minheap
	queryInfos     []VertexInfo     // berisi travelTime, parent, heapNodeId (index dari heapNode di heap array), scanned dari node. node bisa berupa edgeId/overlayVertexId dari graph. ingat, this crp query impl. support turn costs
	storage        QueryInfoStorage // map dari edgeId/overlayVertexId dari graph & overlay graph ke index dari queryInfos
	maxEdgesInCell int
	storageType    QueryInfoStorageType
}

func NewQueryHeap[T comparable](baseSize, maxEdgesInCell int, tipe QueryInfoStorageType, preallocateMinHeap bool) *QueryHeap[T] {

	minHeap := NewFourAryHeap[T]()
	if preallocateMinHeap {
		// buat clone queryHeap dari crpQuery di alternativeRoutes gak perlu preallocate heap
		allocateHeapCapacity := maxEdgesInCell*2 + OVERLAY_INFO_SIZE
		minHeap.Preallocate(allocateHeapCapacity)
	}

	switch tipe {
	case TWO_LEVEL_STORAGE:
		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, baseSize),
			storage:        NewTwoLevelStorage(baseSize, maxEdgesInCell),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
		}
	case ARRAY_STORAGE:
		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, baseSize),
			storage:        NewArrayStorage(baseSize),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
		}
	case MAP_STORAGE:
		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, baseSize),
			storage:        NewMapStorage(baseSize),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
		}
	default:
		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, baseSize),
			storage:        NewTwoLevelStorage(baseSize, maxEdgesInCell),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
		}
	}
}

// updatePosition. buat update heapNodeId dari queryInfo (dipake pas heapifyUp dan heapifyDown)
func (qh *QueryHeap[T]) updatePosition(queryInfoId, newHeapNodeId int) {
	qh.queryInfos[queryInfoId].SetHeapNodeId(newHeapNodeId)
}

// Insert. insert node ke priority queue
// node/id bisa berupa edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T]) Insert(id Index, priority float64, vInfo VertexInfo, queryKey T) {
	newQueryInfoid := len(qh.queryInfos)

	qh.queryInfos = append(qh.queryInfos, vInfo)

	qh.storage.Set(id, newQueryInfoid)

	heapNode := NewPriorityQueueNode[T](priority,
		queryKey, newQueryInfoid)
	qh.heap.Insert(heapNode, newQueryInfoid, qh.updatePosition)
}

// ExtractMin. remove and extract heapNode dari heap dengan lowest priority
func (qh *QueryHeap[T]) ExtractMin() PriorityQueueNode[T] {
	topNode, err := qh.heap.ExtractMin(qh.updatePosition)
	util.AssertPanic(err == nil, "pq is empty")
	return topNode
}

// DecreaseKey. decreaseKey() operation dari min heap. decrease priority dari node ke newPriority
// node/id bisa berupa edgeId/overlayVertexId dari graph & overlay graph
// newPriority adlh priority dari Multilevel-Dijkstra / Multilevel-ALT (A*, landmarks, and triangle intequality), kalau ALT priority dari pq beda sama estimated sp cost/qInfoPriority
func (qh *QueryHeap[T]) DecreaseKey(id Index, newPriority, qInfoPriority float64, newPar VertexEdgePair) {
	qInfoId := qh.storage.Get(id)
	heapNodeId := qh.queryInfos[qInfoId].GetHeapNodeId()
	qh.queryInfos[qInfoId].UpdateParent(newPar)
	qh.queryInfos[qInfoId].UpdateTravelTime(qInfoPriority)
	qh.heap.DecreaseKey(heapNodeId, newPriority, qh.updatePosition)
}

// Get. Get queryInfo dari node
// node/id bisa berupa edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T]) GetPriority(id Index) float64 {
	qInfoId := qh.storage.Get(id)
	if qInfoId == math.MaxInt {
		return pkg.INF_WEIGHT
	}
	return qh.queryInfos[qInfoId].GetTravelTime()
}

// Clear. ya clear
// dipake karena queryheap reussable objects (routing engine pakai sync.Pool)
func (qh *QueryHeap[T]) Clear() {
	qh.storage.Clear()
	qh.queryInfos = qh.queryInfos[:0] //  buat slice length jadi 0, tapi capacity tetep sama, buat prevent array doubling dari dynamic array (slice)
	qh.heap.Clear()
}

// Get. get queryInfo dari node
// node/id bisa berupa edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T]) Get(id Index) VertexInfo {
	qInfoId := qh.storage.Get(id)

	return qh.queryInfos[qInfoId]
}

func (qh *QueryHeap[T]) PreallocateHeap(initHeapSize int) {
	qh.heap.Preallocate(initHeapSize)
}

// Size. return heap size
func (qh *QueryHeap[T]) Size() int {
	return qh.heap.Size()
}

// GetMinRank. get priority/rank of top heapNode
func (qh *QueryHeap[T]) GetMinrank() float64 {
	return qh.heap.GetMinrank()
}

// Scan. mark node as scanned
// node/id bisa berupa edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T]) Scan(id Index) {
	qInfoId := qh.storage.Get(id)
	qh.queryInfos[qInfoId].Scan()
}

func (qh *QueryHeap[T]) SetFirstOverlayEntryExitId(id Index, firstEntryExitId Index) {
	qInfoId := qh.storage.Get(id)
	qh.queryInfos[qInfoId].SetFirstOverlayEntryExitId(firstEntryExitId)
}

func (qh *QueryHeap[T]) Set(id Index, vInfo VertexInfo, queryKey T) {
	newQueryInfoid := len(qh.queryInfos)

	qh.queryInfos = append(qh.queryInfos, vInfo)
	qh.storage.Set(id, newQueryInfoid)
}

func (qh *QueryHeap[T]) IsEmpty() bool {
	return qh.heap.isEmpty()
}

func (qh *QueryHeap[T]) SetQueryLevel(id Index, qLevel uint8) {
	qInfoId := qh.storage.Get(id)
	qh.queryInfos[qInfoId].parent.SetQueryLevel(qLevel)
}

func (qh *QueryHeap[T]) IsScanned(id Index) bool {
	qInfoId := qh.storage.Get(id)
	if qInfoId == math.MaxInt { // belum ke label & ke scan
		return false
	}
	return qh.queryInfos[qInfoId].IsScanned()
}

func (qh *QueryHeap[T]) IsLabelled(id Index) bool {
	qInfoId := qh.storage.Get(id)
	if qInfoId == math.MaxInt { // belum ke label & ke scan
		return false
	}
	return true
}

// ForLabelledItems. get all items inserted to pq.
// karena kita support turn costs:
// offsetedVId berupa edgeId atau overlay vertex id.
func (qh *QueryHeap[T]) ForLabelledItems(handle func(offsetedVId Index, vInfo VertexInfo)) {
	qh.storage.ForAllItems(func(offsetedVId Index, queryInfoId int) {
		if queryInfoId == math.MaxInt {
			return //  belum ke label & ke scan
		}
		handle(offsetedVId, qh.queryInfos[queryInfoId])
	})
}

// Clone. clone queryheap
// dipakai di alternative routes finder
func (qh *QueryHeap[T]) Clone() *QueryHeap[T] {

	newQheap := NewQueryHeap[T](len(qh.queryInfos), qh.maxEdgesInCell, qh.storageType, false)
	newQheap.storage = qh.storage.Clone()
	queryInfosClone := make([]VertexInfo, len(qh.queryInfos))
	copy(queryInfosClone, qh.queryInfos)
	newQheap.queryInfos = queryInfosClone
	return newQheap
}

package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// inspired by OSRM QueryHeap

type QueryHeap[T comparable] struct {
	heap           *DAryHeap[T]     // 4-ary minheap
	queryInfos     []VertexInfo     // berisi travelTime, parent, heapNodeId (index dari heapNode di heap array), scanned dari node. node bisa berupa edgeId/overlayVertexId dari graph. ingat, this crp query impl. support turn costs
	storage        QueryInfoStorage // map dari edgeId/overlayVertexId dari graph & overlay graph ke index dari queryInfos
	maxEdgesInCell uint32
	storageType    QueryInfoStorageType
	scanned        ScannedSetStorage
}

func NewQueryHeap[T comparable](baseSize, maxEdgesInCell uint32, tipe QueryInfoStorageType, preallocateMinHeap bool) *QueryHeap[T] {
	minHeap := NewFourAryHeap[T]()
	approxMaxSearchSize := maxEdgesInCell*2 + OVERLAY_INFO_SIZE

	if preallocateMinHeap {
		// buat clone queryHeap dari crpQuery di alternativeRoutes gak perlu preallocate heap
		minHeap.Preallocate(approxMaxSearchSize)
	}

	switch tipe {
	case TWO_LEVEL_STORAGE:
		scanned := NewScannedBitsetStorage(approxMaxSearchSize)
		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, approxMaxSearchSize),
			storage:        NewTwoLevelStorage(baseSize, maxEdgesInCell),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
			scanned:        scanned,
		}
	case ARRAY_STORAGE:
		scanned := NewScannedBitsetStorage(approxMaxSearchSize)

		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, approxMaxSearchSize),
			storage:        NewArrayStorage(baseSize),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
			scanned:        scanned,
		}
	case MAP_STORAGE:
		scanned := NewScannedSettorage(approxMaxSearchSize)

		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, approxMaxSearchSize),
			storage:        NewMapStorage(baseSize),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
			scanned:        scanned,
		}
	default:
		scanned := NewScannedBitsetStorage(approxMaxSearchSize)
		return &QueryHeap[T]{
			heap:           minHeap,
			queryInfos:     make([]VertexInfo, 0, approxMaxSearchSize),
			storage:        NewTwoLevelStorage(baseSize, maxEdgesInCell),
			maxEdgesInCell: maxEdgesInCell,
			storageType:    tipe,
			scanned:        scanned,
		}
	}
}

// updatePosition. buat update heapNodeId dari queryInfo (dipake pas heapifyUp dan heapifyDown)
func (qh *QueryHeap[T]) updatePosition(queryInfoId uint32, newHeapNodeId uint32) {
	qh.queryInfos[queryInfoId].SetHeapNodeId(newHeapNodeId)
}

// Insert. insert node ke priority queue
// node/id bisa berupa edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T]) Insert(id Index, priority float64, vInfo VertexInfo, queryKey T) {
	newQueryInfoid := uint32(len(qh.queryInfos))

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

// Get. Get queryInfo travel time dari node
// node/id bisa berupa edgeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T]) GetPriority(id Index) float64 {
	qInfoId := qh.storage.Get(id)
	if qInfoId == math.MaxUint32 {
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
	util.AssertPanic(qInfoId != math.MaxUint32, "qInfoId must not equal to math.MaxUint32")
	return qh.queryInfos[qInfoId]
}

func (qh *QueryHeap[T]) PreallocateHeap(initHeapSize int) {
	qh.heap.Preallocate(uint32(initHeapSize))
}

// Size. return heap size
func (qh *QueryHeap[T]) Size() uint32 {
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
	qh.scanned.Set(qInfoId)
}

func (qh *QueryHeap[T]) SetFirstOverlayEntryExitId(id Index, firstEntryExitId Index) {
	qInfoId := qh.storage.Get(id)
	qh.queryInfos[qInfoId].SetFirstOverlayEntryExitId(firstEntryExitId)
}

func (qh *QueryHeap[T]) Set(id Index, vInfo VertexInfo, queryKey T) {
	qInfoId := qh.storage.Get(id)
	if qInfoId == math.MaxUint32 {
		newQueryInfoid := uint32(len(qh.queryInfos))
		qh.queryInfos = append(qh.queryInfos, vInfo)
		qh.storage.Set(id, newQueryInfoid)
		return
	}

	qh.queryInfos[qInfoId].UpdateParent(vInfo.GetParent())
	qh.queryInfos[qInfoId].UpdateTravelTime(vInfo.GetTravelTime())
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
	if qInfoId == math.MaxUint32 { // belum ke label & ke scan
		return false
	}
	return qh.scanned.Test(qInfoId)
}

func (qh *QueryHeap[T]) IsLabelled(id Index) bool {
	qInfoId := qh.storage.Get(id)
	if qInfoId == math.MaxUint32 { // belum ke label & ke scan
		return false
	}
	return true
}

// ForLabelledItems. get all items inserted to pq.
// karena kita support turn costs:
// offsetedVId bisa berupa edgeId atau overlay vertex id.
func (qh *QueryHeap[T]) ForLabelledItems(handle func(offsetedVId Index, vInfo VertexInfo)) {
	qh.storage.ForAllItems(func(offsetedVId Index, queryInfoId uint32) {
		if queryInfoId == math.MaxUint32 {
			return //  belum ke label & ke scan
		}
		handle(offsetedVId, qh.queryInfos[queryInfoId])
	})
}

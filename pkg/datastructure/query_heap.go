package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// inspired by OSRM QueryHeap https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp
// and from implicit 4-ary min-heap code implementation of this paper: https://sidsen.azurewebsites.net//papers/heaps-alenex14.pdf (code: http://code.google.com/p/priority-queue-testing/)

type QueryHeap[T comparable, W util.Numeric] struct {
	heap         []PriorityQueueNode[T, W] // implicit 4-ary min-heap
	verticesData []VertexData[W]           // contains cost, parent, heapNodeId (index dari heapNode di 4-ary minheap array) for all vertices inserted into the min-heap.

	verticesIndex     IndexStorage // map from nodeId/overlayVertexId to index in verticesData
	explored          *ExploredBitsetStorage
	verticesIndexType IndexStorageType
	d                 Index // d-ary heap

	// CRP Query Offsetting Context
	isCRPQuery          bool
	isLevelOneCellQuery bool
	sCellOffset         Index
	sCellSize           Index
	tCellOffset         Index
	tCellSize           Index
	maxVerticesInCell   Index
	overlayOffset       Index // 2 * maxVerticesInCell
	numVertices         Index // graph.NumberOfVertices()
}

func NewQueryHeap[T comparable, W util.Numeric](baseSize Index, tipe IndexStorageType) *QueryHeap[T, W] {

	approxMaxSearchSize := Index(BASE_VERTICES_SIZE)
	minHeap := make([]PriorityQueueNode[T, W], 0, approxMaxSearchSize)
	verticesData := make([]VertexData[W], 0, approxMaxSearchSize)
	switch tipe {
	case ARRAY_STORAGE:
		explored := NewExploredBitsetStorage(approxMaxSearchSize)

		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewArrayStorage(baseSize),
			verticesIndexType: tipe,
			explored:          explored,
			d:                 4,
		}
	case MAP_STORAGE:
		explored := NewExploredBitsetStorage(approxMaxSearchSize)

		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewMapStorage(approxMaxSearchSize),
			verticesIndexType: tipe,
			explored:          explored,
			d:                 4,
		}
	default:
		explored := NewExploredBitsetStorage(approxMaxSearchSize)

		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewArrayStorage(approxMaxSearchSize),
			verticesIndexType: tipe,
			explored:          explored,
			d:                 4,
		}
	}
}

// parent get index dari parent
//
//go:inline
func (qh *QueryHeap[T, W]) parent(index Index) Index {
	shift := qh.d >> 1
	return (index - 1) >> shift
}

//go:inline
func (qh *QueryHeap[T, W]) swap(i, j Index) {
	qh.heap[i], qh.heap[j] = qh.heap[j], qh.heap[i]
	qh.verticesData[qh.heap[i].vertexIndex].heapNodeId = i
	qh.verticesData[qh.heap[j].vertexIndex].heapNodeId = j
}

// heapifyUp mempertahankan min-heap property. check apakah parent dari node index lebih besar kalau iya swap parent dengan node index, then recursive ke parent.  O(logN) complete 2/4-ary tree height.
//
//go:inline
func (qh *QueryHeap[T, W]) heapifyUp(index Index) {
	for index != 0 {
		parentIndex := qh.parent(index)
		if util.Lt(qh.heap[index].rank, qh.heap[parentIndex].rank) {
			qh.swap(index, parentIndex)
			index = parentIndex
		} else {
			break
		}
	}
}

// Insert. insert node ke priority queue
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
//
//go:inline
func (qh *QueryHeap[T, W]) Insert(id Index, priority W, vData VertexData[W], queryKey T) {
	mid := qh.mapId(id)

	newVIndex := Index(len(qh.verticesData))
	heapNodeId := Index(qh.Size())
	vData.heapNodeId = heapNodeId
	qh.verticesData = append(qh.verticesData, vData)
	qh.verticesIndex.Set(mid, newVIndex)

	node := NewPriorityQueueNode[T, W](priority,
		queryKey, newVIndex)
	qh.heap = append(qh.heap, node)
	qh.heapifyUp(heapNodeId)
}

// isEmpty check apakah heap kosong
func (qh *QueryHeap[T, W]) isEmpty() bool {
	return len(qh.heap) == 0
}

// heapifyDown mempertahankan min-heap property. check apakah nilai salah satu children dari node index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) complete 2/4-ary tree height.
//
//go:inline
func (qh *QueryHeap[T, W]) heapifyDown(index Index) {

	shift := qh.d >> 1
	n := qh.Size()
	for {
		leftMostChild := (index << shift) + 1
		if leftMostChild >= n { // this node (index) dont have any children nodes
			break
		}
		sentinel := leftMostChild + qh.d
		sentinel = min(sentinel, n)

		smallest := leftMostChild
		for i := leftMostChild + 1; i < sentinel; i++ {
			if util.Lt(qh.heap[i].rank, qh.heap[smallest].rank) {
				smallest = i
			}
		}

		if util.Lt(qh.heap[smallest].rank, qh.heap[index].rank) {
			qh.swap(index, smallest)
			index = smallest
		} else {
			break
		}
	}
}

// ExtractMin. remove and extract heapNode dari heap dengan lowest priority
//
//go:inline
func (qh *QueryHeap[T, W]) ExtractMin() PriorityQueueNode[T, W] {
	if qh.isEmpty() {
		return PriorityQueueNode[T, W]{}
	}
	root := qh.heap[0]
	lastId := qh.Size() - 1

	if lastId > 0 {
		qh.swap(0, lastId)
		qh.heap = qh.heap[:lastId]
		qh.heapifyDown(0)
	} else {
		qh.heap = qh.heap[:0]
	}

	return root
}

// DecreaseKey. decreaseKey() operation dari min heap. decrease priority dari node ke newPriority
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
// newPriority adlh priority dari vertex di 4-ary min heap, kalau ALT priority dari pq beda sama estimated sp cost/vCost
// vCost adalah estimate sp cost dari vertex
//
//go:inline
func (qh *QueryHeap[T, W]) DecreaseKey(id Index, newPriority, vCost W, newPar ParentVertex) {
	mid := qh.mapId(id)
	vIndex := qh.verticesIndex.Get(mid)
	heapNodeId := qh.verticesData[vIndex].heapNodeId
	qh.verticesData[vIndex].parent = newPar
	qh.verticesData[vIndex].cost = vCost
	qh.heap[heapNodeId].rank = newPriority

	qh.heapifyUp(heapNodeId)
}

// Get. Get est sp cost dari node
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
//
//go:inline
func (qh *QueryHeap[T, W]) GetCost(id Index) W {
	mid := qh.mapId(id)
	vIndex := qh.verticesIndex.Get(mid)
	if vIndex == math.MaxUint32 {
		return util.Infinity[W]()
	}
	return qh.verticesData[vIndex].cost
}

// Clear. ya clear
// dipake karena queryheap reussable objects (routing engine pakai sync.Pool)
//
//go:inline
func (qh *QueryHeap[T, W]) Clear() {
	qh.verticesIndex.Clear()
	qh.verticesData = qh.verticesData[:0] //  buat slice length jadi 0, tapi capacity tetep sama, buat prevent array doubling dari dynamic array (slice)
	// ingat: reslicing slice gak bakal bikin slice baru/resliced slices tetep refer ke original slice (https://go.dev/blog/slices-intro)
	qh.heap = qh.heap[:0]
	qh.explored.Clear()
}

// Get. get VertexData dari node
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
//
//go:inline
func (qh *QueryHeap[T, W]) Get(id Index) VertexData[W] {
	mid := qh.mapId(id)
	vIndex := qh.verticesIndex.Get(mid)
	return qh.verticesData[vIndex]
}

func (qh *QueryHeap[T, W]) PreallocateHeap(maxSearchSize int) {
	if cap(qh.heap) < maxSearchSize {
		qh.heap = make([]PriorityQueueNode[T, W], 0, maxSearchSize)
	}
}

// Size. return heap size
func (qh *QueryHeap[T, W]) Size() Index {

	return Index(len(qh.heap))
}

// GetMinRank. get priority/rank of top heapNode
//
//go:inline
func (qh *QueryHeap[T, W]) GetMinrank() W {
	if len(qh.heap) == 0 {
		return util.Infinity[W]()
	}
	return qh.heap[0].rank
}

// Explore. mark node as explored
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
//
//go:inline
func (qh *QueryHeap[T, W]) Explore(id Index) {
	mid := qh.mapId(id)
	vIndex := qh.verticesIndex.Get(mid)
	qh.explored.Set(vIndex)
}

//go:inline
func (qh *QueryHeap[T, W]) Set(id Index, vData VertexData[W], queryKey T) {
	mid := qh.mapId(id)
	vIndex := qh.verticesIndex.Get(mid)
	if vIndex == math.MaxUint32 {

		newVIndex := Index(len(qh.verticesData))
		qh.verticesData = append(qh.verticesData, vData)
		qh.verticesIndex.Set(mid, newVIndex)
		return
	}

	qh.verticesData[vIndex].parent = vData.parent
	qh.verticesData[vIndex].cost = vData.cost
}

func (qh *QueryHeap[T, W]) IsEmpty() bool {
	return len(qh.heap) == 0
}

//go:inline
func (qh *QueryHeap[T, W]) IsExplored(id Index) bool {
	mid := qh.mapId(id)
	vIndex := qh.verticesIndex.Get(mid)
	if vIndex == math.MaxUint32 { // belum ke label & ke explored
		return false
	}
	return qh.explored.Test(vIndex)
}

//go:inline
func (qh *QueryHeap[T, W]) IsLabelled(id Index) bool {
	mid := qh.mapId(id)
	vIndex := qh.verticesIndex.Get(mid)
	if vIndex == math.MaxUint32 { // belum ke label & ke explored
		return false
	}
	return true
}

/*
--- Customizable Route Planning (CRP) (Delling et al. (2015)) Query optimization related. ----
ref1:  Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable
Route Planning in Road Networks. Transportation Science, No. 2, Volume 51,
pages 566-591. Available at: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

this explanation taken from ref1 :
"A second optimization uses the fact that the maximum heap size can be computed in advance.
Any search will scan at most all overlay vertices and the vertices in the cells of s and t on the lowest level. This allows us to preallocate all necessary data structures.
To index the heap, we use local identifiers (for vertices in the source and target cells, with different offsets), and overlay identifiers otherwise." (Delling et al. (2015))


another CRP implementation that implement this optimization: https://github.com/michaelwegner/CRP/blob/master/algorithm/CRPQuery.cpp
search: "vId -= forwardSOffset;".

this can be done because of graph.vertices is sorted by its vertex level-1 cellNumber, in such a way that vertices located within the same level-1 cell have consecutive IDs.
*/

// mapId maps an input vertex Id into the contiguous offseted index range [0, 2*maxV + numOverlayVertices).
//
//go:inline
func (qh *QueryHeap[T, W]) mapId(id Index) Index {
	if !qh.isCRPQuery && !qh.isLevelOneCellQuery {
		return id
	}
	if qh.isLevelOneCellQuery {
		return id - qh.sCellOffset
	}

	// crp query
	if id >= qh.numVertices {
		// Overlay vertex: (in query phase implementation (multilevel_dijkstra.go) we offset the overlay vertex id as:  id + numVertices
		// make the offseted overlay/boundary vertex id in range  [2*maxV, 2*maxV + numOverlayVertices)
		return qh.overlayOffset + (id - qh.numVertices)
	}
	if id >= qh.sCellOffset && id < qh.sCellOffset+qh.sCellSize {
		// vertex inside source level-1 cell Cs. make the offseted vertex id in range [0, maxV)
		return id - qh.sCellOffset
	}

	// vertex inside target level-1 cell Ct. make the offseted vertex i in range [maxV, 2*maxV)
	return qh.maxVerticesInCell + (id - qh.tCellOffset)
}

// InitCRPQuery initializes the contiguous cell offset context for the query.
func (qh *QueryHeap[T, W]) InitCRPQuery(sOffset, sSize, tOffset, tSize, maxV, nv Index) {
	qh.isCRPQuery = true
	qh.sCellOffset = sOffset
	qh.sCellSize = sSize
	qh.tCellOffset = tOffset
	qh.tCellSize = tSize
	qh.maxVerticesInCell = maxV
	qh.overlayOffset = 2 * maxV
	qh.numVertices = nv
}

// SetIsCRPQuery sets whether this query heap uses CRP query offsetting.
func (qh *QueryHeap[T, W]) SetIsCRPQuery() {
	qh.isCRPQuery = true
}

func (qh *QueryHeap[T, W]) SetLevelOneCellQuery() {
	qh.isLevelOneCellQuery = true
}

// InitCRPQuery initializes the contiguous cell offset context for the query.
func (qh *QueryHeap[T, W]) InitLevelOneCellQuery(sOffset Index) {
	qh.isLevelOneCellQuery = true
	qh.sCellOffset = sOffset
}

// --- PQ Node type related ---

func NewDijkstraKey(node Index) QueryKey {
	return QueryKey{node: node}
}

type QueryKey struct {
	node       Index // vertexId or boundary/overlay vertexId
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

type PriorityQueueNode[T comparable, W util.Numeric] struct {
	item        T
	rank        W
	vertexIndex Index
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

func NewPriorityQueueNode[T comparable, W util.Numeric](rank W, item T, vIndex Index) PriorityQueueNode[T, W] {
	return PriorityQueueNode[T, W]{rank: rank, item: item, vertexIndex: vIndex}
}

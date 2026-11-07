package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// inspired by OSRM QueryHeap https://github.com/Project-OSRM/osrm-backend/blob/master/include/util/query_heap.hpp
// and from implicit 4-ary min-heap code implementation of this paper: https://sidsen.azurewebsites.net//papers/heaps-alenex14.pdf (code: http://code.google.com/p/priority-queue-testing/)

// QueryHeap represents query heap.
type QueryHeap[T comparable, W util.Numeric] struct {
	verticesData []VertexData[T, W]     // contains cost, parent, heapNodeId (index dari heapNode di 4-ary minheap array) for all vertices inserted into the min-heap.
	heap         []PriorityQueueNode[W] // implicit 4-ary min-heap

	verticesIndex     IndexStorage // map from nodeId/overlayVertexId to index in verticesData
	explored          *ExploredBitsetStorage
	verticesIndexType IndexStorageType
	inf               W

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

const (
	defaultDAryHeap = 4
	defaultShift    = 4 >> 1
)

// NewQueryHeap creates and returns query heap.
func NewQueryHeap[T comparable, W util.Numeric](baseSize, approxMaxExploredSize Index, tipe IndexStorageType) *QueryHeap[T, W] {

	minHeap := make([]PriorityQueueNode[W], 0, approxMaxExploredSize)
	verticesData := make([]VertexData[T, W], 0, approxMaxExploredSize)
	inf := util.Infinity[W]()
	switch tipe {
	case ArrayCRPStorageMode:
		explored := NewExploredBitsetStorage(approxMaxExploredSize)

		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewArrayCRPStorage(baseSize),
			verticesIndexType: tipe,
			explored:          explored,
			inf:               inf,
		}
	case MapStorageMode:
		explored := NewExploredBitsetStorage(approxMaxExploredSize)
		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewMapStorage(),
			verticesIndexType: tipe,
			explored:          explored,
			inf:               inf,
		}
	case ArrayStorageUniMode:
		explored := NewExploredBitsetStorage(0)
		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewArrayCRPStorage(baseSize),
			verticesIndexType: tipe,
			explored:          explored,
			inf:               inf,
		}
	case ArrayStorageMode:
		explored := NewExploredBitsetStorage(approxMaxExploredSize)
		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewArrayStorage(baseSize),
			verticesIndexType: tipe,
			explored:          explored,
			inf:               inf,
		}
	default:
		explored := NewExploredBitsetStorage(approxMaxExploredSize)
		return &QueryHeap[T, W]{
			heap:              minHeap,
			verticesData:      verticesData,
			verticesIndex:     NewArrayCRPStorage(approxMaxExploredSize),
			verticesIndexType: tipe,
			explored:          explored,
			inf:               inf,
		}
	}
}

// parent get index dari parent
func parent(index Index) Index {
	return (index - 1) >> defaultShift
}

func (qh *QueryHeap[T, W]) swap(i, j Index) {
	qh.heap[i], qh.heap[j] = qh.heap[j], qh.heap[i]
	qh.verticesData[qh.heap[i].vertexIndex].heapNodeId = i
	qh.verticesData[qh.heap[j].vertexIndex].heapNodeId = j
}

// heapifyUp mempertahankan min-heap property. check apakah parent dari node index lebih besar kalau iya swap parent dengan node index, then recursive ke parent.  O(logN) complete 2-ary (or 4-ary) tree height.
// func (qh *QueryHeap[T, W]) heapifyUp(index Index) {

// 	for index != 0 {
// 		parentIndex := parent(index)
// 		if util.Lt(qh.heap[index].rank, qh.heap[parentIndex].rank) {
// 			qh.swap(index, parentIndex)
// 			index = parentIndex
// 		} else {
// 			break
// 		}
// 	}
// }

// heapifyDown mempertahankan min-heap property. check apakah nilai salah satu children dari node index lebih kecil kalau iya swap, then recursive ke children yang kecil tadi.  O(logN) complete 2-ary (or 4-ary) tree height.
func (qh *QueryHeap[T, W]) heapifyDown(index Index) {

	n := qh.Size()
	for {
		leftMostChild := (index << defaultShift) + 1
		if leftMostChild >= n { // this node (index) dont have any children nodes
			break
		}

		sentinel := leftMostChild + defaultDAryHeap
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

// optHeapifyUp maintains min-heap property. optimized version of heapifyUp().
// place the bubble node (or current node) to the right position by checking if parent node bigger than current node,
// if yes, then place the parent node to current node (bubble down the parent), and recurse to parent node,
// repeat this process until parent node is smaller than or equal to this current node (or stop when the current node is root node),
// then in the end, we place the bubble node to the current node index.
// O(logN) complete 2-ary (or 4-ary) tree height.
// inspired by pater sanders binary heap  optimized implementation in section A.1.2: https://dl.acm.org/doi/epdf/10.1145/351827.384249
// https://web.archive.org/web/20050119031404/http://www.mpi-sb.mpg.de/~sanders/programs/spq/heap4.h
func (qh *QueryHeap[T, W]) optHeapifyUp(index Index) {
	if index == 0 {
		return
	}

	heap := qh.heap
	vData := qh.verticesData
	bubble := heap[index]
	pred := parent(index)
	for index > 0 {
		par := heap[pred]
		if util.Le(par.rank, bubble.rank) {
			break
		}
		// if parent.rank > bubble.rank
		heap[index] = par
		vData[par.vertexIndex].heapNodeId = index

		index = pred
		pred = parent(pred)
	}

	heap[index] = bubble
	vData[heap[index].vertexIndex].heapNodeId = index
}

// IsEmpty check apakah heap kosong
func (qh *QueryHeap[T, W]) IsEmpty() bool {
	return len(qh.heap) == 0
}

// Insert insert node to priority queue
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) Insert(id Index, rank W, vData VertexData[T, W]) {
	id = qh.mapId(id)

	newVIndex := Index(len(qh.verticesData))
	heapNodeId := Index(qh.Size())
	vData.heapNodeId = heapNodeId
	qh.verticesData = append(qh.verticesData, vData)
	qh.verticesIndex.Set(id, newVIndex)

	node := NewPriorityQueueNode[W](rank,
		newVIndex)
	qh.heap = append(qh.heap, node)
	qh.optHeapifyUp(heapNodeId)
}

// ExtractMin remove and extract heapNode dari heap dengan lowest rank
func (qh *QueryHeap[T, W]) ExtractMin() (T, W) {
	if qh.IsEmpty() {
		var zero T
		return zero, W(0)
	}

	rootvIndex := qh.heap[0].vertexIndex
	lastId := qh.Size() - 1
	if lastId > 0 {
		lastNode := qh.heap[lastId]
		qh.heap[0] = lastNode
		qh.verticesData[lastNode.vertexIndex].heapNodeId = 0
		qh.heap = qh.heap[:lastId]
		qh.heapifyDown(0)
	} else {
		qh.heap = qh.heap[:0]
	}

	rootItem := qh.verticesData[rootvIndex].item
	rootCost := qh.verticesData[rootvIndex].cost
	return rootItem, rootCost
}

// DecreaseKey decreaseKey() operation dari min heap. decrease rank dari node ke newRank
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
// newRank adlh rank dari vertex di 4-ary min heap, kalau ALT rank dari pq beda sama estimated sp cost/vCost
// vCost adalah estimate sp cost dari vertex
func (qh *QueryHeap[T, W]) DecreaseKey(id Index, newRank, vCost W, newPar Index) {
	id = qh.mapId(id)
	vIndex := qh.verticesIndex.Get(id)
	heapNodeId := qh.verticesData[vIndex].heapNodeId
	qh.verticesData[vIndex].parent = newPar
	qh.verticesData[vIndex].cost = vCost
	qh.heap[heapNodeId].rank = newRank
	qh.optHeapifyUp(heapNodeId)
}

// Cost Get est sp cost dari node
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) Cost(id Index) W {
	id = qh.mapId(id)
	vIndex := qh.verticesIndex.Get(id)
	if vIndex == math.MaxUint32 {
		return qh.inf
	}
	return qh.verticesData[vIndex].cost
}

// Clear ya clear
// dipake karena queryheap reussable objects (routing engine pakai sync.Pool)
func (qh *QueryHeap[T, W]) Clear() {
	qh.verticesIndex.Clear()
	qh.verticesData = qh.verticesData[:0] //  buat slice length jadi 0, tapi capacity tetep sama, buat prevent array doubling dari dynamic array (slice)
	// ingat: reslicing slice gak bakal bikin slice baru/resliced slices tetep refer ke original slice (https://go.dev/blog/slices-intro)
	qh.heap = qh.heap[:0]
	qh.explored.Clear()
}

// Get get VertexData dari node
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) Get(id Index) VertexData[T, W] {
	id = qh.mapId(id)
	vIndex := qh.verticesIndex.Get(id)
	return qh.verticesData[vIndex]
}

// Size return heap size
func (qh *QueryHeap[T, W]) Size() Index {
	return Index(len(qh.heap))
}

// MinRank returns rank of top heapNode.
func (qh *QueryHeap[T, W]) MinRank() W {
	if len(qh.heap) == 0 {
		return qh.inf
	}
	return qh.heap[0].rank
}

// Explore mark node as explored
// node/id bisa berupa nodeId/overlayVertexId dari graph & overlay graph
func (qh *QueryHeap[T, W]) Explore(id Index) {
	id = qh.mapId(id)
	vIndex := qh.verticesIndex.Get(id)
	qh.explored.Set(vIndex)
}

// Set sets set vertex data of node id.
func (qh *QueryHeap[T, W]) Set(id Index, vData VertexData[T, W]) {
	id = qh.mapId(id)
	vIndex := qh.verticesIndex.Get(id)
	if vIndex == math.MaxUint32 {
		newVIndex := Index(len(qh.verticesData))
		qh.verticesData = append(qh.verticesData, vData)
		qh.verticesIndex.Set(id, newVIndex)
		return
	}

	qh.verticesData[vIndex].parent = vData.parent
	qh.verticesData[vIndex].cost = vData.cost
}

// IsExplored reports whether a vertex (with identifier id) explored.
func (qh *QueryHeap[T, W]) IsExplored(id Index) bool {
	id = qh.mapId(id)
	vIndex := qh.verticesIndex.Get(id)
	if vIndex == math.MaxUint32 { // belum ke label & ke explored
		return false
	}
	return qh.explored.Test(vIndex)
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

this can be done because of graph.vertices is sorted by its vertex level-1 cellNumber, in such a way that vertices located within the same level-1 cell have consecutive Ids.
*/

// mapId maps an input vertex Id into the contiguous offseted index range [0, 2*maxV + numOverlayVertices).
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

// InitCRPQuery initializes the contiguous cell offset context for the CRP query.
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

// SetLevelOneCellQuery sets level one cell query.
func (qh *QueryHeap[T, W]) SetLevelOneCellQuery() {
	qh.isLevelOneCellQuery = true
}

// InitLevelOneCellQuery initializes the contiguous cell offset context for the level-1 cell CRP query.
func (qh *QueryHeap[T, W]) InitLevelOneCellQuery(sOffset Index) {
	qh.isLevelOneCellQuery = true
	qh.sCellOffset = sOffset
}

// --- PQ Node type related ---

// QueryKey represents query key.
type QueryKey struct {
	node Index // vertexId or boundary/overlay vertexId
	flag uint8 // query level [0, 7] and overlay boolean.
}

// GetNode returns node.
func (qk QueryKey) GetNode() Index {
	return qk.node
}

// IsOverlay reports whether overlay.
func (qk QueryKey) IsOverlay() bool {
	return qk.flag&overlayFlag != 0
}

// QueryLevel returns query level.
func (qk QueryKey) QueryLevel() int {
	return int(qk.flag & ^(overlayFlag))
}

// NewQKey creates and returns qkey.
func NewQKey(node Index, queryLevel uint8, overlay bool) QueryKey {
	if !overlay {
		return QueryKey{node: node, flag: queryLevel}
	}
	flag := queryLevel | overlayFlag
	return QueryKey{node: node, flag: flag}
}

// PriorityQueueNode represents priority queue node.
type PriorityQueueNode[W util.Numeric] struct {
	rank        W
	vertexIndex Index
}

// NewPriorityQueueNode creates and returns priority queue node.
func NewPriorityQueueNode[T comparable, W util.Numeric](rank W, vIndex Index) PriorityQueueNode[W] {
	return PriorityQueueNode[W]{rank: rank, vertexIndex: vIndex}
}

const (
	overlayFlag uint8 = 1 << 3
)

// VertexData represents vertex data.
type VertexData[T comparable, W util.Numeric] struct {
	item       T
	parent     Index
	cost       W
	heapNodeId Index // 4 byte
}

// NewVData creates and returns vertex data.
func NewVData[T comparable, W util.Numeric](cost W, item T, parent Index) VertexData[T, W] {
	return VertexData[T, W]{
		cost:       cost,
		parent:     parent,
		item:       item,
		heapNodeId: 0,
	}
}

// GetParent returns parent.
func (vi VertexData[T, W]) GetParent() Index {
	return vi.parent
}

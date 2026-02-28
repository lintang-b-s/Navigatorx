package routing

import (
	"math"
	"time"

	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type CRPRoutingEngine struct {
	graph        *da.Graph
	overlayGraph *da.OverlayGraph
	metrics      *met.Metric
	logger       *zap.Logger
	puCache      *lru.Cache[PUCacheKey, []da.Index]

	customizer   Customizer
	costFunction CostFunction
}

func NewCRPRoutingEngine(graph *da.Graph,
	overlayGraph *da.OverlayGraph, metrics *met.Metric,
	logger *zap.Logger, puCache *lru.Cache[PUCacheKey, []da.Index],
	customizer Customizer, costFunction CostFunction) *CRPRoutingEngine {
	return &CRPRoutingEngine{
		graph:        graph,
		metrics:      metrics,
		overlayGraph: overlayGraph,
		logger:       logger,
		puCache:      puCache,
		customizer:   customizer,
		costFunction: costFunction,
	}
}

func (crp *CRPRoutingEngine) GetGraph() *da.Graph {
	return crp.graph
}

func (crp *CRPRoutingEngine) GetOverlayGraph() *da.OverlayGraph {
	return crp.overlayGraph
}

type CRPBidirectionalSearch struct {
	engine             *CRPRoutingEngine
	shortestTravelTime float64
	forwardMid         vertexEdgePair
	backwardMid        vertexEdgePair

	forwardInfo   *TwoLevelStorage[da.CRPQueryKey]
	backwardInfo  *TwoLevelStorage[da.CRPQueryKey]
	stallingEntry []float64
	stallingExit  []float64

	forwardPq  *da.MinHeap[da.CRPQueryKey]
	backwardPq *da.MinHeap[da.CRPQueryKey]
	fScanned   []bool
	bScanned   []bool

	sCellNumber da.Pv
	tCellNumber da.Pv

	viaVertices []da.ViaVertex

	sForwardId  da.Index
	tBackwardId da.Index

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numScannedVertices   int
	runtime              int64
	pathUnpackingRuntime int64
	lastpqSum            float64
}

func NewCRPBidirectionalSearch(engine *CRPRoutingEngine, upperBound float64) *CRPBidirectionalSearch {
	return &CRPBidirectionalSearch{
		engine:        engine,
		forwardInfo:   NewTwoLevelStorage[da.CRPQueryKey](0, 0),
		backwardInfo:  NewTwoLevelStorage[da.CRPQueryKey](0, 0),
		forwardPq:     da.NewFourAryHeap[da.CRPQueryKey](),
		backwardPq:    da.NewFourAryHeap[da.CRPQueryKey](),
		forwardMid:    newVertexEdgePair(0, 0, false),
		backwardMid:   newVertexEdgePair(0, 0, true),
		viaVertices:   make([]da.ViaVertex, 0),
		upperBound:    upperBound,
		stallingEntry: make([]float64, 0),
		stallingExit:  make([]float64, 0),
		fScanned:      make([]bool, 0),
		bScanned:      make([]bool, 0),

		numScannedVertices: 0,
		lastpqSum:          0,
	}
}

/*
implementation of:
1. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
2. query phase: Delling, D. et al. (20data11) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.

time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
extract-min at most O(m_p+n_o) operations

multilevel-dijkstra / query phase of customizable route planning only search at most edges & vertices that in lowest level cells that containing s or t, and all overlay vertices & shortcuts all cells in each level (other than lowest level cells that containing s or t )
thus we can preallocate the capacity of distance slices and heap as max number of edges in each cell * 2 + number of overlayVertices

*/

func (bs *CRPBidirectionalSearch) ShortestPathSearch(asId, atId da.Index) (float64, float64, []da.Coordinate,
	[]da.OutEdge, bool) {
	// Our query algorithm takes as input a source arc as , a target arc at, the original graph G, the overlay graph
	// H = ∪i Hi , and computes the shortest path between the head vertex s of as and the tail vertex t of at.
	// asId exitPoint of outEdge u->s
	// atId entryPoint of inEdge t->v
	now := time.Now()

	s := bs.engine.graph.GetOutEdge(asId).GetHead()
	t := bs.engine.graph.GetInEdge(atId).GetTail()

	if s == t {
		return 0, 0, []da.Coordinate{}, []da.OutEdge{}, true
	}

	bs.Preallocate()

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	// strategy: use outEdges for forward search, use inEdges for backward search
	// for iterating outEdges, we need entryOffset. for iterating inEdges, we need exitOffset.

	sForwardId := bs.engine.graph.GetEntryOffset(s) + da.Index(bs.engine.graph.GetOutEdge(asId).GetEntryPoint())
	tBackwardId := bs.engine.graph.GetExitOffset(t) + da.Index(bs.engine.graph.GetInEdge(atId).GetExitPoint())

	sForwardId = bs.engine.offsetForward(s, sForwardId, bs.engine.graph.GetCellNumber(s), bs.sCellNumber)
	tBackwardId = bs.engine.offsetBackward(t, tBackwardId, bs.engine.graph.GetCellNumber(t), bs.sCellNumber)

	bs.sForwardId = sForwardId
	bs.tBackwardId = tBackwardId

	bs.shortestTravelTime = 2 * pkg.INF_WEIGHT

	shNode := da.NewPriorityQueueNode(0, da.NewCRPQueryKey(s, sForwardId, false))
	thNode := da.NewPriorityQueueNode(0, da.NewCRPQueryKey(t, tBackwardId, false))
	bs.forwardPq.Insert(shNode)
	bs.backwardPq.Insert(thNode)
	bs.forwardInfo.Set(sForwardId, NewVertexInfo[da.CRPQueryKey](0, newVertexEdgePair(da.INVALID_VERTEX_ID, sForwardId, false), shNode))
	bs.backwardInfo.Set(tBackwardId, NewVertexInfo[da.CRPQueryKey](0, newVertexEdgePair(da.INVALID_VERTEX_ID, tBackwardId, true), thNode))

	close := func(id da.Index, scanned []bool, info *TwoLevelStorage[da.CRPQueryKey]) {
		// scan item (can be edgeId or overlay vertex id)
		scanned[id] = true
		info.Get(id).Scan()
	}

	for bs.forwardPq.Size() > 0 && bs.backwardPq.Size() > 0 {
		minForward := bs.forwardPq.GetMinrank()
		minBackward := bs.backwardPq.GetMinrank()
		if da.Ge(minForward+minBackward, bs.shortestTravelTime*(bs.upperBound)) {
			bs.lastpqSum = minBackward + minForward
			break
		}

		// Customizable Route Planning In Road Networks, Delling et al., page 14:
		// Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
		// overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
		// version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
		// lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
		// the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
		// search backward on graph level 1
		// basically same as forward search, but using inEdges and exitPoint instead of outEdges and entryPoint
		queryKey, _ := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()
		if !uItem.IsOverlay() {
			close(uItem.GetEntryExitPoint(), bs.fScanned, bs.forwardInfo)
			bs.forwardGraphSearch(uItem, s)
		} else {
			close(bs.engine.offsetOverlay(uItem.GetNode()), bs.fScanned, bs.forwardInfo)
			bs.forwardOverlayGraphSearch(uItem)
		}

		queryKey, _ = bs.backwardPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			close(uItem.GetEntryExitPoint(), bs.bScanned, bs.backwardInfo)
			bs.backwardGraphSearch(uItem, t)
		} else {
			close(bs.engine.offsetOverlay(uItem.GetNode()), bs.bScanned, bs.backwardInfo)
			bs.backwardOverlayGraphSearch(uItem)
		}

		bs.numScannedVertices += 2
	}

	if bs.shortestTravelTime == 2*pkg.INF_WEIGHT {
		return pkg.INF_WEIGHT, pkg.INF_WEIGHT, []da.Coordinate{}, []da.OutEdge{}, false
	}

	packedPath := bs.engine.RetrievePackedPath(bs.forwardMid, bs.backwardMid,
		bs.forwardInfo, bs.backwardInfo, bs.sForwardId, bs.tBackwardId, bs.sCellNumber)

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	unpacker := NewPathUnpacker(bs.engine, bs.engine.metrics, bs.engine.puCache, true, false)
	finalPath, finalEdgePath, totalDistance := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)

	bs.pathUnpackingRuntime = unpacker.GetStats()

	return bs.shortestTravelTime, totalDistance, finalPath, finalEdgePath, true
}

/*
graphSearch. turn-aware bidirectional dijkstra search on graph level 1.

Customizable Route Planning In Road Networks, Delling et al., page 7-8:

We implement this by main-taining triples (v, i, d) in the heap, where v is a vertex, i the order of an entry point at v, and d a distance
label. The algorithm is initialized by (s, i, 0) indicating that we start the query from entry point i at vertex
s. (Note that one can generalize this to allow queries starting anywhere along an arc by inserting s, i with
an offset into the queue.) The distance value d of a label (v, i, d) then indicates the cost of the best path
seen so far from the source to the entry point i at vertex v.

To implement bidirectional search on the compact model, we maintain a tentative
shortest path distance µ, initialized by ∞. Then, we perform a forward search from an entry point at s,
operating as described above, and a backward search from an exit point of t that operates on the exit points
of the vertices.

Whenever we scan a vertex that has been seen from the other side, we evaluate all possible turns between all entry and exit points of the intersection and check
whether we can improve µ. We can stop the search as soon as the sum of the minimum keys in both priority
queues exceeds µ. Note that this algorithm basically performs a search from the head vertex (s) of an arc to
the tail (t) of another.

Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
*/
func (bs *CRPBidirectionalSearch) forwardGraphSearch(uItem da.CRPQueryKey, source da.Index) {

	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1

	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	uEntryPoint := bs.engine.getEntryPoint(uId, uEntryId, bs.engine.graph.GetEntryOffset(uId))

	// stalling
	uInDeg := bs.engine.graph.GetInDegree(uId)
	otherUEntryId := bs.engine.offsetForward(uId, bs.engine.graph.GetEntryOffset(uId), bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	for j := da.Index(0); j < uInDeg; j++ {

		stallingOffset := uInDeg*uEntryPoint + j
		bui := math.Max(0, bs.forwardInfo.Get(uEntryId).GetTravelTime()+
			bs.engine.metrics.GetEntryStallingTableCost(uId, stallingOffset))

		if val := bs.stallingEntry[otherUEntryId]; da.Eq(val, pkg.INF_WEIGHT) {
			bs.stallingEntry[otherUEntryId] = bui
		} else {
			bs.stallingEntry[otherUEntryId] = math.Min(bs.stallingEntry[otherUEntryId], bui)
		}
		otherUEntryId++
	}

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
		vId := outArc.GetHead()

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.metrics.GetWeight(outArc)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := bs.forwardInfo.Get(uEntryId).GetTravelTime() + edgeWeight + turnCost

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}
		vEntryId := bs.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())
		if vQueryLevel == 0 {

			vEntryId = bs.engine.offsetForward(vId, vEntryId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			// relax edge
			vAlreadyLabelled := da.Lt(bs.forwardInfo.Get(vEntryId).GetTravelTime(), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.forwardInfo.Get(vEntryId).GetTravelTime())) {
				if bvi := bs.stallingEntry[vEntryId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
					// stalled
					return
				}

				if vAlreadyLabelled {
					vhNode := bs.forwardInfo.Get(vEntryId).GetHeapNode()
					// newTravelTime is better, update the forwardInfo
					bs.forwardInfo.Get(vEntryId).UpdateTravelTime(newTravelTime)

					// is key already in the priority queue, decrease its key
					bs.forwardInfo.Get(vEntryId).UpdateParent(newVertexEdgePair(uId, uEntryId, false))
					bs.forwardPq.DecreaseKey(vhNode, newTravelTime)
				} else if !vAlreadyLabelled {
					vhNode := da.NewPriorityQueueNode(
						newTravelTime, da.NewCRPQueryKey(vId, vEntryId, false))

					// newTravelTime is better, update the forwardInfo
					bs.forwardInfo.Set(vEntryId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
						newVertexEdgePair(uId, uEntryId, false), vhNode))

					// is key not in the priority queue, insert it
					bs.forwardPq.Insert(vhNode)
				}
			}

			// check wether we already Labelled an exit point of vId

			exitOffset := bs.engine.graph.GetExitOffset(vId)

			exitOffset = bs.engine.offsetBackward(vId, exitOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vExitId := exitOffset

			// traverse outEdges of v
			bs.engine.graph.ForOutEdgesOf(vId, da.Index(outArc.GetEntryPoint()), func(e2 *da.OutEdge,
				exitPoint da.Index, turnType2 pkg.TurnType) {
				// Customizable Route Planning In Road Networks, Page 8: Whenever we scan a vertex that has been seen from
				// the other side, we evaluate all possible turns between all entry and exit points of the intersection and check
				// whether we can improve µ.
				// basically: check if forward and backward search already Labelled entry and exit point of v. if so, check whether we can improve the shortest path
				// if head of outEdge v->w already Labelled by backward search, and its forwardTravelTime + backwardTravelTime is better than shortestPath, then update shortestPath
				scannedByBackwardSearch := bs.bScanned[vExitId]
				if scannedByBackwardSearch && da.Lt(bs.forwardInfo.Get(vEntryId).GetTravelTime()+bs.engine.metrics.GetTurnCost(turnType2)+
					bs.backwardInfo.Get(vExitId).GetTravelTime(), bs.shortestTravelTime) {

					bs.shortestTravelTime = bs.forwardInfo.Get(vEntryId).GetTravelTime() + bs.engine.metrics.GetTurnCost(turnType2) +
						bs.backwardInfo.Get(vExitId).GetTravelTime()

					bs.forwardMid = newVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = newVertexEdgePair(vId, vExitId, true)
					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
				}
				vExitId++
			})

		} else {
			// v is in another cell on higher level
			// CRP In Road Networks: Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			// update the forward info of overlay vertex v
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := bs.engine.graph.GetOverlayVertex(vId, outArc.GetEntryPoint(), false)
			overlayVId := bs.engine.offsetOverlay(v)
			vAlreadyLabelled := da.Lt(bs.forwardInfo.Get(overlayVId).GetTravelTime(), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.forwardInfo.Get(overlayVId).GetTravelTime())) {

				if !vAlreadyLabelled {
					vhNode := da.NewPriorityQueueNode(
						newTravelTime, da.NewCRPQueryKey(v, da.Index(vQueryLevel), true))

					vertexInfo := NewVertexInfo[da.CRPQueryKey](newTravelTime,
						newVertexEdgePair(uId, uEntryId, false), vhNode)
					vertexInfo.parent.setFirstOverlayEntryExitId(vEntryId)

					bs.forwardInfo.Set(overlayVId, vertexInfo)

					bs.forwardPq.Insert(vhNode)
				} else {
					vhNode := bs.forwardInfo.Get(overlayVId).GetHeapNode()
					bs.forwardInfo.Get(overlayVId).UpdateTravelTime(newTravelTime)
					bs.forwardInfo.Get(overlayVId).UpdateParent(newVertexEdgePair(uId, uEntryId, false))
					bs.forwardInfo.Get(overlayVId).parent.setFirstOverlayEntryExitId(vEntryId)

					bs.forwardPq.DecreaseKey(vhNode, newTravelTime)
				}
			}

			scannedByBackwardSearch := bs.bScanned[overlayVId]
			// if v Labelled by backward search, check whether we can improve the shortestPath
			if scannedByBackwardSearch && da.Lt(bs.forwardInfo.Get(overlayVId).GetTravelTime()+bs.backwardInfo.Get(overlayVId).GetTravelTime(), bs.shortestTravelTime) {
				bs.shortestTravelTime = bs.forwardInfo.Get(overlayVId).GetTravelTime() + bs.backwardInfo.Get(overlayVId).GetTravelTime()

				bs.forwardMid = newVertexEdgePair(vId, overlayVId, false)
				bs.backwardMid = newVertexEdgePair(vId, overlayVId, true)

				vOverlay := bs.engine.overlayGraph.GetVertex(v)
				vInEdge := bs.engine.graph.GetInEdge(vOverlay.GetOriginalEdge())

				vExitId := bs.engine.graph.GetExitOffset(vInEdge.GetTail()) + da.Index(vInEdge.GetExitPoint())
				vEntryId := bs.engine.graph.GetEntryOffset(vId) + bs.engine.graph.GetEntryOrder(vId, vInEdge.GetEdgeId())

				vExitId = bs.engine.offsetBackward(uId, vExitId, vOverlay.GetCellNumber(), bs.sCellNumber)
				vEntryId = bs.engine.offsetForward(vId, vEntryId, vOverlay.GetCellNumber(), bs.sCellNumber)
				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, vId, true))
			}
		}
	})
}

func (bs *CRPBidirectionalSearch) backwardGraphSearch(uItem da.CRPQueryKey, target da.Index) {
	// search backward on graph level 1
	// basically same as forward search, but using inEdges and exitPoint instead of outEdges and entryPoint

	uId := uItem.GetNode()
	uExitId := uItem.GetEntryExitPoint() // index of outEdge that have endpoint from vertex uId

	uExitPoint := bs.engine.getExitPoint(uId, uExitId, bs.engine.graph.GetExitOffset(uId))

	// stalling
	uOutDeg := bs.engine.graph.GetOutDegree(uId)
	otherUExitId := bs.engine.offsetBackward(uId, bs.engine.graph.GetExitOffset(uId),
		bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	for j := da.Index(0); j < uOutDeg; j++ {

		stallingOffset := uOutDeg*uExitPoint + j
		bui := math.Max(0, bs.backwardInfo.Get(uExitId).GetTravelTime()+
			bs.engine.metrics.GetExitStallingTableCost(uId, stallingOffset))

		if val := bs.stallingExit[otherUExitId]; da.Eq(val, pkg.INF_WEIGHT) {
			bs.stallingExit[otherUExitId] = bui
		} else {
			bs.stallingExit[otherUExitId] = math.Min(bs.stallingExit[otherUExitId], bui)
		}
		otherUExitId++
	}

	bs.engine.graph.ForInEdgesOf(uId, uExitPoint, func(inArc *da.InEdge, entryPoint da.Index, turnType pkg.TurnType) {
		vId := inArc.GetTail()

		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.metrics.GetWeight(inArc)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)

		if uId == target {
			turnCost = 0
		}

		newTravelTime := bs.backwardInfo.Get(uExitId).GetTravelTime() + edgeWeight + turnCost
		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		vExitId := bs.engine.graph.GetExitOffset(vId) + da.Index(inArc.GetExitPoint())

		if vQueryLevel == 0 {

			vExitId = bs.engine.offsetBackward(vId, vExitId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// relax edge
			vAlreadyLabelled := da.Lt(bs.backwardInfo.Get(vExitId).GetTravelTime(), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.backwardInfo.Get(vExitId).GetTravelTime())) {

				if bvi := bs.stallingExit[vExitId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
					// stalled
					return
				}

				if vAlreadyLabelled {
					vhNode := bs.backwardInfo.Get(vExitId).GetHeapNode()
					bs.backwardInfo.Get(vExitId).UpdateTravelTime(newTravelTime)
					bs.backwardInfo.Get(vExitId).UpdateParent(newVertexEdgePair(uId, uExitId, true))

					bs.backwardPq.DecreaseKey(vhNode, newTravelTime)

				} else {

					vhNode := da.NewPriorityQueueNode(
						newTravelTime, da.NewCRPQueryKey(vId, vExitId, false))
					bs.backwardInfo.Set(vExitId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
						newVertexEdgePair(uId, uExitId, true), vhNode))

					bs.backwardPq.Insert(vhNode)
				}
			}

			// check wether we already Labelled an entry point
			entryOffset := bs.engine.graph.GetEntryOffset(vId)

			entryOffset = bs.engine.offsetForward(vId, entryOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vEntryId := entryOffset

			bs.engine.graph.ForInEdgesOf(vId, da.Index(inArc.GetExitPoint()), func(inArc2 *da.InEdge,
				entryPoint2 da.Index, turnType2 pkg.TurnType) {
				scannedByForwardSearch := bs.fScanned[vEntryId]
				if scannedByForwardSearch && da.Lt(bs.forwardInfo.Get(vEntryId).GetTravelTime()+bs.engine.metrics.GetTurnCost(turnType2)+
					bs.backwardInfo.Get(vExitId).GetTravelTime(), bs.shortestTravelTime) {

					bs.shortestTravelTime = bs.forwardInfo.Get(vEntryId).GetTravelTime() + bs.engine.metrics.GetTurnCost(turnType2) +
						bs.backwardInfo.Get(vExitId).GetTravelTime()

					bs.forwardMid = newVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = newVertexEdgePair(vId, vExitId, true)

					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
				}
				vEntryId++
			})

		} else {
			// v is in another cell on higher level
			// Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			v, _ := bs.engine.graph.GetOverlayVertex(vId, inArc.GetExitPoint(), true)
			overlayVId := bs.engine.offsetOverlay(v)
			vAlreadyLabelled := da.Lt(bs.backwardInfo.Get(overlayVId).GetTravelTime(), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.backwardInfo.Get(overlayVId).GetTravelTime())) {

				if !vAlreadyLabelled {
					vhNode := da.NewPriorityQueueNode(
						newTravelTime, da.NewCRPQueryKey(v, da.Index(vQueryLevel), true))

					vVertexInfo := NewVertexInfo[da.CRPQueryKey](newTravelTime,
						newVertexEdgePair(uId, uExitId, true), vhNode)
					vVertexInfo.parent.setFirstOverlayEntryExitId(vExitId)

					bs.backwardInfo.Set(overlayVId, vVertexInfo)

					bs.backwardPq.Insert(vhNode)
				} else {
					vhNode := bs.backwardInfo.Get(overlayVId).GetHeapNode()
					bs.backwardInfo.Get(overlayVId).UpdateTravelTime(newTravelTime)
					bs.backwardInfo.Get(overlayVId).UpdateParent(newVertexEdgePair(uId, uExitId, true))
					bs.backwardInfo.Get(overlayVId).parent.setFirstOverlayEntryExitId(vExitId)

					bs.backwardPq.DecreaseKey(vhNode, newTravelTime)
				}
			}

			scannedByForwardSearch := bs.fScanned[overlayVId]
			if scannedByForwardSearch && da.Lt(bs.forwardInfo.Get(overlayVId).GetTravelTime()+bs.backwardInfo.Get(overlayVId).GetTravelTime(), bs.shortestTravelTime) {
				bs.shortestTravelTime = bs.forwardInfo.Get(overlayVId).GetTravelTime() + bs.backwardInfo.Get(overlayVId).GetTravelTime()

				bs.forwardMid = newVertexEdgePair(vId, overlayVId, false)
				bs.backwardMid = newVertexEdgePair(vId, overlayVId, true)

				vOverlay := bs.engine.overlayGraph.GetVertex(v)
				vOutEdge := bs.engine.graph.GetOutEdge(vOverlay.GetOriginalEdge())

				vEntryId := bs.engine.graph.GetEntryOffset(vOutEdge.GetHead()) + da.Index(vOutEdge.GetEntryPoint())
				vExitId := bs.engine.graph.GetExitOffset(vId) + bs.engine.graph.GetExitOrder(vId, vOutEdge.GetEdgeId())

				vExitId = bs.engine.offsetBackward(vId, vExitId, vOverlay.GetCellNumber(), bs.sCellNumber)
				vEntryId = bs.engine.offsetForward(uId, vEntryId, vOverlay.GetCellNumber(), bs.sCellNumber)
				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, vId, true))
			}
		}
	})
}

/*
Customizable Route Planning In Road Networks, Delling et al., page 14:
Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
*/
func (bs *CRPBidirectionalSearch) forwardOverlayGraphSearch(uItem da.CRPQueryKey) {
	// search on overlay graph

	u := uItem.GetNode()              // overlay vertex id
	uId := bs.engine.offsetOverlay(u) // offseted overlay vertex id
	uVertex := bs.engine.overlayGraph.GetVertex(u)
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	// for each out neighbors of u in level l, check if v already Labelled by backward search. if so, check whether we can improve shortestPath
	// then if v not already Labelled or newTravelTime to v is better, traverse to the next cell entry vertex w using outEdge of v.
	bs.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.forwardInfo.Get(uId).GetTravelTime() + shortcutOutEdgeWeight

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}
		overlayVId := bs.engine.offsetOverlay(v)

		// traverse edge to next cell
		vOriEdgeId := vVertex.GetOriginalEdge()
		outEdge := bs.engine.graph.GetOutEdge(vOriEdgeId)
		edgeWeight := bs.engine.metrics.GetWeight(outEdge)

		/*
			We apply several optimizations. First, by construction, each exit vertex u in the overlay has a single
			outgoing arc (u, v). Therefore, during the search we do not add u to the priority queue; instead, we traverse
			the arc (u, v) immediately and process v.
		*/
		// w is in the next cell from v cell

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		originalWId := wVertex.GetOriginalVertex()

		// relax edge
		vAlreadyLabelled := da.Lt(bs.forwardInfo.Get(overlayVId).GetTravelTime(), pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.forwardInfo.Get(overlayVId).GetTravelTime())) {
			bs.forwardInfo.Set(overlayVId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
				newVertexEdgePair(uVertex.GetOriginalVertex(), uId, false), nil))

			// karena kita langsung scan v & traverse to its neighbor (exit vertex dari suatu cell), kita harus tandain kalau v udah di scan
			bs.fScanned[overlayVId] = true
			bs.forwardInfo.Get(overlayVId).Scan()

			newTravelTime = bs.forwardInfo.Get(overlayVId).GetTravelTime() + edgeWeight
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := bs.engine.graph.GetEntryOffset(originalWId) + da.Index(outEdge.GetEntryPoint())

				wEntryId = bs.engine.offsetForward(originalWId, wEntryId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				wAlreadyLabelled := da.Lt(bs.forwardInfo.Get(wEntryId).GetTravelTime(), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.forwardInfo.Get(wEntryId).GetTravelTime())) {
					if wAlreadyLabelled {
						whNode := bs.forwardInfo.Get(wEntryId).GetHeapNode()
						bs.forwardInfo.Get(wEntryId).UpdateTravelTime(newTravelTime)
						bs.forwardInfo.Get(wEntryId).UpdateParent(newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))
						bs.forwardPq.DecreaseKey(whNode, newTravelTime)

					} else {
						whNode := da.NewPriorityQueueNode(
							newTravelTime, da.NewCRPQueryKey(originalWId, wEntryId, false))

						bs.forwardInfo.Set(wEntryId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
							newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false), whNode))

						bs.forwardPq.Insert(whNode)
					}
				}

				// check whether we already Labelled an exit point
				exitOffset := bs.engine.graph.GetExitOffset(originalWId)

				exitOffset = bs.engine.offsetBackward(originalWId, exitOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				wExitId := exitOffset
				bs.engine.graph.ForOutEdgesOf(originalWId, da.Index(outEdge.GetEntryPoint()), func(e *da.OutEdge, exitPoint da.Index, turn pkg.TurnType) {
					// basically: check if forward and backward search already Labelled entry and exit point of w. if so, check whether we can improve the shortest path
					scannedByBackwardSearch := bs.bScanned[wExitId]
					if scannedByBackwardSearch && da.Lt(bs.forwardInfo.Get(wEntryId).GetTravelTime()+bs.engine.metrics.GetTurnCost(turn)+
						bs.backwardInfo.Get(wExitId).GetTravelTime(), bs.shortestTravelTime) {

						bs.shortestTravelTime = bs.forwardInfo.Get(wEntryId).GetTravelTime() + bs.engine.metrics.GetTurnCost(turn) +
							bs.backwardInfo.Get(wExitId).GetTravelTime()

						bs.forwardMid = newVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = newVertexEdgePair(originalWId, wExitId, true)

						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(originalWId, wEntryId, wExitId, originalWId, false))

					}
					wExitId++
				})
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardOverlayPq, because we need to traverse & relax shortcut edges in overlay graph
				overlayWId := bs.engine.offsetOverlay(w)
				wAlreadyLabelled := da.Lt(bs.forwardInfo.Get(overlayWId).GetTravelTime(), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.forwardInfo.Get(overlayWId).GetTravelTime())) {

					if !wAlreadyLabelled {
						whNode := da.NewPriorityQueueNode(
							newTravelTime, da.NewCRPQueryKey(w, da.Index(wQueryLevel), true))
						bs.forwardInfo.Set(overlayWId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
							newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false), whNode))

						bs.forwardPq.Insert(whNode)
					} else {
						whNode := bs.forwardInfo.Get(overlayWId).GetHeapNode()
						bs.forwardInfo.Get(overlayWId).UpdateTravelTime(newTravelTime)
						bs.forwardInfo.Get(overlayWId).UpdateParent(newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))
						bs.forwardPq.DecreaseKey(whNode, newTravelTime)
					}
				}

				scannedByBackwardSearch := bs.bScanned[overlayWId]
				if scannedByBackwardSearch && da.Lt(bs.forwardInfo.Get(overlayWId).GetTravelTime()+bs.backwardInfo.Get(overlayWId).GetTravelTime(), bs.shortestTravelTime) {
					// if overlay vertex w Labelled by backward search, check whether we can improve the shortestPath
					bs.shortestTravelTime = bs.forwardInfo.Get(overlayWId).GetTravelTime() + bs.backwardInfo.Get(overlayWId).GetTravelTime()

					bs.forwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, false)
					bs.backwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, true)

					wInEdge := bs.engine.graph.GetInEdge(wVertex.GetOriginalEdge())
					wExitId := bs.engine.graph.GetExitOffset(wInEdge.GetTail()) + da.Index(wInEdge.GetExitPoint())
					wEntryId := bs.engine.graph.GetEntryOffset(originalWId) + bs.engine.graph.GetEntryOrder(originalWId, wInEdge.GetEdgeId())

					wExitId = bs.engine.offsetBackward(vVertex.GetOriginalVertex(), wExitId, wVertex.GetCellNumber(), bs.sCellNumber)
					wEntryId = bs.engine.offsetForward(originalWId, wEntryId, wVertex.GetCellNumber(), bs.sCellNumber)
					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayWId, wEntryId, wExitId, originalWId, true))
				}
			}
		}

		scannedByBackwardSearch := bs.bScanned[overlayVId]
		if scannedByBackwardSearch && da.Lt(bs.forwardInfo.Get(overlayVId).GetTravelTime()+bs.backwardInfo.Get(overlayVId).GetTravelTime(), bs.shortestTravelTime) {

			bs.shortestTravelTime = bs.forwardInfo.Get(overlayVId).GetTravelTime() + bs.backwardInfo.Get(overlayVId).GetTravelTime()

			bs.forwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
			bs.backwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

			originalVId := vVertex.GetOriginalVertex()
			vOutEdge := bs.engine.graph.GetOutEdge(vVertex.GetOriginalEdge())
			vEntryId := bs.engine.graph.GetEntryOffset(vOutEdge.GetHead()) + da.Index(vOutEdge.GetEntryPoint())
			vExitId := bs.engine.graph.GetExitOffset(originalVId) + bs.engine.graph.GetExitOrder(originalVId, vOutEdge.GetEdgeId())

			vExitId = bs.engine.offsetBackward(originalVId, vExitId, vVertex.GetCellNumber(), bs.sCellNumber)
			vEntryId = bs.engine.offsetForward(originalWId, vEntryId, vVertex.GetCellNumber(), bs.sCellNumber)
			bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, originalVId, true)) // should use overlay vId (to calculate plateau, we need bactrack forwardInfo)
		}
	})
}

func (bs *CRPBidirectionalSearch) backwardOverlayGraphSearch(uItem da.CRPQueryKey) {
	// search backward on overlay graph
	// basically same as forward search on overlayGraph, but using inEdges and exitPoint instead of outEdges and entryPoint

	u := uItem.GetNode()

	uId := bs.engine.offsetOverlay(u) // offseted overlay id
	uVertex := bs.engine.overlayGraph.GetVertex(u)

	uQueryLevel := uItem.GetEntryExitPoint()

	bs.engine.overlayGraph.ForInNeighborsOf(u, int(uQueryLevel), func(v da.Index,
		wOffset da.Index) {

		shortcutInEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.backwardInfo.Get(uId).GetTravelTime() + shortcutInEdgeWeight

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		overlayVId := bs.engine.offsetOverlay(v)
		// traverse edge to next cell
		vOriEdgeId := vVertex.GetOriginalEdge()
		inEdge := bs.engine.graph.GetInEdge(vOriEdgeId)

		inEdgeWeight := bs.engine.metrics.GetWeight(inEdge)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		originalWId := wVertex.GetOriginalVertex()

		// relax edge
		vAlreadyLabelled := da.Lt(bs.backwardInfo.Get(overlayVId).GetTravelTime(), pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.backwardInfo.Get(overlayVId).GetTravelTime())) {

			bs.backwardInfo.Set(overlayVId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
				newVertexEdgePair(uVertex.GetOriginalVertex(), uId, true), nil))

			bs.bScanned[overlayVId] = true
			bs.backwardInfo.Get(overlayVId).Scan()

			newTravelTime = bs.backwardInfo.Get(overlayVId).GetTravelTime() + inEdgeWeight

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			if wQueryLevel == 0 {

				wExitId := bs.engine.graph.GetExitOffset(originalWId) + da.Index(inEdge.GetExitPoint())

				wExitId = bs.engine.offsetBackward(originalWId, wExitId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax edge
				wAlreadyLabelled := da.Lt(bs.backwardInfo.Get(wExitId).GetTravelTime(), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.backwardInfo.Get(wExitId).GetTravelTime())) {
					if wAlreadyLabelled {
						whNode := bs.backwardInfo.Get(wExitId).GetHeapNode()

						bs.backwardInfo.Get(wExitId).UpdateTravelTime(newTravelTime)
						bs.backwardInfo.Get(wExitId).UpdateParent(newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))
						bs.backwardPq.DecreaseKey(whNode, newTravelTime)
					} else {
						whNode := da.NewPriorityQueueNode(
							newTravelTime, da.NewCRPQueryKey(originalWId, wExitId, false))
						bs.backwardInfo.Set(wExitId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
							newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true), whNode))

						bs.backwardPq.Insert(whNode)
					}
				}

				// check whether we already scanned an entry point in forward search
				entryOffset := bs.engine.graph.GetEntryOffset(originalWId)

				entryOffset = bs.engine.offsetForward(originalWId, entryOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				wEntryId := entryOffset

				bs.engine.graph.ForInEdgesOf(originalWId, da.Index(inEdge.GetExitPoint()), func(e *da.InEdge,
					entryPoint da.Index, turn pkg.TurnType) {
					scannedByForwardSearch := bs.fScanned[wEntryId]
					if scannedByForwardSearch && da.Lt(bs.forwardInfo.Get(wEntryId).GetTravelTime()+bs.engine.metrics.GetTurnCost(turn)+
						bs.backwardInfo.Get(wExitId).GetTravelTime(), bs.shortestTravelTime) {

						bs.shortestTravelTime = bs.forwardInfo.Get(wEntryId).GetTravelTime() + bs.engine.metrics.GetTurnCost(turn) +
							bs.backwardInfo.Get(wExitId).GetTravelTime()

						bs.forwardMid = newVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = newVertexEdgePair(originalWId, wExitId, true)
						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(originalWId, wEntryId, wExitId, originalWId, false))

					}
					wEntryId++
				})
			} else {
				overlayWId := bs.engine.offsetOverlay(w)
				wAlreadyLabelled := da.Lt(bs.backwardInfo.Get(overlayWId).GetTravelTime(), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.backwardInfo.Get(overlayWId).GetTravelTime())) {

					if !wAlreadyLabelled {
						whNode := da.NewPriorityQueueNode(
							newTravelTime, da.NewCRPQueryKey(w, da.Index(wQueryLevel), true))
						bs.backwardInfo.Set(overlayWId, NewVertexInfo[da.CRPQueryKey](newTravelTime,
							newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true), whNode))

						bs.backwardPq.Insert(whNode)
					} else {
						whNode := bs.backwardInfo.Get(overlayWId).GetHeapNode()
						bs.backwardInfo.Get(overlayWId).UpdateTravelTime(newTravelTime)
						bs.backwardInfo.Get(overlayWId).UpdateParent(newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))
						bs.backwardPq.DecreaseKey(whNode, newTravelTime)
					}
				}

				scannedByForwardSearch := bs.fScanned[overlayWId]
				if scannedByForwardSearch && da.Lt(bs.forwardInfo.Get(overlayWId).GetTravelTime()+bs.backwardInfo.Get(overlayWId).GetTravelTime(), bs.shortestTravelTime) {
					bs.shortestTravelTime = bs.forwardInfo.Get(overlayWId).GetTravelTime() + bs.backwardInfo.Get(overlayWId).GetTravelTime()

					bs.forwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, false)
					bs.backwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, true)

					wOutEdge := bs.engine.graph.GetOutEdge(wVertex.GetOriginalEdge())
					wEntryId := bs.engine.graph.GetEntryOffset(wOutEdge.GetHead()) + da.Index(wOutEdge.GetEntryPoint())
					wExitId := bs.engine.graph.GetExitOffset(originalWId) + bs.engine.graph.GetExitOrder(originalWId, wOutEdge.GetEdgeId())

					wExitId = bs.engine.offsetBackward(originalWId, wExitId, wVertex.GetCellNumber(), bs.sCellNumber)
					wEntryId = bs.engine.offsetForward(vVertex.GetOriginalVertex(), wEntryId, wVertex.GetCellNumber(), bs.sCellNumber)
					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayWId, wEntryId, wExitId, originalWId, true))
				}
			}
		}

		scannedByForwardSearch := bs.fScanned[overlayVId]
		if scannedByForwardSearch && da.Lt(bs.backwardInfo.Get(overlayVId).GetTravelTime()+bs.forwardInfo.Get(overlayVId).GetTravelTime(), bs.shortestTravelTime) {

			bs.shortestTravelTime = bs.backwardInfo.Get(overlayVId).GetTravelTime() + bs.forwardInfo.Get(overlayVId).GetTravelTime()

			bs.forwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
			bs.backwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

			originalVId := vVertex.GetOriginalVertex()
			vInEdge := bs.engine.graph.GetInEdge(vVertex.GetOriginalEdge())
			vExitId := bs.engine.graph.GetExitOffset(vInEdge.GetTail()) + da.Index(vInEdge.GetExitPoint())
			vEntryId := bs.engine.graph.GetEntryOffset(originalVId) + bs.engine.graph.GetEntryOrder(originalVId, vInEdge.GetEdgeId())

			vExitId = bs.engine.offsetBackward(originalWId, vExitId, vVertex.GetCellNumber(), bs.sCellNumber)
			vEntryId = bs.engine.offsetForward(originalVId, vEntryId, vVertex.GetCellNumber(), bs.sCellNumber)
			bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, originalVId, true))
		}
	})
}

func (bs *CRPBidirectionalSearch) Preallocate() {
	maxEdgesInCell := bs.engine.graph.GetMaxEdgesInCell()
	numberOfOverlayVertices := bs.engine.overlayGraph.NumberOfOverlayVertices()
	maxSearchSize := int(maxEdgesInCell)*2 + numberOfOverlayVertices

	bs.forwardInfo = NewTwoLevelStorage[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell))
	bs.backwardInfo = NewTwoLevelStorage[da.CRPQueryKey](int(maxEdgesInCell)*2, int(maxEdgesInCell))

	bs.stallingEntry = make([]float64, maxSearchSize)
	bs.stallingExit = make([]float64, maxSearchSize)

	initInfWeight(bs.stallingEntry)
	initInfWeight(bs.stallingExit)

	bs.fScanned = make([]bool, maxSearchSize)
	bs.bScanned = make([]bool, maxSearchSize)

	bs.forwardPq.Preallocate(maxSearchSize)
	bs.backwardPq.Preallocate(maxSearchSize)
}

func (bs *CRPBidirectionalSearch) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf
	/*
		The efficiency of a run of a P2P algorithm is defined as
		the number of vertices on the shortest path divided by
		the number of vertices scanned by the algorithm.1 We
		report efficiency in percent. An optimal algorithm that
		scans only the shortest path vertices has 100% efficiency.
	*/

	efficiency := float64(n) / float64(bs.numScannedVertices)
	return efficiency, bs.numScannedVertices, bs.runtime, bs.pathUnpackingRuntime
}

func (bs *CRPBidirectionalSearch) GetLastPQSum() float64 {
	return bs.lastpqSum
}

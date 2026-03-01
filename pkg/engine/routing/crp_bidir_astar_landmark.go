package routing

import (
	"math"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
)

type CRPALTBidirectionalSearch struct {
	engine             *CRPRoutingEngine
	shortestTravelTime float64

	forwardMid  da.VertexEdgePair
	backwardMid da.VertexEdgePair

	stallingEntry []float64
	stallingExit  []float64

	forwardPq  *da.QueryHeap[da.CRPQueryKey]
	backwardPq *da.QueryHeap[da.CRPQueryKey]
	fScanned   []bool
	bScanned   []bool

	lm              *landmark.Landmark
	activeLandmarks []da.Index

	sCellNumber da.Pv
	tCellNumber da.Pv

	viaVertices []da.ViaVertex

	sForwardId  da.Index
	tBackwardId da.Index

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numScannedVertices        int
	numScannedOverlayVertices int
	runtime                   int64
	pathUnpackingRuntime      int64

	lastpqSum float64
}

func NewCRPALTBidirectionalSearch(engine *CRPRoutingEngine, upperBound float64, lm *landmark.Landmark) *CRPALTBidirectionalSearch {
	crpQuery := &CRPALTBidirectionalSearch{
		engine: engine,

		fScanned: make([]bool, 0),
		bScanned: make([]bool, 0),

		forwardMid:      da.NewVertexEdgePair(0, 0, false),
		backwardMid:     da.NewVertexEdgePair(0, 0, true),
		viaVertices:     make([]da.ViaVertex, 0),
		upperBound:      upperBound,
		stallingEntry:   make([]float64, 0),
		stallingExit:    make([]float64, 0),
		lm:              lm,
		activeLandmarks: make([]da.Index, 0),

		numScannedVertices:        0,
		runtime:                   0,
		pathUnpackingRuntime:      0,
		lastpqSum:                 0,
		numScannedOverlayVertices: 0,
	}

	crpQuery.Preallocate()
	return crpQuery
}

/*
implementation of:
1. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
2. query phase (Goal-direction): Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.
3. ALT query phase: Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
4. bidirectional A*: Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.
5. consistent heuristic for A* & optimality of A*: Hart, P.E., Nilsson, N.J. and Raphael, B. (1968) “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics, 4(2), pp. 100–107. Available at: https://doi.org/10.1109/TSSC.1968.300136.

time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
extract-min at most O(m_p+n_o) operations

multilevel-ALT (A*, Landmarks , and Triangle Inequality) only search at most edges & vertices that in lowest level cells that containing s or t, and all overlay vertices & shortcuts all cells in each level (other than lowest level cells that containing s or t )
thus we can preallocate the capacity of distance slices and heap as max number of edges in each cell * 2 + number of overlayVertices

https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf or https://web.stanford.edu/class/archive/cs/cs221/cs221.1196/lectures/search2-6pp.pdf
Hart et al (1967) [5] proved that the solution given by the A* algorithm will be optimal iff the heuristic is consistent (i.e. h(u) <= h(u,v) + h(v), h(u,v) sp distance from u to v), a consistent heuristic implies that the heuristic is also admissible (i.e h(u) <= spdist(u,t))
ALT (A*, Landmark, and triangle inequality) (Goldberg, A.V. and Harrelson, lm. (2005)) provides a consistent heuristic (by triangle inequality)
Ikeda et al. (1994) [4]  proved that Bidirectional A* is equivalent to bidirectional dijkstra with modified edge length iff the bidirectional A* implemented using heuristic function 1/2(hs(v)-ht(v)) for forward search and 1/2(ht(v)-hs(v)) for backward search, hs and ht are consistent/feasible heuristic function

todo: investigate kenapa kalau load tests dengan > 300 users, p95 latency nya naik drastis, (kayaknya berkatan dengan preallocate lagi?)
*/

func (bs *CRPALTBidirectionalSearch) ShortestPathSearch(asId, atId da.Index) (float64, float64, []da.Coordinate,
	[]da.OutEdge, bool) {
	// Our query algorithm takes as input a source arc as , a target arc at, the original graph G, the overlay graph
	// H = ∪i Hi , and computes the shortest path between the head vertex s of as and the tail vertex t of at.
	// asId exitPoint of outEdge u->s
	// atId entryPoint of inEdge t->v
	defer bs.Done()
	now := time.Now()

	s := bs.engine.graph.GetOutEdge(asId).GetHead()
	t := bs.engine.graph.GetInEdge(atId).GetTail()

	if s == t {
		return 0, 0, []da.Coordinate{}, []da.OutEdge{}, true
	}

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

	sVertexInfo := da.NewVertexInfo[da.CRPQueryKey](0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, sForwardId, false))
	tVertexInfo := da.NewVertexInfo[da.CRPQueryKey](0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, tBackwardId, true))
	sQueryKey := da.NewCRPQueryKey(s, sForwardId, false)
	tQueryKey := da.NewCRPQueryKey(t, tBackwardId, false)
	bs.forwardPq.Insert(sForwardId, 0, sVertexInfo, sQueryKey)
	bs.backwardPq.Insert(tBackwardId, 0, tVertexInfo, tQueryKey)

	bs.activeLandmarks = bs.lm.SelectBestQueryLandmarks(s, t)

	close := func(id da.Index, scanned []bool, queryHeap *da.QueryHeap[da.CRPQueryKey]) {
		// scan item (can be edgeId or overlay vertex id)
		scanned[id] = true
		queryHeap.Scan(id)
	}

	for bs.forwardPq.Size() > 0 && bs.backwardPq.Size() > 0 {
		minForward := bs.forwardPq.GetMinrank()
		minBackward := bs.backwardPq.GetMinrank()
		if da.Ge(minForward+minBackward, (bs.shortestTravelTime)*(bs.upperBound)) {
			bs.lastpqSum = minForward + minBackward
			break
		}

		// Customizable Route Planning In Road Networks, Delling et al., page 14:
		// Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
		// overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
		// version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
		// lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
		// the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;

		queryKey := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()

		if !uItem.IsOverlay() {
			close(uItem.GetEntryExitPoint(), bs.fScanned, bs.forwardPq)
			bs.forwardGraphSearch(uItem, s, t)
		} else {
			close(bs.engine.offsetOverlay(uItem.GetNode()), bs.fScanned, bs.forwardPq)
			bs.forwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		queryKey = bs.backwardPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			close(uItem.GetEntryExitPoint(), bs.bScanned, bs.backwardPq)
			bs.backwardGraphSearch(uItem, s, t)
		} else {
			close(bs.engine.offsetOverlay(uItem.GetNode()), bs.bScanned, bs.backwardPq)
			bs.backwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		bs.numScannedVertices += 2
	}

	if bs.shortestTravelTime == 2*pkg.INF_WEIGHT {
		return pkg.INF_WEIGHT, pkg.INF_WEIGHT, []da.Coordinate{}, []da.OutEdge{}, false
	}

	packedPath := bs.engine.RetrievePackedPath(bs.forwardMid, bs.backwardMid,
		bs.forwardPq, bs.backwardPq, bs.sForwardId, bs.tBackwardId, bs.sCellNumber)

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	unpacker := NewPathUnpackerALT(bs.engine, bs.engine.metrics, bs.engine.puCache, true, bs.lm)
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
func (bs *CRPALTBidirectionalSearch) forwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {

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
		bui := math.Max(0, bs.forwardPq.GetPriority(uEntryId)+
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

		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
		pfv, _ := bs.lm.FindTighestConsistentLowerBound(vId, source, target, bs.activeLandmarks)

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := bs.forwardPq.GetPriority(uEntryId) + edgeWeight + turnCost

		priority := newTravelTime + pfv

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}
		vEntryId := bs.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())
		if vQueryLevel == 0 {

			vEntryId = bs.engine.offsetForward(vId, vEntryId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			// relax edge
			vAlreadyLabelled := da.Lt(bs.forwardPq.GetPriority(vEntryId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.forwardPq.GetPriority(vEntryId))) {
				if bvi := bs.stallingEntry[vEntryId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
					// stalled
					return
				}

				if vAlreadyLabelled {
					// newTravelTime is better, update the forwardInfo
					// is key already in the priority queue, decrease its key

					newPar := da.NewVertexEdgePair(uId, uEntryId, false)
					bs.forwardPq.DecreaseKey(vEntryId, priority, newTravelTime, newPar)
				} else if !vAlreadyLabelled {

					vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
						da.NewVertexEdgePair(uId, uEntryId, false))
					queryKey := da.NewCRPQueryKey(vId, vEntryId, false)
					// is key not in the priority queue, insert it
					bs.forwardPq.Insert(vEntryId, priority, vertexInfo, queryKey)
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
				if scannedByBackwardSearch && da.Lt(bs.forwardPq.GetPriority(vEntryId)+bs.engine.metrics.GetTurnCost(turnType2)+
					bs.backwardPq.GetPriority(vExitId), bs.shortestTravelTime) {

					bs.shortestTravelTime = bs.forwardPq.GetPriority(vEntryId) + bs.engine.metrics.GetTurnCost(turnType2) +
						bs.backwardPq.GetPriority(vExitId)

					bs.forwardMid = da.NewVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = da.NewVertexEdgePair(vId, vExitId, true)
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
			vAlreadyLabelled := da.Lt(bs.forwardPq.GetPriority(overlayVId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.forwardPq.GetPriority(overlayVId))) {

				if !vAlreadyLabelled {

					vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
						da.NewVertexEdgePair(uId, uEntryId, false))
					vertexInfo.SetFirstOverlayEntryExitId(vEntryId)

					queryKey := da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)
					bs.forwardPq.Insert(overlayVId, priority, vertexInfo, queryKey)
				} else {

					bs.forwardPq.SetFirstOverlayEntryExitId(overlayVId, vEntryId)
					newPar := da.NewVertexEdgePair(uId, uEntryId, false)
					bs.forwardPq.DecreaseKey(overlayVId, priority, newTravelTime, newPar)
				}
			}

			scannedByBackwardSearch := bs.bScanned[overlayVId]
			// if v Labelled by backward search, check whether we can improve the shortestPath
			if scannedByBackwardSearch && da.Lt(bs.forwardPq.GetPriority(overlayVId)+bs.backwardPq.GetPriority(overlayVId), bs.shortestTravelTime) {
				bs.shortestTravelTime = bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)

				bs.forwardMid = da.NewVertexEdgePair(vId, overlayVId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, overlayVId, true)

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

func (bs *CRPALTBidirectionalSearch) backwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
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
		bui := math.Max(0, bs.backwardPq.GetPriority(uExitId)+
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

		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
		_, prv := bs.lm.FindTighestConsistentLowerBound(vId, source, target, bs.activeLandmarks)

		newTravelTime := bs.backwardPq.GetPriority(uExitId) + edgeWeight + turnCost
		priority := newTravelTime + prv
		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		vExitId := bs.engine.graph.GetExitOffset(vId) + da.Index(inArc.GetExitPoint())

		if vQueryLevel == 0 {

			vExitId = bs.engine.offsetBackward(vId, vExitId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// relax edge
			vAlreadyLabelled := da.Lt(bs.backwardPq.GetPriority(vExitId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.backwardPq.GetPriority(vExitId))) {

				if bvi := bs.stallingExit[vExitId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
					// stalled
					return
				}

				if vAlreadyLabelled {
					newPar := da.NewVertexEdgePair(uId, uExitId, true)
					bs.backwardPq.DecreaseKey(vExitId, priority, newTravelTime, newPar)
				} else {
					vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
						da.NewVertexEdgePair(uId, uExitId, false))
					queryKey := da.NewCRPQueryKey(vId, vExitId, false)
					bs.backwardPq.Insert(vExitId, priority, vertexInfo, queryKey)
				}
			}

			// check wether we already Labelled an entry point
			entryOffset := bs.engine.graph.GetEntryOffset(vId)

			entryOffset = bs.engine.offsetForward(vId, entryOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vEntryId := entryOffset

			bs.engine.graph.ForInEdgesOf(vId, da.Index(inArc.GetExitPoint()), func(inArc2 *da.InEdge,
				entryPoint2 da.Index, turnType2 pkg.TurnType) {
				scannedByForwardSearch := bs.fScanned[vEntryId]
				if scannedByForwardSearch && da.Lt(bs.forwardPq.GetPriority(vEntryId)+bs.engine.metrics.GetTurnCost(turnType2)+
					bs.backwardPq.GetPriority(vExitId), bs.shortestTravelTime) {

					bs.shortestTravelTime = bs.forwardPq.GetPriority(vEntryId) + bs.engine.metrics.GetTurnCost(turnType2) +
						bs.backwardPq.GetPriority(vExitId)

					bs.forwardMid = da.NewVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = da.NewVertexEdgePair(vId, vExitId, true)

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
			vAlreadyLabelled := da.Lt(bs.backwardPq.GetPriority(overlayVId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.backwardPq.GetPriority(overlayVId))) {

				if !vAlreadyLabelled {

					vVertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
						da.NewVertexEdgePair(uId, uExitId, true))
					vVertexInfo.SetFirstOverlayEntryExitId(vExitId)
					queryKey := da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)
					bs.backwardPq.Insert(overlayVId, priority, vVertexInfo, queryKey)
				} else {
					bs.backwardPq.SetFirstOverlayEntryExitId(overlayVId, vExitId)
					newPar := da.NewVertexEdgePair(uId, uExitId, true)
					bs.backwardPq.DecreaseKey(overlayVId, priority, newTravelTime, newPar)
				}
			}

			scannedByForwardSearch := bs.fScanned[overlayVId]
			if scannedByForwardSearch && da.Lt(bs.forwardPq.GetPriority(overlayVId)+bs.backwardPq.GetPriority(overlayVId), bs.shortestTravelTime) {
				bs.shortestTravelTime = bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)

				bs.forwardMid = da.NewVertexEdgePair(vId, overlayVId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, overlayVId, true)

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
func (bs *CRPALTBidirectionalSearch) forwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
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

		newTravelTime := bs.forwardPq.GetPriority(uId) + shortcutOutEdgeWeight
		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function

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
		vAlreadyLabelled := da.Lt(bs.forwardPq.GetPriority(overlayVId), pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.forwardPq.GetPriority(overlayVId))) {
			bs.forwardPq.Set(overlayVId, da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
				da.NewVertexEdgePair(uVertex.GetOriginalVertex(), uId, false)), da.NewCRPQueryKey(da.INVALID_VERTEX_ID,
				da.INVALID_EDGE_ID, true))

			// karena kita langsung scan v & traverse to its neighbor (exit vertex dari suatu cell), kita harus tandain kalau v udah di scan
			bs.fScanned[overlayVId] = true
			bs.forwardPq.Scan(overlayVId)

			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			pfw, _ := bs.lm.FindTighestConsistentLowerBound(originalWId, source, target, bs.activeLandmarks)

			newTravelTime = bs.forwardPq.GetPriority(overlayVId) + edgeWeight
			priority := newTravelTime + pfw
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := bs.engine.graph.GetEntryOffset(originalWId) + da.Index(outEdge.GetEntryPoint())

				wEntryId = bs.engine.offsetForward(originalWId, wEntryId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				wAlreadyLabelled := da.Lt(bs.forwardPq.GetPriority(wEntryId), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.forwardPq.GetPriority(wEntryId))) {
					if wAlreadyLabelled {

						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
						bs.forwardPq.DecreaseKey(wEntryId, priority, newTravelTime, newPar)
					} else {

						vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))
						queryKey := da.NewCRPQueryKey(originalWId, wEntryId, false)
						bs.forwardPq.Insert(wEntryId, priority, vertexInfo, queryKey)
					}
				}

				// check whether we already Labelled an exit point
				exitOffset := bs.engine.graph.GetExitOffset(originalWId)

				exitOffset = bs.engine.offsetBackward(originalWId, exitOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				wExitId := exitOffset
				bs.engine.graph.ForOutEdgesOf(originalWId, da.Index(outEdge.GetEntryPoint()), func(e *da.OutEdge, exitPoint da.Index, turn pkg.TurnType) {
					// basically: check if forward and backward search already Labelled entry and exit point of w. if so, check whether we can improve the shortest path
					scannedByBackwardSearch := bs.bScanned[wExitId]
					if scannedByBackwardSearch && da.Lt(bs.forwardPq.GetPriority(wEntryId)+bs.engine.metrics.GetTurnCost(turn)+
						bs.backwardPq.GetPriority(wExitId), bs.shortestTravelTime) {

						bs.shortestTravelTime = bs.forwardPq.GetPriority(wEntryId) + bs.engine.metrics.GetTurnCost(turn) +
							bs.backwardPq.GetPriority(wExitId)

						bs.forwardMid = da.NewVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = da.NewVertexEdgePair(originalWId, wExitId, true)

						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(originalWId, wEntryId, wExitId, originalWId, false))

					}
					wExitId++
				})
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardOverlayPq, because we need to traverse & relax shortcut edges in overlay graph
				overlayWId := bs.engine.offsetOverlay(w)
				wAlreadyLabelled := da.Lt(bs.forwardPq.GetPriority(overlayWId), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.forwardPq.GetPriority(overlayWId))) {

					if !wAlreadyLabelled {

						vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))
						queryKey := da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)
						bs.forwardPq.Insert(overlayWId, priority, vertexInfo, queryKey)
					} else {

						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
						bs.forwardPq.DecreaseKey(overlayWId, priority, newTravelTime, newPar)
					}
				}

				scannedByBackwardSearch := bs.bScanned[overlayWId]
				if scannedByBackwardSearch && da.Lt(bs.forwardPq.GetPriority(overlayWId)+bs.backwardPq.GetPriority(overlayWId), bs.shortestTravelTime) {
					// if overlay vertex w Labelled by backward search, check whether we can improve the shortestPath
					bs.shortestTravelTime = bs.forwardPq.GetPriority(overlayWId) + bs.backwardPq.GetPriority(overlayWId)

					bs.forwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, false)
					bs.backwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, true)

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
		if scannedByBackwardSearch && da.Lt(bs.forwardPq.GetPriority(overlayVId)+bs.backwardPq.GetPriority(overlayVId), bs.shortestTravelTime) {

			bs.shortestTravelTime = bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)

			bs.forwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
			bs.backwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

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

func (bs *CRPALTBidirectionalSearch) backwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
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

		newTravelTime := bs.backwardPq.GetPriority(uId) + shortcutInEdgeWeight

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
		vAlreadyLabelled := da.Lt(bs.backwardPq.GetPriority(overlayVId), pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bs.backwardPq.GetPriority(overlayVId))) {

			bs.backwardPq.Set(overlayVId, da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
				da.NewVertexEdgePair(uVertex.GetOriginalVertex(), uId, true)), da.NewCRPQueryKey(da.INVALID_VERTEX_ID,
				da.INVALID_EDGE_ID, true))

			bs.bScanned[overlayVId] = true
			bs.backwardPq.Scan(overlayVId)

			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			_, prw := bs.lm.FindTighestConsistentLowerBound(originalWId, source, target, bs.activeLandmarks)

			newTravelTime = bs.backwardPq.GetPriority(overlayVId) + inEdgeWeight
			priority := newTravelTime + prw

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			if wQueryLevel == 0 {

				wExitId := bs.engine.graph.GetExitOffset(originalWId) + da.Index(inEdge.GetExitPoint())

				wExitId = bs.engine.offsetBackward(originalWId, wExitId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax edge
				wAlreadyLabelled := da.Lt(bs.backwardPq.GetPriority(wExitId), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.backwardPq.GetPriority(wExitId))) {
					if wAlreadyLabelled {
						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)
						bs.backwardPq.DecreaseKey(wExitId, priority, newTravelTime, newPar)
					} else {
						queryKey := da.NewCRPQueryKey(originalWId, wExitId, false)

						vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))
						bs.backwardPq.Insert(wExitId, priority, vertexInfo, queryKey)
					}
				}

				// check whether we already scanned an entry point in forward search
				entryOffset := bs.engine.graph.GetEntryOffset(originalWId)

				entryOffset = bs.engine.offsetForward(originalWId, entryOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				wEntryId := entryOffset

				bs.engine.graph.ForInEdgesOf(originalWId, da.Index(inEdge.GetExitPoint()), func(e *da.InEdge,
					entryPoint da.Index, turn pkg.TurnType) {
					scannedByForwardSearch := bs.fScanned[wEntryId]
					if scannedByForwardSearch && da.Lt(bs.forwardPq.GetPriority(wEntryId)+bs.engine.metrics.GetTurnCost(turn)+
						bs.backwardPq.GetPriority(wExitId), bs.shortestTravelTime) {

						bs.shortestTravelTime = bs.forwardPq.GetPriority(wEntryId) + bs.engine.metrics.GetTurnCost(turn) +
							bs.backwardPq.GetPriority(wExitId)

						bs.forwardMid = da.NewVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = da.NewVertexEdgePair(originalWId, wExitId, true)
						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(originalWId, wEntryId, wExitId, originalWId, false))

					}
					wEntryId++
				})
			} else {
				overlayWId := bs.engine.offsetOverlay(w)
				wAlreadyLabelled := da.Lt(bs.backwardPq.GetPriority(overlayWId), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bs.backwardPq.GetPriority(overlayWId))) {

					if !wAlreadyLabelled {
						queryKey := da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)

						vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))
						bs.backwardPq.Insert(overlayWId, priority, vertexInfo, queryKey)
					} else {
						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)
						bs.backwardPq.DecreaseKey(overlayWId, priority, newTravelTime, newPar)
					}
				}

				scannedByForwardSearch := bs.fScanned[overlayWId]
				if scannedByForwardSearch && da.Lt(bs.forwardPq.GetPriority(overlayWId)+bs.backwardPq.GetPriority(overlayWId), bs.shortestTravelTime) {
					bs.shortestTravelTime = bs.forwardPq.GetPriority(overlayWId) + bs.backwardPq.GetPriority(overlayWId)

					bs.forwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, false)
					bs.backwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, true)

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
		if scannedByForwardSearch && da.Lt(bs.backwardPq.GetPriority(overlayVId)+bs.forwardPq.GetPriority(overlayVId), bs.shortestTravelTime) {

			bs.shortestTravelTime = bs.backwardPq.GetPriority(overlayVId) + bs.forwardPq.GetPriority(overlayVId)

			bs.forwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
			bs.backwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

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

func (bs *CRPALTBidirectionalSearch) Preallocate() {
	maxEdgesInCell := bs.engine.graph.GetMaxEdgesInCell()
	numberOfOverlayVertices := bs.engine.overlayGraph.NumberOfOverlayVertices()
	maxSearchSize := int(maxEdgesInCell)*2 + numberOfOverlayVertices

	bs.forwardPq = bs.engine.fBufPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	bs.backwardPq = bs.engine.bBufPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	bs.forwardPq.Clear()
	bs.backwardPq.Clear()

	bs.stallingEntry = make([]float64, maxSearchSize)
	bs.stallingExit = make([]float64, maxSearchSize)

	initInfWeight(bs.stallingEntry)
	initInfWeight(bs.stallingExit)

	bs.fScanned = make([]bool, maxSearchSize)
	bs.bScanned = make([]bool, maxSearchSize)

	// https://drive.google.com/file/d/1eViVblw6g0E4WrAn22-q0AhdVYTgCy9F/view?usp=sharing
	// kalo kita preallocate heap capacity dengan maxSearchSize, load tests pakai 900 vu, makan memory banyak
	allocateHeapCapacity := int(maxEdgesInCell)*2 + da.OVERLAY_INFO_SIZE
	bs.forwardPq.PreallocateHeap(allocateHeapCapacity)
	bs.backwardPq.PreallocateHeap(allocateHeapCapacity)
}

func (bs *CRPALTBidirectionalSearch) Done() {
	bs.engine.fBufPool.Put(bs.forwardPq)
	bs.engine.bBufPool.Put(bs.backwardPq)
}

func (bs *CRPALTBidirectionalSearch) GetStats(n int) (float64, int, int64, int64) {
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

func (bs *CRPALTBidirectionalSearch) GetLastPQSum() float64 {
	return bs.lastpqSum
}

func (bs *CRPALTBidirectionalSearch) GetActiveLandmarks() []da.Index {
	return bs.activeLandmarks
}

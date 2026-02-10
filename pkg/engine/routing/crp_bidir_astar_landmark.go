package routing

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPALTBidirectionalSearch struct {
	engine             *CRPRoutingEngine
	shortestTimeTravel float64
	forwardMid         vertexEdgePair
	backwardMid        vertexEdgePair

	forwardInfo   map[da.Index]VertexInfo
	backwardInfo  map[da.Index]VertexInfo
	stallingEntry map[da.Index]float64
	stallingExit  map[da.Index]float64

	forwardPq  *da.MinHeap[da.CRPQueryKey]
	backwardPq *da.MinHeap[da.CRPQueryKey]
	fScanned   map[da.CRPQueryKey]struct{}
	bScanned   map[da.CRPQueryKey]struct{}

	lm *landmark.Landmark

	sCellNumber da.Pv
	tCellNumber da.Pv

	viaVertices []da.ViaVertex

	sForwardId  da.Index
	tBackwardId da.Index

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numSettledNodes int
}

func NewCRPALTBidirectionalSearch(engine *CRPRoutingEngine, upperBound float64, lm *landmark.Landmark) *CRPALTBidirectionalSearch {
	return &CRPALTBidirectionalSearch{
		engine:       engine,
		forwardInfo:  make(map[da.Index]VertexInfo),
		backwardInfo: make(map[da.Index]VertexInfo),
		forwardPq:    da.NewFourAryHeap[da.CRPQueryKey](),
		backwardPq:   da.NewFourAryHeap[da.CRPQueryKey](),
		fScanned:     make(map[da.CRPQueryKey]struct{}),
		bScanned:     make(map[da.CRPQueryKey]struct{}),

		forwardMid:    newVertexEdgePair(0, 0, false),
		backwardMid:   newVertexEdgePair(0, 0, true),
		viaVertices:   make([]da.ViaVertex, 0),
		upperBound:    upperBound,
		stallingEntry: make(map[da.Index]float64),
		stallingExit:  make(map[da.Index]float64),
		lm:            lm,

		numSettledNodes: 0,
	}
}

/*
implementation of:
1. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
2. query phase: Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.
3. ALT query phase: Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
4. Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortucts within any partition
let n,m,k denote the number vertices of the original graph,edges of the original graph, and partitioning depth, respectively.
time complexity of CRP query is: O((n_p + m_p + k *  \hat{m_p}) * log n)
*/

func (bs *CRPALTBidirectionalSearch) ShortestPathSearch(asId, atId da.Index) (float64, float64, []da.Coordinate,
	[]da.OutEdge, bool) {
	// Our query algorithm takes as input a source arc as , a target arc at, the original graph G, the overlay graph
	// H = ∪i Hi , and computes the shortest path between the head vertex s of as and the tail vertex t of at.
	// asId exitPoint of outEdge u->s
	// atId entryPoint of inEdge t->v

	if asId == atId {
		return 0, 0, []da.Coordinate{}, []da.OutEdge{}, true
	}

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

	sForwardId = offsetForward(sForwardId, bs.engine.graph.GetCellNumber(s), bs.sCellNumber)
	tBackwardId = offsetBackward(tBackwardId, bs.engine.graph.GetCellNumber(t), bs.sCellNumber)

	bs.sForwardId = sForwardId
	bs.tBackwardId = tBackwardId

	bs.shortestTimeTravel = 2 * pkg.INF_WEIGHT

	bs.forwardInfo[sForwardId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, sForwardId, false))
	bs.backwardInfo[tBackwardId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, tBackwardId, true))

	bs.forwardPq.Insert(da.NewPriorityQueueNode(0, da.NewCRPQueryKey(s, sForwardId, false)))
	bs.backwardPq.Insert(da.NewPriorityQueueNode(0, da.NewCRPQueryKey(t, tBackwardId, false)))

	_, prt := bs.lm.FindTighestConsistentLowerBound(t, s, t)

	for bs.forwardPq.Size() > 0 && bs.backwardPq.Size() > 0 {
		minForward := bs.forwardPq.GetMinrank()
		minBackward := bs.backwardPq.GetMinrank()
		if minForward+minBackward > (bs.shortestTimeTravel+prt)*(bs.upperBound) {
			/*
				https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf
				stopping criterion for Bidirectional A* Search:
				top_f+top_r > \mu + p_r(t)
			*/
			break
		}

		// Customizable Route Planning In Road Networks, Daniel Delling, page 14:
		// Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
		// overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
		// version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
		// lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
		// the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
		queryKey, _ := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.fScanned[uItem] = struct{}{}
			bs.forwardGraphSearch(uItem, s, t)
		} else {
			bs.fScanned[da.NewCRPQueryKey(uItem.GetNode(), 0, true)] = struct{}{}
			bs.forwardOverlayGraphSearch(uItem, s, t)
		}

		queryKey, _ = bs.backwardPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.bScanned[uItem] = struct{}{}
			bs.backwardGraphSearch(uItem, s, t)
		} else {
			bs.bScanned[da.NewCRPQueryKey(uItem.GetNode(), 0, true)] = struct{}{}
			bs.backwardOverlayGraphSearch(uItem, s, t)
		}

		bs.numSettledNodes += 2
	}

	if bs.shortestTimeTravel == 2*pkg.INF_WEIGHT {
		return pkg.INF_WEIGHT, pkg.INF_WEIGHT, []da.Coordinate{}, []da.OutEdge{}, false
	}

	idPath := make([]vertexEdgePair, 0) // contains all outedges that make up the shortest path

	mid := bs.forwardMid

	if !isOverlay(mid.getEdge()) {
		adjustedMidEdge := adjustForwardOffBit(mid.getEdge())
		mid.setEdge(adjustedMidEdge)

		_, midOutEdge := bs.engine.graph.GetHeadOfInedgeWithOutEdge(mid.getEdge())
		mid.setEdge(midOutEdge.GetEdgeId())
		tail := bs.engine.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
		if tail != midOutEdge.GetHead() {
			idPath = append(idPath, mid)
		}
	}

	fMidEdge := bs.forwardMid.getEdge()
	curInfo := bs.forwardInfo[fMidEdge]

	for curInfo.GetParent().edge != sForwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.getEdge()
		parentCopy := parent

		if isOverlay(parentCopy.getEdge()) {

			// shortcut
			adjustedForwardEdge := adjustOverlay(parentCopy.getEdge())
			parentCopy.setEdge(adjustedForwardEdge)
		} else {

			adjustedForwardEdge := adjustForwardOffBit(parentCopy.getEdge())

			// jadiin outEdge semua
			inEdge := bs.engine.graph.GetInEdge(adjustedForwardEdge)
			_, outEdge := bs.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			parentCopy.setEdge(outEdge.GetEdgeId())
			parentCopy.setisOutEdge(true)
		}

		idPath = append(idPath, parentCopy)
		curInfo = bs.forwardInfo[parentEdge]
	}

	idPath = util.ReverseG[vertexEdgePair](idPath)

	mid = bs.backwardMid
	if isOverlay(mid.getEdge()) {
		// overlay vertex
		adjustedMidEdge := adjustOverlay(mid.getEdge())
		mid.setEdge(adjustedMidEdge)
		idPath = append(idPath, mid)

	} else {

		adjustedMidEdge := adjustBackwardOffbit(mid.getEdge())
		mid.setEdge(adjustedMidEdge)

		midOutEdge := bs.engine.graph.GetOutEdge(mid.getEdge())
		tail := bs.engine.graph.GetTailFromOutEdge(midOutEdge.GetEdgeId())
		if tail != midOutEdge.GetHead() {
			idPath = append(idPath, mid)
		}
	}

	bMidEdge := bs.backwardMid.getEdge()
	curInfo = bs.backwardInfo[bMidEdge]

	for curInfo.GetParent().edge != tBackwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.getEdge()
		parentCopy := parent

		if isOverlay(parentCopy.getEdge()) {

			// overlay vertex
			adjustedParentEdge := adjustOverlay(parentCopy.getEdge())
			parentCopy.setEdge(adjustedParentEdge)
		} else {

			adjustedParentEdge := adjustBackwardOffbit(parentCopy.getEdge())
			parentCopy.setEdge(adjustedParentEdge)
		}

		idPath = append(idPath, parentCopy)
		curInfo = bs.backwardInfo[parentEdge]
	}

	unpacker := NewPathUnpacker(bs.engine.graph, bs.engine.overlayGraph, bs.engine.metrics, bs.engine.puCache, true, false)
	finalPath, finalEdgePath, totalDistance := unpacker.unpackPath(idPath, bs.sCellNumber, bs.tCellNumber)

	return bs.shortestTimeTravel, totalDistance, finalPath, finalEdgePath, true
}

/*
graphSearch. turn-aware bidirectional dijkstra search on graph level 1.

Customizable Route Planning In Road Networks, Daniel Delling, page 7-8:

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

	uEntryPoint := adjustForward(uEntryId, bs.engine.graph.GetEntryOffset(uId))

	// stalling
	uInDeg := bs.engine.graph.GetInDegree(uId)
	otherUEntryId := offsetForward(bs.engine.graph.GetEntryOffset(uId), bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	for j := da.Index(0); j < uInDeg; j++ {

		stallingOffset := uInDeg*uEntryPoint + j
		bui := math.Max(0, bs.forwardInfo[uEntryId].GetTravelTime()+
			bs.engine.metrics.GetEntryStallingTableCost(uId, stallingOffset))

		if _, exists := bs.stallingEntry[otherUEntryId]; !exists {
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

		edgeWeight := bs.engine.metrics.GetWeight(outArc, 0)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}

		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
		pfv, _ := bs.lm.FindTighestConsistentLowerBound(vId, source, target)

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := bs.forwardInfo[uEntryId].GetTravelTime() + edgeWeight + turnCost

		priority := newTravelTime + pfv

		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}

		vEntryId := bs.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())
		vEntryId = offsetForward(vEntryId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

		if vQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			_, vAlreadyLabelled := bs.forwardInfo[vEntryId]
			if vAlreadyLabelled && newTravelTime >= bs.forwardInfo[vEntryId].GetTravelTime() {
				// newTravelTime is not better, do nothing
				return
			}

			if bvi, exists := bs.stallingEntry[vEntryId]; exists && newTravelTime >= bvi {
				// stalled
				return
			}

			// newTravelTime is better, update the forwardInfo
			bs.forwardInfo[vEntryId] = NewVertexInfo(newTravelTime,
				newVertexEdgePair(uId, uEntryId, false))

			if vAlreadyLabelled {
				// is key already in the priority queue, decrease its key
				bs.forwardPq.DecreaseKey(da.NewPriorityQueueNode(
					priority, da.NewCRPQueryKey(vId, vEntryId, false)),
				)
			} else if !vAlreadyLabelled {
				// is key not in the priority queue, insert it
				bs.forwardPq.Insert(da.NewPriorityQueueNode(
					priority, da.NewCRPQueryKey(vId, vEntryId, false)),
				)
			}

			// check wether we already Labelled an exit point of vId

			exitOffset := bs.engine.graph.GetExitOffset(vId)

			exitOffset = offsetBackward(exitOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vExitId := exitOffset

			// traverse outEdges of v
			bs.engine.graph.ForOutEdgesOf(vId, da.Index(outArc.GetEntryPoint()), func(e2 *da.OutEdge,
				exitPoint da.Index, turnType2 pkg.TurnType) {
				// Customizable Route Planning In Road Networks, Page 8: Whenever we scan a vertex that has been seen from
				// the other side, we evaluate all possible turns between all entry and exit points of the intersection and check
				// whether we can improve µ.
				// basically: check if forward and backward search already Labelled entry and exit point of v. if so, check whether we can improve the shortest path
				// if head of outEdge v->w already Labelled by backward search, and its forwardTravelTime + backwardTravelTime is better than shortestPath, then update shortestPath

				_, scannedByBackwardSearch := bs.bScanned[da.NewCRPQueryKey(vId, vExitId, false)]
				if scannedByBackwardSearch && bs.forwardInfo[vEntryId].GetTravelTime()+bs.engine.metrics.GetTurnCost(turnType2)+
					bs.backwardInfo[vExitId].GetTravelTime() < bs.shortestTimeTravel {

					bs.shortestTimeTravel = bs.forwardInfo[vEntryId].GetTravelTime() + bs.engine.metrics.GetTurnCost(turnType2) +
						bs.backwardInfo[vExitId].GetTravelTime()

					bs.forwardMid = newVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = newVertexEdgePair(vId, vExitId, true)
					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId))
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
			overlayVId := onOverlayBit(v)
			_, vAlreadyLabelled := bs.forwardInfo[overlayVId]
			if !vAlreadyLabelled || (vAlreadyLabelled && newTravelTime < bs.forwardInfo[overlayVId].GetTravelTime()) {

				vertexInfo := NewVertexInfo(newTravelTime,
					newVertexEdgePair(uId, uEntryId, false))
				bs.forwardInfo[vEntryId] = vertexInfo

				vVertexInfo := NewVertexInfo(newTravelTime,
					newVertexEdgePair(vId, vEntryId, false))
				bs.forwardInfo[overlayVId] = vVertexInfo
				if !vAlreadyLabelled {
					bs.forwardPq.Insert(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)),
					)
				} else {
					bs.forwardPq.DecreaseKey(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)),
					)
				}

				_, scannedByBackwardSearch := bs.bScanned[da.NewCRPQueryKey(v, 0, true)]
				// if v Labelled by backward search, check whether we can improve the shortestPath
				if scannedByBackwardSearch && bs.forwardInfo[overlayVId].GetTravelTime()+bs.backwardInfo[overlayVId].GetTravelTime() < bs.shortestTimeTravel {
					bs.shortestTimeTravel = bs.forwardInfo[overlayVId].GetTravelTime() + bs.backwardInfo[overlayVId].GetTravelTime()

					bs.forwardMid = newVertexEdgePair(vId, overlayVId, false)
					bs.backwardMid = newVertexEdgePair(vId, overlayVId, true)

					vOverlay := bs.engine.overlayGraph.GetVertex(v)
					vInEdge := bs.engine.graph.GetInEdge(vOverlay.GetOriginalEdge())

					vExitId := bs.engine.graph.GetExitOffset(vInEdge.GetTail()) + da.Index(vInEdge.GetExitPoint())
					vEntryId := bs.engine.graph.GetEntryOffset(vId) + bs.engine.graph.GetEntryOrder(vId, vInEdge.GetEdgeId())

					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, vId))
				}
			}
		}
	})
}

func (bs *CRPALTBidirectionalSearch) backwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search backward on graph level 1
	// basically same as forward search, but using inEdges and exitPoint instead of outEdges and entryPoint

	uId := uItem.GetNode()
	uExitId := uItem.GetEntryExitPoint() // index of outEdge that have endpoint from vertex uId

	uExitPoint := adjustBackward(uExitId, bs.engine.graph.GetExitOffset(uId))

	// stalling
	uOutDeg := bs.engine.graph.GetOutDegree(uId)
	otherUExitId := offsetBackward(bs.engine.graph.GetExitOffset(uId),
		bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	for j := da.Index(0); j < uOutDeg; j++ {

		stallingOffset := uOutDeg*uExitPoint + j
		bui := math.Max(0, bs.backwardInfo[uExitId].GetTravelTime()+
			bs.engine.metrics.GetExitStallingTableCost(uId, stallingOffset))

		if _, exists := bs.stallingExit[otherUExitId]; !exists {
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

		edgeWeight := bs.engine.metrics.GetWeight(inArc, 0)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)

		if uId == target {
			turnCost = 0
		}

		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
		_, prv := bs.lm.FindTighestConsistentLowerBound(vId, source, target)

		newTravelTime := bs.backwardInfo[uExitId].GetTravelTime() + edgeWeight + turnCost
		priority := newTravelTime + prv
		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}

		vExitId := bs.engine.graph.GetExitOffset(vId) + da.Index(inArc.GetExitPoint())

		vExitId = offsetBackward(vExitId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

		if vQueryLevel == 0 {

			_, vAlreadyLabelled := bs.backwardInfo[vExitId]

			if vAlreadyLabelled && newTravelTime >= bs.backwardInfo[vExitId].GetTravelTime() {
				return
			}

			if bvi, exists := bs.stallingExit[vExitId]; exists && newTravelTime >= bvi {
				// stalled
				return
			}

			bs.backwardInfo[vExitId] = NewVertexInfo(newTravelTime,
				newVertexEdgePair(uId, uExitId, true))

			if vAlreadyLabelled {
				bs.backwardPq.DecreaseKey(da.NewPriorityQueueNode(
					priority, da.NewCRPQueryKey(vId, vExitId, false)),
				)
			} else {
				bs.backwardPq.Insert(da.NewPriorityQueueNode(
					priority, da.NewCRPQueryKey(vId, vExitId, false)),
				)
			}

			// check wether we already Labelled an entry point
			entryOffset := bs.engine.graph.GetEntryOffset(vId)

			entryOffset = offsetForward(entryOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vEntryId := entryOffset

			bs.engine.graph.ForInEdgesOf(vId, da.Index(inArc.GetExitPoint()), func(inArc2 *da.InEdge,
				entryPoint2 da.Index, turnType2 pkg.TurnType) {
				_, scannedByForwardSearch := bs.fScanned[da.NewCRPQueryKey(vId, vEntryId, false)]
				if scannedByForwardSearch && bs.forwardInfo[vEntryId].GetTravelTime()+bs.engine.metrics.GetTurnCost(turnType2)+
					bs.backwardInfo[vExitId].GetTravelTime() < bs.shortestTimeTravel {

					bs.shortestTimeTravel = bs.forwardInfo[vEntryId].GetTravelTime() + bs.engine.metrics.GetTurnCost(turnType2) +
						bs.backwardInfo[vExitId].GetTravelTime()

					bs.forwardMid = newVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = newVertexEdgePair(vId, vExitId, true)

					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId))
				}
				vEntryId++
			})

		} else {
			// v is in another cell on higher level
			// Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			v, _ := bs.engine.graph.GetOverlayVertex(vId, inArc.GetExitPoint(), true)
			overlayVId := onOverlayBit(v)
			_, vAlreadyLabelled := bs.backwardInfo[overlayVId]
			if !vAlreadyLabelled || (vAlreadyLabelled && newTravelTime < bs.backwardInfo[overlayVId].GetTravelTime()) {

				vertexInfo := NewVertexInfo(newTravelTime,
					newVertexEdgePair(uId, uExitId, true))

				bs.backwardInfo[vExitId] = vertexInfo

				vVertexInfo := NewVertexInfo(newTravelTime,
					newVertexEdgePair(vId, vExitId, true))

				bs.backwardInfo[overlayVId] = vVertexInfo

				if !vAlreadyLabelled {
					bs.backwardPq.Insert(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)),
					)
				} else {
					bs.backwardPq.DecreaseKey(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)),
					)
				}

				_, scannedByForwardSearch := bs.fScanned[da.NewCRPQueryKey(v, 0, true)]
				if scannedByForwardSearch && bs.forwardInfo[overlayVId].GetTravelTime()+bs.backwardInfo[overlayVId].GetTravelTime() < bs.shortestTimeTravel {
					bs.shortestTimeTravel = bs.forwardInfo[overlayVId].GetTravelTime() + bs.backwardInfo[overlayVId].GetTravelTime()

					bs.forwardMid = newVertexEdgePair(vId, overlayVId, false)
					bs.backwardMid = newVertexEdgePair(vId, overlayVId, true)

					vOverlay := bs.engine.overlayGraph.GetVertex(v)
					vOutEdge := bs.engine.graph.GetOutEdge(vOverlay.GetOriginalEdge())

					vEntryId := bs.engine.graph.GetEntryOffset(vOutEdge.GetHead()) + da.Index(vOutEdge.GetEntryPoint())
					vExitId := bs.engine.graph.GetExitOffset(vId) + bs.engine.graph.GetExitOrder(vId, vOutEdge.GetEdgeId())

					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, vId))
				}
			}
		}
	})
}

/*
Customizable Route Planning In Road Networks, Daniel Delling, page 14:
Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
*/
func (bs *CRPALTBidirectionalSearch) forwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search on overlay graph

	u := uItem.GetNode()            // overlay vertex id
	uId := onBit(u, OVERLAY_OFFSET) // overlay vertex id , overlayOffset to get unique id in forwardInfo & backwardInfo
	uVertex := bs.engine.overlayGraph.GetVertex(u)
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	// for each out neighbors of u in level l, check if v already Labelled by backward search. if so, check whether we can improve shortestPath
	// then if v not already Labelled or newTravelTime to v is better, traverse to the next cell entry vertex w using outEdge of v.
	bs.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset, 0)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.forwardInfo[uId].GetTravelTime() + shortcutOutEdgeWeight
		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function

		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}
		overlayVId := onOverlayBit(v)
		_, vAlreadyLabelled := bs.forwardInfo[overlayVId]
		if !vAlreadyLabelled || (vAlreadyLabelled && newTravelTime < bs.forwardInfo[overlayVId].GetTravelTime()) {
			bs.forwardInfo[overlayVId] = NewVertexInfo(newTravelTime,
				newVertexEdgePair(uVertex.GetOriginalVertex(), uId, false))

			// karena kita langsung scan v & traverse to its neighbor (exit vertex dari suatu cell), kita harus tandain kalau v udah di scan
			bs.fScanned[da.NewCRPQueryKey(v, 0, true)] = struct{}{}

			_, scannedByBackwardSearch := bs.bScanned[da.NewCRPQueryKey(v, 0, true)]
			if scannedByBackwardSearch && bs.forwardInfo[overlayVId].GetTravelTime()+bs.backwardInfo[overlayVId].GetTravelTime() < bs.shortestTimeTravel {

				bs.shortestTimeTravel = bs.forwardInfo[overlayVId].GetTravelTime() + bs.backwardInfo[overlayVId].GetTravelTime()
				bs.forwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
				bs.backwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

				originalVId := vVertex.GetOriginalVertex()
				vOutEdge := bs.engine.graph.GetOutEdge(vVertex.GetOriginalEdge())
				vEntryId := bs.engine.graph.GetEntryOffset(vOutEdge.GetHead()) + da.Index(vOutEdge.GetEntryPoint())
				vExitId := bs.engine.graph.GetExitOffset(originalVId) + bs.engine.graph.GetExitOrder(originalVId, vOutEdge.GetEdgeId())

				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, originalVId)) // should use overlay vId (to calculate plateau, we need bactrack forwardInfo)
			}

			// traverse edge to next cell
			vOriEdgeId := vVertex.GetOriginalEdge()
			outEdge := bs.engine.graph.GetOutEdge(vOriEdgeId)
			edgeWeight := bs.engine.metrics.GetWeight(outEdge, 0)

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

			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			pfw, _ := bs.lm.FindTighestConsistentLowerBound(originalWId, source, target)

			newTravelTime = bs.forwardInfo[overlayVId].GetTravelTime() + edgeWeight
			priority := newTravelTime + pfw
			if newTravelTime >= pkg.INF_WEIGHT {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := bs.engine.graph.GetEntryOffset(originalWId) + da.Index(outEdge.GetEntryPoint())

				wEntryId = offsetForwardOverlay(wVertex, wEntryId, bs.sCellNumber)

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				_, wAlreadyLabelled := bs.forwardInfo[wEntryId]
				if wAlreadyLabelled && newTravelTime >= bs.forwardInfo[wEntryId].GetTravelTime() {
					return
				}

				bs.forwardInfo[wEntryId] = NewVertexInfo(newTravelTime,
					newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))

				if wAlreadyLabelled {
					bs.forwardPq.DecreaseKey(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(originalWId, wEntryId, false)),
					)
				} else {
					bs.forwardPq.Insert(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(originalWId, wEntryId, false)),
					)
				}
				// check whether we already Labelled an exit point
				exitOffset := bs.engine.graph.GetExitOffset(originalWId)

				exitOffset = offsetBackwardOverlay(wVertex, exitOffset, bs.sCellNumber)

				wExitId := exitOffset
				bs.engine.graph.ForOutEdgesOf(originalWId, da.Index(outEdge.GetEntryPoint()), func(e *da.OutEdge, exitPoint da.Index, turn pkg.TurnType) {
					// basically: check if forward and backward search already Labelled entry and exit point of w. if so, check whether we can improve the shortest path
					_, scannedByBackwardSearch := bs.bScanned[da.NewCRPQueryKey(originalWId, wExitId, false)]
					if scannedByBackwardSearch && bs.forwardInfo[wEntryId].GetTravelTime()+bs.engine.metrics.GetTurnCost(turn)+
						bs.backwardInfo[wExitId].GetTravelTime() < bs.shortestTimeTravel {

						bs.shortestTimeTravel = bs.forwardInfo[wEntryId].GetTravelTime() + bs.engine.metrics.GetTurnCost(turn) +
							bs.backwardInfo[wExitId].GetTravelTime()

						bs.forwardMid = newVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = newVertexEdgePair(originalWId, wExitId, true)

						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(originalWId, wEntryId, wExitId, originalWId))

					}
					wExitId++
				})
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardOverlayPq, because we need to traverse & relax shortcut edges in overlay graph
				wId := onBit(w, OVERLAY_OFFSET)
				_, wAlreadyLabelled := bs.forwardInfo[wId]
				if !wAlreadyLabelled || (wAlreadyLabelled && newTravelTime < bs.forwardInfo[wId].GetTravelTime()) {
					bs.forwardInfo[wId] = NewVertexInfo(newTravelTime,
						newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))

					if !wAlreadyLabelled {
						bs.forwardPq.Insert(da.NewPriorityQueueNode(
							priority, da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)),
						)
					} else {
						bs.forwardPq.DecreaseKey(da.NewPriorityQueueNode(
							priority, da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)),
						)
					}

					_, scannedByBackwardSearch := bs.bScanned[da.NewCRPQueryKey(w, 0, true)]
					if scannedByBackwardSearch && bs.forwardInfo[wId].GetTravelTime()+bs.backwardInfo[wId].GetTravelTime() < bs.shortestTimeTravel {
						// if overlay vertex w Labelled by backward search, check whether we can improve the shortestPath
						bs.shortestTimeTravel = bs.forwardInfo[wId].GetTravelTime() + bs.backwardInfo[wId].GetTravelTime()

						bs.forwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId, false)
						bs.backwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId, true)

						wInEdge := bs.engine.graph.GetInEdge(wVertex.GetOriginalEdge())
						wExitId := bs.engine.graph.GetExitOffset(wInEdge.GetTail()) + da.Index(wInEdge.GetExitPoint())
						wEntryId := bs.engine.graph.GetEntryOffset(originalWId) + bs.engine.graph.GetEntryOrder(originalWId, wInEdge.GetEdgeId())

						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(wId, wEntryId, wExitId, originalWId))
					}
				}
			}
		}
	})
}

func (bs *CRPALTBidirectionalSearch) backwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search backward on overlay graph
	// basically same as forward search on overlayGraph, but using inEdges and exitPoint instead of outEdges and entryPoint

	u := uItem.GetNode()

	uId := onBit(u, OVERLAY_OFFSET)
	uVertex := bs.engine.overlayGraph.GetVertex(u)

	uQueryLevel := uItem.GetEntryExitPoint()

	bs.engine.overlayGraph.ForInNeighborsOf(u, int(uQueryLevel), func(v da.Index,
		wOffset da.Index) {

		shortcutInEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset, 0)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.backwardInfo[uId].GetTravelTime() + shortcutInEdgeWeight

		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}

		overlayVId := onOverlayBit(v)
		_, vAlreadyLabelled := bs.backwardInfo[overlayVId]
		if !vAlreadyLabelled || (vAlreadyLabelled && newTravelTime < bs.backwardInfo[overlayVId].GetTravelTime()) {

			bs.backwardInfo[overlayVId] = NewVertexInfo(newTravelTime,
				newVertexEdgePair(uVertex.GetOriginalVertex(), uId, true))

			bs.bScanned[da.NewCRPQueryKey(v, 0, true)] = struct{}{}

			_, scannedByForwardSearch := bs.fScanned[da.NewCRPQueryKey(v, 0, true)]
			if scannedByForwardSearch && bs.backwardInfo[overlayVId].GetTravelTime()+bs.forwardInfo[overlayVId].GetTravelTime() < bs.shortestTimeTravel {

				bs.shortestTimeTravel = bs.backwardInfo[overlayVId].GetTravelTime() + bs.forwardInfo[overlayVId].GetTravelTime()
				bs.forwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
				bs.backwardMid = newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

				originalVId := vVertex.GetOriginalVertex()
				vInEdge := bs.engine.graph.GetInEdge(vVertex.GetOriginalEdge())
				vExitId := bs.engine.graph.GetExitOffset(vInEdge.GetTail()) + da.Index(vInEdge.GetExitPoint())
				vEntryId := bs.engine.graph.GetEntryOffset(originalVId) + bs.engine.graph.GetEntryOrder(originalVId, vInEdge.GetEdgeId())

				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(overlayVId, vEntryId, vExitId, originalVId))
			}

			// traverse edge to next cell
			vOriEdgeId := vVertex.GetOriginalEdge()
			inEdge := bs.engine.graph.GetInEdge(vOriEdgeId)

			inEdgeWeight := bs.engine.metrics.GetWeight(inEdge, 0)

			w := vVertex.GetNeighborOverlayVertex()
			wVertex := bs.engine.overlayGraph.GetVertex(w)
			wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
				wVertex.GetCellNumber())
			originalWId := wVertex.GetOriginalVertex()

			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			_, prw := bs.lm.FindTighestConsistentLowerBound(originalWId, source, target)

			newTravelTime = bs.backwardInfo[overlayVId].GetTravelTime() + inEdgeWeight
			priority := newTravelTime + prw

			if newTravelTime >= pkg.INF_WEIGHT {
				return
			}

			if wQueryLevel == 0 {

				wExitId := bs.engine.graph.GetExitOffset(originalWId) + da.Index(inEdge.GetExitPoint())

				wExitId = offsetBackwardOverlay(wVertex, wExitId, bs.sCellNumber)

				_, wAlreadyLabelled := bs.backwardInfo[wExitId]
				if newTravelTime >= bs.backwardInfo[wExitId].GetTravelTime() && wAlreadyLabelled {
					return
				}

				bs.backwardInfo[wExitId] = NewVertexInfo(newTravelTime,
					newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))

				if wAlreadyLabelled {
					bs.backwardPq.DecreaseKey(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(originalWId, wExitId, false)),
					)
				} else {

					bs.backwardPq.Insert(da.NewPriorityQueueNode(
						priority, da.NewCRPQueryKey(originalWId, wExitId, false)),
					)
				}

				// check whether we already scanned an entry point in forward search
				entryOffset := bs.engine.graph.GetEntryOffset(originalWId)

				entryOffset = offsetForwardOverlay(wVertex, entryOffset, bs.sCellNumber)

				wEntryId := entryOffset

				bs.engine.graph.ForInEdgesOf(originalWId, da.Index(inEdge.GetExitPoint()), func(e *da.InEdge,
					entryPoint da.Index, turn pkg.TurnType) {
					_, scannedByForwardSearch := bs.fScanned[da.NewCRPQueryKey(originalWId, wEntryId, false)]
					if scannedByForwardSearch && bs.forwardInfo[wEntryId].GetTravelTime()+bs.engine.metrics.GetTurnCost(turn)+
						bs.backwardInfo[wExitId].GetTravelTime() < bs.shortestTimeTravel {

						bs.shortestTimeTravel = bs.forwardInfo[wEntryId].GetTravelTime() + bs.engine.metrics.GetTurnCost(turn) +
							bs.backwardInfo[wExitId].GetTravelTime()

						bs.forwardMid = newVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = newVertexEdgePair(originalWId, wExitId, true)
						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(originalWId, wEntryId, wExitId, originalWId))

					}
					wEntryId++
				})
			} else {
				wId := onBit(w, OVERLAY_OFFSET)
				_, wAlreadyLabelled := bs.backwardInfo[wId]
				if !wAlreadyLabelled || (wAlreadyLabelled && newTravelTime < bs.backwardInfo[wId].GetTravelTime()) {

					bs.backwardInfo[wId] = NewVertexInfo(newTravelTime,
						newVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))

					if !wAlreadyLabelled {
						bs.backwardPq.Insert(da.NewPriorityQueueNode(
							priority, da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)),
						)
					} else {
						bs.backwardPq.DecreaseKey(da.NewPriorityQueueNode(
							priority, da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)),
						)
					}

					_, scannedByForwardSearch := bs.fScanned[da.NewCRPQueryKey(w, 0, true)]
					if scannedByForwardSearch && bs.forwardInfo[wId].GetTravelTime()+bs.backwardInfo[wId].GetTravelTime() < bs.shortestTimeTravel {
						bs.shortestTimeTravel = bs.forwardInfo[wId].GetTravelTime() + bs.backwardInfo[wId].GetTravelTime()

						bs.forwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId, false)
						bs.backwardMid = newVertexEdgePair(wVertex.GetOriginalVertex(), wId, true)

						wOutEdge := bs.engine.graph.GetOutEdge(wVertex.GetOriginalEdge())
						wEntryId := bs.engine.graph.GetEntryOffset(wOutEdge.GetHead()) + da.Index(wOutEdge.GetEntryPoint())
						wExitId := bs.engine.graph.GetExitOffset(originalWId) + bs.engine.graph.GetExitOrder(originalWId, wOutEdge.GetEdgeId())

						bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(wId, wEntryId, wExitId, originalWId))
					}
				}
			}
		}
	})
}

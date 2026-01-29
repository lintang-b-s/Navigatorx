package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPUniDijkstra struct {
	engine             *CRPRoutingEngine
	shortestTimeTravel float64

	forwardInfo   map[da.Index]VertexInfo
	stallingEntry map[da.Index]float64
	stallingExit  map[da.Index]float64

	pq        *da.MinHeap[da.CRPQueryKey]
	overlayPq *da.MinHeap[da.CRPQueryKey]
	tEntryId  da.Index

	daySpeedProfile map[int64]*da.PWL

	sCellNumber da.Pv
	tCellNumber da.Pv

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numSettledNodes int
}

func NewCRPUniDijkstra(engine *CRPRoutingEngine) *CRPUniDijkstra {
	return &CRPUniDijkstra{
		engine:      engine,
		forwardInfo: make(map[da.Index]VertexInfo),
		pq:          da.NewFourAryHeap[da.CRPQueryKey](),
		overlayPq:   da.NewFourAryHeap[da.CRPQueryKey](),

		stallingEntry: make(map[da.Index]float64),
		stallingExit:  make(map[da.Index]float64),

		numSettledNodes: 0,

		tEntryId: da.INVALID_VERTEX_ID,
	}
}

/*
implementation of:
1. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
2. query phase: Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.

time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortucts within any partition
let n,m,k denote the number vertices,edges, and partitioning depth, respectively.
time complexity of CRP query is: O((n_p + m_p + k * \hat{m_p}) * log n)
*/
func (us *CRPUniDijkstra) ShortestPathSearch(asId, atId da.Index) (float64, float64, []da.Coordinate,
	[]da.OutEdge, bool) {
	// Our query algorithm takes as input a source arc as , a target arc at, the original graph G, the overlay graph
	// H = ∪i Hi , and computes the shortest path between the head vertex s of as and the tail vertex t of at.
	// asId exitPoint of outEdge u->s
	// atId entryPoint of inEdge t->v

	s := us.engine.graph.GetOutEdge(asId).GetHead()
	t := us.engine.graph.GetInEdge(atId).GetTail()

	us.sCellNumber = us.engine.graph.GetCellNumber(s)
	us.tCellNumber = us.engine.graph.GetCellNumber(t)

	// for iterating outEdges, we need entryOffset.
	sForwardId := us.engine.graph.GetEntryOffset(s) + da.Index(us.engine.graph.GetOutEdge(asId).GetEntryPoint())

	sForwardId = offsetForward(sForwardId, us.engine.graph.GetCellNumber(s), us.sCellNumber)

	us.shortestTimeTravel = 2 * pkg.INF_WEIGHT

	us.forwardInfo[sForwardId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	us.pq.Insert(da.NewPriorityQueueNode(0, da.NewCRPQueryKey(s, sForwardId)))

	finished := false

	for !us.pq.IsEmpty() || !us.overlayPq.IsEmpty() {

		if finished {
			break
		}
		if us.overlayPq.IsEmpty() || (!us.pq.IsEmpty() && us.pq.GetMinrank() < us.overlayPq.GetMinrank()) {
			// search on graph level 1
			finished = us.graphSearchUni(s, t)
			us.numSettledNodes++
		} else {
			// search on overlay graph
			us.overlayGraphSearchUni()
			us.numSettledNodes++
		}

	}

	if us.shortestTimeTravel == 2*pkg.INF_WEIGHT {
		return pkg.INF_WEIGHT, pkg.INF_WEIGHT, []da.Coordinate{}, []da.OutEdge{}, false
	}

	idPath := make([]vertexEdgePair, 0) // contains all outedges that make up the shortest path
	curInfo := us.forwardInfo[us.tEntryId]
	tEntryIdAdj := adjustForwardOffBit(da.Index(us.tEntryId))

	_, tOutEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(tEntryIdAdj)
	toutEdgeId := tOutEdge.GetEdgeId()
	tpair := newVertexEdgePair(t, toutEdgeId, true)
	idPath = append(idPath, tpair)

	for curInfo.GetParent().edge != sForwardId {
		parent := curInfo.GetParent()
		parentCopy := parent

		if isOverlay(parentCopy.getEdge()) {
			// shortcut
			ovTT := us.forwardInfo[parentCopy.getEdge()].GetTravelTime()
			parentCopy.setTravelTime(ovTT)

			adjustedForwardEdge := adjustOverlay(parentCopy.getEdge())
			parentCopy.setEdge(adjustedForwardEdge)

		} else {

			adjustedForwardEdge := adjustForwardOffBit(parentCopy.getEdge())
			parentCopy.setEdge(adjustedForwardEdge)

			inEdge := us.engine.graph.GetInEdge(parentCopy.getEdge())
			_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			parentCopy.setEdge(outEdge.GetEdgeId())

		}

		parentCopy.setisOutEdge(true)
		idPath = append(idPath, parentCopy)

		curInfo = us.forwardInfo[parent.getEdge()]
	}

	idPath = util.ReverseG[vertexEdgePair](idPath)

	unpacker := NewPathUnpacker(us.engine.graph, us.engine.overlayGraph, us.engine.metrics, us.engine.puCache, true)
	finalPath, finalEdgePath, totalDistance := unpacker.unpackPath(idPath, us.sCellNumber, us.tCellNumber)

	return us.shortestTimeTravel, totalDistance, finalPath, finalEdgePath, true
}

func (us *CRPUniDijkstra) graphSearchUni(source, target da.Index) bool {

	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1
	queryKey, _ := us.pq.ExtractMin()
	uItem := queryKey.GetItem()

	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId
	if uId == target {
		us.shortestTimeTravel = us.forwardInfo[uEntryId].GetTravelTime()
		us.tEntryId = uEntryId
		return true
	}
	uEntryPoint := adjustForward(uEntryId, us.engine.graph.GetEntryOffset(uId))

	// traverse outEdges of u
	us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {

		vId := outArc.GetHead()
		if vId == uId {
			return
		}

		// get query level of v l_st(v)
		vQueryLevel := us.engine.overlayGraph.GetQueryLevel(us.sCellNumber, us.tCellNumber,
			us.engine.graph.GetCellNumber(vId))

		edgeWeight := us.engine.metrics.GetWeight(outArc, 0)

		turnCost := us.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newArrTime := us.forwardInfo[uEntryId].GetTravelTime() + edgeWeight + turnCost

		if newArrTime >= pkg.INF_WEIGHT {
			return
		}

		vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())
		vEntryId = offsetForward(vEntryId, us.engine.graph.GetCellNumber(vId), us.sCellNumber)

		if vQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			_, vAlreadyVisited := us.forwardInfo[vEntryId]
			if vAlreadyVisited && newArrTime >= us.forwardInfo[vEntryId].GetTravelTime() {
				// newArrTime is not better, do nothing

				return
			}

			if bvi, exists := us.stallingEntry[vEntryId]; exists && newArrTime >= bvi {
				// stalled
				return
			}

			// newArrTime is better, update the forwardInfo
			us.forwardInfo[vEntryId] = NewVertexInfo(newArrTime,
				newVertexEdgePair(uId, uEntryId, false))

			if vAlreadyVisited {
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(da.NewPriorityQueueNode(
					newArrTime, da.NewCRPQueryKey(vId, vEntryId)),
				)
			} else if !vAlreadyVisited {
				// is key not in the priority queue, insert it
				us.pq.Insert(da.NewPriorityQueueNode(
					newArrTime, da.NewCRPQueryKey(vId, vEntryId)),
				)
			}

		} else {
			// v is in another cell on higher level
			// CRP In Road Networks: Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			// update the forward info of overlay vertex v
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := us.engine.graph.GetOverlayVertex(vId, outArc.GetEntryPoint(), false)
			overlayVId := onOverlayBit(v)
			_, vAlreadyVisited := us.forwardInfo[overlayVId]
			if !vAlreadyVisited || (vAlreadyVisited && newArrTime < us.forwardInfo[overlayVId].GetTravelTime()) {

				vVertex := us.engine.overlayGraph.GetVertex(v)

				maxBoundaryVerticeLevel := vVertex.GetEntryPointSize()
				if maxBoundaryVerticeLevel < vQueryLevel {
					// to handle the case where the query level v is greater than the level where vertex v becomes the boundary vertex
					vQueryLevel = uint8(maxBoundaryVerticeLevel)
				}

				vertexInfo := NewVertexInfo(newArrTime,
					newVertexEdgePair(uId, uEntryId, false))

				us.forwardInfo[vEntryId] = vertexInfo

				vVertexInfo := NewVertexInfo(newArrTime,
					newVertexEdgePair(vId, vEntryId, false))

				us.forwardInfo[overlayVId] = vVertexInfo
				if !vAlreadyVisited {

					us.overlayPq.Insert(da.NewPriorityQueueNode(
						newArrTime, da.NewCRPQueryKey(v, da.Index(vQueryLevel))),
					)
				} else {
					us.overlayPq.DecreaseKey(da.NewPriorityQueueNode(
						newArrTime, da.NewCRPQueryKey(v, da.Index(vQueryLevel))),
					)
				}
			}
		}
	})

	return false
}

func (us *CRPUniDijkstra) overlayGraphSearchUni() {
	// search on overlay graph
	queryKey, _ := us.overlayPq.ExtractMin()
	uItem := queryKey.GetItem()

	u := uItem.GetNode() // overlay vertex id

	uId := onOverlayBit(u) // overlay vertex id | overlayOffset to get unique id in forwardInfo & backwardInfo
	uVertex := us.engine.overlayGraph.GetVertex(u)
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	// for each out neighbors of u in level l, check if v already visited by backward search. if so, check whether we can improve shortestPath
	// then if v not already visited or newArrTime to v is better, traverse to the next cell entry vertex w using outEdge of v.
	us.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := us.engine.metrics.GetShortcutWeight(wOffset, 0)

		newArrTime := us.forwardInfo[uId].GetTravelTime() + shortcutOutEdgeWeight
		if newArrTime >= pkg.INF_WEIGHT {
			return
		}
		vVertex := us.engine.overlayGraph.GetVertex(v)

		vId := onOverlayBit(v)
		_, vAlreadyVisited := us.forwardInfo[vId]
		if !vAlreadyVisited || (vAlreadyVisited && newArrTime < us.forwardInfo[vId].GetTravelTime()) {
			us.forwardInfo[vId] = NewVertexInfo(newArrTime,
				newVertexEdgePair(uVertex.GetOriginalVertex(), uId, false))

			// traverse edge to next cell
			vOriEdgeId := vVertex.GetOriginalEdge()
			outEdge := us.engine.graph.GetOutEdge(vOriEdgeId)
			edgeWeight := us.engine.metrics.GetWeight(outEdge, 0)

			/*
				We apply several optimizations. First, by construction, each exit vertex u in the overlay has a single
				outgoing arc (u, v). Therefore, during the search we do not add u to the priority queue; instead, we traverse
				the arc (u, v) immediately and process v.
			*/
			// w is in the next cell from v cell
			w := vVertex.GetNeighborOverlayVertex()
			wVertex := us.engine.overlayGraph.GetVertex(w)
			wQueryLevel := us.engine.overlayGraph.GetQueryLevel(us.sCellNumber, us.tCellNumber,
				wVertex.GetCellNumber())
			originalW := wVertex.GetOriginalVertex()

			newArrTime = us.forwardInfo[vId].GetTravelTime() + edgeWeight

			if newArrTime >= pkg.INF_WEIGHT {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := us.engine.graph.GetEntryOffset(originalW) + da.Index(outEdge.GetEntryPoint())

				wEntryId = offsetForwardOverlay(wVertex, wEntryId, us.sCellNumber)

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				_, wAlreadyVisited := us.forwardInfo[wEntryId]
				if wAlreadyVisited && newArrTime >= us.forwardInfo[wEntryId].GetTravelTime() {

					return
				}

				us.forwardInfo[wEntryId] = NewVertexInfo(newArrTime,
					newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

				if wAlreadyVisited {
					us.pq.DecreaseKey(da.NewPriorityQueueNode(
						newArrTime, da.NewCRPQueryKey(originalW, wEntryId)),
					)
				} else {
					us.pq.Insert(da.NewPriorityQueueNode(
						newArrTime, da.NewCRPQueryKey(originalW, wEntryId)),
					)
				}

			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to overlayPq, because we need to traverse & relax shortcut edges in overlay graph
				wId := onOverlayBit(w)
				_, wAlreadyVisited := us.forwardInfo[wId]
				if !wAlreadyVisited || (wAlreadyVisited && newArrTime < us.forwardInfo[wId].GetTravelTime()) {
					us.forwardInfo[wId] = NewVertexInfo(newArrTime,
						newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

					maxBoundaryVerticeLevel := wVertex.GetEntryPointSize()
					if maxBoundaryVerticeLevel < wQueryLevel {
						wQueryLevel = uint8(maxBoundaryVerticeLevel)
					}

					if !wAlreadyVisited {
						us.overlayPq.Insert(da.NewPriorityQueueNode(
							newArrTime, da.NewCRPQueryKey(w, da.Index(wQueryLevel))),
						)
					} else {
						us.overlayPq.DecreaseKey(da.NewPriorityQueueNode(
							newArrTime, da.NewCRPQueryKey(w, da.Index(wQueryLevel))),
						)
					}
				}
			}
		}
	})
}

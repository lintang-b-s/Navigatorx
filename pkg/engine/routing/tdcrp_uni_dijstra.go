package routing

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPUnidirectionalSearch struct {
	engine             *CRPRoutingEngine
	shortestTimeTravel float64

	forwardInfo   map[datastructure.Index]VertexInfo
	stallingEntry map[datastructure.Index]float64
	stallingExit  map[datastructure.Index]float64

	pq        *datastructure.MinHeap[datastructure.CRPQueryKey]
	overlayPq *datastructure.MinHeap[datastructure.CRPQueryKey]

	sCellNumber datastructure.Pv
	tCellNumber datastructure.Pv

	forwardSOffsetBit int // set 32-th bit on
	forwardTOffsetBit int // set 31-th bit on
	overlayOffset     int // set 30-th bit on
	viaVertices       []datastructure.ViaVertex

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	penalty                 bool // wether to use CRP-π or not (Evolution and Evaluation of the Penalty Method for Alternative Graphs, Kobitzsch et al.)
	penaltyEdgeCost         map[datastructure.PenaltiedEdge]float64
	penaltyShortcutEdgeCost map[datastructure.Index]float64
	numSettledNodes         int
}

func NewCRPUnidirectionalSearch(engine *CRPRoutingEngine) *CRPUnidirectionalSearch {
	return &CRPUnidirectionalSearch{
		engine:      engine,
		forwardInfo: make(map[datastructure.Index]VertexInfo),
		pq:          datastructure.NewFourAryHeap[datastructure.CRPQueryKey](),
		overlayPq:   datastructure.NewFourAryHeap[datastructure.CRPQueryKey](),

		viaVertices:             make([]datastructure.ViaVertex, 0),
		stallingEntry:           make(map[datastructure.Index]float64),
		stallingExit:            make(map[datastructure.Index]float64),
		penalty:                 false,
		penaltyEdgeCost:         make(map[datastructure.PenaltiedEdge]float64),
		penaltyShortcutEdgeCost: make(map[datastructure.Index]float64),
		numSettledNodes:         0,
		forwardSOffsetBit:       31,
		forwardTOffsetBit:       30,
		overlayOffset:           29,
	}
}

func (us *CRPUnidirectionalSearch) ShortestPathSearchUni(asId, atId datastructure.Index) (float64, float64, []datastructure.Coordinate,
	[]datastructure.OutEdge, bool) {
	// Our query algorithm takes as input a source arc as , a target arc at, the original graph G, the overlay graph
	// H = ∪i Hi , and computes the shortest path between the head vertex s of as and the tail vertex t of at.
	// asId exitPoint of outEdge u->s
	// atId entryPoint of inEdge t->v

	s := us.engine.graph.GetOutEdge(asId).GetHead()
	t := us.engine.graph.GetInEdge(atId).GetTail()

	us.sCellNumber = us.engine.graph.GetCellNumber(s)
	us.tCellNumber = us.engine.graph.GetCellNumber(t)

	// for iterating outEdges, we need entryOffset.
	sForwardId := us.engine.graph.GetEntryOffset(s) + datastructure.Index(us.engine.graph.GetOutEdge(asId).GetEntryPoint())

	sForwardId = us.offsetForward(s, sForwardId)

	tEntryId := us.engine.graph.GetEntryOffset(t) + atId

	us.shortestTimeTravel = 2 * pkg.INF_WEIGHT

	us.forwardInfo[sForwardId] = NewVertexInfo(0, newVertexEdgePair(s, asId, false))

	us.pq.Insert(datastructure.NewPriorityQueueNode(0, datastructure.NewCRPQueryKey(s, sForwardId)))

	finished := false

	for !us.pq.IsEmpty() || !us.overlayPq.IsEmpty() {

		if finished {
			break
		}
		// Customizable Route Planning In Road Networks, Daniel Delling, page 14:
		// Each iteration of the algorithm takes the minimum-distance entry from the queue, representing either an
		// overlay vertex u or a pair (u, i) from the original graph. If the entry is a pair, we scan it using the turn-aware
		// version of Dijkstra’s algorithm (and look at its neighbors in G). Otherwise, we use the overlay graph at level
		// lst (u), which does not have turns. In either case, the neighbors v of u are added to the priority queue with
		// the appropriate distance labels. Note that a level transition occurs when u and v have different query levels;
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
		return pkg.INF_WEIGHT, pkg.INF_WEIGHT, []datastructure.Coordinate{}, []datastructure.OutEdge{}, false
	}

	idPath := make([]vertexEdgePair, 0) // contains all outedges that make up the shortest path
	curInfo := us.forwardInfo[tEntryId]
	for curInfo.GetParent().edge != asId {
		parent := curInfo.GetParent()
		parentCopy := parent

		if parentCopy.getEdge()&datastructure.Index(1<<us.overlayOffset) != 0 {
			// shortcut
			adjustedForwardEdge := (parentCopy.getEdge() & ^datastructure.Index(1<<us.overlayOffset)) | datastructure.Index(1<<UNPACK_OVERLAY_OFFSET)
			parentCopy.setEdge(adjustedForwardEdge)
		} else {

			if parentCopy.getEdge()&datastructure.Index(1<<us.forwardSOffsetBit) != 0 {
				adjustedForwardEdge := parentCopy.getEdge() & ^datastructure.Index(1<<us.forwardSOffsetBit)
				parentCopy.setEdge(adjustedForwardEdge)
			} else {
				adjustedForwardEdge := parentCopy.getEdge() & ^datastructure.Index(1<<us.forwardTOffsetBit)
				parentCopy.setEdge(adjustedForwardEdge)
			}

			if !parentCopy.isOut() {
				inEdge := us.engine.graph.GetInEdge(parentCopy.getEdge())
				_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
				parentCopy.setEdge(outEdge.GetEdgeId())
			}
		}

		parentCopy.setisOutEdge(true)
		idPath = append(idPath, parentCopy)

		curInfo = us.forwardInfo[parent.getEdge()]
	}

	idPath = util.ReverseG[vertexEdgePair](idPath)

	unpacker := NewPathUnpacker(us.engine.graph, us.engine.overlayGraph, us.engine.metrics, us.engine.puCache)
	finalPath, finalEdgePath, totalDistance := unpacker.unpackPath(idPath, us.sCellNumber, us.tCellNumber)

	return us.shortestTimeTravel, totalDistance, finalPath, finalEdgePath, true
}

func (us *CRPUnidirectionalSearch) graphSearchUni(source, target datastructure.Index) bool {

	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1
	queryKey, _ := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId
	if uId == target {
		return true
	}
	uEntryPoint := us.adjustForward(uId, uEntryId)

	// stalling
	uInDeg := us.engine.graph.GetInDegree(uId)
	otherUEntryId := us.offsetForward(uId, us.engine.graph.GetEntryOffset(uId))

	for j := datastructure.Index(0); j < uInDeg; j++ {

		stallingOffset := uInDeg*uEntryPoint + j
		bui := math.Max(0, us.forwardInfo[uEntryId].GetTravelTime()+
			us.engine.metrics.GetEntryStallingTableCost(uId, stallingOffset))

		if _, exists := us.stallingEntry[otherUEntryId]; !exists {
			us.stallingEntry[otherUEntryId] = bui
		} else {
			us.stallingEntry[otherUEntryId] = math.Min(us.stallingEntry[otherUEntryId], bui)
		}
		otherUEntryId++
	}

	// traverse outEdges of u
	us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
		vId := outArc.GetHead()

		// get query level of v l_st(v)
		vQueryLevel := us.engine.overlayGraph.GetQueryLevel(us.sCellNumber, us.tCellNumber,
			us.engine.graph.GetCellNumber(vId))

		edgeWeight := us.engine.metrics.GetWeight(outArc)

		if penaltyCost, penalized := us.penaltyEdgeCost[datastructure.NewPenaltiedEdge(outArc.GetEdgeId(), true)]; us.penalty && penalized {
			edgeWeight = penaltyCost
		}

		turnCost := us.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}
		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := us.forwardInfo[uEntryId].GetTravelTime() + edgeWeight + turnCost

		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}

		if vQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			vEntryId := us.engine.graph.GetEntryOffset(vId) + datastructure.Index(outArc.GetEntryPoint())
			vEntryId = us.offsetForward(vId, vEntryId)

			_, vAlreadyVisited := us.forwardInfo[vEntryId]
			if vAlreadyVisited && newTravelTime >= us.forwardInfo[vEntryId].GetTravelTime() {
				// newTravelTime is not better, do nothing
				return
			}

			if bvi, exists := us.stallingEntry[vEntryId]; exists && newTravelTime >= bvi {
				// stalled
				return
			}

			// newTravelTime is better, update the forwardInfo
			us.forwardInfo[vEntryId] = NewVertexInfo(newTravelTime,
				newVertexEdgePair(uId, uEntryId, false))

			if vAlreadyVisited {
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(datastructure.NewPriorityQueueNode(
					newTravelTime, datastructure.NewCRPQueryKey(vId, vEntryId)),
				)
			} else if !vAlreadyVisited {
				// is key not in the priority queue, insert it
				us.pq.Insert(datastructure.NewPriorityQueueNode(
					newTravelTime, datastructure.NewCRPQueryKey(vId, vEntryId)),
				)
			}

		} else {
			// v is in another cell on higher level
			// CRP In Road Networks: Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			// update the forward info of overlay vertex v
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := us.engine.graph.GetOverlayVertex(vId, outArc.GetEntryPoint(), false)
			overlayVId := v | datastructure.Index(1<<us.overlayOffset)
			_, vAlreadyVisited := us.forwardInfo[overlayVId]
			if !vAlreadyVisited || (vAlreadyVisited && newTravelTime < us.forwardInfo[overlayVId].GetTravelTime()) {

				vertexInfo := NewVertexInfo(newTravelTime,
					newVertexEdgePair(uId, uEntryId, false))

				vVertex := us.engine.overlayGraph.GetVertex(v)

				maxBoundaryVerticeLevel := vVertex.GetEntryPointSize()
				if maxBoundaryVerticeLevel < vQueryLevel {
					// to handle the case where the query level v is greater than the level where vertex v becomes the boundary vertex
					vQueryLevel = uint8(maxBoundaryVerticeLevel)
				}

				us.forwardInfo[overlayVId] = vertexInfo
				if !vAlreadyVisited {
					us.overlayPq.Insert(datastructure.NewPriorityQueueNode(
						newTravelTime, datastructure.NewCRPQueryKey(v, datastructure.Index(vQueryLevel))),
					)
				} else {
					us.overlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(
						newTravelTime, datastructure.NewCRPQueryKey(v, datastructure.Index(vQueryLevel))),
					)
				}
			}
		}
	})

	return false
}

func (us *CRPUnidirectionalSearch) overlayGraphSearchUni() {
	// search on overlay graph
	queryKey, _ := us.overlayPq.ExtractMin()
	uItem := queryKey.GetItem()
	u := uItem.GetNode() // overlay vertex id

	uId := u | datastructure.Index(1<<us.overlayOffset) // overlay vertex id | overlayOffset to get unique id in forwardInfo & backwardInfo
	uVertex := us.engine.overlayGraph.GetVertex(u)
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	// for each out neighbors of u in level l, check if v already visited by backward search. if so, check whether we can improve shortestPath
	// then if v not already visited or newTravelTime to v is better, traverse to the next cell entry vertex w using outEdge of v.
	us.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v datastructure.Index, wOffset datastructure.Index) {
		shortcutOutEdgeWeight := us.engine.metrics.GetShortcutWeight(wOffset)
		if penaltyCost, penalized := us.penaltyShortcutEdgeCost[wOffset]; us.penalty && penalized {
			shortcutOutEdgeWeight = penaltyCost
		}

		newTravelTime := us.forwardInfo[uId].GetTravelTime() + shortcutOutEdgeWeight
		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}
		vId := v | datastructure.Index(1<<us.overlayOffset)
		_, vAlreadyVisited := us.forwardInfo[vId]
		if !vAlreadyVisited || newTravelTime < us.forwardInfo[vId].GetTravelTime() {
			us.forwardInfo[vId] = NewVertexInfo(newTravelTime,
				newVertexEdgePair(uVertex.GetOriginalVertex(), uId, false))

			vVertex := us.engine.overlayGraph.GetVertex(v)

			// traverse edge to next cell
			outEdge := us.engine.graph.GetOutEdge(vVertex.GetOriginalEdge())
			edgeWeight := us.engine.metrics.GetWeight(outEdge)

			if penaltyCost, penalized := us.penaltyEdgeCost[datastructure.NewPenaltiedEdge(outEdge.GetEdgeId(), true)]; us.penalty && penalized {
				edgeWeight = penaltyCost
			}

			newTravelTime = us.forwardInfo[vId].GetTravelTime() + edgeWeight

			if newTravelTime >= pkg.INF_WEIGHT {
				return
			}

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
			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := us.engine.graph.GetEntryOffset(originalW) + datastructure.Index(outEdge.GetEntryPoint())

				wEntryId = us.offsetForwardOverlay(wVertex, wEntryId)

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				_, wAlreadyVisited := us.forwardInfo[wEntryId]
				if wAlreadyVisited && newTravelTime >= us.forwardInfo[wEntryId].GetTravelTime() {
					return
				}

				us.forwardInfo[wEntryId] = NewVertexInfo(newTravelTime,
					newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

				if wAlreadyVisited {
					us.pq.DecreaseKey(datastructure.NewPriorityQueueNode(
						newTravelTime, datastructure.NewCRPQueryKey(originalW, wEntryId)),
					)
				} else {
					us.pq.Insert(datastructure.NewPriorityQueueNode(
						newTravelTime, datastructure.NewCRPQueryKey(originalW, wEntryId)),
					)
				}

			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to overlayPq, because we need to traverse & relax shortcut edges in overlay graph
				wId := w | datastructure.Index(1<<us.overlayOffset)
				_, wAlreadyVisited := us.forwardInfo[wId]
				if !wAlreadyVisited || (wAlreadyVisited && newTravelTime < us.forwardInfo[wId].GetTravelTime()) {
					us.forwardInfo[wId] = NewVertexInfo(newTravelTime,
						newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

					maxBoundaryVerticeLevel := wVertex.GetEntryPointSize()
					if maxBoundaryVerticeLevel < wQueryLevel {
						wQueryLevel = uint8(maxBoundaryVerticeLevel)
					}

					if !wAlreadyVisited {
						us.overlayPq.Insert(datastructure.NewPriorityQueueNode(
							newTravelTime, datastructure.NewCRPQueryKey(w, datastructure.Index(wQueryLevel))),
						)
					} else {
						us.overlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(
							newTravelTime, datastructure.NewCRPQueryKey(w, datastructure.Index(wQueryLevel))),
						)
					}

				}
			}
		}
	})

}

func (us *CRPUnidirectionalSearch) offsetForward(u, uEntryOffset datastructure.Index) datastructure.Index {
	if us.engine.graph.GetCellNumber(u) == us.sCellNumber {

		return uEntryOffset | datastructure.Index(1<<us.forwardSOffsetBit)
	} else {

		return uEntryOffset | datastructure.Index(1<<us.forwardTOffsetBit)
	}
}

func (us *CRPUnidirectionalSearch) offsetForwardOverlay(u *datastructure.OverlayVertex, uEntryOffset datastructure.Index) datastructure.Index {
	if u.GetCellNumber() == us.sCellNumber {
		return uEntryOffset | datastructure.Index(1<<us.forwardSOffsetBit)
	} else {
		return uEntryOffset | datastructure.Index(1<<us.forwardTOffsetBit)
	}
}

func (us *CRPUnidirectionalSearch) adjustForwardOffBit(uEntryOffset datastructure.Index) datastructure.Index {
	if uEntryOffset&datastructure.Index(1<<us.forwardSOffsetBit) != 0 {
		return uEntryOffset & ^datastructure.Index(1<<us.forwardSOffsetBit)
	} else {
		return uEntryOffset & ^datastructure.Index(1<<us.forwardTOffsetBit)
	}
}

func (us *CRPUnidirectionalSearch) adjustForward(u, uEntryOffset datastructure.Index) datastructure.Index {
	if uEntryOffset&datastructure.Index(1<<us.forwardSOffsetBit) != 0 {
		return (uEntryOffset & ^datastructure.Index(1<<us.forwardSOffsetBit)) - us.engine.graph.GetEntryOffset(u)
	} else {
		return (uEntryOffset & ^datastructure.Index(1<<us.forwardTOffsetBit)) - us.engine.graph.GetEntryOffset(u)

	}
}

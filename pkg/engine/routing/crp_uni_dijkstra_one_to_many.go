package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPUniDijkstraOneToMany struct {
	engine              *CRPRoutingEngine
	shortestTimeTravels map[da.Index]float64

	forwardInfo   []*VertexInfo[da.CRPQueryKey]
	stallingEntry []float64
	stallingExit  []float64

	pq        *da.MinHeap[da.CRPQueryKey]
	tEntryIds map[target]da.Index

	sCellNumber  da.Pv
	tCellNumbers []da.Pv

	targetsSettled map[da.Index]struct{}

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numSettledNodes int
}

func NewCRPUniDijkstraOneToMany(engine *CRPRoutingEngine) *CRPUniDijkstraOneToMany {
	return &CRPUniDijkstraOneToMany{
		engine:      engine,
		forwardInfo: make([]*VertexInfo[da.CRPQueryKey], 0),
		pq:          da.NewFourAryHeap[da.CRPQueryKey](),

		stallingEntry: make([]float64, 0),
		stallingExit:  make([]float64, 0),

		numSettledNodes: 0,

		tEntryIds:           make(map[target]da.Index, 0),
		shortestTimeTravels: make(map[da.Index]float64),
		targetsSettled:      make(map[da.Index]struct{}),
	}
}

/*
implementation of:
1. one-to-many crp-query: https://patentimages.storage.googleapis.com/00/16/32/08bc539e7761fd/US20140107921A1.pdf or https://patents.google.com/patent/US20140107921A1/en

2. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.

time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortucts within any partition
let n,m,k denote the number vertices,edges, and partitioning depth, respectively.
time complexity of CRP query is: O((n_p + m_p + k * \hat{m_p}) * log n)

if len(atIds) approaches n, u should use plain dijkstra in dijkstra.go
*/
func (us *CRPUniDijkstraOneToMany) ShortestPathOneToManySearch(asId da.Index, atIds []da.Index) (map[da.Index]float64, map[da.Index]float64, map[da.Index][]da.Coordinate,
	map[da.Index][]da.OutEdge) {

	// Our query algorithm takes as input a source arc as , a target arc at, the original graph G, the overlay graph
	// H = ∪i Hi , and computes the shortest path between the head vertex s of as and the tail vertex t of at.
	// asId exitPoint of outEdge u->s
	// atId entryPoint of inEdge t->v

	us.Preallocate()

	s := us.engine.graph.GetOutEdge(asId).GetHead()
	us.sCellNumber = us.engine.graph.GetCellNumber(s)

	ts := make([]target, 0, len(atIds))
	us.tCellNumbers = make([]da.Pv, 0, len(atIds))
	for _, atId := range atIds {
		t := us.engine.graph.GetInEdge(atId).GetTail()
		ts = append(ts, newTarget(t, atId))
		us.tCellNumbers = append(us.tCellNumbers, us.engine.graph.GetCellNumber(t))
	}

	// for iterating outEdges, we need entryOffset.
	sForwardId := us.engine.graph.GetEntryOffset(s) + da.Index(us.engine.graph.GetOutEdge(asId).GetEntryPoint())

	shNode := da.NewPriorityQueueNode(0, da.NewCRPQueryKey(s, sForwardId, false))
	us.pq.Insert(shNode)

	us.forwardInfo[sForwardId] = NewVertexInfo[da.CRPQueryKey](0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), shNode)

	finished := false

	for !us.pq.IsEmpty() {

		if finished {
			break
		}

		queryKey, _ := us.pq.ExtractMin()
		uItem := queryKey.GetItem()
		if !uItem.IsOverlay() {
			// search on graph level 1
			finished = us.graphSearchUni(uItem, s, ts)
			us.numSettledNodes++
		} else {
			// search on overlay graph
			us.overlayGraphSearchUni(uItem)
			us.numSettledNodes++
		}

	}

	tdists := make(map[da.Index]float64, len(atIds))
	tfinalPath := make(map[da.Index][]da.Coordinate, len(atIds))
	tfinalEdgePath := make(map[da.Index][]da.OutEdge, len(atIds))

	for t, tEntryId := range us.tEntryIds {
		if t.getatId() == asId || t.gettId() == s {
			tdists[t.getatId()] = 0

			tfinalPath[t.getatId()] = make([]da.Coordinate, 0)
			tfinalEdgePath[t.getatId()] = make([]da.OutEdge, 0)
			continue
		}
		idPath := make([]vertexEdgePair, 0) // contains all outedges that make up the shortest path
		curInfo := us.forwardInfo[tEntryId]

		_, tOutEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(tEntryId)
		toutEdgeId := tOutEdge.GetEdgeId()
		tpair := newVertexEdgePair(t.gettId(), toutEdgeId, true)
		idPath = append(idPath, tpair)

		for curInfo.GetParent().edge != sForwardId {
			parent := curInfo.GetParent()
			parentCopy := parent

			if parentCopy.getEdge() >= da.Index(us.engine.graph.NumberOfEdges()) {
				// shortcut
				adjustedForwardEdge := onBit(parentCopy.getEdge()-da.Index(us.engine.graph.NumberOfEdges()), UNPACK_OVERLAY_OFFSET)
				parentCopy.setEdge(adjustedForwardEdge)

			} else {

				parentCopy.setEdge(parentCopy.getEdge())

				inEdge := us.engine.graph.GetInEdge(parentCopy.getEdge())
				_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
				parentCopy.setEdge(outEdge.GetEdgeId())
			}

			parentCopy.setisOutEdge(true)
			idPath = append(idPath, parentCopy)

			curInfo = us.forwardInfo[parent.getEdge()]
		}

		idPath = util.ReverseG[vertexEdgePair](idPath)

		unpacker := NewPathUnpacker(us.engine, us.engine.metrics, us.engine.puCache, true, true)
		finalPath, finalEdgePath, totalDistance := unpacker.unpackPath(idPath, us.sCellNumber, us.engine.graph.GetCellNumber(t.gettId()))
		tdists[t.getatId()] = totalDistance
		tfinalPath[t.getatId()] = finalPath
		tfinalEdgePath[t.getatId()] = finalEdgePath
	}

	return us.shortestTimeTravels, tdists, tfinalPath, tfinalEdgePath
}

func (us *CRPUniDijkstraOneToMany) graphSearchUni(uItem da.CRPQueryKey, source da.Index, targets []target) bool {

	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1

	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	for _, t := range targets {

		_, alreadySettled := us.targetsSettled[t.gettId()]
		if alreadySettled {
			continue
		}
		if uId == t.gettId() {
			us.targetsSettled[t.gettId()] = struct{}{}
			us.shortestTimeTravels[t.getatId()] = us.forwardInfo[uEntryId].GetTravelTime()
			us.tEntryIds[t] = uEntryId
		}
	}

	if len(us.targetsSettled) == len(targets) {
		return true
	}

	uEntryPoint := uEntryId - us.engine.graph.GetEntryOffset(uId)

	// traverse outEdges of u
	us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {

		vId := outArc.GetHead()
		if vId == uId {
			return
		}

		// get query level of v l_st(v)
		lowestVQueryLevel := uint8(255)

		for _, tcellNumber := range us.tCellNumbers {
			vQueryLevel := us.engine.overlayGraph.GetQueryLevel(us.sCellNumber, tcellNumber,
				us.engine.graph.GetCellNumber(vId))
			if vQueryLevel < lowestVQueryLevel {
				lowestVQueryLevel = vQueryLevel
			}
		}

		edgeWeight := us.engine.metrics.GetWeight(outArc)

		turnCost := us.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := us.forwardInfo[uEntryId].GetTravelTime() + edgeWeight + turnCost

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())

		if lowestVQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			vAlreadyLabelled := da.Lt(us.forwardInfo[vEntryId].GetTravelTime(), pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newTravelTime, us.forwardInfo[vEntryId].GetTravelTime()) {
				// newTravelTime is not better, do nothing

				return
			}

			if bvi := us.stallingEntry[vEntryId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Ge(newTravelTime, bvi) {
				// stalled
				return
			}

			if vAlreadyLabelled {
				vhNode := us.forwardInfo[vEntryId].GetHeapNode()
				us.forwardInfo[vEntryId].UpdateTravelTime(newTravelTime)
				us.forwardInfo[vEntryId].UpdateParent(newVertexEdgePair(uId, uEntryId, false))

				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vhNode, newTravelTime)
			} else if !vAlreadyLabelled {

				vhNode := da.NewPriorityQueueNode(
					newTravelTime, da.NewCRPQueryKey(vId, vEntryId, false))
				// newTravelTime is better, update the forwardInfo
				us.forwardInfo[vEntryId] = NewVertexInfo[da.CRPQueryKey](newTravelTime,
					newVertexEdgePair(uId, uEntryId, false), vhNode)

				// is key not in the priority queue, insert it
				us.pq.Insert(vhNode)
			}

		} else {
			// v is in another cell on higher level
			// CRP In Road Networks: Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			// update the forward info of overlay vertex v
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := us.engine.graph.GetOverlayVertex(vId, outArc.GetEntryPoint(), false)
			overlayVId := v + da.Index(us.engine.graph.NumberOfEdges())

			vAlreadyLabelled := da.Lt(us.forwardInfo[overlayVId].GetTravelTime(), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, us.forwardInfo[overlayVId].GetTravelTime())) {

				if !vAlreadyLabelled {
					vhNode := da.NewPriorityQueueNode(
						newTravelTime, da.NewCRPQueryKey(v, da.Index(lowestVQueryLevel), true))

					vVertexInfo := NewVertexInfo[da.CRPQueryKey](newTravelTime,
						newVertexEdgePair(vId, vEntryId, false), vhNode)

					us.forwardInfo[overlayVId] = vVertexInfo

					us.pq.Insert(vhNode)
				} else {
					vhNode := us.forwardInfo[overlayVId].GetHeapNode()
					us.forwardInfo[overlayVId].UpdateTravelTime(newTravelTime)
					us.forwardInfo[overlayVId].UpdateParent(newVertexEdgePair(uId, uEntryId, false))
					us.forwardInfo[overlayVId].parent.setFirstOverlayEntryExitId(vEntryId)

					us.pq.DecreaseKey(vhNode, newTravelTime)
				}
			}
		}
	})

	return false
}

func (us *CRPUniDijkstraOneToMany) overlayGraphSearchUni(uItem da.CRPQueryKey) {
	// search on overlay graph

	u := uItem.GetNode() // overlay vertex id

	uId := u + da.Index(us.engine.graph.NumberOfEdges()) // overlay vertex id | overlayOffset to get unique id in forwardInfo & backwardInfo
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	// for each out neighbors of u in level l, check if v already Labelledby backward search. if so, check whether we can improve shortestPath
	// then if v not already Labelledor newTravelTime to v is better, traverse to the next cell entry vertex w using outEdge of v.
	us.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := us.engine.metrics.GetShortcutWeight(wOffset)

		newTravelTime := us.forwardInfo[uId].GetTravelTime() + shortcutOutEdgeWeight
		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}
		vVertex := us.engine.overlayGraph.GetVertex(v)

		vId := v + da.Index(us.engine.graph.NumberOfEdges())
		vAlreadyLabelled := da.Lt(us.forwardInfo[vId].GetTravelTime(), pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && newTravelTime < us.forwardInfo[vId].GetTravelTime()) {

			us.forwardInfo[vId].parent.setQueryLevel(uint8(uQueryLevel))

			// traverse edge to next cell
			vOriEdgeId := vVertex.GetOriginalEdge()
			outEdge := us.engine.graph.GetOutEdge(vOriEdgeId)
			edgeWeight := us.engine.metrics.GetWeight(outEdge)

			/*
				We apply several optimizations. First, by construction, each exit vertex u in the overlay has a single
				outgoing arc (u, v). Therefore, during the search we do not add u to the priority queue; instead, we traverse
				the arc (u, v) immediately and process v.
			*/
			// w is in the next cell from v cell
			w := vVertex.GetNeighborOverlayVertex()
			wVertex := us.engine.overlayGraph.GetVertex(w)

			lowestWQueryLevel := uint8(255)

			for _, tcellNumber := range us.tCellNumbers {
				wQueryLevel := us.engine.overlayGraph.GetQueryLevel(us.sCellNumber, tcellNumber,
					wVertex.GetCellNumber())

				if wQueryLevel < lowestWQueryLevel {
					lowestWQueryLevel = wQueryLevel
				}
			}

			originalW := wVertex.GetOriginalVertex()

			newTravelTime = us.forwardInfo[vId].GetTravelTime() + edgeWeight

			if newTravelTime >= pkg.INF_WEIGHT {
				return
			}

			if lowestWQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := us.engine.graph.GetEntryOffset(originalW) + da.Index(outEdge.GetEntryPoint())

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				wAlreadyLabelled := da.Lt(us.forwardInfo[wEntryId].GetTravelTime(), pkg.INF_WEIGHT)
				if wAlreadyLabelled && da.Ge(newTravelTime, us.forwardInfo[wEntryId].GetTravelTime()) {

					return
				}

				if wAlreadyLabelled {
					whNode := us.forwardInfo[wEntryId].GetHeapNode()
					us.forwardInfo[wEntryId].UpdateTravelTime(newTravelTime)
					us.forwardInfo[wEntryId].UpdateParent(newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

					us.pq.DecreaseKey(whNode, newTravelTime)
				} else {
					whNode := da.NewPriorityQueueNode(
						newTravelTime, da.NewCRPQueryKey(originalW, wEntryId, false))

					us.forwardInfo[wEntryId] = NewVertexInfo[da.CRPQueryKey](newTravelTime,
						newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false), whNode)

					us.pq.Insert(whNode)
				}

			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to pq, because we need to traverse & relax shortcut edges in overlay graph
				wId := w + da.Index(us.engine.graph.NumberOfEdges())
				wAlreadyLabelled := da.Lt(us.forwardInfo[wId].GetTravelTime(), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, us.forwardInfo[wId].GetTravelTime())) {

					if !wAlreadyLabelled {
						whNode := da.NewPriorityQueueNode(
							newTravelTime, da.NewCRPQueryKey(w, da.Index(lowestWQueryLevel), true))
						us.forwardInfo[wId] = NewVertexInfo[da.CRPQueryKey](newTravelTime,
							newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false), whNode)

						us.pq.Insert(whNode)
					} else {
						whNode := us.forwardInfo[wId].GetHeapNode()
						us.forwardInfo[wId].UpdateTravelTime(newTravelTime)
						us.forwardInfo[wId].UpdateParent(newVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

						us.pq.DecreaseKey(whNode, newTravelTime)
					}
				}
			}
		}
	})
}

func (bs *CRPUniDijkstraOneToMany) Preallocate() {

	maxSearchSize := bs.engine.graph.NumberOfEdges() + bs.engine.overlayGraph.NumberOfOverlayVertices()
	bs.forwardInfo = make([]*VertexInfo[da.CRPQueryKey], maxSearchSize)

	initInfWeightVertexInfo(bs.forwardInfo)

	bs.stallingEntry = make([]float64, maxSearchSize)
	bs.stallingExit = make([]float64, maxSearchSize)

	initInfWeight(bs.stallingEntry)
	initInfWeight(bs.stallingExit)
}

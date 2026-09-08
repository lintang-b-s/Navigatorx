package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPUniDijkstraOneToMany[W util.RoutingNumber] struct {
	engine        *CRPRoutingEngine[W]
	shortestCosts map[da.Index]W

	stallingEntry []W
	stallingExit  []W

	pq        *da.QueryHeap[da.CRPQueryKey, W]
	tEntryIds map[target]da.Index

	sCellNumber  da.Pv
	tCellNumbers []da.Pv

	targetsSettled map[da.Index]struct{}

	numSettledNodes int
}

func NewCRPUniDijkstraOneToMany[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *CRPUniDijkstraOneToMany[W] {
	crpQuery := &CRPUniDijkstraOneToMany[W]{
		engine: engine,

		stallingEntry: make([]W, 0),
		stallingExit:  make([]W, 0),

		numSettledNodes: 0,

		tEntryIds:      make(map[target]da.Index, 0),
		shortestCosts:  make(map[da.Index]W),
		targetsSettled: make(map[da.Index]struct{}),
	}
	crpQuery.Preallocate()
	return crpQuery
}

/*
implementation of:
1. one-to-many crp-query: https://patentimages.storage.googleapis.com/00/16/32/08bc539e7761fd/US20140107921A1.pdf or https://patents.google.com/patent/US20140107921A1/en

2. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.

let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
extract-min at most O(m_p+n_o) operations

if len(atIds) approaches n, u should use plain dijkstra in dijkstra.go
*/
func (us *CRPUniDijkstraOneToMany[W]) ShortestPathOneToManySearch(asId da.Index, atIds []da.Index) (map[da.Index]W, map[da.Index]float64, map[da.Index][]da.Coordinate,
	map[da.Index][]da.Index) {

	us.Preallocate()

	asEdge := us.engine.graph.GetOutEdge(asId)
	s := asEdge.GetHead()

	us.sCellNumber = us.engine.graph.GetCellNumber(s)

	ts := make([]target, 0, len(atIds))
	us.tCellNumbers = make([]da.Pv, 0, len(atIds))
	for _, atId := range atIds {
		atEdge := us.engine.graph.GetInEdge(atId)
		t := atEdge.GetTail()
		ts = append(ts, newTarget(t, atId))
		us.tCellNumbers = append(us.tCellNumbers, us.engine.graph.GetCellNumber(t))
	}

	// for iterating outEdges, we need entryOffset.
	sForwardId := us.engine.graph.GetEntryOffset(s) + da.Index(asEdge.GetEntryPoint())

	sQueryKey := da.NewCRPQueryKey(s, sForwardId, false)
	sVertexData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	us.pq.Insert(sForwardId, 0, sVertexData, sQueryKey)

	finished := false

	for !us.pq.IsEmpty() {

		if finished {
			break
		}

		queryKey := us.pq.ExtractMin()
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
	tfinalEdgePath := make(map[da.Index][]da.Index, len(atIds))

	for t, tEntryId := range us.tEntryIds {
		if t.getatId() == asId || t.gettId() == s {
			tdists[t.getatId()] = 0

			tfinalPath[t.getatId()] = make([]da.Coordinate, 0)
			tfinalEdgePath[t.getatId()] = make([]da.Index, 0)
			continue
		}
		idPath := make([]da.VertexEdgePair, 0) // contains all outedges that make up the shortest path
		vData := us.pq.Get(tEntryId)

		_, tOutEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(tEntryId)
		toutEdgeId := tOutEdge
		tpair := da.NewVertexEdgePair(t.gettId(), toutEdgeId, true)
		idPath = append(idPath, tpair)

		for vData.GetParent().GetEdge() != sForwardId {
			parent := vData.GetParent()
			parentCopy := parent

			if parentCopy.GetEdge() >= da.Index(us.engine.graph.NumberOfEdges()) {
				// shortcut
				adjustedForwardEdge := onBit(parentCopy.GetEdge()-da.Index(us.engine.graph.NumberOfEdges()), UNPACK_OVERLAY_OFFSET)
				parentCopy.SetEdge(adjustedForwardEdge)

			} else {

				parentCopy.SetEdge(parentCopy.GetEdge())

				inEdge := us.engine.graph.GetInEdge(parentCopy.GetEdge())
				_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
				parentCopy.SetEdge(outEdge)
			}

			idPath = append(idPath, parentCopy)

			vData = us.pq.Get(parent.GetEdge())
		}

		util.ReverseG[da.VertexEdgePair](idPath)

		unpacker := NewPathUnpacker(us.engine)
		edgeIdPath, _ := unpacker.unpackPath(idPath, us.sCellNumber, us.engine.graph.GetCellNumber(t.gettId()), true)
		finalPath, totalDistance := us.engine.GetEdgePath(edgeIdPath)
		tdists[t.getatId()] = totalDistance
		tfinalPath[t.getatId()] = *finalPath
		tfinalEdgePath[t.getatId()] = edgeIdPath
		us.engine.PutCoordsToPool(finalPath)
	}

	return us.shortestCosts, tdists, tfinalPath, tfinalEdgePath
}

func (us *CRPUniDijkstraOneToMany[W]) graphSearchUni(uItem da.CRPQueryKey, source da.Index, targets []target) bool {

	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	for _, t := range targets {

		_, alreadySettled := us.targetsSettled[t.gettId()]
		if alreadySettled {
			continue
		}
		if uId == t.gettId() {
			us.targetsSettled[t.gettId()] = struct{}{}
			us.shortestCosts[t.getatId()] = us.pq.GetCost(uEntryId)
			us.tEntryIds[t] = uEntryId
		}
	}

	if len(us.targetsSettled) == len(targets) {
		return true
	}

	uEntryPoint := uEntryId - us.engine.graph.GetEntryOffset(uId)

	// traverse outEdges of u
	us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(eId, head da.Index, exitPoint, entryPoint, turnTableId da.Index,
		turnType pkg.TurnType,
		hwType pkg.OsmHighwayType) {

		vId := head

		// get query level of v l_st(v)
		lowestVQueryLevel := uint8(255)

		for _, tcellNumber := range us.tCellNumbers {
			vQueryLevel := us.engine.overlayGraph.GetQueryLevel(us.sCellNumber, tcellNumber,
				us.engine.graph.GetCellNumber(vId))
			if vQueryLevel < lowestVQueryLevel {
				lowestVQueryLevel = vQueryLevel
			}
		}

		edgeWeight := us.engine.getWeight(eId, true)

		turnCost := us.engine.metrics.GetTurnCost(turnTableId)

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newVCost := us.pq.GetCost(uEntryId) + edgeWeight + turnCost

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}

		vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(entryPoint)

		if lowestVQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			vAlreadyLabelled := util.Lt(us.pq.GetCost(vEntryId), util.Infinity[W]())
			if vAlreadyLabelled && util.Ge(newVCost, us.pq.GetCost(vEntryId)) {
				// newVCost is not better, do nothing

				return
			}

			if bvi := us.stallingEntry[vEntryId]; util.Lt(bvi, util.Infinity[W]()) && util.Gt(newVCost, bvi) {
				// stalled
				return
			}

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, uEntryId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vEntryId, newVCost, newVCost,
					newPar)
			} else if !vAlreadyLabelled {

				queryKey := da.NewCRPQueryKey(vId, vEntryId, false)
				// newVCost is better, update the forwardData
				vData := da.NewVertexData(newVCost,
					da.NewVertexEdgePair(uId, uEntryId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vEntryId, newVCost, vData, queryKey)
			}

		} else {
			// v is in another cell on higher level

			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := us.engine.graph.GetOverlayVertex(vId, entryPoint, false)
			overlayVId := v + da.Index(us.engine.graph.NumberOfEdges())

			vAlreadyLabelled := util.Lt(us.pq.GetCost(overlayVId), util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newVCost, us.pq.GetCost(overlayVId))) {

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKey(v, da.Index(lowestVQueryLevel), true)

					vData := da.NewVertexData(newVCost,
						da.NewVertexEdgePair(vId, vEntryId, false))

					us.pq.Insert(overlayVId, newVCost, vData, queryKey)
				} else {
					newPar := da.NewVertexEdgePair(uId, uEntryId, false)
					us.pq.DecreaseKey(overlayVId, newVCost, newVCost,
						newPar)
				}
			}
		}
	})

	return false
}

func (us *CRPUniDijkstraOneToMany[W]) overlayGraphSearchUni(uItem da.CRPQueryKey) {
	// search on overlay graph

	u := uItem.GetNode() // overlay vertex id

	uId := u + da.Index(us.engine.graph.NumberOfEdges())
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	us.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := us.engine.metrics.GetShortcutWeight(wOffset)

		newVCost := us.pq.GetCost(uId) + shortcutOutEdgeWeight
		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}
		vVertex := us.engine.overlayGraph.GetVertex(v)

		vId := v + da.Index(us.engine.graph.NumberOfEdges())
		vAlreadyLabelled := util.Lt(us.pq.GetCost(vId), util.Infinity[W]())
		if !vAlreadyLabelled || (vAlreadyLabelled && newVCost < us.pq.GetCost(vId)) {

			us.pq.SetQueryLevel(vId, uint8(uQueryLevel))

			// traverse edge to next cell
			vCutEdgeId := vVertex.GetCutEdge()

			edgeWeight := us.engine.getWeight(vCutEdgeId, true)
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

			originalW := wVertex.GetOrigVId()

			newVCost = us.pq.GetCost(vId) + edgeWeight

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			if lowestWQueryLevel == 0 {
				// w is in the same cell as s or t
				entryPoint := us.engine.graph.GetEntryPointOfOutEdge(vCutEdgeId)
				wEntryId := us.engine.graph.GetEntryOffset(originalW) + entryPoint

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				wAlreadyLabelled := util.Lt(us.pq.GetCost(wEntryId), util.Infinity[W]())
				if wAlreadyLabelled && util.Ge(newVCost, us.pq.GetCost(wEntryId)) {

					return
				}

				if wAlreadyLabelled {
					newPar := da.NewVertexEdgePair(vVertex.GetOrigVId(), vId, false)
					us.pq.DecreaseKey(wEntryId, newVCost, newVCost, newPar)
				} else {
					queryKey := da.NewCRPQueryKey(originalW, wEntryId, false)
					vData := da.NewVertexData(newVCost,
						da.NewVertexEdgePair(vVertex.GetOrigVId(), vId, false))

					us.pq.Insert(wEntryId, newVCost, vData, queryKey)
				}

			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to pq, because we need to traverse & relax shortcut edges in overlay graph
				wId := w + da.Index(us.engine.graph.NumberOfEdges())
				wAlreadyLabelled := util.Lt(us.pq.GetCost(wId), util.Infinity[W]())
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newVCost, us.pq.GetCost(wId))) {

					if !wAlreadyLabelled {
						queryKey := da.NewCRPQueryKey(w, da.Index(lowestWQueryLevel), true)
						vData := da.NewVertexData(newVCost,
							da.NewVertexEdgePair(vVertex.GetOrigVId(), vId, false))

						us.pq.Insert(wId, newVCost, vData, queryKey)
					} else {
						newPar := da.NewVertexEdgePair(vVertex.GetOrigVId(), vId, false)

						us.pq.DecreaseKey(wId, newVCost, newVCost, newPar)
					}
				}
			}
		}
	})
}

func (us *CRPUniDijkstraOneToMany[W]) Preallocate() {
	maxEdgesInCell := us.engine.graph.GetMaxEdgesInCell()

	maxSearchSize := us.engine.graph.NumberOfEdges() + us.engine.overlayGraph.NumberOfOverlayVertices()
	us.pq = da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxSearchSize), uint32(maxEdgesInCell), da.TWO_LEVEL_STORAGE, true)
	us.pq.PreallocateHeap(maxSearchSize)
	us.stallingEntry = make([]W, maxSearchSize)
	us.stallingExit = make([]W, maxSearchSize)

	initInfWeight(us.stallingEntry)
	initInfWeight(us.stallingExit)
}

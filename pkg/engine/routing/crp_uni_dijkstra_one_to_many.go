package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPUniDijkstraOneToMany struct {
	engine              *CRPRoutingEngine
	shortestTravelTimes map[da.Index]float64

	stallingEntry []float64
	stallingExit  []float64

	pq        *da.QueryHeap[da.CRPQueryKey]
	tEntryIds map[target]da.Index

	sCellNumber  da.Pv
	tCellNumbers []da.Pv

	targetsSettled map[da.Index]struct{}

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numSettledNodes int
}

func NewCRPUniDijkstraOneToMany(engine *CRPRoutingEngine) *CRPUniDijkstraOneToMany {
	crpQuery := &CRPUniDijkstraOneToMany{
		engine: engine,

		stallingEntry: make([]float64, 0),
		stallingExit:  make([]float64, 0),

		numSettledNodes: 0,

		tEntryIds:           make(map[target]da.Index, 0),
		shortestTravelTimes: make(map[da.Index]float64),
		targetsSettled:      make(map[da.Index]struct{}),
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

let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
extract-min at most O(m_p+n_o) operations

if len(atIds) approaches n, u should use plain dijkstra in dijkstra.go
*/
func (us *CRPUniDijkstraOneToMany) ShortestPathOneToManySearch(asId da.Index, atIds []da.Index) (map[da.Index]float64, map[da.Index]float64, map[da.Index][]da.Coordinate,
	map[da.Index][]da.OutEdge) {

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

	sQueryKey := da.NewCRPQueryKey(s, sForwardId, false)
	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	us.pq.Insert(sForwardId, 0, sVertexInfo, sQueryKey)

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
	tfinalEdgePath := make(map[da.Index][]da.OutEdge, len(atIds))

	for t, tEntryId := range us.tEntryIds {
		if t.getatId() == asId || t.gettId() == s {
			tdists[t.getatId()] = 0

			tfinalPath[t.getatId()] = make([]da.Coordinate, 0)
			tfinalEdgePath[t.getatId()] = make([]da.OutEdge, 0)
			continue
		}
		idPath := make([]da.VertexEdgePair, 0) // contains all outedges that make up the shortest path
		curInfo := us.pq.Get(tEntryId)

		_, tOutEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(tEntryId)
		toutEdgeId := tOutEdge.GetEdgeId()
		tpair := da.NewVertexEdgePair(t.gettId(), toutEdgeId, true)
		idPath = append(idPath, tpair)

		for curInfo.GetParent().GetEdge() != sForwardId {
			parent := curInfo.GetParent()
			parentCopy := parent

			if parentCopy.GetEdge() >= da.Index(us.engine.graph.NumberOfEdges()) {
				// shortcut
				adjustedForwardEdge := onBit(parentCopy.GetEdge()-da.Index(us.engine.graph.NumberOfEdges()), UNPACK_OVERLAY_OFFSET)
				parentCopy.SetEdge(adjustedForwardEdge)

			} else {

				parentCopy.SetEdge(parentCopy.GetEdge())

				inEdge := us.engine.graph.GetInEdge(parentCopy.GetEdge())
				_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
				parentCopy.SetEdge(outEdge.GetEdgeId())
			}

			idPath = append(idPath, parentCopy)

			curInfo = us.pq.Get(parent.GetEdge())
		}

		idPath = util.ReverseG[da.VertexEdgePair](idPath)

		unpacker := NewPathUnpacker(us.engine, us.engine.metrics, us.engine.puCache, true, true)
		finalPath, finalEdgePath, totalDistance := unpacker.unpackPath(idPath, us.sCellNumber, us.engine.graph.GetCellNumber(t.gettId()))
		tdists[t.getatId()] = totalDistance
		tfinalPath[t.getatId()] = finalPath
		tfinalEdgePath[t.getatId()] = finalEdgePath
	}

	return us.shortestTravelTimes, tdists, tfinalPath, tfinalEdgePath
}

func (us *CRPUniDijkstraOneToMany) graphSearchUni(uItem da.CRPQueryKey, source da.Index, targets []target) bool {

	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	for _, t := range targets {

		_, alreadySettled := us.targetsSettled[t.gettId()]
		if alreadySettled {
			continue
		}
		if uId == t.gettId() {
			us.targetsSettled[t.gettId()] = struct{}{}
			us.shortestTravelTimes[t.getatId()] = us.pq.GetPriority(uEntryId)
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
		newTravelTime := us.pq.GetPriority(uEntryId) + edgeWeight + turnCost

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())

		if lowestVQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			vAlreadyLabelled := da.Lt(us.pq.GetPriority(vEntryId), pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newTravelTime, us.pq.GetPriority(vEntryId)) {
				// newTravelTime is not better, do nothing

				return
			}

			if bvi := us.stallingEntry[vEntryId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
				// stalled
				return
			}

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, uEntryId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vEntryId, newTravelTime, newTravelTime,
					newPar)
			} else if !vAlreadyLabelled {

				queryKey := da.NewCRPQueryKey(vId, vEntryId, false)
				// newTravelTime is better, update the forwardInfo
				vertexInfo := da.NewVertexInfo(newTravelTime,
					da.NewVertexEdgePair(uId, uEntryId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vEntryId, newTravelTime, vertexInfo, queryKey)
			}

		} else {
			// v is in another cell on higher level

			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := us.engine.graph.GetOverlayVertex(vId, outArc.GetEntryPoint(), false)
			overlayVId := v + da.Index(us.engine.graph.NumberOfEdges())

			vAlreadyLabelled := da.Lt(us.pq.GetPriority(overlayVId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, us.pq.GetPriority(overlayVId))) {

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKey(v, da.Index(lowestVQueryLevel), true)

					vertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(vId, vEntryId, false))

					us.pq.Insert(overlayVId, newTravelTime, vertexInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePair(uId, uEntryId, false)
					us.pq.DecreaseKey(overlayVId, newTravelTime, newTravelTime,
						newPar)
				}
			}
		}
	})

	return false
}

func (us *CRPUniDijkstraOneToMany) overlayGraphSearchUni(uItem da.CRPQueryKey) {
	// search on overlay graph

	u := uItem.GetNode() // overlay vertex id

	uId := u + da.Index(us.engine.graph.NumberOfEdges())
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	us.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := us.engine.metrics.GetShortcutWeight(wOffset)

		newTravelTime := us.pq.GetPriority(uId) + shortcutOutEdgeWeight
		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}
		vVertex := us.engine.overlayGraph.GetVertex(v)

		vId := v + da.Index(us.engine.graph.NumberOfEdges())
		vAlreadyLabelled := da.Lt(us.pq.GetPriority(vId), pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && newTravelTime < us.pq.GetPriority(vId)) {

			us.pq.SetQueryLevel(vId, uint8(uQueryLevel))

			// traverse edge to next cell
			vOriEdgeId := vVertex.GetOriginalEdge()
			outEdge := us.engine.graph.GetOutEdge(vOriEdgeId)
			edgeWeight := us.engine.metrics.GetWeight(outEdge)

		
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

			newTravelTime = us.pq.GetPriority(vId) + edgeWeight

			if newTravelTime >= pkg.INF_WEIGHT {
				return
			}

			if lowestWQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := us.engine.graph.GetEntryOffset(originalW) + da.Index(outEdge.GetEntryPoint())

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				wAlreadyLabelled := da.Lt(us.pq.GetPriority(wEntryId), pkg.INF_WEIGHT)
				if wAlreadyLabelled && da.Ge(newTravelTime, us.pq.GetPriority(wEntryId)) {

					return
				}

				if wAlreadyLabelled {
					newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), vId, false)
					us.pq.DecreaseKey(wEntryId, newTravelTime, newTravelTime, newPar)
				} else {
					queryKey := da.NewCRPQueryKey(originalW, wEntryId, false)
					vertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

					us.pq.Insert(wEntryId, newTravelTime, vertexInfo, queryKey)
				}

			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to pq, because we need to traverse & relax shortcut edges in overlay graph
				wId := w + da.Index(us.engine.graph.NumberOfEdges())
				wAlreadyLabelled := da.Lt(us.pq.GetPriority(wId), pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, us.pq.GetPriority(wId))) {

					if !wAlreadyLabelled {
						queryKey := da.NewCRPQueryKey(w, da.Index(lowestWQueryLevel), true)
						vertexInfo := da.NewVertexInfo(newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), vId, false))

						us.pq.Insert(wId, newTravelTime, vertexInfo, queryKey)
					} else {
						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), vId, false)

						us.pq.DecreaseKey(wId, newTravelTime, newTravelTime, newPar)
					}
				}
			}
		}
	})
}

func (bs *CRPUniDijkstraOneToMany) Preallocate() {
	maxEdgesInCell := bs.engine.graph.GetMaxEdgesInCell()

	maxSearchSize := bs.engine.graph.NumberOfEdges() + bs.engine.overlayGraph.NumberOfOverlayVertices()
	bs.pq = da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, int(maxEdgesInCell), da.TWO_LEVEL_STORAGE)
	bs.pq.PreallocateHeap(maxSearchSize)
	bs.stallingEntry = make([]float64, maxSearchSize)
	bs.stallingExit = make([]float64, maxSearchSize)

	initInfWeight(bs.stallingEntry)
	initInfWeight(bs.stallingExit)
}

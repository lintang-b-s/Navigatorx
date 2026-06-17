package routing

import (
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PathUnpacker[W util.RoutingNumber] struct {
	eng     *CRPRoutingEngine[W]
	runtime int64
}

func NewPathUnpacker[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *PathUnpacker[W] {
	return &PathUnpacker[W]{
		eng:     engine,
		runtime: 0,
	}
}

/*
[1] Path Unpacking phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.

unpackPath. unpack a level-i shortcut (v, w) by running Dijkstra  between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
jika level i-1 >= 1, kita menggunakan shortcut edges (dari subcells dari level-i cell containing the shortcut) di overlay graph level i-1
jika level i-1 = 0, kita menggunakan base edges yang terletak pada level-i cell containing the shortcut

this path unpacking use dijkstra

time complexity:
let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (compact graph CRP graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
let q = number of shorcut edges in packedPath
let L = highest level of multilevel partition
time complexity of unpackPath: O(q * ( L *  (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p) ) )
*/
func (pu *PathUnpacker[W]) unpackPath(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv, oneToMany bool) ([]da.Index, map[uint64]uint8) {

	unpackedEdgePath := make([]da.Index, 0, len(packedPath))

	now := time.Now()

	shortcutPathSet := make(map[uint64]uint8)

	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.GetEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)

			unpackedEdgePath = append(unpackedEdgePath, cur.GetEdge())

			i++
		} else {
			// overlay vertex
			entryOverlayId := offBit(cur.GetEdge(), UNPACK_OVERLAY_OFFSET)

			entryVertex := pu.eng.overlayGraph.GetVertex(entryOverlayId)
			entryCellNumber := entryVertex.GetCellNumber()
			var queryLevel uint8

			if !oneToMany {
				queryLevel = pu.eng.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			} else {
				queryLevel = cur.GetQueryLevel()
			}

			exitOverlayId := offBit(packedPath[i+1].GetEdge(), UNPACK_OVERLAY_OFFSET)
			exitVertex := pu.eng.overlayGraph.GetVertex(exitOverlayId)

			enOriVId := entryVertex.GetOriginalVertex()
			exOriVId := exitVertex.GetOriginalVertex()
			shortcutPathSet[util.Bitpack(uint32(enOriVId), uint32(exOriVId))] = queryLevel

			unpackedEdgePath = pu.unpackInLevelCell(entryOverlayId, exitOverlayId, queryLevel, unpackedEdgePath)
			i += 2
		}
	}

	unpackedEdgePath = removeConsecutiveDuplicates(unpackedEdgePath)

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedEdgePath, shortcutPathSet
}

func (pu *PathUnpacker[W]) unpackInLevelCell(sourceOverlayId da.Index,
	targetOverlayId da.Index,
	level uint8,
	edgePath []da.Index,
) []da.Index {

	if level == 1 {
		sourceOverlayVertex := pu.eng.overlayGraph.GetVertex(sourceOverlayId)
		sourceEntryId := sourceOverlayVertex.GetOriginalEdge()
		targetOverlayVertex := pu.eng.overlayGraph.GetVertex(targetOverlayId)
		neighborOfTarget := targetOverlayVertex.GetNeighborOverlayVertex()
		neighborOverlayVertex := pu.eng.overlayGraph.GetVertex(neighborOfTarget)
		targetEntryId := neighborOverlayVertex.GetOriginalEdge()

		edgePath = pu.unpackInLowestLevelCell(sourceEntryId, targetEntryId,
			sourceOverlayId, targetOverlayId, edgePath)
		return edgePath
	}

	if overlayPath, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {
		for i := 0; i < len(overlayPath); i += 2 {
			edgePath = pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, edgePath)
		}
		return edgePath
	}

	sVertex := pu.eng.overlayGraph.GetVertex(sourceOverlayId)
	sourceCellNumber := sVertex.GetCellNumber()

	pq := pu.eng.pufOverlayHeapPool.Get().(*da.QueryHeap[da.Index, W])

	pq.Clear()

	truncatedSourceCellNumber := pu.eng.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	sVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	pq.Insert(sourceOverlayId, 0, sVertexInfo, sourceOverlayId)

	labelled := func(pq *da.QueryHeap[da.Index, W], v da.Index) bool {

		ok := util.Lt(pq.GetPriority(v), util.Infinity[W]())
		return ok
	}

	for pq.Size() > 0 {

		u := pq.ExtractMin()

		uOverlayId := u.GetItem()
		pq.Scan(uOverlayId)

		if uOverlayId == targetOverlayId {
			break
		}

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.eng.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.eng.metrics.GetShortcutWeight(wOffset)
			vOverlayVertex := pu.eng.overlayGraph.GetVertex(vOverlayId)

			newTravelTime := pq.GetPriority(uOverlayId) + shortcutOutEdgeWeight

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			vAlreadyLabelled := labelled(pq, vOverlayId)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(vOverlayId))) {
				// relax shortcut edge

				pq.Scan(vOverlayId) // langsung scan exit overlay vertex v
				uOverlayVertex := pu.eng.overlayGraph.GetVertex(uOverlayId)
				originalUId := uOverlayVertex.GetOriginalVertex()

				if vOverlayId == targetOverlayId {
					// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
					// if v is the target overlay vertex, insert/decrease its key  pq

					if !vAlreadyLabelled {
						wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(originalUId,
							uOverlayId, true))
						pq.Insert(vOverlayId, newTravelTime, wVertexInfo, uOverlayId)
					} else {
						wNewPar := da.NewVertexEdgePair(originalUId,
							uOverlayId, true)
						pq.DecreaseKey(vOverlayId, newTravelTime, newTravelTime, wNewPar)
					}

				} else {
					pq.Set(vOverlayId, da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(originalUId,
						uOverlayId, true)), vOverlayId)
				}

				// visit next cell neighbor
				wNeighborId := vOverlayVertex.GetNeighborOverlayVertex()
				wNeigborVertex := pu.eng.overlayGraph.GetVertex(wNeighborId)

				wCellNumber := wNeigborVertex.GetCellNumber()
				truncatedWCellNumber := pu.eng.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
				if truncatedWCellNumber != truncatedSourceCellNumber {
					// if w is not in the same cell as sourceOverlayId in level l, dont visit w
					return
				}

				// get out edge that point to wEntryVertex from vOverlayId
				newTravelTime += pu.eng.getWeight(vOverlayVertex.GetOriginalEdge(), true)
				// relax edge
				wAlreadyLabelled := labelled(pq, wNeighborId)
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(wNeighborId))) {
					if !wAlreadyLabelled {
						wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
							vOverlayId, true))
						pq.Insert(wNeighborId, newTravelTime, wVertexInfo, wNeighborId)
					} else {
						wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
							vOverlayId, true)
						pq.DecreaseKey(wNeighborId, newTravelTime, newTravelTime, wNewPar)
					}
				}

			}

		})

	}

	overlayPath := make([]da.Index, 0, 64)
	overlayPath = append(overlayPath, targetOverlayId)

	curOverlayId := pq.Get(targetOverlayId).GetParent().GetEdge()
	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pq.Get(curOverlayId).GetParent().GetEdge()
	}

	util.ReverseG(overlayPath)

	pu.eng.puCache.Set(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath)

	pq.Clear()
	pu.eng.pufOverlayHeapPool.Put(pq)

	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]
		edgePath = pu.unpackInLevelCell(curV, nextV, level-1, edgePath)
	}

	return edgePath
}

func (pu *PathUnpacker[W]) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	sourceOverlayId, targetOverlayId da.Index, edgePath []da.Index) []da.Index {

	if edgeIds, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
		// fetch from cache
		return append(edgePath, edgeIds...)
	}

	// sourceEntryId inEdge that point to source vertex
	pq := pu.eng.pufBaseHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey, W])
	pq.Clear()

	// sourceEntryId: id buat inEdge u->s
	// targetEntryId: id buat inEdge t->v

	// get source vertex
	sourceVertex := pu.eng.graph.GetVertex(pu.eng.graph.GetHeadFromInEdge(sourceEntryId))

	s := sourceVertex.GetID()

	// s and t in same cell in level 1 and both are overlay vertices
	// s is entry vertex, t is exit vertex of this cell

	_, sOutEdge := pu.eng.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)

	sourceCellNumber := pu.eng.graph.GetCellNumber(s)

	offSourceEntryId := pu.eng.offsetForward(s, sourceEntryId, sourceCellNumber, sourceCellNumber)

	sQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(s, offSourceEntryId, sOutEdge)
	sInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))

	pq.Insert(offSourceEntryId, 0, sInfo, sQueryKey)

	offTargetEntryId := da.INVALID_EDGE_ID

	for pq.Size() > 0 {

		queryKey := pq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryId := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutInEdgeId()

		pq.Scan(uEntryId)

		adjuEntryId := pu.eng.adjustForward(uId, uEntryId)

		if adjuEntryId == targetEntryId {
			offTargetEntryId = uEntryId
			break
		}

		// relax all out edges of u
		pu.eng.graph.ForOutEdgesOf(uId, pu.eng.graph.GetEntryOrder(uId, adjuEntryId), func(eId, head da.Index, exitPoint, entryPoint, turnTableId da.Index, turnType pkg.TurnType,
			hwType pkg.OsmHighwayType) {
			vId := head

			vEntryId := pu.eng.graph.GetEntryOffset(vId) + entryPoint
			edgeWeight := pu.eng.getWeight(eId, true)

			newTravelTime := pq.GetPriority(uEntryId) + edgeWeight + pu.eng.metrics.GetTurnCost(turnTableId)

			if pu.eng.graph.GetCellNumber(vId) != sourceCellNumber && vEntryId != targetEntryId {
				// do not cross cell boundary
				return
			}

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			offVEntryId := pu.eng.offsetForward(vId, vEntryId, pu.eng.graph.GetCellNumber(vId), sourceCellNumber)

			// relax edge
			vAlreadyLabelled := util.Lt(pq.GetPriority(offVEntryId), util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(offVEntryId))) {

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, eId)
					vInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))

					pq.Insert(offVEntryId, newTravelTime, vInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false)
					pq.DecreaseKey(offVEntryId, newTravelTime, newTravelTime, newPar)
				}
			}

		})

	}

	startLen := len(edgePath)

	_, midOutEdgeId := pu.eng.graph.GetHeadOfInedgeWithOutEdge(targetEntryId)
	if util.Gt(pu.eng.getWeight(midOutEdgeId, true), W(0)) {
		edgePath = append(edgePath, midOutEdgeId)
	}

	uId := offTargetEntryId
	for pq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := pq.Get(uId).GetParent().GetOutInEdgeId()

		edgePath = append(edgePath, prevOutEdgeId)

		pEId := pq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	subPath := edgePath[startLen:]
	util.ReverseG(subPath)

	subPathCop := make([]da.Index, len(subPath))
	copy(subPathCop, subPath) // harus di copy, karena bisa aja keubah di remove removeConsecutiveDuplicates
	pu.eng.puCache.Set(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), subPathCop)

	pq.Clear()
	pu.eng.pufBaseHeapPool.Put(pq)

	return edgePath
}

func (pu *PathUnpacker[W]) GetStats() int64 {
	return pu.runtime
}

func (crp *CRPRoutingEngine[W]) GetEdgePath(edgeIdPath []da.Index, reserve int) (*da.Coordinates, float64) {

	totalDistance := 0.0

	capacity := 0

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		totalDistance += crp.GetSegmentLength(eId, true)
		capacity += crp.graph.GetEdgeGeometryLength(eId)
	}

	path := crp.GetCoordsFromPool()
	path.Grow(capacity + reserve)

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		crp.graph.AppendPathWithEdgeGeometry(path, eId)
	}

	return path, totalDistance
}

func (crp *CRPRoutingEngine[W]) GetPathDistance(edgeIdPath []da.Index) float64 {

	totalDistance := 0.0

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		totalDistance += crp.GetSegmentLength(eId, true)
	}

	return totalDistance
}

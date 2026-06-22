package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PathUnpackerALTNoTurnCost[W util.RoutingNumber] struct {
	eng *CRPRoutingEngine[W]

	runtime int64
}

func NewPathUnpackerALTNoTurnCost[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *PathUnpackerALTNoTurnCost[W] {
	pu := engine.pathUnpackerNoTurnCostPool.Get().(*PathUnpackerALTNoTurnCost[W])
	return pu
}

func newPathUnpackerALTNoTurnCostAlloc[W util.RoutingNumber](engine *CRPRoutingEngine[W]) *PathUnpackerALTNoTurnCost[W] {
	pu := &PathUnpackerALTNoTurnCost[W]{}
	pu.eng = engine
	pu.runtime = 0
	return pu
}

func (pu *PathUnpackerALTNoTurnCost[W]) Reset() {
	pu.runtime = 0
}

// DonePooled returns a PathUnpackerALTNoTurnCost instance back to the engine pool so
// the next call can reuse the already-paid-for allocation.
func (pu *PathUnpackerALTNoTurnCost[W]) DonePooled() {
	pu.eng.pathUnpackerNoTurnCostPool.Put(pu)
}

func (pu *PathUnpackerALTNoTurnCost[W]) unpackPath(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv) []da.Index {

	unpackedVertexPath := make([]da.Index, 0, len(packedPath)) // unpacked vertex path
	now := time.Now()

	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.GetVertex(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)

			unpackedVertexPath = append(unpackedVertexPath, cur.GetVertex())
			i++
		} else {
			// overlay vertex
			entryOverlayId := offBit(cur.GetVertex(), UNPACK_OVERLAY_OFFSET)

			entryVertex := pu.eng.overlayGraph.GetVertex(entryOverlayId)
			entryCellNumber := entryVertex.GetCellNumber()

			queryLevel := pu.eng.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			exitOverlayId := offBit(packedPath[i+1].GetVertex(), UNPACK_OVERLAY_OFFSET)

			unpackedVertexPath = pu.unpackInLevelCell(entryOverlayId, exitOverlayId, queryLevel, unpackedVertexPath)
			i += 2
		}
	}

	unpackedVertexPath = removeConsecutiveDuplicates(unpackedVertexPath)

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedVertexPath
}

func (pu *PathUnpackerALTNoTurnCost[W]) unpackInLevelCell(sourceOverlayId da.Index,
	targetOverlayId da.Index,
	level uint8,
	vertexPath []da.Index,
) []da.Index {

	if level == 1 {

		vertexPath = pu.unpackInLowestLevelCell(
			sourceOverlayId, targetOverlayId, vertexPath)
		return vertexPath
	}

	if overlayPath, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {
		for i := 0; i < len(overlayPath); i += 2 {
			vertexPath = pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, vertexPath)
		}
		return vertexPath
	}

	sVertex := pu.eng.overlayGraph.GetVertex(sourceOverlayId)
	sourceCellNumber := sVertex.GetCellNumber()

	pq := pu.eng.pufOverlayHeapPool.Get().(*da.QueryHeap[da.Index, W])

	truncatedSourceCellNumber := pu.eng.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	tVertex := pu.eng.overlayGraph.GetVertex(targetOverlayId)

	sVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	pq.Insert(sourceOverlayId, 0, sVertexInfo, sourceOverlayId)

	s := sVertex.GetOriginalVertex()
	t := tVertex.GetOriginalVertex()
	activeLandmarks := pu.eng.lm.SelectBestQueryLandmarks(s, t)

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
			originalVId := vOverlayVertex.GetOriginalVertex()
			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			pfv := pu.eng.lm.FindTighestLowerBound(originalVId, t, activeLandmarks)

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			priority := newTravelTime + pfv

			vAlreadyLabelled := util.Lt(pq.GetPriority(vOverlayId), util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(vOverlayId))) {
				// relax shortcut edge

				pq.Scan(vOverlayId) // langsung scan exit overlay vertex v

				vNewPar := da.NewVertexEdgePair(uOverlayId,
					da.INVALID_EDGE_ID, true)
				if vOverlayId == targetOverlayId {

					// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
					// if v is the target overlay vertex, insert/decrease its key  pq
					if !vAlreadyLabelled {
						vVertexInfo := da.NewVertexInfo(newTravelTime, vNewPar)
						pq.Insert(vOverlayId, priority, vVertexInfo, uOverlayId)
					} else {

						pq.DecreaseKey(vOverlayId, priority, newTravelTime, vNewPar)
					}

				} else {
					pq.Set(vOverlayId, da.NewVertexInfo(newTravelTime, vNewPar), vOverlayId)
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

				wOriginalId := wNeigborVertex.GetOriginalVertex()
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfw := pu.eng.lm.FindTighestLowerBound(wOriginalId, t, activeLandmarks)
				priority = newTravelTime + pfw
				wPar := da.NewVertexEdgePair(vOverlayId, da.INVALID_EDGE_ID, true)

				// relax edge
				wAlreadyLabelled := util.Lt(pq.GetPriority(wNeighborId), util.Infinity[W]())
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(wNeighborId))) {
					if !wAlreadyLabelled {
						wVertexInfo := da.NewVertexInfo(newTravelTime, wPar)
						pq.Insert(wNeighborId, priority, wVertexInfo, wNeighborId)
					} else {
						pq.DecreaseKey(wNeighborId, priority, newTravelTime, wPar)
					}
				}

			}

		})

	}

	overlayPath := make([]da.Index, 0, 64)
	overlayPath = append(overlayPath, targetOverlayId)

	curOverlayId := pq.Get(targetOverlayId).GetParent().GetVertex()
	for curOverlayId != da.INVALID_VERTEX_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pq.Get(curOverlayId).GetParent().GetVertex()
	}

	util.ReverseG(overlayPath)

	pu.eng.puCache.Set(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath)

	pq.Clear()
	pu.eng.pufOverlayHeapPool.Put(pq)

	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		vertexPath = pu.unpackInLevelCell(curV, nextV, level-1, vertexPath)
	}

	return vertexPath
}

func (pu *PathUnpackerALTNoTurnCost[W]) unpackInLowestLevelCell(
	sourceOverlayId, targetOverlayId da.Index, vertexPath []da.Index) []da.Index {

	if edgeIds, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
		// fetch from cache
		return append(vertexPath, edgeIds...)
	}

	// sourceEntryId inEdge that point to source vertex
	pq := pu.eng.pufBaseNoTurnCostHeapPool.Get().(*da.QueryHeap[da.Index, W])

	sOverlayVertex := pu.eng.overlayGraph.GetVertex(sourceOverlayId)
	s := sOverlayVertex.GetOriginalVertex()

	tOverlayVertex := pu.eng.overlayGraph.GetVertex(targetOverlayId)
	t := tOverlayVertex.GetOriginalVertex()

	sInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	sourceCellNumber := pu.eng.graph.GetCellNumber(s)

	pq.Insert(s, 0, sInfo, s)

	activeLandmarks := pu.eng.lm.SelectBestQueryLandmarks(s, t)

	for pq.Size() > 0 {

		queryKey := pq.ExtractMin()

		uId := queryKey.GetItem()

		pq.Scan(uId)
		uTravelTime := pq.GetPriority(uId)

		if uId == t {
			break
		}

		// relax all out edges of u
		pu.eng.graph.ForOutEdgesOfNoTurnCost(uId, func(eId, head, _ da.Index) {

			vId := head
			edgeWeight := pu.eng.getWeight(eId, true)

			newTravelTime := uTravelTime + edgeWeight
			if pu.eng.graph.GetCellNumber(vId) != sourceCellNumber && vId != t {
				// do not cross cell boundary
				return
			}

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			// relax edge
			vAlreadyLabelled := util.Lt(pq.GetPriority(vId), util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(vId))) {
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfv := pu.eng.lm.FindTighestLowerBound(vId, t, activeLandmarks)
				priority := newTravelTime + pfv

				if !vAlreadyLabelled {
					vInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))

					pq.Insert(vId, priority, vInfo, vId)
				} else {
					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)
					pq.DecreaseKey(vId, priority, newTravelTime, newPar)
				}
			}
		})
	}

	startLen := len(vertexPath)

	uId := t
	vertexPath = append(vertexPath, t)
	for pq.Get(uId).GetParent().GetVertex() != da.INVALID_VERTEX_ID {
		prevVertex := pq.Get(uId).GetParent().GetVertex()
		vertexPath = append(vertexPath, prevVertex)
		uId = prevVertex
	}

	subPath := vertexPath[startLen:]
	util.ReverseG(subPath)

	subPathCop := make([]da.Index, len(subPath))
	copy(subPathCop, subPath) // harus di copy, karena bisa aja keubah di remove removeConsecutiveDuplicates
	pu.eng.puCache.Set(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), subPathCop)

	pq.Clear()
	pu.eng.pufBaseNoTurnCostHeapPool.Put(pq)

	return vertexPath
}

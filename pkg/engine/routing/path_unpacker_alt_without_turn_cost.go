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

func (pu *PathUnpackerALTNoTurnCost[W]) unpackPath(packedPath []da.VertexEdgePair, sCellNum, tCellNum da.Pv) []da.Index {

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
			entryOvId := offBit(cur.GetVertex(), UNPACK_OVERLAY_OFFSET)

			entryVertex := pu.eng.overlayGraph.GetVertex(entryOvId)
			entryCellNum := entryVertex.GetCellNumber()

			queryLevel := pu.eng.overlayGraph.GetQueryLevel(sCellNum, tCellNum, entryCellNum)

			exitOvId := offBit(packedPath[i+1].GetVertex(), UNPACK_OVERLAY_OFFSET)

			unpackedVertexPath = pu.unpackInLevelCell(entryOvId, exitOvId, queryLevel, unpackedVertexPath)
			i += 2
		}
	}

	unpackedVertexPath = removeConsecutiveDuplicates(unpackedVertexPath)

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedVertexPath
}

func (pu *PathUnpackerALTNoTurnCost[W]) unpackInLevelCell(sOvId da.Index,
	tOvId da.Index,
	level uint8,
	verticesPath []da.Index,
) []da.Index {

	if level == 1 {

		verticesPath = pu.unpackInLowestLevelCell(
			sOvId, tOvId, verticesPath)
		return verticesPath
	}

	if overlayPath, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sOvId, tOvId, level)); ok {
		for i := 0; i < len(overlayPath); i += 2 {
			verticesPath = pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, verticesPath)
		}
		return verticesPath
	}

	sVertex := pu.eng.overlayGraph.GetVertex(sOvId)
	sCellNum := sVertex.GetCellNumber()

	pq := pu.eng.pufOverlayHeapPool.Get().(*da.QueryHeap[da.Index, W])

	truncSourceCellNum := pu.eng.overlayGraph.GetLevelData().TruncateToLevel(sCellNum, level)

	tVertex := pu.eng.overlayGraph.GetVertex(tOvId)

	sVertexData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	pq.Insert(sOvId, 0, sVertexData, sOvId)

	s := sVertex.GetOrigVId()
	t := tVertex.GetOrigVId()
	activeLandmarks := pu.eng.lm.SelectBestQueryLandmarks(s, t)

	for pq.Size() > 0 {

		u := pq.ExtractMin()

		uOvId := u.GetItem()
		pq.Explore(uOvId)

		if uOvId == tOvId {
			break
		}

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.eng.overlayGraph.ForOutNeighborsOf(uOvId, int(level-1), func(vOvId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.eng.metrics.GetShortcutWeight(wOffset)
			vOv := pu.eng.overlayGraph.GetVertex(vOvId)

			newVCost := pq.GetCost(uOvId) + shortcutOutEdgeWeight
			originalVId := vOv.GetOrigVId()
			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			pfv := pu.eng.lm.FindTighestLowerBound(originalVId, t, activeLandmarks)

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			priority := newVCost + pfv

			vLabelled := util.Lt(pq.GetCost(vOvId), util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, pq.GetCost(vOvId))) {
				// relax shortcut edge

				pq.Explore(vOvId) // langsung scan exit overlay vertex v

				vNewPar := da.NewVertexEdgePair(uOvId,
					da.INVALID_EDGE_ID, true)
				if vOvId == tOvId {

					// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
					// if v is the target overlay vertex, insert/decrease its key  pq
					if !vLabelled {
						vVertexData := da.NewVertexData(newVCost, vNewPar)
						pq.Insert(vOvId, priority, vVertexData, uOvId)
					} else {

						pq.DecreaseKey(vOvId, priority, newVCost, vNewPar)
					}

				} else {
					pq.Set(vOvId, da.NewVertexData(newVCost, vNewPar), vOvId)
				}

				// visit next cell neighbor
				wNeighborId := vOv.GetNeighborOverlayVertex()
				wNeigborVertex := pu.eng.overlayGraph.GetVertex(wNeighborId)

				wCellNum := wNeigborVertex.GetCellNumber()
				truncatedWCellNum := pu.eng.overlayGraph.GetLevelData().TruncateToLevel(wCellNum, uint8(level))
				if truncatedWCellNum != truncSourceCellNum {
					// if w is not in the same cell as sOvId in level l, dont visit w
					return
				}

				// get out edge that point to wEntryVertex from vOvId
				newVCost += pu.eng.getWeight(vOv.GetCutEdge(), true)

				wOriginalId := wNeigborVertex.GetOrigVId()
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfw := pu.eng.lm.FindTighestLowerBound(wOriginalId, t, activeLandmarks)
				priority = newVCost + pfw
				wPar := da.NewVertexEdgePair(vOvId, da.INVALID_EDGE_ID, true)

				// relax edge
				wLabelled := util.Lt(pq.GetCost(wNeighborId), util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, pq.GetCost(wNeighborId))) {
					if !wLabelled {
						wVertexData := da.NewVertexData(newVCost, wPar)
						pq.Insert(wNeighborId, priority, wVertexData, wNeighborId)
					} else {
						pq.DecreaseKey(wNeighborId, priority, newVCost, wPar)
					}
				}
			}
		})
	}

	overlayPath := make([]da.Index, 0, 64)
	overlayPath = append(overlayPath, tOvId)

	curOvId := pq.Get(tOvId).GetParent().GetVertex()
	for curOvId != da.INVALID_VERTEX_ID {
		overlayPath = append(overlayPath, curOvId)
		curOvId = pq.Get(curOvId).GetParent().GetVertex()
	}

	util.ReverseG(overlayPath)

	pu.eng.puCache.Set(da.NewPUCacheKey(sOvId, tOvId, level), overlayPath)

	pq.Clear()
	pu.eng.pufOverlayHeapPool.Put(pq)

	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		verticesPath = pu.unpackInLevelCell(curV, nextV, level-1, verticesPath)
	}

	return verticesPath
}

func (pu *PathUnpackerALTNoTurnCost[W]) unpackInLowestLevelCell(
	sOvId, tOvId da.Index, verticesPath []da.Index) []da.Index {

	if edgeIds, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sOvId, tOvId, 1)); ok {
		// fetch from cache
		return append(verticesPath, edgeIds...)
	}

	pq := pu.eng.pufBaseNoTurnCostHeapPool.Get().(*da.QueryHeap[da.Index, W])

	sOv := pu.eng.overlayGraph.GetVertex(sOvId)
	s := sOv.GetOrigVId()

	tOv := pu.eng.overlayGraph.GetVertex(tOvId)
	t := tOv.GetOrigVId()

	sData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	sCellNum := pu.eng.graph.GetCellNumber(s)

	pq.Insert(s, 0, sData, s)

	activeLandmarks := pu.eng.lm.SelectBestQueryLandmarks(s, t)

	for pq.Size() > 0 {

		queryKey := pq.ExtractMin()

		uId := queryKey.GetItem()

		pq.Explore(uId)
		uCost := pq.GetCost(uId)

		if uId == t {
			break
		}

		// relax all out edges of u
		pu.eng.graph.ForOutEdgesOfNoTurnCost(uId, func(eId, head, _ da.Index) {

			vId := head
			edgeWeight := pu.eng.getWeight(eId, true)

			newVCost := uCost + edgeWeight
			if pu.eng.graph.GetCellNumber(vId) != sCellNum && vId != t {
				// do not cross cell boundary
				return
			}

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			// relax edge
			vLabelled := util.Lt(pq.GetCost(vId), util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, pq.GetCost(vId))) {
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfv := pu.eng.lm.FindTighestLowerBound(vId, t, activeLandmarks)
				priority := newVCost + pfv

				if !vLabelled {
					vData := da.NewVertexData(newVCost, da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))

					pq.Insert(vId, priority, vData, vId)
				} else {
					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)
					pq.DecreaseKey(vId, priority, newVCost, newPar)
				}
			}
		})
	}

	startLen := len(verticesPath)

	uId := t
	verticesPath = append(verticesPath, t)
	for pq.Get(uId).GetParent().GetVertex() != da.INVALID_VERTEX_ID {
		prevVertex := pq.Get(uId).GetParent().GetVertex()
		verticesPath = append(verticesPath, prevVertex)
		uId = prevVertex
	}

	subPath := verticesPath[startLen:]
	util.ReverseG(subPath)

	subPathCop := make([]da.Index, len(subPath))
	copy(subPathCop, subPath) // harus di copy, karena bisa aja keubah di remove removeConsecutiveDuplicates
	pu.eng.puCache.Set(da.NewPUCacheKey(sOvId, tOvId, 1), subPathCop)

	pq.Clear()
	pu.eng.pufBaseNoTurnCostHeapPool.Put(pq)

	return verticesPath
}

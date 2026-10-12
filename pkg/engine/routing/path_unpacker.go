package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PathUnpackerALT[W util.RoutingNumber] struct {
	eng        *CRPRoutingEngine[W]
	metrics    *met.Metric[W]
	lm         *landmark.Landmark[W]
	scSet      map[uint64]uint8
	buildSCSet bool
	runtime    int64
}

func newPathUnpackerALT[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
	metrics *met.Metric[W],
	lm *landmark.Landmark[W],
	buildSCSet bool,
) *PathUnpackerALT[W] {
	pu := &PathUnpackerALT[W]{eng: engine, metrics: metrics, lm: lm}

	pu.runtime = 0
	if buildSCSet {
		pu.scSet = make(map[uint64]uint8, 10)
		pu.buildSCSet = true
	}

	return pu
}

// adapted from: https://github.com/michaelwegner/CRP/blob/master/algorithm/PathUnpacker.cpp

/*
unpackPath unpack shortcut edges in packed path returned from CRP query phase.

ref1:  Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable
Route Planning in Road Networks. Transportation Science, No. 2, Volume 51,
pages 566-591. Available at: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

the explanation below taken from ref1:
"To obtain the complete path description as a sequence of arcs (or vertices) in the original graph, each shortcut in
the result must be translated (unpacked ) into the corresponding subpath." (Delling et al. (2015))

"we recursively unpack a level-i shortcut (v, w) by running bidirectional Dijkstra between v and w on level i − 1, restricted
to subcells of the level-i cell containing the shortcut."  (Delling et al. (2015))
*/
func (pu *PathUnpackerALT[W]) unpackPath(packedPath []da.ParentVertex, sCellNum, tCellNum da.Pv) []da.Index {

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
			exitVertex := pu.eng.overlayGraph.GetVertex(exitOvId)
			if pu.buildSCSet {
				envId := uint32(entryVertex.GetOrigVId())
				exvId := uint32(exitVertex.GetOrigVId())
				pu.scSet[util.Bitpack(uint32(envId), uint32(exvId))] = queryLevel
			}
			unpackedVertexPath = pu.unpackInLevelCell(entryOvId, exitOvId, queryLevel, unpackedVertexPath)
			i += 2
		}
	}

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedVertexPath
}

func (pu *PathUnpackerALT[W]) unpackInLevelCell(sOvId da.Index,
	tOv da.Index,
	level uint8,
	verticesPath []da.Index,
) []da.Index {

	if level == 1 {

		verticesPath = pu.unpackInLowestLevelCell(
			sOvId, tOv, verticesPath)
		return verticesPath
	}

	if overlayPath, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sOvId, tOv, level)); ok {
		for i := 0; i < len(overlayPath); i += 2 {
			verticesPath = pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, verticesPath)
		}
		return verticesPath
	}

	sVertex := pu.eng.overlayGraph.GetVertex(sOvId)
	sCellNum := sVertex.GetCellNumber()

	pq := pu.eng.puovHeapPool.Get().(*da.QueryHeap[da.Index, W])

	truncSourceCellNum := pu.eng.overlayGraph.TruncateToLevel(sCellNum, level)

	tVertex := pu.eng.overlayGraph.GetVertex(tOv)

	sVertexData := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID))
	pq.Insert(sOvId, 0, sVertexData, sOvId)

	s := sVertex.GetOrigVId()
	t := tVertex.GetOrigVId()
	activeLandmarks := pu.lm.SelectBestQueryLandmarks(s, t)

	for pq.Size() > 0 {
		uKey := pq.ExtractMin()
		uOv := uKey.GetItem()
		pq.Explore(uOv)

		if uOv == tOv {
			break
		}

		// traverse all out neighbor of u in level l-1 in the same cell as u
		ovOffset, wOffset, eNum, numExit := pu.eng.overlayGraph.NumOfShortcuts(uOv, int(level-1), true)

		for i := da.Index(0); i < eNum; i++ {
			vOv, ewOffset := pu.eng.GetOV(ovOffset, wOffset, numExit, i, true)
			shortcutWeight := pu.eng.metrics.GetShortcutWeight(ewOffset)
			vVertex := pu.eng.overlayGraph.GetVertex(vOv)

			newVCost := pq.GetCost(uOv) + shortcutWeight
			oldVCost := pq.GetCost(vOv)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				// relax shortcut edge
				v := vVertex.GetOrigVId()
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				hv := pu.lm.ComputeTighestLowerBound(v, t, activeLandmarks)
				priority := newVCost + hv
				vNewPar := da.NewParentVertex(uOv)
				if vOv == tOv {

					// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
					// if v is the target overlay vertex, insert/decrease its key  pq
					if !vLabelled {
						vVertexData := da.NewVData(newVCost, vNewPar)
						pq.Insert(vOv, priority, vVertexData, uOv)
					} else {

						pq.DecreaseKey(vOv, priority, newVCost, vNewPar)
					}

				} else {
					pq.Set(vOv, da.NewVData(newVCost, vNewPar), vOv)
				}

				// visit next cell neighbor
				wOv := vVertex.GetNeighborOverlayVertex()
				wVertex := pu.eng.overlayGraph.GetVertex(wOv)

				wCellNum := wVertex.GetCellNumber()
				truncatedWCellNum := pu.eng.overlayGraph.TruncateToLevel(wCellNum, uint8(level))
				if truncatedWCellNum != truncSourceCellNum {
					// if w is not in the same cell as sOvId in level l, dont visit w
					continue
				}

				// get out edge that point to wEntryVertex from v
				newVCost += pu.metrics.GetWeight(vVertex.GetCutEdge())
				wPar := da.NewParentVertex(vOv)
				// relax edge
				wLabelled := util.Lt(pq.GetCost(wOv), util.Infinity[W]())
				if util.Lt(newVCost, pq.GetCost(wOv)) {
					wOriginalId := wVertex.GetOrigVId()
					// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
					hw := pu.lm.ComputeTighestLowerBound(wOriginalId, t, activeLandmarks)
					priority = newVCost + hw

					if !wLabelled {
						wVertexData := da.NewVData(newVCost, wPar)
						pq.Insert(wOv, priority, wVertexData, wOv)
					} else {
						pq.DecreaseKey(wOv, priority, newVCost, wPar)
					}
				}
			}
		}
	}

	overlayPath := make([]da.Index, 0, 64)
	overlayPath = append(overlayPath, tOv)

	curOvId := pq.Get(tOv).GetParent().GetVertex()
	for curOvId != da.INVALID_VERTEX_ID {
		overlayPath = append(overlayPath, curOvId)
		curOvId = pq.Get(curOvId).GetParent().GetVertex()
	}

	util.ReverseG(overlayPath)

	pu.eng.puCache.Set(da.NewPUCacheKey(sOvId, tOv, level), overlayPath)

	pq.Clear()
	pu.eng.puovHeapPool.Put(pq)

	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		verticesPath = pu.unpackInLevelCell(curV, nextV, level-1, verticesPath)
	}

	return verticesPath
}

func (pu *PathUnpackerALT[W]) unpackInLowestLevelCell(
	sOvId, tOvId da.Index, verticesPath []da.Index) []da.Index {

	if edgeIds, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sOvId, tOvId, 1)); ok {
		// fetch from cache
		return append(verticesPath, edgeIds...)
	}

	pq := pu.eng.puHeapPool.Get().(*da.QueryHeap[da.Index, W])

	sOv := pu.eng.overlayGraph.GetVertex(sOvId)
	s := sOv.GetOrigVId()

	tOv := pu.eng.overlayGraph.GetVertex(tOvId)
	t := tOv.GetOrigVId()

	sData := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID))
	sCellNum := pu.eng.graph.GetCellNumber(s)

	offset, _ := pu.eng.graph.GetVertexCellOffset(s)
	pq.InitLevelOneCellQuery(offset)

	pq.Insert(s, 0, sData, s)

	activeLandmarks := pu.lm.SelectBestQueryLandmarks(s, t)

	for pq.Size() > 0 {
		queryKey := pq.ExtractMin()
		u := queryKey.GetItem()
		pq.Explore(u)
		uCost := pq.GetCost(u)

		if u == t {
			break
		}

		// relax all out edges of u
		firstOut, eNum := pu.eng.graph.NumOutEdges(u)
		for i := da.Index(0); i < eNum; i++ {
			eId := firstOut + i
			v := pu.eng.graph.GetHead(eId)
			edgeWeight := pu.metrics.GetWeight(eId)

			newVCost := uCost + edgeWeight
			if pu.eng.graph.GetCellNumber(v) != sCellNum && v != t {
				// do not cross cell boundary
				continue
			}

			// relax edge
			oldVCost := pq.GetCost(v)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfv := pu.lm.ComputeTighestLowerBound(v, t, activeLandmarks)
				priority := newVCost + pfv

				if !vLabelled {
					vData := da.NewVData(newVCost, da.NewParentVertex(u))
					pq.Insert(v, priority, vData, v)
				} else {
					newPar := da.NewParentVertex(u)
					pq.DecreaseKey(v, priority, newVCost, newPar)
				}
			}
		}
	}

	startLen := len(verticesPath)

	u := t
	verticesPath = append(verticesPath, t)
	for pq.Get(u).GetParent().GetVertex() != da.INVALID_VERTEX_ID {
		prevVertex := pq.Get(u).GetParent().GetVertex()
		verticesPath = append(verticesPath, prevVertex)
		u = prevVertex
	}

	subPath := verticesPath[startLen:]
	util.ReverseG(subPath)

	subPathCop := make([]da.Index, len(subPath))
	copy(subPathCop, subPath) // harus di copy, karena bisa aja keubah di remove removeConsecutiveDuplicates
	pu.eng.puCache.Set(da.NewPUCacheKey(sOvId, tOvId, 1), subPathCop)

	pq.Clear()
	pu.eng.puHeapPool.Put(pq)

	return verticesPath
}

func (pu *PathUnpackerALT[W]) GetStats() int64 {
	return pu.runtime
}

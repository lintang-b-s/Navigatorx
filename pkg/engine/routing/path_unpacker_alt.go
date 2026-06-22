package routing

import (
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PathUnpackerALT[W util.RoutingNumber] struct {
	eng *CRPRoutingEngine[W]

	runtime int64
}

func NewPathUnpackerALT[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *PathUnpackerALT[W] {
	pu := engine.pathUnpackerPool.Get().(*PathUnpackerALT[W])
	return pu
}

func newPathUnpackerALTAlloc[W util.RoutingNumber](engine *CRPRoutingEngine[W]) *PathUnpackerALT[W] {
	pu := &PathUnpackerALT[W]{}
	pu.eng = engine
	pu.runtime = 0
	return pu
}

func (pu *PathUnpackerALT[W]) Reset() {
	pu.runtime = 0
}

// DonePooled returns a PathUnpackerALT instance back to the engine pool so
// the next call can reuse the already-paid-for allocation.
func (pu *PathUnpackerALT[W]) DonePooled() {
	pu.eng.pathUnpackerPool.Put(pu)
}

/*
[1] Path Unpacking phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
[2] Haeupler, B. et al. (2025) “Bidirectional Dijkstra's Algorithm is Instance-Optimal,” in 2025 Symposium on Simplicity in Algorithms (SOSA). Society for Industrial and Applied Mathematics (Proceedings), pp. 202–215. Available at: https://doi.org/10.1137/1.9781611978315.16.
[3] I. Pohl. Bi-directional Search. In Machine Intelligence, volume 6, pages 124–140. Edinburgh Univ. Press, Edinburgh, 1971.
[4] Goldberg, A.V. and Harrelson,  (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
[5] bidirectional A*: Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.
[6] Cormen, T.H. et al. (2009) Introduction to Algorithms. 3th ed. Cambridge, MA, USA: MIT Press
[7] https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf

unpackPath. unpack a level-i shortcut (v, w) by running  ALT (A* algorithm, landmarks, and triangle inequality ) [4] between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
jika level i-1 >= 1, kita menggunakan shortcut edges (dari subcells dari level-i cell containing the shortcut) di overlay graph level i-1
jika level i-1 = 0, kita menggunakan base edges yang terletak pada level-i cell containing the shortcut

this path unpacking use  ALT (A* algorithm, landmarks, and triangle inequality ) in ref[4]
A*, landmarks, and triangle inequality (ALT) [4] adalah algoritma A* yang fungsi heuristik/potential nya memanfaatkan precomputed landmark shortest path distances (see ref[4] for the details)
fungsi heuristik/potential yang digunakan ALT memiliki sifat konsisten/feasible (lihat ref [4])
potential function adalah fungsi dari vertices ke bilangan real, fungsi potensial \pi_t(v) memberikan estimate sp distance dari v ke t
diberikan fungsi potensial \pi, kita mendefinisikan reduced cost dari sebuah edge dengan l_{\pi}(v,w)=l(v,w)-\pi(v)+\pi(w)
fungsi potensial \pi dikakan konsisten atau feasible jika l_{\pi} >= 0 untuk semua edges

dari Lemma 25.1 (Reweighting does not change shortest paths) pada ref 6:
misal p=(v0,v1,...,vk) adalah any path dari v0 ke vk. then p is a shortest path from v0 to vk with weight function l if and only if it is a shortest path with weight function l_{\pi}

time complexity:
let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (compact graph CRP graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
let q = number of shorcut edges in packedPath
let L = highest level of multilevel partition
time complexity of unpackPath: O(q * ( L *  (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p) ) )
*/
func (pu *PathUnpackerALT[W]) unpackPath(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Index, map[uint64]uint8) {

	unpackedEdgePath := make([]da.Index, 0, len(packedPath))

	now := time.Now()

	// todo: sebelumnya pakai worker pools di worker_pool.go, di benchmark ada additional 40000 B/op, bikin worker pool yang allocate less space ?
	// DONE: ternyata tanpa worker pool buat path unpacking lebih less space dan lebih bagus hasil load testnya

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

			queryLevel := pu.eng.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

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

// unpackPathEdgesOnly is the same as unpackPath but skips allocating the
// shortcutPathSet map.
func (pu *PathUnpackerALT[W]) unpackPathEdgesOnly(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv) []da.Index {
	unpackedEdgePath := make([]da.Index, 0, len(packedPath))

	now := time.Now()
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

			queryLevel := pu.eng.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			exitOverlayId := offBit(packedPath[i+1].GetEdge(), UNPACK_OVERLAY_OFFSET)

			unpackedEdgePath = pu.unpackInLevelCell(entryOverlayId, exitOverlayId, queryLevel, unpackedEdgePath)
			i += 2
		}
	}

	unpackedEdgePath = removeConsecutiveDuplicates(unpackedEdgePath)

	pu.runtime = time.Since(now).Milliseconds()
	return unpackedEdgePath
}

func (pu *PathUnpackerALT[W]) unpackInLevelCell(sourceOverlayId da.Index,
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
				uOverlayVertex := pu.eng.overlayGraph.GetVertex(uOverlayId)
				originalUId := uOverlayVertex.GetOriginalVertex()

				if vOverlayId == targetOverlayId {
					// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
					// if v is the target overlay vertex, insert/decrease its key  pq
					if !vAlreadyLabelled {
						wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(originalUId,
							uOverlayId, true))
						pq.Insert(vOverlayId, priority, wVertexInfo, uOverlayId)
					} else {
						wNewPar := da.NewVertexEdgePair(originalUId,
							uOverlayId, true)
						pq.DecreaseKey(vOverlayId, priority, newTravelTime, wNewPar)
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

				wOriginalId := wNeigborVertex.GetOriginalVertex()
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfw := pu.eng.lm.FindTighestLowerBound(wOriginalId, t, activeLandmarks)
				priority = newTravelTime + pfw

				// relax edge
				wAlreadyLabelled := util.Lt(pq.GetPriority(wNeighborId), util.Infinity[W]())
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(wNeighborId))) {
					if !wAlreadyLabelled {
						wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
							vOverlayId, true))
						pq.Insert(wNeighborId, priority, wVertexInfo, wNeighborId)
					} else {
						wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
							vOverlayId, true)
						pq.DecreaseKey(wNeighborId, priority, newTravelTime, wNewPar)
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

func (pu *PathUnpackerALT[W]) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	sourceOverlayId, targetOverlayId da.Index, edgePath []da.Index) []da.Index {

	if edgeIds, ok := pu.eng.puCache.GetIfPresent(da.NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
		// fetch from cache
		return append(edgePath, edgeIds...)
	}

	// sourceEntryId inEdge that point to source vertex
	pq := pu.eng.pufBaseHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey, W])

	// sourceEntryId: id buat inEdge u->s
	// targetEntryId: id buat inEdge t->v

	// get source vertex
	sourceVertex := pu.eng.graph.GetVertex(pu.eng.graph.GetHeadFromInEdge(sourceEntryId))
	tInEdge := pu.eng.graph.GetInEdge(targetEntryId)
	t := tInEdge.GetTail()
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

	activeLandmarks := pu.eng.lm.SelectBestQueryLandmarks(s, t)
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
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfv := pu.eng.lm.FindTighestLowerBound(vId, t, activeLandmarks)
				priority := newTravelTime + pfv

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, eId)
					vInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))

					pq.Insert(offVEntryId, priority, vInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false)
					pq.DecreaseKey(offVEntryId, priority, newTravelTime, newPar)
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

func (pu *PathUnpackerALT[W]) GetStats() int64 {
	return pu.runtime
}

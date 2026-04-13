package routing

import (
	"time"

	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PathUnpackerALT struct {
	engine *CRPRoutingEngine

	metrics *metrics.Metric

	puCache *ristretto.Cache[[]byte, []da.Index]
	lm      *landmark.Landmark

	useCache             bool
	oneToMany            bool
	runtime              int64
	forAlternativeRoutes bool
}

func NewPathUnpackerALT(engine *CRPRoutingEngine, metrics *metrics.Metric,
	puCache *ristretto.Cache[[]byte, []da.Index], useCache bool, lm *landmark.Landmark,
) *PathUnpackerALT {
	return &PathUnpackerALT{
		engine:  engine,
		metrics: metrics,

		puCache:  puCache,
		useCache: useCache,
		runtime:  0,
		lm:       lm,
	}
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
let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (turn-based graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
let q = number of shorcut edges in packedPath
let L = highest level of multilevel partition
time complexity of unpackPath: O(q * ( L *  (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p) ) )
*/
func (pu *PathUnpackerALT) unpackPath(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Index, map[uint64]uint8) {

	unpackedEdgePath := make([]da.Index, 0, len(packedPath))

	now := time.Now()

	// todo: sebelumnya pakai worker pools di worker_pool.go, di benchmark ada additional 40000 B/op, bikin worker pool yang allocate less space ?
	// mungkin tanpa pake wp di worker_pool.go

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

			entryVertex := pu.engine.overlayGraph.GetVertex(entryOverlayId)
			entryCellNumber := entryVertex.GetCellNumber()
			var queryLevel uint8

			if !pu.oneToMany {
				queryLevel = pu.engine.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			} else {
				queryLevel = cur.GetQueryLevel()
			}

			exitOverlayId := offBit(packedPath[i+1].GetEdge(), UNPACK_OVERLAY_OFFSET)
			exitVertex := pu.engine.overlayGraph.GetVertex(exitOverlayId)

			enOriVId := entryVertex.GetOriginalVertex()
			exOriVId := exitVertex.GetOriginalVertex()
			shortcutPathSet[util.Bitpack(uint32(enOriVId), uint32(exOriVId))] = queryLevel

			shortcutEdgeIdsPath := pu.unpackInLevelCell(entryOverlayId, exitOverlayId, queryLevel)
			unpackedEdgePath = append(unpackedEdgePath, shortcutEdgeIdsPath...) // golang slice append copy shortcutEdgeIdsPath ke unpackedEdgePath :https://go.dev/doc/effective_go#slices
			i += 2
		}
	}

	unpackedEdgePath = removeDuplicates(unpackedEdgePath)

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedEdgePath, shortcutPathSet
}

func (pu *PathUnpackerALT) unpackInLevelCell(sourceOverlayId da.Index,
	targetOverlayId da.Index,
	level uint8,
) []da.Index {

	if level == 1 {
		sourceOverlayVertex := pu.engine.overlayGraph.GetVertex(sourceOverlayId)
		sourceEntryId := sourceOverlayVertex.GetOriginalEdge()
		targetOverlayVertex := pu.engine.overlayGraph.GetVertex(targetOverlayId)
		neighborOfTarget := targetOverlayVertex.GetNeighborOverlayVertex()
		neighborOverlayVertex := pu.engine.overlayGraph.GetVertex(neighborOfTarget)
		targetEntryId := neighborOverlayVertex.GetOriginalEdge()

		edgePath := pu.unpackInLowestLevelCell(sourceEntryId, targetEntryId,
			sourceOverlayId, targetOverlayId)
		return edgePath
	}

	if pu.useCache {
		if overlayPath, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {

			edgePath := make([]da.Index, 0, UNPACKER_EDGE_PATH_SIZE)
			for i := 0; i < len(overlayPath); i += 2 {
				downLevelEdgePath := pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1)
				edgePath = append(edgePath, downLevelEdgePath...)
			}

			return edgePath
		}
	}

	sVertex := pu.engine.overlayGraph.GetVertex(sourceOverlayId)
	sourceCellNumber := sVertex.GetCellNumber()

	pq := pu.engine.pufOverlayHeapPool.Get().(*da.QueryHeap[da.Index])

	done := func() {
		pq.Clear()
		pu.engine.pufOverlayHeapPool.Put(pq)

	}

	truncatedSourceCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	tVertex := pu.engine.overlayGraph.GetVertex(targetOverlayId)
	targetCellNumber := tVertex.GetCellNumber()
	truncatedTargetCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(targetCellNumber, level)

	util.AssertPanic(truncatedSourceCellNumber == truncatedTargetCellNumber, "cell number/id dari sourceOverlay vertex dan targetOverlay vertex haruslah sama")

	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	pq.Insert(sourceOverlayId, 0, sVertexInfo, sourceOverlayId)

	s := sVertex.GetOriginalVertex()
	t := tVertex.GetOriginalVertex()
	activeLandmarks := pu.lm.SelectBestQueryLandmarks(s, t)

	labelled := func(pq *da.QueryHeap[da.Index], v da.Index) bool {

		ok := util.Lt(pq.GetPriority(v), pkg.INF_WEIGHT)
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
		pu.engine.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)
			vOverlayVertex := pu.engine.overlayGraph.GetVertex(vOverlayId)

			newTravelTime := pq.GetPriority(uOverlayId) + shortcutOutEdgeWeight
			originalVId := vOverlayVertex.GetOriginalVertex()
			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			pfv, _ := pu.lm.FindTighestConsistentLowerBound(originalVId, s, t, activeLandmarks)

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			priority := newTravelTime + pfv

			vAlreadyLabelled := labelled(pq, vOverlayId)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(vOverlayId))) {
				// relax shortcut edge

				pq.Scan(vOverlayId) // langsung scan exit overlay vertex v
				uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)
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
				wNeigborVertex := pu.engine.overlayGraph.GetVertex(wNeighborId)

				wCellNumber := wNeigborVertex.GetCellNumber()
				truncatedWCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
				if truncatedWCellNumber != truncatedSourceCellNumber {
					// if w is not in the same cell as sourceOverlayId in level l, dont visit w
					return
				}

				// get out edge that point to wEntryVertex from vOverlayId
				newTravelTime += pu.engine.GetWeight(vOverlayVertex.GetOriginalEdge(), true)

				wOriginalId := wNeigborVertex.GetOriginalVertex()
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfw, _ := pu.lm.FindTighestConsistentLowerBound(wOriginalId, s, t, activeLandmarks)
				priority = newTravelTime + pfw

				// relax edge
				wAlreadyLabelled := labelled(pq, wNeighborId)
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

	overlayPath := make([]da.Index, 0, UNPACKER_OVERLAY_PATH_SIZE)

	overlayPath = append(overlayPath, targetOverlayId)

	curOverlayId := pq.Get(targetOverlayId).GetParent().GetEdge()
	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pq.Get(curOverlayId).GetParent().GetEdge()
	}

	util.ReverseG(overlayPath)

	util.AssertPanic(len(overlayPath)%2 == 0, "harusnya len(overlayPath) genap")

	if pu.useCache {
		pu.puCache.Set(NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath, 1)
	}

	done()

	edgePath := make([]da.Index, 0, UNPACKER_EDGE_PATH_SIZE)
	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		downLevelEdgePath := pu.unpackInLevelCell(curV, nextV, level-1)
		edgePath = append(edgePath, downLevelEdgePath...)
	}

	return edgePath
}

func (pu *PathUnpackerALT) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	sourceOverlayId, targetOverlayId da.Index) []da.Index {

	if pu.useCache {
		if edgeIds, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
			// fetch from cache

			return edgeIds
		}
	}

	// sourceEntryId inEdge that point to source vertex
	pq := pu.engine.pufBaseHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey])

	done := func() {
		pq.Clear()
		pu.engine.pufBaseHeapPool.Put(pq)
	}
	defer done()

	// sourceEntryId: id buat inEdge u->s
	// targetEntryId: id buat inEdge t->v

	// get source vertex
	sourceVertex := pu.engine.graph.GetVertex(pu.engine.graph.GetHeadFromInEdge(sourceEntryId))
	tInEdge := pu.engine.graph.GetInEdge(targetEntryId)
	t := tInEdge.GetTail()
	s := sourceVertex.GetID()

	// s and t in same cell in level 1 and both are overlay vertices
	// s is entry vertex, t is exit vertex of this cell

	_, sOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)

	sourceCellNumber := pu.engine.graph.GetCellNumber(s)

	offSourceEntryId := pu.engine.offsetForward(s, sourceEntryId, sourceCellNumber, sourceCellNumber)

	sQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(s, offSourceEntryId, sOutEdge)
	sInfo := da.NewVertexInfo(0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))

	pq.Insert(offSourceEntryId, 0, sInfo, sQueryKey)

	activeLandmarks := pu.lm.SelectBestQueryLandmarks(s, t)
	offTargetEntryId := da.INVALID_EDGE_ID

	for pq.Size() > 0 {

		queryKey := pq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryId := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutInEdgeId()

		pq.Scan(uEntryId)

		adjuEntryId := pu.engine.adjustForward(uId, uEntryId)

		if adjuEntryId == targetEntryId {
			offTargetEntryId = uEntryId
			break
		}

		// relax all out edges of u
		pu.engine.graph.ForOutEdgesOf(uId, pu.engine.graph.GetEntryOrder(uId, adjuEntryId), func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType,
			hwType pkg.OsmHighwayType) {

			vId := head

			vEntryId := pu.engine.graph.GetEntryOffset(vId) + entryPoint
			edgeWeight := pu.engine.GetWeight(eId, true)

			newTravelTime := pq.GetPriority(uEntryId) + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.engine.graph.GetCellNumber(vId) != sourceCellNumber && vEntryId != targetEntryId {
				// do not cross cell boundary
				return
			}

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVEntryId := pu.engine.offsetForward(vId, vEntryId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			// relax edge
			vAlreadyLabelled := util.Lt(pq.GetPriority(offVEntryId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, pq.GetPriority(offVEntryId))) {
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfv, _ := pu.lm.FindTighestConsistentLowerBound(vId, s, t, activeLandmarks)
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

	edgeIdPath := make([]da.Index, 0, UNPACKER_EDGE_PATH_SIZE)

	_, midOutEdgeId := pu.engine.graph.GetHeadOfInedgeWithOutEdge(targetEntryId)

	if util.Gt(pu.engine.GetWeight(midOutEdgeId, true), 0) {
		edgeIdPath = append(edgeIdPath, midOutEdgeId)
	}

	uId := offTargetEntryId
	for pq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := pq.Get(uId).GetParent().GetOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)

		pEId := pq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	util.ReverseG(edgeIdPath)

	if pu.useCache {
		// https://github.com/dgraph-io/ristretto is thread-safe
		pu.puCache.Set(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), edgeIdPath, 1)
	}

	return edgeIdPath
}

func (pu *PathUnpackerALT) GetStats() int64 {
	return pu.runtime
}

func (pu *PathUnpackerALT) setForAlternativeRoutes() {
	pu.forAlternativeRoutes = true
}

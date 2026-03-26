package routing

import (
	"sync"
	"time"

	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PathUnpackerALT struct {
	engine *CRPRoutingEngine

	metrics *metrics.Metric

	puCache *lru.Cache[PUCacheKey, []da.Index]
	lm      *landmark.Landmark

	useCache             bool
	oneToMany            bool
	runtime              int64
	lock                 sync.RWMutex
	forAlternativeRoutes bool
}

func NewPathUnpackerALT(engine *CRPRoutingEngine, metrics *metrics.Metric,
	puCache *lru.Cache[PUCacheKey, []da.Index], useCache bool, lm *landmark.Landmark,
) *PathUnpackerALT {
	return &PathUnpackerALT{
		engine:  engine,
		metrics: metrics,

		puCache:  puCache,
		useCache: useCache,
		runtime:  0,
		lock:     sync.RWMutex{},
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

unpackPath. unpack a level-i shortcut (v, w) by running Bidirectional ALT [4] between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
jika level i-1 >= 1, kita menggunakan shortcut edges (dari subcells dari level-i cell containing the shortcut) di overlay graph level i-1
jika level i-1 = 0, kita menggunakan base edges yang terletak pada level-i cell containing the shortcut

this path unpacking use bidirectional ALT in ref[4]
Bidirectional A*, landmarks, and triangle inequality (ALT) [4] adalah algoritma bidirectional A* yang fungsi heuristik/potential nya memanfaatkan precomputed landmark shortest path distances (see ref[4] for the details)
fungsi heuristik/potential yang digunakan bidirectional ALT memiliki sifat konsisten/feasible
potential function adalah fungsi dari vertices ke bilangan real, fungsi potensial \pi_t(v) memberikan estimate sp distance dari v ke t
diberikan fungsi potensial \pi, kita mendefinisikan reduced cost dari sebuah edge dengan l_{\pi}(v,w)=l(v,w)-\pi(v)+\pi(w)
fungsi potensial \pi dikakan konsisten atau feasible jika l_{\pi} >= 0 untuk semua edges

pada bidirectional A*,kita perlu adjust fungsi potensial agar tetap bersifat konsisten. misal \pi_t(v) adalah estimate sp distance dari v ke t dan \pi_s(v) estimate sp distance dari s ke v
[5] dan [4], kita menggunakan fungsi potensial p_t(v)=\frac{\pi_t(v)-\pi_s(v)}{2} untuk forward search dan p_s(v)=-p_t(v) untuk backward search
[5] dan [4] membuktikan bahwa bidirectional A* dengan fungsi potensial p_t dan p_s diatas ekuivalen dengan menjalankan algoritma bidirectional dijkstra dengan bobot edge l_p(v,w)=l(v,w)+p_t(w)-p_t(v)=l(v,w)-p_s(w)+p_s(v) >= 0
dari Lemma 25.1 (Reweighting does not change shortest paths) pada ref 6:
misal p=(v0,v1,...,vk) adalah any path dari v0 ke vk. then p is a shortest path from v0 to vk with weight function l if and only if it is a shortest path with weight function l_p

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

	unpackedEdgePathComp := make([][]da.Index, len(packedPath))
	now := time.Now()

	shortcutPathSet := make(map[uint64]uint8)

	workerSize := PATH_UNPACKER_WORKERS
	if pu.forAlternativeRoutes {
		workerSize = PATH_UNPACKER_ALTERNATIVE_WORKERS
	}

	workers := concurrent.NewWorkerPool[pathUnpackingParam, any](workerSize, len(packedPath))

	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.GetEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)

			unpackedEdgePathComp[i] = append(unpackedEdgePathComp[i], cur.GetEdge())

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
			shortcutPathSet[bitpack(enOriVId, exOriVId)] = queryLevel

			workers.AddJob(NewPathUnpackingParam(entryOverlayId, exitOverlayId, queryLevel, &unpackedEdgePathComp[i]))

			i += 2
		}
	}

	workers.Close()
	workers.Start(pu.unpackInLevelCell)
	workers.WaitDirect()

	size := 0

	for i := 0; i < len(unpackedEdgePathComp); i++ {
		size += len(unpackedEdgePathComp[i])
	}

	unpackedEdgePath := make([]da.Index, size)

	id := 0
	for i := 0; i < len(unpackedEdgePathComp); i++ {
		for j := 0; j < len(unpackedEdgePathComp[i]); j++ {
			unpackedEdgePath[id] = unpackedEdgePathComp[i][j]
			id++
		}
	}

	unpackedEdgePath = removeDuplicates(unpackedEdgePath)

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedEdgePath, shortcutPathSet
}

func (pu *PathUnpackerALT) unpackInLevelCell(param pathUnpackingParam,
) any {
	sourceOverlayId := param.getSourceOverlayId()
	targetOverlayId := param.getTargetOverlayId()
	level := param.getLevel()
	unpackedEdgePath := param.getUnpackedEdgePath()
	if level == 1 {
		sourceEntryId := pu.engine.overlayGraph.GetVertex(sourceOverlayId).GetOriginalEdge()
		neighborOfTarget := pu.engine.overlayGraph.GetVertex(targetOverlayId).GetNeighborOverlayVertex()
		targetEntryId := pu.engine.overlayGraph.GetVertex(neighborOfTarget).GetOriginalEdge()

		pu.unpackInLowestLevelCell(sourceEntryId, targetEntryId, unpackedEdgePath,
			sourceOverlayId, targetOverlayId)
		return nil
	}

	if pu.useCache {
		if overlayPath, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {
			// buat tests, gak ambil dari cache
			for i := 0; i < len(overlayPath); i += 2 {
				pu.unpackInLevelCell(NewPathUnpackingParam(overlayPath[i], overlayPath[i+1], level-1, unpackedEdgePath))
			}
			return nil
		}
	}

	sVertex := pu.engine.overlayGraph.GetVertex(sourceOverlayId)
	sourceCellNumber := sVertex.GetCellNumber()

	fOverlayPq := pu.engine.pufOverlayHeapPool.Get().(*da.QueryHeap[da.Index])
	bOverlayPq := pu.engine.pubOverlayHeapPool.Get().(*da.QueryHeap[da.Index])
	fOverlayPq.Clear()
	bOverlayPq.Clear()

	done := func() {
		pu.engine.pufOverlayHeapPool.Put(fOverlayPq)
		pu.engine.pubOverlayHeapPool.Put(bOverlayPq)
	}

	truncatedSourceCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	tVertex := pu.engine.overlayGraph.GetVertex(targetOverlayId)
	targetCellNumber := tVertex.GetCellNumber()
	truncatedTargetCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(targetCellNumber, level)

	util.AssertPanic(truncatedSourceCellNumber == truncatedTargetCellNumber, "cell number/id dari sourceOverlay vertex dan targetOverlay vertex haruslah sama")

	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	fOverlayPq.Insert(sourceOverlayId, 0, sVertexInfo, sourceOverlayId)

	tVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	bOverlayPq.Insert(targetOverlayId, 0, tVertexInfo, targetOverlayId)

	fastestTT := 2 * pkg.INF_WEIGHT

	s := sVertex.GetOriginalVertex()
	t := tVertex.GetOriginalVertex()
	activeLandmarks := pu.lm.SelectBestQueryLandmarks(s, t)

	labelled := func(pq *da.QueryHeap[da.Index], v da.Index) bool {

		ok := util.Lt(pq.GetPriority(v), pkg.INF_WEIGHT)
		return ok
	}

	mid := da.INVALID_VERTEX_ID

	for fOverlayPq.Size() > 0 && bOverlayPq.Size() > 0 {
		minForward := fOverlayPq.GetMinrank()
		minBackward := bOverlayPq.GetMinrank()
		if util.Ge(minForward+minBackward, fastestTT) {
			break
		}
		u := fOverlayPq.ExtractMin()

		uOverlayId := u.GetItem()
		fOverlayPq.Scan(uOverlayId)

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)

			newTravelTime := fOverlayPq.GetPriority(uOverlayId) + shortcutOutEdgeWeight
			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := labelled(fOverlayPq, vOverlayId)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, fOverlayPq.GetPriority(vOverlayId))) {
				uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)
				// if v is the target overlay vertex, update the pq
				fOverlayPq.Set(vOverlayId, da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
					uOverlayId, true)), vOverlayId)

				fOverlayPq.Scan(vOverlayId)

			}

			scannedByBackwardSearch := bOverlayPq.IsScanned(vOverlayId)
			if scannedByBackwardSearch && util.Lt(fOverlayPq.GetPriority(vOverlayId)+bOverlayPq.GetPriority(vOverlayId), fastestTT) {
				fastestTT = fOverlayPq.GetPriority(vOverlayId) + bOverlayPq.GetPriority(vOverlayId)
				mid = vOverlayId
			}

			// visit next cell neighbor
			wNeighborId := pu.engine.overlayGraph.GetVertex(vOverlayId).GetNeighborOverlayVertex()
			wNeigborVertex := pu.engine.overlayGraph.GetVertex(wNeighborId)

			wCellNumber := wNeigborVertex.GetCellNumber()
			truncatedWCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
			if truncatedWCellNumber != truncatedSourceCellNumber {
				// if w is not in the same cell as sourceOverlayId in level l, dont visit w
				return
			}

			// get out edge that point to wEntryVertex from vOverlayId
			vOverlayVertex := pu.engine.overlayGraph.GetVertex(vOverlayId)
			vOutEdge := pu.engine.graph.GetOutEdge(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vOutEdge)

			wOriginalId := wNeigborVertex.GetOriginalVertex()
			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			pfv, _ := pu.lm.FindTighestConsistentLowerBound(wOriginalId, s, t, activeLandmarks)
			priority := newTravelTime + pfv

			// relax edge
			wAlreadyLabelled := labelled(fOverlayPq, wNeighborId)
			if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, fOverlayPq.GetPriority(wNeighborId))) {
				if !wAlreadyLabelled {
					wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true))
					fOverlayPq.Insert(wNeighborId, priority, wVertexInfo, wNeighborId)
				} else {
					wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true)
					fOverlayPq.DecreaseKey(wNeighborId, priority, newTravelTime, wNewPar)
				}
			}

			scannedByBackwardSearch = bOverlayPq.IsScanned(wNeighborId)
			if scannedByBackwardSearch && util.Lt(fOverlayPq.GetPriority(wNeighborId)+bOverlayPq.GetPriority(wNeighborId), fastestTT) {
				fastestTT = fOverlayPq.GetPriority(wNeighborId) + bOverlayPq.GetPriority(wNeighborId)
				mid = wNeighborId
			}
		})

		u = bOverlayPq.ExtractMin()

		uOverlayId = u.GetItem()
		bOverlayPq.Scan(uOverlayId)

		// traverse all in neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForInNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutInEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)

			newTravelTime := bOverlayPq.GetPriority(uOverlayId) + shortcutInEdgeWeight
			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := labelled(bOverlayPq, vOverlayId)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, bOverlayPq.GetPriority(vOverlayId))) {
				uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)
				vVertex := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
					uOverlayId, true))
				bOverlayPq.Set(vOverlayId, vVertex, vOverlayId)

				bOverlayPq.Scan(vOverlayId)
			}

			scannedByForwardSearch := fOverlayPq.IsScanned(vOverlayId)
			if scannedByForwardSearch && util.Lt(fOverlayPq.GetPriority(vOverlayId)+bOverlayPq.GetPriority(vOverlayId), fastestTT) {
				fastestTT = fOverlayPq.GetPriority(vOverlayId) + bOverlayPq.GetPriority(vOverlayId)
				mid = vOverlayId
			}

			// visit next cell neighbor
			wNeighborId := pu.engine.overlayGraph.GetVertex(vOverlayId).GetNeighborOverlayVertex()
			wNeigborVertex := pu.engine.overlayGraph.GetVertex(wNeighborId)

			wCellNumber := wNeigborVertex.GetCellNumber()
			truncatedWCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
			if truncatedWCellNumber != truncatedSourceCellNumber {
				// if w is not in the same cell as sourceOverlayId in level l, dont visit w
				return
			}

			vOverlayVertex := pu.engine.overlayGraph.GetVertex(vOverlayId)
			vInEdge := pu.engine.graph.GetInEdge(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vInEdge)
			wOriginalid := wNeigborVertex.GetOriginalVertex()

			// relax edge
			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			_, prv := pu.lm.FindTighestConsistentLowerBound(wOriginalid, s, t, activeLandmarks)
			priority := newTravelTime + prv
			wAlreadyLabelled := labelled(bOverlayPq, wNeighborId)
			if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, bOverlayPq.GetPriority(wNeighborId))) {
				if !wAlreadyLabelled {
					wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true))
					bOverlayPq.Insert(wNeighborId, priority, wVertexInfo, wNeighborId)
				} else {
					wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true)
					bOverlayPq.DecreaseKey(wNeighborId, priority, newTravelTime, wNewPar)
				}
			}

			scannedByForwardSearch = fOverlayPq.IsScanned(wNeighborId)
			if scannedByForwardSearch && util.Lt(fOverlayPq.GetPriority(wNeighborId)+bOverlayPq.GetPriority(wNeighborId), fastestTT) {
				fastestTT = fOverlayPq.GetPriority(wNeighborId) + bOverlayPq.GetPriority(wNeighborId)
				mid = wNeighborId
			}
		})
	}

	overlayPath := make([]da.Index, 0, 8)

	overlayPath = append(overlayPath, mid)

	curOverlayId := fOverlayPq.Get(mid).GetParent().GetEdge()
	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = fOverlayPq.Get(curOverlayId).GetParent().GetEdge()
	}

	util.ReverseG(overlayPath)

	curOverlayId = bOverlayPq.Get(mid).GetParent().GetEdge()

	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = bOverlayPq.Get(curOverlayId).GetParent().GetEdge()
	}

	fOverlayPq.Clear()
	bOverlayPq.Clear()

	util.AssertPanic(len(overlayPath)%2 == 0, "harusnya len(overlayPath) genap")

	if pu.useCache {
		pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath)
	}

	done()
	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		pu.unpackInLevelCell(NewPathUnpackingParam(curV, nextV, level-1, unpackedEdgePath))
	}
	return nil
}

func (pu *PathUnpackerALT) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	unpackedEdgePath *[]da.Index,
	sourceOverlayId, targetOverlayId da.Index) {

	if pu.useCache {
		if edgeIds, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
			// fetch from cache
			for _, edgeId := range edgeIds {

				*unpackedEdgePath = append(*unpackedEdgePath, edgeId)
			}

			return
		}
	}

	// sourceEntryId inEdge that point to source vertex
	fpq := pu.engine.pufBaseHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	bpq := pu.engine.pubBaseHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	fpq.Clear()
	bpq.Clear()

	done := func() {
		pu.engine.pufBaseHeapPool.Put(fpq)
		pu.engine.pubBaseHeapPool.Put(bpq)
	}
	defer done()

	// sourceEntryId: id buat inEdge u->s
	// targetEntryId: id buat inEdge t->v

	// get source vertex
	sourceVertex := pu.engine.graph.GetVertex(pu.engine.graph.GetHeadFromInEdge(sourceEntryId))
	t := pu.engine.graph.GetInEdge(targetEntryId).GetTail()
	s := sourceVertex.GetID()

	// s and t in same cell in level 1 and both are overlay vertices
	// s is entry vertex, t is exit vertex of this cell

	_, sOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)
	_, tOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(targetEntryId)

	sourceCellNumber := pu.engine.graph.GetCellNumber(s)
	targetCellNumber := pu.engine.graph.GetCellNumber(t)

	offSourceEntryId := pu.engine.offsetForward(s, sourceEntryId, sourceCellNumber, sourceCellNumber)

	tExitId := tOutEdge.GetEdgeId()
	offTargetExitId := pu.engine.offsetBackward(t, tExitId, targetCellNumber, sourceCellNumber)

	sQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(s, offSourceEntryId, sOutEdge.GetEdgeId())
	sInfo := da.NewVertexInfo(0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))

	fpq.Insert(offSourceEntryId, 0, sInfo, sQueryKey)

	tQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(t, offTargetExitId, tOutEdge.GetEdgeId())
	tInfo := da.NewVertexInfo(0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))
	bpq.Insert(offTargetExitId, 0, tInfo, tQueryKey)

	fastestTT := 2 * pkg.INF_WEIGHT

	fMid := da.INVALID_EDGE_ID
	bMid := da.INVALID_EDGE_ID

	offFMid := da.INVALID_EDGE_ID
	offBMid := da.INVALID_EDGE_ID

	activeLandmarks := pu.lm.SelectBestQueryLandmarks(s, t)

	for fpq.Size() > 0 && bpq.Size() > 0 {
		minForward := fpq.GetMinrank()
		minBackward := bpq.GetMinrank()
		if util.Ge(minForward+minBackward, fastestTT) {

			break
		}

		queryKey := fpq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryId := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutInEdgeId()

		fpq.Scan(uEntryId)

		adjuEntryId := pu.engine.adjustForward(uId, uEntryId)

		// relax all out edges of u
		pu.engine.graph.ForOutEdgesOf(uId, pu.engine.graph.GetEntryOrder(uId, adjuEntryId), func(e *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
			vId := e.GetHead()

			vEntryId := pu.engine.graph.GetEntryOffset(vId) + da.Index(e.GetEntryPoint())
			edgeWeight := pu.metrics.GetWeight(e)

			newTravelTime := fpq.GetPriority(uEntryId) + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.engine.graph.GetCellNumber(vId) != sourceCellNumber {
				// do not cross cell boundary
				return
			}

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVEntryId := pu.engine.offsetForward(vId, vEntryId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			// relax edge
			vAlreadyLabelled := util.Lt(fpq.GetPriority(offVEntryId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, fpq.GetPriority(offVEntryId))) {
				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				pfv, _ := pu.lm.FindTighestConsistentLowerBound(vId, s, t, activeLandmarks)
				priority := newTravelTime + pfv

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, e.GetEdgeId())
					vInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))

					fpq.Insert(offVEntryId, priority, vInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false)
					fpq.DecreaseKey(offVEntryId, priority, newTravelTime, newPar)
				}

			}

			// check wether we already scannned an exit point of vId

			exitOffset := pu.engine.graph.GetExitOffset(vId)

			exitOffset = pu.engine.offsetBackward(vId, exitOffset, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			offVExitId := exitOffset

			// traverse outEdges of v
			pu.engine.graph.ForOutEdgesOf(vId, da.Index(e.GetEntryPoint()), func(e2 *da.OutEdge,
				exitPoint da.Index, turnType2 pkg.TurnType) {

				//  check if forward and backward search already scanned entry and exit point of v. if so, check whether we can improve the shortest path
				scannedByBackwardSearch := bpq.IsScanned(offVExitId)
				if scannedByBackwardSearch && util.Lt(fpq.GetPriority(offVEntryId)+pu.engine.metrics.GetTurnCost(turnType2)+
					bpq.GetPriority(offVExitId), fastestTT) {

					fastestTT = fpq.GetPriority(offVEntryId) + pu.engine.metrics.GetTurnCost(turnType2) +
						bpq.GetPriority(offVExitId)

					fMid = pu.engine.adjustForward(vId, offVEntryId)
					bMid = pu.engine.adjustBackward(vId, offVExitId)
					offFMid = offVEntryId
					offBMid = offVExitId

				}
				offVExitId++
			})

		})

		// backward search
		queryKey = bpq.ExtractMin()

		uItem = queryKey.GetItem()
		uId = uItem.GetNode()
		uExitId := uItem.GetEntryExitPoint()
		uOutEdgeId = uItem.GetOutInEdgeId()
		bpq.Scan(uExitId)

		adjuExitId := pu.engine.adjustBackward(uId, uExitId)

		// relax all in edges of u
		pu.engine.graph.ForInEdgesOf(uId, pu.engine.graph.GetExitOrder(uId, adjuExitId), func(e *da.InEdge, entryPoint da.Index, turnType pkg.TurnType) {
			vId := e.GetTail()

			vExitId := pu.engine.graph.GetExitOffset(vId) + da.Index(e.GetExitPoint())
			edgeWeight := pu.metrics.GetWeight(e)

			newTravelTime := bpq.GetPriority(uExitId) + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.engine.graph.GetCellNumber(vId) != sourceCellNumber {
				// do not cross cell boundary
				return
			}

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVExitId := pu.engine.offsetBackward(vId, vExitId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			// relax edge
			vAlreadyLabelled := util.Lt(bpq.GetPriority(offVExitId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, bpq.GetPriority(offVExitId))) {

				// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
				_, prv := pu.lm.FindTighestConsistentLowerBound(vId, s, t, activeLandmarks)
				priority := newTravelTime + prv

				if !vAlreadyLabelled {
					_, outEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(e.GetEdgeId())
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVExitId, outEdge.GetEdgeId())
					vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false))

					bpq.Insert(offVExitId, priority, vertexInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false)
					bpq.DecreaseKey(offVExitId, priority, newTravelTime, newPar)
				}
			}

			// check wether we already Labelled an entry point of vId
			entryOffset := pu.engine.graph.GetEntryOffset(vId)

			entryOffset = pu.engine.offsetForward(vId, entryOffset, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			offVEntryId := entryOffset

			// traverse outEdges of v
			pu.engine.graph.ForInEdgesOf(vId, da.Index(e.GetExitPoint()), func(e2 *da.InEdge,
				entryPoint da.Index, turnType2 pkg.TurnType) {
				//  check if forward and backward search already scanned entry and exit point of v. if so, check whether we can improve the shortest path
				scannedByForwardSearch := fpq.IsScanned(offVEntryId)
				if scannedByForwardSearch && util.Lt(fpq.GetPriority(offVEntryId)+pu.engine.metrics.GetTurnCost(turnType2)+
					bpq.GetPriority(offVExitId), fastestTT) {

					fastestTT = fpq.GetPriority(offVEntryId) + pu.engine.metrics.GetTurnCost(turnType2) +
						bpq.GetPriority(offVExitId)

					fMid = pu.engine.adjustForward(vId, offVEntryId)
					bMid = pu.engine.adjustBackward(vId, offVExitId)
					offFMid = offVEntryId
					offBMid = offVExitId
				}
				offVEntryId++
			})
		})

	}

	edgeIdPath := make([]da.Index, 0, 10)

	// u->mid
	_, midOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(fMid)
	if util.Gt(pu.metrics.GetWeight(midOutEdge), 0) {
		edgeIdPath = append(edgeIdPath, midOutEdge.GetEdgeId())
	}

	uId := offFMid
	for fpq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := fpq.Get(uId).GetParent().GetOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)

		pEId := fpq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	util.ReverseG(edgeIdPath)

	// mid<-v
	midOutEdge = pu.engine.graph.GetOutEdge(bMid)
	if util.Gt(pu.metrics.GetWeight(midOutEdge), 0) {
		edgeIdPath = append(edgeIdPath, midOutEdge.GetEdgeId())
	}

	uId = offBMid
	for bpq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = targetEntry, include sp edges didalam current cell & sp edge exit cell ini
		prevOutEdgeId := bpq.Get(uId).GetParent().GetOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)

		pEId := bpq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	pu.lock.Lock()
	*unpackedEdgePath = append(*unpackedEdgePath, edgeIdPath...)
	pu.lock.Unlock()

	fpq.Clear()
	bpq.Clear()

	if pu.useCache {
		// github.com/hashicorp/golang-lru/v2 is thread-safe
		pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), edgeIdPath)
	}
}

func (pu *PathUnpackerALT) GetStats() int64 {
	return pu.runtime
}

func (pu *PathUnpackerALT) setForAlternativeRoutes() {
	pu.forAlternativeRoutes = true
}

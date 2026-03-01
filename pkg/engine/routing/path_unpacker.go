package routing

import (
	"sync"
	"time"

	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PUCacheKey struct {
	startOverlayId  da.Index
	targetOverlayId da.Index
	level           uint8
}

func NewPUCacheKey(start, target da.Index, level uint8) PUCacheKey {
	return PUCacheKey{start, target, level}
}

type PathUnpacker struct {
	engine *CRPRoutingEngine

	metrics *metrics.Metric

	puCache *lru.Cache[PUCacheKey, []da.Index]

	useCache  bool
	oneToMany bool
	runtime   int64
	lock      sync.RWMutex
}

func NewPathUnpacker(engine *CRPRoutingEngine, metrics *metrics.Metric,
	puCache *lru.Cache[PUCacheKey, []da.Index], useCache, oneToMany bool) *PathUnpacker {
	return &PathUnpacker{
		engine:  engine,
		metrics: metrics,

		puCache:   puCache,
		useCache:  useCache,
		oneToMany: oneToMany,
		runtime:   0,
		lock:      sync.RWMutex{},
	}
}

/*
[1] Path Unpacking phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
[2] Haeupler, B. et al. (2025) “Bidirectional Dijkstra's Algorithm is Instance-Optimal,” in 2025 Symposium on Simplicity in Algorithms (SOSA). Society for Industrial and Applied Mathematics (Proceedings), pp. 202–215. Available at: https://doi.org/10.1137/1.9781611978315.16.
[3] I. Pohl. Bi-directional Search. In Machine Intelligence, volume 6, pages 124–140. Edinburgh Univ. Press, Edinburgh, 1971.

unpackPath. unpack a level-i shortcut (v, w) by running Bidirectional Dijkstra between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.

this path unpacking use bidirectional dijkstra (see algorithm 2 in ref [2])
proof of correctness of bidirectional dijkstra algorithm can be found in [2] or [3]

time complexity:
let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (turn-based graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
let q = number of shorcut edges in packedPath
time complexity of unpackPath: O(\sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))

todo: bikin path unpacker pakai A*, landmarks, triangle inequality (ALT) (Goldberg, A.V. and Harrelson, lm. (2005))
*/
func (pu *PathUnpacker) unpackPath(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Coordinate, []da.OutEdge, float64) {
	unpackedPath := make([]da.Coordinate, 0, 50)
	unpackedEdgePathComp := make([][]da.OutEdge, len(packedPath))
	totalDistance := 0.0
	now := time.Now()

	workers := concurrent.NewWorkerPool[pathUnpackingParam, any](4, len(packedPath))

	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.GetEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)

			outEdge := pu.engine.graph.GetOutEdge(cur.GetEdge())
			unpackedEdgePathComp[i] = append(unpackedEdgePathComp[i], *outEdge)

			i++
		} else {
			// overlay vertex
			entryVertex := offBit(cur.GetEdge(), UNPACK_OVERLAY_OFFSET)

			entryCellNumber := pu.engine.overlayGraph.GetVertex(entryVertex).GetCellNumber()
			var queryLevel uint8

			if !pu.oneToMany {
				queryLevel = pu.engine.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			} else {
				queryLevel = cur.GetQueryLevel()
			}

			exitVertex := offBit(packedPath[i+1].GetEdge(), UNPACK_OVERLAY_OFFSET)

			workers.AddJob(NewPathUnpackingParam(entryVertex, exitVertex, queryLevel, &unpackedEdgePathComp[i]))

			i += 2
		}
	}

	workers.Close()
	workers.Start(pu.unpackInLevelCell)
	workers.WaitDirect()

	unpackedEdgePath := make([]da.OutEdge, 0, 50)

	for i := 0; i < len(unpackedEdgePathComp); i++ {
		unpackedEdgePath = append(unpackedEdgePath, unpackedEdgePathComp[i]...)
	}

	unpackedEdgePath = removeDuplicates(unpackedEdgePath)

	for _, e := range unpackedEdgePath {
		eGeom := pu.engine.graph.GetEdgeGeometry(e.GetEdgeId())
		unpackedPath = append(unpackedPath, eGeom...)
		totalDistance += e.GetLength()
	}

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	// todo: polyline simplification unpackedPath
	return unpackedPath, unpackedEdgePath, totalDistance
}

func (pu *PathUnpacker) unpackInLevelCell(param pathUnpackingParam,
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
			// fetch from cache, cuma dipakai di server time-independent
			// buat tests, gak ambil dari cache
			for i := 0; i < len(overlayPath); i += 2 {
				pu.unpackInLevelCell(NewPathUnpackingParam(overlayPath[i], overlayPath[i+1], level-1, unpackedEdgePath))
			}
			return nil
		}
	}

	sVertex := pu.engine.overlayGraph.GetVertex(sourceOverlayId)
	sourceCellNumber := sVertex.GetCellNumber()

	// number of overlay vertices in this cell + number of overlayVertices in all direct subcells (subcells in level-1) of this cell
	numOverlayVerticesInThisCell := pu.engine.overlayGraph.GetNumOfOverlayVerticesOfCell(sourceCellNumber, level)
	maxSearchSize := util.MaxInt(numOverlayVerticesInThisCell, 512)

	maxEdgesInCell := pu.engine.graph.GetMaxEdgesInCell()

	fOverlayPq := da.NewQueryHeap[da.Index](maxSearchSize, int(maxEdgesInCell), da.MAP_STORAGE)
	bOverlayPq := da.NewQueryHeap[da.Index](maxSearchSize, int(maxEdgesInCell), da.MAP_STORAGE)
	fOverlayPq.PreallocateHeap(maxSearchSize)
	bOverlayPq.PreallocateHeap(maxSearchSize)

	truncatedSourceCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	tVertex := pu.engine.overlayGraph.GetVertex(targetOverlayId)
	targetCellNumber := tVertex.GetCellNumber()
	truncatedTargetCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(targetCellNumber, level)

	util.AssertPanic(truncatedSourceCellNumber == truncatedTargetCellNumber, "cell number/id dari sourceOverlay vertex dan targetOverlay vertex haruslah sama")

	sVertexInfo := da.NewVertexInfo[da.Index](0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	fOverlayPq.Insert(sourceOverlayId, 0, sVertexInfo, sourceOverlayId)

	tVertexInfo := da.NewVertexInfo[da.Index](0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	bOverlayPq.Insert(targetOverlayId, 0, tVertexInfo, targetOverlayId)

	fastestTT := 2 * pkg.INF_WEIGHT

	fScanned := make(map[da.Index]bool, maxSearchSize)
	bScanned := make(map[da.Index]bool, maxSearchSize)

	labelled := func(pq *da.QueryHeap[da.Index], v da.Index) bool {

		ok := da.Lt(pq.GetPriority(v), pkg.INF_WEIGHT)
		return ok
	}

	mid := da.INVALID_VERTEX_ID

	for fOverlayPq.Size() > 0 && bOverlayPq.Size() > 0 {
		minForward := fOverlayPq.GetMinrank()
		minBackward := bOverlayPq.GetMinrank()
		if da.Ge(minForward+minBackward, fastestTT) {
			break
		}
		u := fOverlayPq.ExtractMin()

		uOverlayId := u.GetItem()
		fScanned[uOverlayId] = true

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)

			newTravelTime := fOverlayPq.GetPriority(uOverlayId) + shortcutOutEdgeWeight
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := labelled(fOverlayPq, vOverlayId)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, fOverlayPq.GetPriority(vOverlayId))) {
				uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)

				fScanned[vOverlayId] = true
				// if v is the target overlay vertex, update the pq
				fOverlayPq.Set(vOverlayId, da.NewVertexInfo[da.Index](newTravelTime, da.NewVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
					uOverlayId, true)), vOverlayId)

			}

			scannedByBackwardSearch := bScanned[vOverlayId]
			if scannedByBackwardSearch && da.Lt(fOverlayPq.GetPriority(vOverlayId)+bOverlayPq.GetPriority(vOverlayId), fastestTT) {
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

			// relax edge
			wAlreadyLabelled := labelled(fOverlayPq, wNeighborId)
			if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, fOverlayPq.GetPriority(wNeighborId))) {
				if !wAlreadyLabelled {
					wVertexInfo := da.NewVertexInfo[da.Index](newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true))
					fOverlayPq.Insert(wNeighborId, newTravelTime, wVertexInfo, wNeighborId)
				} else {
					wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true)
					fOverlayPq.DecreaseKey(wNeighborId, newTravelTime, newTravelTime, wNewPar)
				}
			}

			scannedByBackwardSearch = bScanned[wNeighborId]
			if scannedByBackwardSearch && da.Lt(fOverlayPq.GetPriority(wNeighborId)+bOverlayPq.GetPriority(wNeighborId), fastestTT) {
				fastestTT = fOverlayPq.GetPriority(wNeighborId) + bOverlayPq.GetPriority(wNeighborId)
				mid = wNeighborId
			}
		})

		u = bOverlayPq.ExtractMin()

		uOverlayId = u.GetItem()
		bScanned[uOverlayId] = true

		// traverse all in neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForInNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutInEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)

			newTravelTime := bOverlayPq.GetPriority(uOverlayId) + shortcutInEdgeWeight
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := labelled(bOverlayPq, vOverlayId)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bOverlayPq.GetPriority(vOverlayId))) {
				uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)

				bScanned[vOverlayId] = true
				vVertex := da.NewVertexInfo[da.Index](newTravelTime, da.NewVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
					uOverlayId, true))
				bOverlayPq.Set(vOverlayId, vVertex, vOverlayId)
			}

			scannedByForwardSearch := fScanned[vOverlayId]
			if scannedByForwardSearch && da.Lt(fOverlayPq.GetPriority(vOverlayId)+bOverlayPq.GetPriority(vOverlayId), fastestTT) {
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

			// relax edge
			wAlreadyLabelled := labelled(bOverlayPq, wNeighborId)
			if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, bOverlayPq.GetPriority(wNeighborId))) {
				if !wAlreadyLabelled {
					wVertexInfo := da.NewVertexInfo[da.Index](newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true))
					bOverlayPq.Insert(wNeighborId, newTravelTime, wVertexInfo, wNeighborId)
				} else {
					wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true)
					bOverlayPq.DecreaseKey(wNeighborId, newTravelTime, newTravelTime, wNewPar)
				}
			}

			scannedByForwardSearch = fScanned[wNeighborId]
			if scannedByForwardSearch && da.Lt(fOverlayPq.GetPriority(wNeighborId)+bOverlayPq.GetPriority(wNeighborId), fastestTT) {
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

	overlayPath = util.ReverseG(overlayPath)

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
	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		pu.unpackInLevelCell(NewPathUnpackingParam(curV, nextV, level-1, unpackedEdgePath))
	}
	return nil
}

func (pu *PathUnpacker) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	unpackedEdgePath *[]da.OutEdge,
	sourceOverlayId, targetOverlayId da.Index) {

	if pu.useCache {
		if edgeIds, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
			// fetch from cache
			for _, edgeId := range edgeIds {
				edge := *pu.engine.graph.GetOutEdge(edgeId)

				*unpackedEdgePath = append(*unpackedEdgePath, edge)
			}

			return
		}
	}

	maxEdgesInCell := pu.engine.graph.GetMaxEdgesInCell()
	maxSearchSize := int(maxEdgesInCell) * 2

	// sourceEntryId inEdge that point to source vertex

	fpq := da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, int(maxEdgesInCell), da.ARRAY_STORAGE)
	bpq := da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, int(maxEdgesInCell), da.ARRAY_STORAGE)
	fpq.PreallocateHeap(maxSearchSize)
	bpq.PreallocateHeap(maxSearchSize)

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
	sInfo := da.NewVertexInfo[da.CRPQueryKey](0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))

	fpq.Insert(offSourceEntryId, 0, sInfo, sQueryKey)

	tQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(t, offTargetExitId, tOutEdge.GetEdgeId())
	tInfo := da.NewVertexInfo[da.CRPQueryKey](0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))
	bpq.Insert(offTargetExitId, 0, tInfo, tQueryKey)

	fScanned := make([]bool, maxSearchSize)
	bScanned := make([]bool, maxSearchSize)

	fastestTT := 2 * pkg.INF_WEIGHT

	fMid := da.INVALID_EDGE_ID
	bMid := da.INVALID_EDGE_ID

	offFMid := da.INVALID_EDGE_ID
	offBMid := da.INVALID_EDGE_ID

	for fpq.Size() > 0 && bpq.Size() > 0 {
		minForward := fpq.GetMinrank()
		minBackward := bpq.GetMinrank()
		if da.Ge(minForward+minBackward, fastestTT) {

			break
		}

		queryKey := fpq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryId := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutInEdgeId()

		fScanned[uEntryId] = true

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

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVEntryId := pu.engine.offsetForward(vId, vEntryId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			// relax edge
			vAlreadyLabelled := da.Lt(fpq.GetPriority(offVEntryId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, fpq.GetPriority(offVEntryId))) {

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, e.GetEdgeId())
					vInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))

					fpq.Insert(offVEntryId, newTravelTime, vInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false)
					fpq.DecreaseKey(offVEntryId, newTravelTime, newTravelTime, newPar)
				}

			}

			// check wether we already Labelled an exit point of vId

			exitOffset := pu.engine.graph.GetExitOffset(vId)

			exitOffset = pu.engine.offsetBackward(vId, exitOffset, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			offVExitId := exitOffset

			// traverse outEdges of v
			pu.engine.graph.ForOutEdgesOf(vId, da.Index(e.GetEntryPoint()), func(e2 *da.OutEdge,
				exitPoint da.Index, turnType2 pkg.TurnType) {
				// Customizable Route Planning In Road Networks, Page 8: Whenever we scan a vertex that has been seen from
				// the other side, we evaluate all possible turns between all entry and exit points of the intersection and check
				// whether we can improve µ.
				// basically: check if forward and backward search already Labelled entry and exit point of v. if so, check whether we can improve the shortest path
				// if head of outEdge v->w already Labelled by backward search, and its forwardTravelTime + backwardTravelTime is better than shortestPath, then update shortestPath
				scannedByBackwardSearch := bScanned[offVExitId]
				if scannedByBackwardSearch && da.Lt(fpq.GetPriority(offVEntryId)+pu.engine.metrics.GetTurnCost(turnType2)+
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
		bScanned[uExitId] = true

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

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVExitId := pu.engine.offsetBackward(vId, vExitId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			// relax edge
			vAlreadyLabelled := da.Lt(bpq.GetPriority(offVExitId), pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, bpq.GetPriority(offVExitId))) {

				if !vAlreadyLabelled {
					_, outEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(e.GetEdgeId())
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVExitId, outEdge.GetEdgeId())
					vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false))

					bpq.Insert(offVExitId, newTravelTime, vertexInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false)
					bpq.DecreaseKey(offVExitId, newTravelTime, newTravelTime, newPar)
				}
			}

			// check wether we already Labelled an entry point of vId
			entryOffset := pu.engine.graph.GetEntryOffset(vId)

			entryOffset = pu.engine.offsetForward(vId, entryOffset, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			offVEntryId := entryOffset

			// traverse outEdges of v
			pu.engine.graph.ForInEdgesOf(vId, da.Index(e.GetExitPoint()), func(e2 *da.InEdge,
				entryPoint da.Index, turnType2 pkg.TurnType) {
				// Customizable Route Planning In Road Networks, Page 8: Whenever we scan a vertex that has been seen from
				// the other side, we evaluate all possible turns between all entry and exit points of the intersection and check
				// whether we can improve µ.
				// basically: check if forward and backward search already Labelled entry and exit point of v. if so, check whether we can improve the shortest path
				scannedByForwardSearch := fScanned[offVEntryId]
				if scannedByForwardSearch && da.Lt(fpq.GetPriority(offVEntryId)+pu.engine.metrics.GetTurnCost(turnType2)+
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
	outEdges := make([]da.OutEdge, 0, 10)

	// u->mid
	_, midOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(fMid)
	if da.Gt(pu.metrics.GetWeight(midOutEdge), 0) {
		outEdges = append(outEdges, *midOutEdge)
		edgeIdPath = append(edgeIdPath, midOutEdge.GetEdgeId())
	}

	uId := offFMid
	for fpq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := fpq.Get(uId).GetParent().GetOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)
		prevOutEdge := *pu.engine.graph.GetOutEdge(prevOutEdgeId)

		outEdges = append(outEdges, prevOutEdge)

		pEId := fpq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	edgeIdPath = util.ReverseG(edgeIdPath)
	outEdges = util.ReverseG(outEdges)

	// mid<-v
	midOutEdge = pu.engine.graph.GetOutEdge(bMid)
	if da.Gt(pu.metrics.GetWeight(midOutEdge), 0) {
		outEdges = append(outEdges, *midOutEdge)
		edgeIdPath = append(edgeIdPath, midOutEdge.GetEdgeId())
	}

	uId = offBMid
	for bpq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = targetEntry, include sp edges didalam current cell & sp edge exit cell ini
		prevOutEdgeId := bpq.Get(uId).GetParent().GetOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)
		prevOutEdge := *pu.engine.graph.GetOutEdge(prevOutEdgeId)

		outEdges = append(outEdges, prevOutEdge)

		pEId := bpq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	pu.lock.Lock()
	*unpackedEdgePath = append(*unpackedEdgePath, outEdges...)
	pu.lock.Unlock()

	fpq.Clear()
	bpq.Clear()

	if pu.useCache {
		// github.com/hashicorp/golang-lru/v2 is thread-safe
		pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), edgeIdPath)
	}
}

func (pu *PathUnpacker) GetStats() int64 {
	return pu.runtime
}

package routing

import (
	"encoding/binary"
	"time"

	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func NewPUCacheKey(start, target da.Index, level uint8) []byte {
	buf := make([]byte, 9)
	binary.LittleEndian.PutUint32(buf[:4], uint32(start))
	binary.LittleEndian.PutUint32(buf[4:8], uint32(target))
	buf[8] = level
	return buf
}

type PathUnpacker struct {
	engine *CRPRoutingEngine

	metrics *metrics.Metric

	puCache *ristretto.Cache[[]byte, []da.Index]

	useCache             bool
	oneToMany            bool
	runtime              int64
	forAlternativeRoutes bool
}

func NewPathUnpacker(engine *CRPRoutingEngine, metrics *metrics.Metric,
	puCache *ristretto.Cache[[]byte, []da.Index], useCache, oneToMany bool) *PathUnpacker {
	return &PathUnpacker{
		engine:  engine,
		metrics: metrics,

		puCache:   puCache,
		useCache:  useCache,
		oneToMany: oneToMany,
		runtime:   0,
	}
}

/*
[1] Path Unpacking phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
[2] Haeupler, B. et al. (2025) “Bidirectional Dijkstra's Algorithm is Instance-Optimal,” in 2025 Symposium on Simplicity in Algorithms (SOSA). Society for Industrial and Applied Mathematics (Proceedings), pp. 202–215. Available at: https://doi.org/10.1137/1.9781611978315.16.
[3] I. Pohl. Bi-directional Search. In Machine Intelligence, volume 6, pages 124–140. Edinburgh Univ. Press, Edinburgh, 1971.

unpackPath. unpack a level-i shortcut (v, w) by running Bidirectional Dijkstra [2] between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
jika level i-1 >= 1, kita menggunakan shortcut edges (dari subcells dari level-i cell containing the shortcut) di overlay graph level i-1
jika level i-1 = 0, kita menggunakan base edges yang terletak pada level-i cell containing the shortcut

this path unpacking use bidirectional dijkstra (see algorithm 2 in ref [2])

time complexity:
let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (turn-based graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
let q = number of shorcut edges in packedPath
let L = highest level of multilevel partition
time complexity of unpackPath: O(q * ( L *  (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p) ) )
*/
func (pu *PathUnpacker) unpackPath(packedPath []da.VertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Index, map[uint64]uint8) {
	unpackedEdgePath := make([]da.Index, 0, len(packedPath))

	now := time.Now()

	// todo: sebelumnya pakai worker pools di worker_pool.go, di benchmark ada additional 40000 B/op, bikin worker pool yang allocate less space ?

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
			unpackedEdgePath = append(unpackedEdgePath, shortcutEdgeIdsPath...)
			i += 2
		}
	}

	unpackedEdgePath = removeDuplicates(unpackedEdgePath)

	dur := time.Since(now).Milliseconds()
	pu.runtime = dur
	return unpackedEdgePath, shortcutPathSet
}

func (pu *PathUnpacker) unpackInLevelCell(sourceOverlayId da.Index,
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
			// buat tests, gak ambil dari cache
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
			vOverlayVertex := pu.engine.overlayGraph.GetVertex(vOverlayId)
			wNeighborId := vOverlayVertex.GetNeighborOverlayVertex()
			wNeigborVertex := pu.engine.overlayGraph.GetVertex(wNeighborId)

			wCellNumber := wNeigborVertex.GetCellNumber()
			truncatedWCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
			if truncatedWCellNumber != truncatedSourceCellNumber {
				// if w is not in the same cell as sourceOverlayId in level l, dont visit w
				return
			}

			// get out edge that point to wEntryVertex from vOverlayId
			vOutEdgeWeight, vOutEdgeLength, vOutEdgeHwType := pu.engine.graph.GetOutEdgeTripleWeight(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vOutEdgeHwType, vOutEdgeWeight, vOutEdgeLength)

			// relax edge
			wAlreadyLabelled := labelled(fOverlayPq, wNeighborId)
			if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, fOverlayPq.GetPriority(wNeighborId))) {
				if !wAlreadyLabelled {
					wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true))
					fOverlayPq.Insert(wNeighborId, newTravelTime, wVertexInfo, wNeighborId)
				} else {
					wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true)
					fOverlayPq.DecreaseKey(wNeighborId, newTravelTime, newTravelTime, wNewPar)
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
			vOverlayVertex := pu.engine.overlayGraph.GetVertex(vOverlayId)
			wNeighborId := vOverlayVertex.GetNeighborOverlayVertex()
			wNeigborVertex := pu.engine.overlayGraph.GetVertex(wNeighborId)

			wCellNumber := wNeigborVertex.GetCellNumber()
			truncatedWCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
			if truncatedWCellNumber != truncatedSourceCellNumber {
				// if w is not in the same cell as sourceOverlayId in level l, dont visit w
				return
			}

			vInEdgeWeight, vInEdgeLength, vInEdgeHwType := pu.engine.graph.GetInEdgeTripleWeight(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vInEdgeHwType, vInEdgeWeight, vInEdgeLength)

			// relax edge

			wAlreadyLabelled := labelled(bOverlayPq, wNeighborId)
			if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, bOverlayPq.GetPriority(wNeighborId))) {
				if !wAlreadyLabelled {
					wVertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true))
					bOverlayPq.Insert(wNeighborId, newTravelTime, wVertexInfo, wNeighborId)
				} else {
					wNewPar := da.NewVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
						vOverlayId, true)
					bOverlayPq.DecreaseKey(wNeighborId, newTravelTime, newTravelTime, wNewPar)
				}
			}

			scannedByForwardSearch = fOverlayPq.IsScanned(wNeighborId)
			if scannedByForwardSearch && util.Lt(fOverlayPq.GetPriority(wNeighborId)+bOverlayPq.GetPriority(wNeighborId), fastestTT) {
				fastestTT = fOverlayPq.GetPriority(wNeighborId) + bOverlayPq.GetPriority(wNeighborId)
				mid = wNeighborId
			}
		})
	}

	overlayPath := make([]da.Index, 0, UNPACKER_OVERLAY_PATH_SIZE)

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

func (pu *PathUnpacker) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	sourceOverlayId, targetOverlayId da.Index) []da.Index {

	if pu.useCache {
		if edgeIds, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
			// fetch from cache

			return edgeIds
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
	tInEdge := pu.engine.graph.GetInEdge(targetEntryId)
	t := tInEdge.GetTail()
	s := sourceVertex.GetID()

	// s and t in same cell in level 1 and both are overlay vertices
	// s is entry vertex, t is exit vertex of this cell

	_, sOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)
	_, tOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(targetEntryId)

	sourceCellNumber := pu.engine.graph.GetCellNumber(s)
	targetCellNumber := pu.engine.graph.GetCellNumber(t)

	offSourceEntryId := pu.engine.offsetForward(s, sourceEntryId, sourceCellNumber, sourceCellNumber)

	tExitId := tOutEdge
	offTargetExitId := pu.engine.offsetBackward(t, tExitId, targetCellNumber, sourceCellNumber)

	sQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(s, offSourceEntryId, sOutEdge)
	sInfo := da.NewVertexInfo(0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))

	fpq.Insert(offSourceEntryId, 0, sInfo, sQueryKey)

	tQueryKey := da.NewCRPQueryKeyWithOutInEdgeId(t, offTargetExitId, tOutEdge)
	tInfo := da.NewVertexInfo(0, da.NewVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false))
	bpq.Insert(offTargetExitId, 0, tInfo, tQueryKey)

	fastestTT := 2 * pkg.INF_WEIGHT

	fMid := da.INVALID_EDGE_ID
	bMid := da.INVALID_EDGE_ID

	offFMid := da.INVALID_EDGE_ID
	offBMid := da.INVALID_EDGE_ID

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
		pu.engine.graph.ForOutEdgesOf(uId, pu.engine.graph.GetEntryOrder(uId, adjuEntryId), func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType,
			hwType pkg.OsmHighwayType) {
			vId := head

			vEntryId := pu.engine.graph.GetEntryOffset(vId) + entryPoint
			edgeWeight := pu.metrics.GetWeight(hwType, weight, length)

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

				if !vAlreadyLabelled {
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, eId)
					vInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))

					fpq.Insert(offVEntryId, newTravelTime, vInfo, queryKey)
				} else {
					newPar := da.NewVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false)
					fpq.DecreaseKey(offVEntryId, newTravelTime, newTravelTime, newPar)
				}
			}

			// check wether we already scannned an exit point of vId

			exitOffset := pu.engine.graph.GetExitOffset(vId)

			exitOffset = pu.engine.offsetBackward(vId, exitOffset, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			offVExitId := exitOffset

			// traverse outEdges of v
			pu.engine.graph.ForOutEdgesOf(vId, entryPoint, func(_, _ da.Index, _, _ float64, _, _ da.Index, turnType2 pkg.TurnType,
				_ pkg.OsmHighwayType) {

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
		pu.engine.graph.ForInEdgesOf(uId, pu.engine.graph.GetExitOrder(uId, adjuExitId), func(eId, tail da.Index, weight, length float64, exitPoint, entryPoint da.Index,
			turnType pkg.TurnType, hwType pkg.OsmHighwayType) {
			vId := tail

			vExitId := pu.engine.graph.GetExitOffset(vId) + exitPoint
			edgeWeight := pu.metrics.GetWeight(hwType, weight, length)

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

				if !vAlreadyLabelled {
					_, outEdgeId := pu.engine.graph.GetHeadOfInedgeWithOutEdge(eId)
					queryKey := da.NewCRPQueryKeyWithOutInEdgeId(vId, offVExitId, outEdgeId)
					vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false))

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
			pu.engine.graph.ForInEdgesOf(vId, exitPoint, func(_, _ da.Index, _, _ float64, _, _ da.Index,
				turnType2 pkg.TurnType, _ pkg.OsmHighwayType) {
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

	edgeIdPath := make([]da.Index, 0, UNPACKER_EDGE_PATH_SIZE)

	// u->mid
	_, midOutEdgeId := pu.engine.graph.GetHeadOfInedgeWithOutEdge(fMid)
	midInEdgeWeight, midInEdgeLength, midInEdgeHwType := pu.engine.graph.GetInEdgeTripleWeight(fMid)
	if util.Gt(pu.metrics.GetWeight(midInEdgeHwType, midInEdgeWeight, midInEdgeLength), 0) {
		edgeIdPath = append(edgeIdPath, midOutEdgeId)
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

	midOutEdgeWeight, midOutEdgeLength, midOutEdgeHwType := pu.engine.graph.GetInEdgeTripleWeight(bMid)

	if util.Gt(pu.metrics.GetWeight(midOutEdgeHwType, midOutEdgeWeight, midOutEdgeLength), 0) {
		edgeIdPath = append(edgeIdPath, bMid)
	}

	uId = offBMid
	for bpq.Get(uId).GetParent().GetEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = targetEntry, include sp edges didalam current cell & sp edge exit cell ini
		prevOutEdgeId := bpq.Get(uId).GetParent().GetOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)

		pEId := bpq.Get(uId).GetParent().GetEdge()

		uId = pEId
	}

	fpq.Clear()
	bpq.Clear()

	if pu.useCache {
		// https://github.com/dgraph-io/ristretto is thread-safe
		pu.puCache.Set(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), edgeIdPath, 1)
	}

	return edgeIdPath
}

func (pu *PathUnpacker) GetStats() int64 {
	return pu.runtime
}

// pas di profiling fungsi ini allocate banyak space, load test 900vus
// htop RES dari 1.9gb ke 3.0 gb, osrm cuma max 770mb pas di load test, -> setelah pake slice pointer receiver: 1.9gb ke 2.8 gb utk sp query dan 3.2 gb untuk alternative routes query
// -> setelah gak pake worker pool di pathUnpaker,  2.8gb alternative routes query
// -> setelah gak pake pointer buat graph []Vertex, []OutEdge, []InEdge, overlay graph []OverlayVertex: setelah read graph 1.1 gb,
// load test 900vus alternative routes query naik ke 2.5 gb
// alokasi gede di GetOsmNodePoints() 32 million allocs, ?
// alokasi gede lain ada di polyline.EncodeCoords() 60 million allocs, todo: investigate ini
// cara cek escape to heap:
// go build -o ./bin/engine_profiling -pgo=./cmd/engine/engine.pgo -gcflags=-m   ./cmd/engine_profiling    2> escape_analysis.txt ;
// di vs code > Source Action > see show compiler optimization
func (re *CRPRoutingEngine) GetEdgePath(edgeIdPath []da.Index) (*da.Coordinates, float64) {

	totalDistance := 0.0

	capacity := 0

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		totalDistance += re.graph.GetOutEdgeLength(eId)
		capacity += re.graph.GetEdgeGeometryLength(eId)
	}

	path := da.NewCoordinatesWithCap(capacity)

	for i := 0; i < len(edgeIdPath); i++ {
		eId := edgeIdPath[i]
		re.graph.AppendPathWithEdgeGeometry(path, eId)
	}

	return path, totalDistance
}

func (pu *PathUnpacker) setForAlternativeRoutes() {
	pu.forAlternativeRoutes = true
}

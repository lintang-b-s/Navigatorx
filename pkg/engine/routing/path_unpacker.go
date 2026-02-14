package routing

import (
	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
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

	fInfo []*VertexInfo[da.Index]
	bInfo []*VertexInfo[da.Index]

	lfInfo []*VertexInfo[da.CRPQueryKey]
	lbInfo []*VertexInfo[da.CRPQueryKey]
	fpq    *da.MinHeap[da.CRPQueryKey]
	bpq    *da.MinHeap[da.CRPQueryKey]

	fOverlayPq *da.MinHeap[da.Index]
	bOverlayPq *da.MinHeap[da.Index]
	puCache    *lru.Cache[PUCacheKey, []da.Index]

	useCache  bool
	oneToMany bool
}

func NewPathUnpacker(engine *CRPRoutingEngine, metrics *metrics.Metric,
	puCache *lru.Cache[PUCacheKey, []da.Index], useCache, oneToMany bool) *PathUnpacker {
	return &PathUnpacker{
		engine:     engine,
		metrics:    metrics,
		fInfo:      make([]*VertexInfo[da.Index], 0),
		bInfo:      make([]*VertexInfo[da.Index], 0),
		lfInfo:     make([]*VertexInfo[da.CRPQueryKey], 0),
		lbInfo:     make([]*VertexInfo[da.CRPQueryKey], 0),
		fpq:        da.NewFourAryHeap[da.CRPQueryKey](),
		bpq:        da.NewFourAryHeap[da.CRPQueryKey](),
		fOverlayPq: da.NewFourAryHeap[da.Index](),
		bOverlayPq: da.NewFourAryHeap[da.Index](),
		puCache:    puCache,
		useCache:   useCache,
		oneToMany:  oneToMany,
	}
}

/*
[1] Path Unpacking phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
[2] Haeupler, B. et al. (2025) “Bidirectional Dijkstra's Algorithm is Instance-Optimal,” in 2025 Symposium on Simplicity in Algorithms (SOSA). Society for Industrial and Applied Mathematics (Proceedings), pp. 202–215. Available at: https://doi.org/10.1137/1.9781611978315.16.
[3] I. Pohl. Bi-directional Search. In Machine Intelligence, volume 6, pages 124–140. Edinburgh Univ. Press, Edinburgh, 1971.

unpackPath. unpack a level-i shortcut (v, w) by running Bidirectional Dijkstra between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.

Customizable Route Planning in Road Networks, (Delling et al.(2015)) page 14:
Path Unpacking. Up to now, we have only discussed how to compute the distance between two arcs.
Following the parent pointers of the meeting vertex (the vertex that was responsible for the last update
of µ) of forward and backward searches, we obtain a path that potentially containing shortcuts. To obtain
the complete path description as a sequence of arcs (or vertices) in the original graph, each shortcut in
the result must be translated (unpacked) into the corresponding subpath. An obvious approach is to store
this information for each shortcut explicitly, but this is wasteful in terms of space. Instead, we recursively
unpack a level-i shortcut (v, w) by running bidirectional Dijkstra between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut. See Fig. 5 for an illustration. This does not increase the
metric-dependent space consumption, and query times are still small enough. Note that disjoint cells can be
handled in parallel.
If even faster unpacking times are needed, the customization phase could store a bit with each arc at
level i−1 indicating whether it appears in a shortcut at level i. Only arcs with this bit set need to be visited
during the unpacking process. This approach increases the metric-dependent amount of data stored only
slightly, and does accelerate queries. Unfortunately, it complicates the customization step.
In practice, we use a simpler approach: we maintain a cache of frequently-used shortcuts. Each entry in
the cache represents a level-i shortcut together with the corresponding sequence of level-(i − 1) shortcuts.
Note that we have one cache with all shortcuts instead of having one cache for each level. As the experiments
will show, with a standard least-recently used (LRU) update policy, even a small cache can accelerate path
unpacking significantly.

this path unpacking use bidirectional dijkstra (see algorithm 2 in ref [2])
proof of correctness of bidirectional dijkstra algorithm can be found in [2] or [3]

todo: bikin path unpacker pakai A*, landmarks, triangle inequality (ALT) (Goldberg, A.V. and Harrelson, lm. (2005))
todo2: investigate kenapa path unpacking ini lebih lemot dari path unpackingnya OSRM (https://github.com/Project-OSRM/osrm-backend/blob/master/include/engine/routing_algorithms/routing_base_mld.hpp)
todo3: mungkin biar lebih cepet, di path upacking overlay cells yang saling disjoint bisa di goroutine yang beda
*/
func (pu *PathUnpacker) unpackPath(packedPath []vertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Coordinate, []da.OutEdge, float64) {
	unpackedPath := make([]da.Coordinate, 0, 50)
	unpackedEdgePath := make([]da.OutEdge, 0, 50)
	totalDistance := 0.0
	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.getEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)

			outEdge := pu.engine.graph.GetOutEdge(cur.getEdge())
			unpackedEdgePath = append(unpackedEdgePath, *outEdge)

			i++
		} else {
			// overlay vertex
			entryVertex := offBit(cur.getEdge(), UNPACK_OVERLAY_OFFSET)

			entryCellNumber := pu.engine.overlayGraph.GetVertex(entryVertex).GetCellNumber()
			var queryLevel uint8

			if !pu.oneToMany {
				queryLevel = pu.engine.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			} else {
				queryLevel = cur.getQueryLevel()

			}

			exitVertex := offBit(packedPath[i+1].getEdge(), UNPACK_OVERLAY_OFFSET)

			pu.unpackInLevelCell(entryVertex, exitVertex, queryLevel, &unpackedEdgePath)

			i += 2
		}
	}

	unpackedEdgePath = removeDuplicates(unpackedEdgePath)

	for _, e := range unpackedEdgePath {
		eGeom := pu.engine.graph.GetEdgeGeometry(e.GetEdgeId())
		unpackedPath = append(unpackedPath, eGeom...)
		totalDistance += e.GetLength()
	}

	// todo: polyline simplification unpackedPath
	return unpackedPath, unpackedEdgePath, totalDistance
}

func (pu *PathUnpacker) unpackInLevelCell(sourceOverlayId, targetOverlayId da.Index, level uint8, unpackedEdgePath *[]da.OutEdge,
) {

	if level == 1 {
		sourceEntryId := pu.engine.overlayGraph.GetVertex(sourceOverlayId).GetOriginalEdge()
		neighborOfTarget := pu.engine.overlayGraph.GetVertex(targetOverlayId).GetNeighborOverlayVertex()
		targetEntryId := pu.engine.overlayGraph.GetVertex(neighborOfTarget).GetOriginalEdge()

		pu.unpackInLowestLevelCell(sourceEntryId, targetEntryId, unpackedEdgePath,
			sourceOverlayId, targetOverlayId)
		return
	}

	if pu.useCache {
		if overlayPath, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {
			// fetch from cache, cuma dipakai di server time-independent
			// buat tests, gak ambil dari cache
			for i := 0; i < len(overlayPath); i += 2 {
				pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, unpackedEdgePath)
			}
			return
		}
	}

	maxSearchSize := pu.engine.overlayGraph.NumberOfOverlayVertices()
	pu.fInfo = make([]*VertexInfo[da.Index], maxSearchSize)
	pu.bInfo = make([]*VertexInfo[da.Index], maxSearchSize)
	initInfWeightVertexInfo(pu.fInfo)
	initInfWeightVertexInfo(pu.bInfo)
	pu.fOverlayPq.Preallocate(maxSearchSize)
	pu.bOverlayPq.Preallocate(maxSearchSize)

	sourceCellNumber := pu.engine.overlayGraph.GetVertex(sourceOverlayId).GetCellNumber()
	truncatedSourceCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	targetCellNumber := pu.engine.overlayGraph.GetVertex(targetOverlayId).GetCellNumber()
	truncatedTargetCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(targetCellNumber, level)

	util.AssertPanic(truncatedSourceCellNumber == truncatedTargetCellNumber, "cell number/id dari sourceOverlay vertex dan targetOverlay vertex haruslah sama")

	shNode := da.NewPriorityQueueNode(0, sourceOverlayId)
	pu.fOverlayPq.Insert(shNode)

	thNode := da.NewPriorityQueueNode(0, targetOverlayId)
	pu.bOverlayPq.Insert(thNode)

	fastestTT := 2 * pkg.INF_WEIGHT
	pu.fInfo[sourceOverlayId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), shNode)
	pu.bInfo[targetOverlayId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), thNode)

	fScanned := make([]bool, maxSearchSize)
	bScanned := make([]bool, maxSearchSize)

	mid := da.INVALID_VERTEX_ID

	for pu.fOverlayPq.Size() > 0 && pu.bOverlayPq.Size() > 0 {
		minForward := pu.fOverlayPq.GetMinrank()
		minBackward := pu.bOverlayPq.GetMinrank()
		if da.Ge(minForward+minBackward, (fastestTT)) {
			break
		}
		u, _ := pu.fOverlayPq.ExtractMin()

		uOverlayId := u.GetItem()
		fScanned[uOverlayId] = true

		uTravelTime := u.GetRank()

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)

			newTravelTime := uTravelTime + shortcutOutEdgeWeight
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := da.Lt(pu.fInfo[vOverlayId].GetTravelTime(), pkg.INF_WEIGHT)

			if vAlreadyLabelled && da.Ge(newTravelTime, pu.fInfo[vOverlayId].GetTravelTime()) {
				return
			}

			uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)

			fScanned[vOverlayId] = true
			// if v is the target overlay vertex, update the pq
			pu.fInfo[vOverlayId] = NewVertexInfo[da.Index](newTravelTime, newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
				uOverlayId, true), nil)

			scannedByBackwardSearch := bScanned[vOverlayId]
			if scannedByBackwardSearch && da.Lt(pu.fInfo[vOverlayId].GetTravelTime()+pu.bInfo[vOverlayId].GetTravelTime(), fastestTT) {
				fastestTT = pu.fInfo[vOverlayId].GetTravelTime() + pu.bInfo[vOverlayId].GetTravelTime()
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

			wAlreadyLabelled := da.Lt(pu.fInfo[wNeighborId].GetTravelTime(), pkg.INF_WEIGHT)
			if !wAlreadyLabelled {
				whNode := da.NewPriorityQueueNode(newTravelTime, wNeighborId)
				pu.fInfo[wNeighborId] = NewVertexInfo(newTravelTime, newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true), whNode)
				pu.fOverlayPq.Insert(whNode)
			} else {
				whNode := pu.fInfo[wNeighborId].GetHeapNode()
				pu.fInfo[wNeighborId].UpdateTravelTime(newTravelTime)
				pu.fInfo[wNeighborId].UpdateParent(newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true))
				pu.fOverlayPq.DecreaseKey(whNode, newTravelTime)
			}

			scannedByBackwardSearch = bScanned[wNeighborId]
			if scannedByBackwardSearch && da.Lt(pu.fInfo[wNeighborId].GetTravelTime()+pu.bInfo[wNeighborId].GetTravelTime(), fastestTT) {
				fastestTT = pu.fInfo[wNeighborId].GetTravelTime() + pu.bInfo[wNeighborId].GetTravelTime()
				mid = wNeighborId

			}
		})

		u, _ = pu.bOverlayPq.ExtractMin()

		uOverlayId = u.GetItem()
		bScanned[uOverlayId] = true

		uTravelTime = u.GetRank()

		// traverse all in neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForInNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutInEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)

			newTravelTime := uTravelTime + shortcutInEdgeWeight
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := da.Lt(pu.bInfo[vOverlayId].GetTravelTime(), pkg.INF_WEIGHT)

			if vAlreadyLabelled && da.Ge(newTravelTime, pu.bInfo[vOverlayId].GetTravelTime()) {
				return
			}

			uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)

			bScanned[vOverlayId] = true
			pu.bInfo[vOverlayId] = NewVertexInfo[da.Index](newTravelTime, newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
				uOverlayId, true), nil)

			scannedByForwardSearch := fScanned[vOverlayId]
			if scannedByForwardSearch && da.Lt(pu.fInfo[vOverlayId].GetTravelTime()+pu.bInfo[vOverlayId].GetTravelTime(), fastestTT) {
				fastestTT = pu.fInfo[vOverlayId].GetTravelTime() + pu.bInfo[vOverlayId].GetTravelTime()
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
			vInEdge := pu.engine.graph.GetInEdge(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vInEdge)

			wAlreadyLabelled := da.Lt(pu.bInfo[wNeighborId].GetTravelTime(), pkg.INF_WEIGHT)
			if !wAlreadyLabelled {
				whNode := da.NewPriorityQueueNode(newTravelTime, wNeighborId)
				pu.bInfo[wNeighborId] = NewVertexInfo(newTravelTime, newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true), whNode)
				pu.bOverlayPq.Insert(whNode)
			} else {
				whNode := pu.bInfo[wNeighborId].GetHeapNode()
				pu.bInfo[wNeighborId].UpdateTravelTime(newTravelTime)
				pu.bInfo[wNeighborId].UpdateParent(newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true))
				pu.bOverlayPq.DecreaseKey(whNode, newTravelTime)
			}

			scannedByForwardSearch = fScanned[wNeighborId]
			if scannedByForwardSearch && da.Lt(pu.fInfo[wNeighborId].GetTravelTime()+pu.bInfo[wNeighborId].GetTravelTime(), fastestTT) {
				fastestTT = pu.fInfo[wNeighborId].GetTravelTime() + pu.bInfo[wNeighborId].GetTravelTime()
				mid = wNeighborId

			}
		})
	}

	overlayPath := make([]da.Index, 0, 8)

	overlayPath = append(overlayPath, mid)

	curOverlayId := pu.fInfo[mid].GetParent().edge
	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pu.fInfo[curOverlayId].GetParent().edge
	}

	overlayPath = util.ReverseG(overlayPath)

	curOverlayId = pu.bInfo[mid].GetParent().edge

	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pu.bInfo[curOverlayId].GetParent().edge
	}

	pu.fOverlayPq.Clear()
	pu.bOverlayPq.Clear()

	util.AssertPanic(len(overlayPath)%2 == 0, "harusnya len(overlayPath) genap")

	if pu.useCache {
		pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath)
	}
	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		pu.unpackInLevelCell(curV, nextV, level-1, unpackedEdgePath)
	}
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
	pu.lfInfo = make([]*VertexInfo[da.CRPQueryKey], maxSearchSize)
	pu.lbInfo = make([]*VertexInfo[da.CRPQueryKey], maxSearchSize)
	initInfWeightVertexInfo(pu.lfInfo)
	initInfWeightVertexInfo(pu.lbInfo)
	pu.fpq.Preallocate(int(maxSearchSize))
	pu.bpq.Preallocate(int(maxSearchSize))

	// sourceEntryId: id buat inEdge u->s
	// targetEntryId: id buat inEdge t->v

	// get source vertex
	sourceVertex := pu.engine.graph.GetVertex(pu.engine.graph.GetHeadFromInEdge(sourceEntryId))
	tVId := pu.engine.graph.GetInEdge(targetEntryId).GetTail()

	// s and t in same cell in level 1 and both are overlay vertices
	// s is entry vertex, t is exit vertex of this cell

	_, sOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)
	_, tOutEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(targetEntryId)

	sourceCellNumber := pu.engine.graph.GetCellNumber(sourceVertex.GetID())
	targetCellNumber := pu.engine.graph.GetCellNumber(tVId)

	offSourceEntryId := pu.engine.offsetForward(sourceVertex.GetID(), sourceEntryId, sourceCellNumber, sourceCellNumber)

	tExitId := tOutEdge.GetEdgeId()
	offTargetExitId := pu.engine.offsetBackward(tVId, tExitId, targetCellNumber, sourceCellNumber)

	shNode := da.NewPriorityQueueNode(0, da.NewCRPQueryKeyWithOutInEdgeId(sourceVertex.GetID(),
		offSourceEntryId, sOutEdge.GetEdgeId()))
	pu.fpq.Insert(shNode)

	thNode := da.NewPriorityQueueNode(0, da.NewCRPQueryKeyWithOutInEdgeId(tVId,
		offTargetExitId, tOutEdge.GetEdgeId()))
	pu.bpq.Insert(thNode)

	pu.lfInfo[offSourceEntryId] = NewVertexInfo(0, newVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false), shNode)

	pu.lbInfo[offTargetExitId] = NewVertexInfo(0, newVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false), thNode)

	fScanned := make([]bool, maxSearchSize)
	bScanned := make([]bool, maxSearchSize)

	fastestTT := 2 * pkg.INF_WEIGHT

	fMid := da.INVALID_EDGE_ID
	bMid := da.INVALID_EDGE_ID

	offFMid := da.INVALID_EDGE_ID
	offBMid := da.INVALID_EDGE_ID

	for pu.fpq.Size() > 0 && pu.bpq.Size() > 0 {
		minForward := pu.fpq.GetMinrank()
		minBackward := pu.bpq.GetMinrank()
		if da.Ge(minForward+minBackward, (fastestTT)) {
			break
		}

		queryKey, _ := pu.fpq.ExtractMin()

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

			newTravelTime := queryKey.GetRank() + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.engine.graph.GetCellNumber(vId) != sourceCellNumber {
				// do not cross cell boundary
				return
			}

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVEntryId := pu.engine.offsetForward(vId, vEntryId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			vAlreadyLabelled := da.Lt(pu.lfInfo[offVEntryId].GetTravelTime(), pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newTravelTime, pu.lfInfo[offVEntryId].GetTravelTime()) {
				return
			}

			if !vAlreadyLabelled {
				vhNode := da.NewPriorityQueueNode(newTravelTime,
					da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, e.GetEdgeId()))
				pu.lfInfo[offVEntryId] = NewVertexInfo(newTravelTime, newVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false), vhNode)

				pu.fpq.Insert(vhNode)
			} else {
				vhNode := pu.lfInfo[offVEntryId].GetHeapNode()
				pu.lfInfo[offVEntryId].UpdateTravelTime(newTravelTime)
				pu.lfInfo[offVEntryId].UpdateParent(newVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))
				pu.fpq.DecreaseKey(vhNode, newTravelTime)
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
				if scannedByBackwardSearch && da.Lt(pu.lfInfo[offVEntryId].GetTravelTime()+pu.engine.metrics.GetTurnCost(turnType2)+
					pu.lbInfo[offVExitId].GetTravelTime(), fastestTT) {

					fastestTT = pu.lfInfo[offVEntryId].GetTravelTime() + pu.engine.metrics.GetTurnCost(turnType2) +
						pu.lbInfo[offVExitId].GetTravelTime()

					fMid = pu.engine.adjustForward(vId, offVEntryId)
					bMid = pu.engine.adjustBackward(vId, offVExitId)
					offFMid = offVEntryId
					offBMid = offVExitId

				}
				offVExitId++
			})

		})

		// backward search
		queryKey, _ = pu.bpq.ExtractMin()

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

			newTravelTime := queryKey.GetRank() + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.engine.graph.GetCellNumber(vId) != sourceCellNumber {
				// do not cross cell boundary
				return
			}

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVExitId := pu.engine.offsetBackward(vId, vExitId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			vAlreadyLabelled := da.Lt(pu.lbInfo[offVExitId].GetTravelTime(), pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newTravelTime, pu.lbInfo[offVExitId].GetTravelTime()) {
				return
			}

			if !vAlreadyLabelled {
				_, outEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(e.GetEdgeId())
				vhNode := da.NewPriorityQueueNode(newTravelTime,
					da.NewCRPQueryKeyWithOutInEdgeId(vId, offVExitId, outEdge.GetEdgeId()))
				pu.lbInfo[offVExitId] = NewVertexInfo(newTravelTime, newVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false), vhNode)

				pu.bpq.Insert(vhNode)
			} else {
				vhNode := pu.lbInfo[offVExitId].GetHeapNode()
				pu.lbInfo[offVExitId].UpdateTravelTime(newTravelTime)
				pu.lbInfo[offVExitId].UpdateParent(newVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false))
				pu.bpq.DecreaseKey(vhNode, newTravelTime)
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
				if scannedByForwardSearch && da.Lt(pu.lfInfo[offVEntryId].GetTravelTime()+pu.engine.metrics.GetTurnCost(turnType2)+
					pu.lbInfo[offVExitId].GetTravelTime(), fastestTT) {

					fastestTT = pu.lfInfo[offVEntryId].GetTravelTime() + pu.engine.metrics.GetTurnCost(turnType2) +
						pu.lbInfo[offVExitId].GetTravelTime()

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
	outEdges = append(outEdges, *midOutEdge)
	edgeIdPath = append(edgeIdPath, midOutEdge.GetEdgeId())

	uId := offFMid
	for pu.lfInfo[uId].parent.getEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := pu.lfInfo[uId].parent.getOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)
		prevOutEdge := *pu.engine.graph.GetOutEdge(prevOutEdgeId)

		outEdges = append(outEdges, prevOutEdge)

		pEId := pu.lfInfo[uId].parent.getEdge()

		uId = pEId
	}

	edgeIdPath = util.ReverseG(edgeIdPath)
	outEdges = util.ReverseG(outEdges)

	// mid<-v
	midOutEdge = pu.engine.graph.GetOutEdge(bMid)
	outEdges = append(outEdges, *midOutEdge)
	edgeIdPath = append(edgeIdPath, midOutEdge.GetEdgeId())

	uId = offBMid
	for pu.lbInfo[uId].parent.getEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = targetEntry, include sp edges didalam current cell & sp edge exit cell ini
		prevOutEdgeId := pu.lbInfo[uId].parent.getOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)
		prevOutEdge := *pu.engine.graph.GetOutEdge(prevOutEdgeId)

		outEdges = append(outEdges, prevOutEdge)

		pEId := pu.lbInfo[uId].parent.getEdge()

		uId = pEId
	}

	*unpackedEdgePath = append(*unpackedEdgePath, outEdges...)

	pu.fpq.Clear()
	pu.bpq.Clear()

	if pu.useCache {
		pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), edgeIdPath)
	}
}

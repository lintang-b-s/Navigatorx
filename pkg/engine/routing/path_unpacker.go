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
func (pu *PathUnpacker) unpackPath(packedPath []vertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Coordinate, []da.OutEdge, float64) {
	unpackedPath := make([]da.Coordinate, 0, 50)
	unpackedEdgePathComp := make([][]da.OutEdge, len(packedPath))
	totalDistance := 0.0
	now := time.Now()

	workers := concurrent.NewWorkerPool[pathUnpackingParam, any](4, len(packedPath))

	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.getEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)

			outEdge := pu.engine.graph.GetOutEdge(cur.getEdge())
			unpackedEdgePathComp[i] = append(unpackedEdgePathComp[i], *outEdge)

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

	sourceCellNumber := pu.engine.overlayGraph.GetVertex(sourceOverlayId).GetCellNumber()

	// number of overlay vertices in this cell + number of overlayVertices in all direct subcells (subcells in level-1) of this cell
	numOverlayVerticesInThisCell := pu.engine.overlayGraph.GetNumOfOverlayVerticesOfCell(sourceCellNumber, level)
	maxSearchSize := util.MaxInt(numOverlayVerticesInThisCell, 512)
	fInfo := make(map[da.Index]*VertexInfo[da.Index], maxSearchSize)
	bInfo := make(map[da.Index]*VertexInfo[da.Index], maxSearchSize)

	fOverlayPq := da.NewFourAryHeap[da.Index]()
	bOverlayPq := da.NewFourAryHeap[da.Index]()
	fOverlayPq.Preallocate(maxSearchSize)
	bOverlayPq.Preallocate(maxSearchSize)

	truncatedSourceCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	targetCellNumber := pu.engine.overlayGraph.GetVertex(targetOverlayId).GetCellNumber()
	truncatedTargetCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(targetCellNumber, level)

	util.AssertPanic(truncatedSourceCellNumber == truncatedTargetCellNumber, "cell number/id dari sourceOverlay vertex dan targetOverlay vertex haruslah sama")

	shNode := da.NewPriorityQueueNode(0, sourceOverlayId)
	fOverlayPq.Insert(shNode)

	thNode := da.NewPriorityQueueNode(0, targetOverlayId)
	bOverlayPq.Insert(thNode)

	fastestTT := 2 * pkg.INF_WEIGHT
	fInfo[sourceOverlayId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), shNode)
	bInfo[targetOverlayId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), thNode)

	fScanned := make(map[da.Index]bool, maxSearchSize)
	bScanned := make(map[da.Index]bool, maxSearchSize)

	labelled := func(dist map[da.Index]*VertexInfo[da.Index], v da.Index) bool {
		// same as check dist[v] < INF_WEIGHT if dist implemented using array/slice
		// https://go.dev/blog/swisstable
		// go1.24 use swiss table for its hash table (open adressing)
		// a=load factor=n/m, n=number of items to be mapped, m=size of hash table
		// open adressing avg case: unsuccesful search & insert in O(1/(1-a)) or O(1)
		// in this function  n=number of overlay vertices in sourceCell + number of overlayVertices in all direct subcells (subcells in level-1) of sourceCell

		_, ok := dist[v]
		return ok
	}

	mid := da.INVALID_VERTEX_ID

	for fOverlayPq.Size() > 0 && bOverlayPq.Size() > 0 {
		minForward := fOverlayPq.GetMinrank()
		minBackward := bOverlayPq.GetMinrank()
		if da.Ge(minForward+minBackward, (fastestTT)) {
			break
		}
		u, _ := fOverlayPq.ExtractMin()

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

			vAlreadyLabelled := labelled(fInfo, vOverlayId)
			if vAlreadyLabelled && da.Ge(newTravelTime, fInfo[vOverlayId].GetTravelTime()) {
				return
			}

			uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)

			fScanned[vOverlayId] = true
			// if v is the target overlay vertex, update the pq
			fInfo[vOverlayId] = NewVertexInfo[da.Index](newTravelTime, newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
				uOverlayId, true), nil)

			scannedByBackwardSearch := bScanned[vOverlayId]
			if scannedByBackwardSearch && da.Lt(fInfo[vOverlayId].GetTravelTime()+bInfo[vOverlayId].GetTravelTime(), fastestTT) {
				fastestTT = fInfo[vOverlayId].GetTravelTime() + bInfo[vOverlayId].GetTravelTime()
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

			wAlreadyLabelled := labelled(fInfo, wNeighborId)
			if !wAlreadyLabelled {
				whNode := da.NewPriorityQueueNode(newTravelTime, wNeighborId)
				fInfo[wNeighborId] = NewVertexInfo(newTravelTime, newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true), whNode)
				fOverlayPq.Insert(whNode)
			} else {
				whNode := fInfo[wNeighborId].GetHeapNode()
				fInfo[wNeighborId].UpdateTravelTime(newTravelTime)
				fInfo[wNeighborId].UpdateParent(newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true))
				fOverlayPq.DecreaseKey(whNode, newTravelTime)
			}

			scannedByBackwardSearch = bScanned[wNeighborId]
			if scannedByBackwardSearch && da.Lt(fInfo[wNeighborId].GetTravelTime()+bInfo[wNeighborId].GetTravelTime(), fastestTT) {
				fastestTT = fInfo[wNeighborId].GetTravelTime() + bInfo[wNeighborId].GetTravelTime()
				mid = wNeighborId
			}
		})

		u, _ = bOverlayPq.ExtractMin()

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

			vAlreadyLabelled := labelled(bInfo, vOverlayId)
			if vAlreadyLabelled && da.Ge(newTravelTime, bInfo[vOverlayId].GetTravelTime()) {
				return
			}

			uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)

			bScanned[vOverlayId] = true
			bInfo[vOverlayId] = NewVertexInfo[da.Index](newTravelTime, newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
				uOverlayId, true), nil)

			scannedByForwardSearch := fScanned[vOverlayId]
			if scannedByForwardSearch && da.Lt(fInfo[vOverlayId].GetTravelTime()+bInfo[vOverlayId].GetTravelTime(), fastestTT) {
				fastestTT = fInfo[vOverlayId].GetTravelTime() + bInfo[vOverlayId].GetTravelTime()
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

			wAlreadyLabelled := labelled(bInfo, wNeighborId)
			if !wAlreadyLabelled {
				whNode := da.NewPriorityQueueNode(newTravelTime, wNeighborId)
				bInfo[wNeighborId] = NewVertexInfo(newTravelTime, newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true), whNode)
				bOverlayPq.Insert(whNode)
			} else {
				whNode := bInfo[wNeighborId].GetHeapNode()
				bInfo[wNeighborId].UpdateTravelTime(newTravelTime)
				bInfo[wNeighborId].UpdateParent(newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true))
				bOverlayPq.DecreaseKey(whNode, newTravelTime)
			}

			scannedByForwardSearch = fScanned[wNeighborId]
			if scannedByForwardSearch && da.Lt(fInfo[wNeighborId].GetTravelTime()+bInfo[wNeighborId].GetTravelTime(), fastestTT) {
				fastestTT = fInfo[wNeighborId].GetTravelTime() + bInfo[wNeighborId].GetTravelTime()
				mid = wNeighborId
			}
		})
	}

	overlayPath := make([]da.Index, 0, 8)

	overlayPath = append(overlayPath, mid)

	curOverlayId := fInfo[mid].GetParent().edge
	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = fInfo[curOverlayId].GetParent().edge
	}

	overlayPath = util.ReverseG(overlayPath)

	curOverlayId = bInfo[mid].GetParent().edge

	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = bInfo[curOverlayId].GetParent().edge
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
	lfInfo := make([]*VertexInfo[da.CRPQueryKey], maxSearchSize)
	lbInfo := make([]*VertexInfo[da.CRPQueryKey], maxSearchSize)
	initInfWeightVertexInfo(lfInfo)
	initInfWeightVertexInfo(lbInfo)
	fpq := da.NewFourAryHeap[da.CRPQueryKey]()
	bpq := da.NewFourAryHeap[da.CRPQueryKey]()
	fpq.Preallocate(int(maxSearchSize))
	bpq.Preallocate(int(maxSearchSize))

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
	fpq.Insert(shNode)

	thNode := da.NewPriorityQueueNode(0, da.NewCRPQueryKeyWithOutInEdgeId(tVId,
		offTargetExitId, tOutEdge.GetEdgeId()))
	bpq.Insert(thNode)

	lfInfo[offSourceEntryId] = NewVertexInfo(0, newVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false), shNode)

	lbInfo[offTargetExitId] = NewVertexInfo(0, newVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false), thNode)

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
		if da.Ge(minForward+minBackward, (fastestTT)) {
			break
		}

		queryKey, _ := fpq.ExtractMin()

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

			vAlreadyLabelled := da.Lt(lfInfo[offVEntryId].GetTravelTime(), pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newTravelTime, lfInfo[offVEntryId].GetTravelTime()) {
				return
			}

			if !vAlreadyLabelled {
				vhNode := da.NewPriorityQueueNode(newTravelTime,
					da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, e.GetEdgeId()))
				lfInfo[offVEntryId] = NewVertexInfo(newTravelTime, newVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false), vhNode)

				fpq.Insert(vhNode)
			} else {
				vhNode := lfInfo[offVEntryId].GetHeapNode()
				lfInfo[offVEntryId].UpdateTravelTime(newTravelTime)
				lfInfo[offVEntryId].UpdateParent(newVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))
				fpq.DecreaseKey(vhNode, newTravelTime)
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
				if scannedByBackwardSearch && da.Lt(lfInfo[offVEntryId].GetTravelTime()+pu.engine.metrics.GetTurnCost(turnType2)+
					lbInfo[offVExitId].GetTravelTime(), fastestTT) {

					fastestTT = lfInfo[offVEntryId].GetTravelTime() + pu.engine.metrics.GetTurnCost(turnType2) +
						lbInfo[offVExitId].GetTravelTime()

					fMid = pu.engine.adjustForward(vId, offVEntryId)
					bMid = pu.engine.adjustBackward(vId, offVExitId)
					offFMid = offVEntryId
					offBMid = offVExitId

				}
				offVExitId++
			})

		})

		// backward search
		queryKey, _ = bpq.ExtractMin()

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

			vAlreadyLabelled := da.Lt(lbInfo[offVExitId].GetTravelTime(), pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newTravelTime, lbInfo[offVExitId].GetTravelTime()) {
				return
			}

			if !vAlreadyLabelled {
				_, outEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(e.GetEdgeId())
				vhNode := da.NewPriorityQueueNode(newTravelTime,
					da.NewCRPQueryKeyWithOutInEdgeId(vId, offVExitId, outEdge.GetEdgeId()))
				lbInfo[offVExitId] = NewVertexInfo(newTravelTime, newVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false), vhNode)

				bpq.Insert(vhNode)
			} else {
				vhNode := lbInfo[offVExitId].GetHeapNode()
				lbInfo[offVExitId].UpdateTravelTime(newTravelTime)
				lbInfo[offVExitId].UpdateParent(newVertexEdgePairWithOutEdgeId(uId, uExitId, uOutEdgeId, false))
				bpq.DecreaseKey(vhNode, newTravelTime)
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
				if scannedByForwardSearch && da.Lt(lfInfo[offVEntryId].GetTravelTime()+pu.engine.metrics.GetTurnCost(turnType2)+
					lbInfo[offVExitId].GetTravelTime(), fastestTT) {

					fastestTT = lfInfo[offVEntryId].GetTravelTime() + pu.engine.metrics.GetTurnCost(turnType2) +
						lbInfo[offVExitId].GetTravelTime()

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
	for lfInfo[uId].parent.getEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := lfInfo[uId].parent.getOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)
		prevOutEdge := *pu.engine.graph.GetOutEdge(prevOutEdgeId)

		outEdges = append(outEdges, prevOutEdge)

		pEId := lfInfo[uId].parent.getEdge()

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
	for lbInfo[uId].parent.getEdge() != da.INVALID_EDGE_ID { // sampai parent.edge = targetEntry, include sp edges didalam current cell & sp edge exit cell ini
		prevOutEdgeId := lbInfo[uId].parent.getOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)
		prevOutEdge := *pu.engine.graph.GetOutEdge(prevOutEdgeId)

		outEdges = append(outEdges, prevOutEdge)

		pEId := lbInfo[uId].parent.getEdge()

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

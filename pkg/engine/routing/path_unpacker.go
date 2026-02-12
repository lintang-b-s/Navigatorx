package routing

import (
	"fmt"

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

	info      []*VertexInfo[da.Index]
	lInfo     []*VertexInfo[da.CRPQueryKey]
	pq        *da.MinHeap[da.CRPQueryKey]
	overlayPq *da.MinHeap[da.Index]
	puCache   *lru.Cache[PUCacheKey, []da.Index]
	useCache  bool
	oneToMany bool
}

func NewPathUnpacker(engine *CRPRoutingEngine, metrics *metrics.Metric,
	puCache *lru.Cache[PUCacheKey, []da.Index], useCache, oneToMany bool) *PathUnpacker {
	return &PathUnpacker{
		engine:  engine,
		metrics: metrics,

		info:      make([]*VertexInfo[da.Index], 0),
		lInfo:     make([]*VertexInfo[da.CRPQueryKey], 0),
		pq:        da.NewFourAryHeap[da.CRPQueryKey](),
		overlayPq: da.NewFourAryHeap[da.Index](),
		puCache:   puCache,
		useCache:  useCache,
		oneToMany: oneToMany,
	}
}

/*
unpackPath. unpack a level-i shortcut (v, w) by running Dijkstra between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
*/
func (pu *PathUnpacker) unpackPath(packedPath []vertexEdgePair, sCellNumber, tCellNumber da.Pv) ([]da.Coordinate, []da.OutEdge, float64) {
	unpackedPath := make([]da.Coordinate, 0, 50)
	unpackedEdgePath := make([]da.OutEdge, 0, 50)
	totalDistance := 0.0
	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.getEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)
			edgeGeometry := pu.engine.graph.GetEdgeGeometry(cur.getEdge())
			unpackedPath = append(unpackedPath, edgeGeometry...)

			outEdge := pu.engine.graph.GetOutEdge(cur.getEdge())
			unpackedEdgePath = append(unpackedEdgePath, *outEdge)
			totalDistance += outEdge.GetLength()

			i++
		} else {
			// overlay vertex
			entryVertex := offBit(cur.getEdge(), UNPACK_OVERLAY_OFFSET)

			entryCellNumber := pu.engine.overlayGraph.GetVertex(entryVertex).GetCellNumber()
			var queryLevel uint8

			if !pu.oneToMany {
				queryLevel = pu.engine.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)
				if queryLevel == 0 {
					fmt.Printf("debug")
				}
			} else {
				queryLevel = cur.getQueryLevel()
				if queryLevel == 0 {
					fmt.Printf("debug")
				}
			}
			if queryLevel == 0 {
				fmt.Printf("debug")
			}
			exitVertex := offBit(packedPath[i+1].getEdge(), UNPACK_OVERLAY_OFFSET)

			pu.unpackInLevelCell(entryVertex, exitVertex, queryLevel, &unpackedPath, &unpackedEdgePath, &totalDistance)

			i += 2
		}
	}

	unpackedEdgePath = removeDuplicates(unpackedEdgePath)

	// todo: polyline simplification unpackedPath
	return unpackedPath, unpackedEdgePath, totalDistance
}

func (pu *PathUnpacker) unpackInLevelCell(sourceOverlayId, targetOverlayId da.Index, level uint8, unpackedPath *[]da.Coordinate,
	unpackedEdgePath *[]da.OutEdge, distance *float64) {

	if level == 1 {
		sourceEntryId := pu.engine.overlayGraph.GetVertex(sourceOverlayId).GetOriginalEdge()
		neighborOfTarget := pu.engine.overlayGraph.GetVertex(targetOverlayId).GetNeighborOverlayVertex()
		targetEntryId := pu.engine.overlayGraph.GetVertex(neighborOfTarget).GetOriginalEdge()

		pu.unpackInLowestLevelCell(sourceEntryId, targetEntryId, unpackedPath, unpackedEdgePath, distance,
			sourceOverlayId, targetOverlayId)
		return
	}

	if pu.useCache {
		if overlayPath, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {
			// fetch from cache, cuma dipakai di server time-independent
			// buat tests, gak ambil dari cache
			for i := 0; i < len(overlayPath); i += 2 {
				pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, unpackedPath, unpackedEdgePath, distance)
			}
			return
		}
	}

	maxSearchSize := pu.engine.overlayGraph.NumberOfOverlayVertices()
	pu.info = make([]*VertexInfo[da.Index], maxSearchSize)
	initInfWeightVertexInfo(pu.info)
	pu.overlayPq.Preallocate(maxSearchSize)

	sourceCellNumber := pu.engine.overlayGraph.GetVertex(sourceOverlayId).GetCellNumber()
	truncatedSourceCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, level)

	targetCellNumber := pu.engine.overlayGraph.GetVertex(targetOverlayId).GetCellNumber()
	truncatedTargetCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(targetCellNumber, level)

	util.AssertPanic(truncatedSourceCellNumber == truncatedTargetCellNumber, "cell number/id dari sourceOverlay vertex dan targetOverlay vertex haruslah sama")

	shNode := da.NewPriorityQueueNode(0, sourceOverlayId)
	pu.overlayPq.Insert(shNode)

	pu.info[sourceOverlayId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false), shNode)

	for pu.overlayPq.Size() != 0 {
		u, _ := pu.overlayPq.ExtractMin()

		uOverlayId := u.GetItem()

		uTravelTime := u.GetRank()
		if uOverlayId == targetOverlayId {
			break
		}

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.engine.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId da.Index, wOffset da.Index) {

			shortcutOutEdgeWeight := pu.metrics.GetShortcutWeight(wOffset)

			newTravelTime := uTravelTime + shortcutOutEdgeWeight
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := da.Lt(pu.info[vOverlayId].GetTravelTime(), pkg.INF_WEIGHT)

			if vAlreadyLabelled && da.Ge(newTravelTime, pu.info[vOverlayId].GetTravelTime()) {
				return
			}

			uOverlayVertex := pu.engine.overlayGraph.GetVertex(uOverlayId)
			vhNode := da.NewPriorityQueueNode(newTravelTime, vOverlayId)
			pu.info[vOverlayId] = NewVertexInfo(newTravelTime, newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
				uOverlayId, true), vhNode)
			if vOverlayId == targetOverlayId {
				// if v is the target overlay vertex, update the pq
				if !vAlreadyLabelled {

					pu.overlayPq.Insert(vhNode)
				} else {
					vhNode = pu.info[vOverlayId].GetHeapNode()
					pu.info[vOverlayId].UpdateTravelTime(newTravelTime)
					pu.info[vOverlayId].UpdateParent(newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
						uOverlayId, true))
					pu.overlayPq.DecreaseKey(vhNode, newTravelTime)
				}
			}

			// visit next cell neighbor
			wNeighborId := pu.engine.overlayGraph.GetVertex(vOverlayId).GetNeighborOverlayVertex()
			wCellNumber := pu.engine.overlayGraph.GetVertex(wNeighborId).GetCellNumber()
			truncatedWCellNumber := pu.engine.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
			if truncatedWCellNumber != truncatedSourceCellNumber {
				// if w is not in the same cell as sourceOverlayId in level l, dont visit w
				return
			}

			// get out edge that point to wEntryVertex from vOverlayId
			vOverlayVertex := pu.engine.overlayGraph.GetVertex(vOverlayId)
			vOutEdge := pu.engine.graph.GetOutEdge(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vOutEdge)

			wAlreadyLabelled := da.Lt(pu.info[wNeighborId].GetTravelTime(), pkg.INF_WEIGHT)
			if !wAlreadyLabelled {
				whNode := da.NewPriorityQueueNode(newTravelTime, wNeighborId)
				pu.info[wNeighborId] = NewVertexInfo(newTravelTime, newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true), whNode)
				pu.overlayPq.Insert(whNode)
			} else {
				whNode := pu.info[wNeighborId].GetHeapNode()
				pu.info[wNeighborId].UpdateTravelTime(newTravelTime)
				pu.info[wNeighborId].UpdateParent(newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
					vOverlayId, true))
				pu.overlayPq.DecreaseKey(whNode, newTravelTime)
			}
		})
	}

	overlayPath := make([]da.Index, 0, 8)
	curOverlayId := targetOverlayId // targetOverlayId = exit vertex

	for curOverlayId != da.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pu.info[curOverlayId].GetParent().edge
	}

	pu.overlayPq.Clear()

	// reverse
	overlayPath = util.ReverseG[da.Index](overlayPath)

	util.AssertPanic(len(overlayPath)%2 == 0, "harusnya len(overlayPath) genap")

	if pu.useCache {
		pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath)
	}
	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]

		pu.unpackInLevelCell(curV, nextV, level-1, unpackedPath, unpackedEdgePath, distance)
	}
}

func (pu *PathUnpacker) unpackInLowestLevelCell(sourceEntryId, targetEntryId da.Index,
	unpackedPath *[]da.Coordinate, unpackedEdgePath *[]da.OutEdge, distance *float64,
	sourceOverlayId, targetOverlayId da.Index) {

	if pu.useCache {
		if edgeIds, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
			// fetch from cache
			for _, edgeId := range edgeIds {
				edge := *pu.engine.graph.GetOutEdge(edgeId)
				edgeGeometry := pu.engine.graph.GetEdgeGeometry(edgeId)

				*unpackedPath = append(*unpackedPath, edgeGeometry...)
				*unpackedEdgePath = append(*unpackedEdgePath, edge)
				*distance += edge.GetLength()
			}

			return
		}
	}

	maxEdgesInCell := pu.engine.graph.GetMaxEdgesInCell()
	maxSearchSize := int(maxEdgesInCell) * 2 //  all edges in this cell + targetEntryId

	// sourceEntryId inEdge that point to source vertex
	pu.lInfo = make([]*VertexInfo[da.CRPQueryKey], maxSearchSize)
	initInfWeightVertexInfo(pu.lInfo)
	pu.pq.Preallocate(int(maxSearchSize))

	// get source vertex
	sourceVertex := pu.engine.graph.GetVertex(pu.engine.graph.GetHeadFromInEdge(sourceEntryId))
	_, outEdge := pu.engine.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)

	sourceCellNumber := pu.engine.graph.GetCellNumber(sourceVertex.GetID())

	offSourceEntryId := pu.engine.offsetForward(sourceVertex.GetID(), sourceEntryId, sourceCellNumber, sourceCellNumber)

	shNode := da.NewPriorityQueueNode(0, da.NewCRPQueryKeyWithOutInEdgeId(sourceVertex.GetID(),
		offSourceEntryId, outEdge.GetEdgeId()))
	pu.pq.Insert(shNode)

	pu.lInfo[offSourceEntryId] = NewVertexInfo(0, newVertexEdgePairWithOutEdgeId(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID,
		da.INVALID_EDGE_ID, false), shNode)

	for pu.pq.Size() != 0 {
		queryKey, _ := pu.pq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryId := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutInEdgeId()

		adjuEntryId := pu.engine.adjustForward(uId, uEntryId)
		if adjuEntryId == targetEntryId {
			break
		}

		// relax all out edges of u
		pu.engine.graph.ForOutEdgesOf(uId, pu.engine.graph.GetEntryOrder(uId, adjuEntryId), func(e *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
			vId := e.GetHead()

			vEntryId := pu.engine.graph.GetEntryOffset(vId) + da.Index(e.GetEntryPoint())
			edgeWeight := pu.metrics.GetWeight(e)

			newTravelTime := queryKey.GetRank() + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.engine.graph.GetCellNumber(vId) != sourceCellNumber && vEntryId != targetEntryId {
				// do not cross cell boundary except to the target entry point
				return
			}

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			offVEntryId := pu.engine.offsetForward(vId, vEntryId, pu.engine.graph.GetCellNumber(vId), sourceCellNumber)

			vAlreadyLabelled := da.Lt(pu.lInfo[offVEntryId].GetTravelTime(), pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newTravelTime, pu.lInfo[offVEntryId].GetTravelTime()) {
				return
			}

			if !vAlreadyLabelled {
				vhNode := da.NewPriorityQueueNode(newTravelTime,
					da.NewCRPQueryKeyWithOutInEdgeId(vId, offVEntryId, e.GetEdgeId()))
				pu.lInfo[offVEntryId] = NewVertexInfo(newTravelTime, newVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false), vhNode)

				pu.pq.Insert(vhNode)
			} else {
				vhNode := pu.lInfo[offVEntryId].GetHeapNode()
				pu.lInfo[offVEntryId].UpdateTravelTime(newTravelTime)
				pu.lInfo[offVEntryId].UpdateParent(newVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))
				pu.pq.DecreaseKey(vhNode, newTravelTime)
			}
		})
	}

	edgeIdPath := make([]da.Index, 0, 10)
	path := make([]da.Coordinate, 0, 10)

	targetVertex := pu.engine.graph.GetVertex(pu.engine.graph.GetHeadFromInEdge(targetEntryId))
	tVId := targetVertex.GetID()

	uId := pu.engine.offsetForward(tVId, targetEntryId, pu.engine.graph.GetCellNumber(tVId), sourceCellNumber)
	backwardEdges := make([]da.OutEdge, 0, 10)

	for pu.lInfo[uId].GetParent().edge != da.INVALID_EDGE_ID { // sampai parent.edge = sourceEntryId, include sp edges didalam current cell & sp edge entry cell ini
		prevOutEdgeId := pu.lInfo[uId].parent.getOutInEdgeId()

		edgeIdPath = append(edgeIdPath, prevOutEdgeId)
		prevOutEdge := *pu.engine.graph.GetOutEdge(prevOutEdgeId)

		backwardEdges = append(backwardEdges, prevOutEdge)
		edgeGeometry := pu.engine.graph.GetEdgeGeometry(prevOutEdgeId)
		revGeom := util.ReverseG(edgeGeometry)
		path = append(path, revGeom...)

		*distance += prevOutEdge.GetLength()

		pEId := pu.lInfo[uId].GetParent().edge

		uId = pEId
	}

	backwardEdges = util.ReverseG(backwardEdges)

	*unpackedEdgePath = append(*unpackedEdgePath, backwardEdges...)
	revEdgeIdPath := util.ReverseG(edgeIdPath)

	reversedPath := util.ReverseG[da.Coordinate](path)
	*unpackedPath = append(*unpackedPath, reversedPath...)

	pu.pq.Clear()

	if pu.useCache {
		pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), revEdgeIdPath)
	}
}

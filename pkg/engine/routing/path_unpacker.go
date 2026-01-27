package routing

import (
	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PUCacheKey struct {
	startOverlayId  datastructure.Index
	targetOverlayId datastructure.Index
	level           int
}

func NewPUCacheKey(start, target datastructure.Index, level int) PUCacheKey {
	return PUCacheKey{start, target, level}
}

type PathUnpacker struct {
	graph        *datastructure.Graph
	overlayGraph *datastructure.OverlayGraph
	metrics      *metrics.Metric

	info      map[datastructure.Index]VertexInfo
	pq        *datastructure.MinHeap[datastructure.CRPQueryKey]
	overlayPq *datastructure.MinHeap[datastructure.Index]
	puCache   *lru.Cache[PUCacheKey, []datastructure.Index]
	crpInfo   map[datastructure.Index]VertexInfo
}

func NewPathUnpacker(graph *datastructure.Graph, overlayGraph *datastructure.OverlayGraph, metrics *metrics.Metric,
	puCache *lru.Cache[PUCacheKey, []datastructure.Index]) *PathUnpacker {
	return &PathUnpacker{
		graph:        graph,
		overlayGraph: overlayGraph,
		metrics:      metrics,

		info:      make(map[datastructure.Index]VertexInfo),
		pq:        datastructure.NewFourAryHeap[datastructure.CRPQueryKey](),
		overlayPq: datastructure.NewFourAryHeap[datastructure.Index](),
		puCache:   puCache,
		crpInfo:   make(map[datastructure.Index]VertexInfo),
	}
}

/*
unpackPath. unpack a level-i shortcut (v, w) by running Dijkstra between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
*/
func (pu *PathUnpacker) unpackPath(packedPath []vertexEdgePair, sCellNumber, tCellNumber datastructure.Pv) ([]datastructure.Coordinate, []datastructure.OutEdge, float64) {
	unpackedPath := make([]datastructure.Coordinate, 0)
	unpackedEdgePath := make([]datastructure.OutEdge, 0, 50)
	totalDistance := 0.0
	for i := 0; i < len(packedPath); {
		cur := packedPath[i]
		if !isBitOn(cur.getEdge(), UNPACK_OVERLAY_OFFSET) {
			// original vertex (non-overlay vertex)
			edgeGeometry := pu.graph.GetEdgeGeometry(cur.getEdge())
			unpackedPath = append(unpackedPath, edgeGeometry...)

			outEdge := pu.graph.GetOutEdge(cur.getEdge())
			unpackedEdgePath = append(unpackedEdgePath, *outEdge)
			totalDistance += outEdge.GetLength()

			i++
		} else {
			// overlay vertex
			entryVertex := offBit(cur.getEdge(), UNPACK_OVERLAY_OFFSET)

			entryCellNumber := pu.overlayGraph.GetVertex(entryVertex).GetCellNumber()
			queryLevel := pu.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			exitVertex := offBit(packedPath[i+1].getEdge(), UNPACK_OVERLAY_OFFSET)

			pu.unpackInLevelCell(entryVertex, exitVertex, int(queryLevel), &unpackedPath, &unpackedEdgePath, &totalDistance, cur.getTime(),
				cur.getTravelTime())

			i += 2
		}
	}

	return unpackedPath, unpackedEdgePath, totalDistance
}

func (pu *PathUnpacker) unpackInLevelCell(sourceOverlayId, targetOverlayId datastructure.Index, level int, unpackedPath *[]datastructure.Coordinate,
	unpackedEdgePath *[]datastructure.OutEdge, distance *float64, tSec, stravelTime float64) {

	if level == 1 {
		sourceEntryId := pu.overlayGraph.GetVertex(sourceOverlayId).GetOriginalEdge()
		neighborOfTarget := pu.overlayGraph.GetVertex(targetOverlayId).GetNeighborOverlayVertex()
		targetEntryId := pu.overlayGraph.GetVertex(neighborOfTarget).GetOriginalEdge()
		pu.unpackInLowestLevelCell(sourceEntryId, targetEntryId, unpackedPath, unpackedEdgePath, distance,
			sourceOverlayId, targetOverlayId, tSec, stravelTime)
		return
	}

	if overlayPath, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, level)); ok {
		// fetch from cache, cuma dipakai di server
		// buat tests, gak ambil dari cache
		for i := 0; i < len(overlayPath); i += 2 {
			pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, unpackedPath, unpackedEdgePath, distance, tSec, stravelTime)
		}
		return
	}

	pu.info = make(map[datastructure.Index]VertexInfo)

	sourceCellNumber := pu.overlayGraph.GetVertex(sourceOverlayId).GetCellNumber()
	truncatedSourceCellNumber := pu.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, uint8(level))

	pu.info[sourceOverlayId] = NewVertexInfo(tSec, newVertexEdgePair(datastructure.INVALID_VERTEX_ID, datastructure.INVALID_EDGE_ID, false))
	pu.overlayPq.Insert(datastructure.NewPriorityQueueNode(tSec, sourceOverlayId))

	for pu.overlayPq.Size() != 0 {
		u, _ := pu.overlayPq.ExtractMin()
		uTime := u.GetRank()

		uOverlayId := u.GetItem()

		uTravelTime := u.GetRank()
		if uOverlayId == targetOverlayId {
			break
		}

		// traverse all out neighbor of u in level l-1 in the same cell as u
		pu.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId datastructure.Index, wOffset datastructure.Index) {

			shortcutOutEdgeWeight := pu.metrics.GetShortcutWeight(wOffset, uTime)

			newTravelTime := uTravelTime + shortcutOutEdgeWeight
			_, vAlreadyVisited := pu.info[vOverlayId]

			if vAlreadyVisited && newTravelTime >= pu.info[vOverlayId].GetTravelTime() {
				return
			}

			uOverlayVertex := pu.overlayGraph.GetVertex(uOverlayId)
			pu.info[vOverlayId] = NewVertexInfo(newTravelTime, newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
				uOverlayId, true))

			if vOverlayId == targetOverlayId {
				// if v is the target overlay vertex, update the pq
				if !vAlreadyVisited {
					pu.overlayPq.Insert(datastructure.NewPriorityQueueNode(newTravelTime, vOverlayId))
				} else {
					pu.overlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(newTravelTime, vOverlayId))
				}
			}

			// visit next cell neighbor
			wNeighborId := pu.overlayGraph.GetVertex(vOverlayId).GetNeighborOverlayVertex()
			wCellNumber := pu.overlayGraph.GetVertex(wNeighborId).GetCellNumber()
			truncatedWCellNumber := pu.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
			if truncatedWCellNumber != truncatedSourceCellNumber {
				// if w is not in the same cell as sourceOverlayId in level l, dont visit w
				return
			}

			// get out edge that point to wEntryVertex from vOverlayId
			vOverlayVertex := pu.overlayGraph.GetVertex(vOverlayId)
			vOutEdge := pu.graph.GetOutEdge(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vOutEdge, newTravelTime)

			_, wAlreadyVisited := pu.info[wNeighborId]
			pu.info[wNeighborId] = NewVertexInfo(newTravelTime, newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
				vOverlayId, true))
			if !wAlreadyVisited {
				pu.overlayPq.Insert(datastructure.NewPriorityQueueNode(newTravelTime, wNeighborId))
			} else {
				pu.overlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(newTravelTime, wNeighborId))
			}
		})
	}

	overlayPath := make([]datastructure.Index, 0, 8)
	curOverlayId := targetOverlayId // targetOverlayId = exit vertex

	for curOverlayId != datastructure.INVALID_EDGE_ID {
		overlayPath = append(overlayPath, curOverlayId)
		curOverlayId = pu.info[curOverlayId].GetParent().edge
	}

	pu.overlayPq.Clear()

	// reverse
	overlayPath = util.ReverseG[datastructure.Index](overlayPath)

	util.AssertPanic(len(overlayPath)%2 == 0, "harusnya len(overlayPath) genap")

	pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, level), overlayPath)
	for i := 0; i < len(overlayPath); i += 2 {
		curV := overlayPath[i]
		nextV := overlayPath[i+1]
		piTime := pu.info[curV].GetTravelTime()

		curVTT := pu.info[curV].GetTravelTime() + stravelTime
		pu.unpackInLevelCell(curV, nextV, level-1, unpackedPath, unpackedEdgePath, distance, piTime, curVTT)
	}
}

func (pu *PathUnpacker) unpackInLowestLevelCell(sourceEntryId, targetEntryId datastructure.Index,
	unpackedPath *[]datastructure.Coordinate, unpackedEdgePath *[]datastructure.OutEdge, distance *float64,
	sourceOverlayId, targetOverlayId datastructure.Index, tSec, stravelTime float64) {
	if edgeIds, ok := pu.puCache.Get(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1)); ok {
		// fetch from cache
		for _, edgeId := range edgeIds {
			edge := *pu.graph.GetOutEdge(edgeId)
			edgeGeometry := pu.graph.GetEdgeGeometry(edgeId)

			*unpackedPath = append(*unpackedPath, edgeGeometry...)
			*unpackedEdgePath = append(*unpackedEdgePath, edge)
			*distance += edge.GetLength()
		}

		return
	}

	// sourceEntryId inEdge that point to source vertex
	pu.info = make(map[datastructure.Index]VertexInfo)
	// get source vertex
	sourceVertex := pu.graph.GetVertex(pu.graph.GetHeadFromInEdge(sourceEntryId))
	_, outEdge := pu.graph.GetHeadOfInedgeWithOutEdge(sourceEntryId)

	sourceCellNumber := pu.graph.GetCellNumber(sourceVertex.GetID())

	pu.info[sourceEntryId] = NewVertexInfo(tSec, newVertexEdgePairWithOutEdgeId(datastructure.INVALID_VERTEX_ID, datastructure.INVALID_EDGE_ID,
		datastructure.INVALID_EDGE_ID, false))

	pu.pq.Insert(datastructure.NewPriorityQueueNode(tSec, datastructure.NewCRPQueryKeyWithOutInEdgeId(sourceVertex.GetID(),
		sourceEntryId, outEdge.GetEdgeId())))

	for pu.pq.Size() != 0 {
		queryKey, _ := pu.pq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uTime := queryKey.GetRank()
		uEntryId := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutInEdgeId()

		if uEntryId == targetEntryId {
			break
		}

		// relax all out edges of u
		pu.graph.ForOutEdgesOf(uId, pu.graph.GetEntryOrder(uId, uEntryId), func(e *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
			vId := e.GetHead()

			vEntryId := pu.graph.GetEntryOffset(vId) + datastructure.Index(e.GetEntryPoint())
			edgeWeight := pu.metrics.GetWeight(e, uTime)

			newTravelTime := queryKey.GetRank() + edgeWeight + pu.metrics.GetTurnCost(turnType)

			if pu.graph.GetCellNumber(vId) != sourceCellNumber && vEntryId != targetEntryId {
				// do not cross cell boundary except to the target entry point
				return
			}

			if newTravelTime >= pkg.INF_WEIGHT {
				return
			}

			_, vAlreadyVisited := pu.info[vEntryId]

			if vAlreadyVisited && newTravelTime >= pu.info[vEntryId].GetTravelTime() {
				return
			}

			pu.info[vEntryId] = NewVertexInfo(newTravelTime, newVertexEdgePairWithOutEdgeId(uId, uEntryId, uOutEdgeId, false))
			if !vAlreadyVisited {
				pu.pq.Insert(datastructure.NewPriorityQueueNode(newTravelTime,
					datastructure.NewCRPQueryKeyWithOutInEdgeId(vId, vEntryId, e.GetEdgeId())))
			} else {
				pu.pq.DecreaseKey(datastructure.NewPriorityQueueNode(newTravelTime,
					datastructure.NewCRPQueryKeyWithOutInEdgeId(vId, vEntryId, e.GetEdgeId())))
			}
		})
	}

	edgeIdPath := make([]datastructure.Index, 0, 10)
	path := make([]datastructure.Coordinate, 0, 10)
	uId := targetEntryId
	backwardEdges := make([]datastructure.OutEdge, 0)

	for pu.info[uId].GetParent().edge != sourceEntryId { // sampai parent.edge = sourceEntryId, cuma include sp edges didalam current cell
		prevEdgeId := pu.info[uId].GetParent().outInEdgeId

		edgeIdPath = append(edgeIdPath, prevEdgeId)
		prevOutEdge := *pu.graph.GetOutEdge(prevEdgeId)

		backwardEdges = append(backwardEdges, prevOutEdge)
		edgeGeometry := pu.graph.GetEdgeGeometry(prevEdgeId)
		revGeom := util.ReverseG(edgeGeometry)
		path = append(path, revGeom...)

		*distance += prevOutEdge.GetLength()

		curInfoS := pu.info[prevEdgeId]
		curInfoS.travelTime += stravelTime
		pu.crpInfo[prevEdgeId] = curInfoS

		uId = pu.info[uId].GetParent().edge
	}

	backwardEdges = util.ReverseG(backwardEdges)

	*unpackedEdgePath = append(*unpackedEdgePath, backwardEdges...)
	revEdgeIdPath := util.ReverseG(edgeIdPath)

	reversedPath := util.ReverseG[datastructure.Coordinate](path)
	*unpackedPath = append(*unpackedPath, reversedPath...)

	pu.pq.Clear()

	pu.puCache.Add(NewPUCacheKey(sourceOverlayId, targetOverlayId, 1), revEdgeIdPath)
}

func (pu *PathUnpacker) GetCRPInfo() map[datastructure.Index]VertexInfo {
	return pu.crpInfo
}

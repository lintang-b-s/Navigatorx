package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type PathUnpacker struct {
	graph        *datastructure.Graph
	overlayGraph *datastructure.OverlayGraph
	metrics      *metrics.Metric

	info      map[datastructure.Index]VertexInfo
	pq        *datastructure.MinHeap[datastructure.CRPQueryKey]
	overlayPq *datastructure.MinHeap[datastructure.Index]
}

func NewPathUnpacker(graph *datastructure.Graph, overlayGraph *datastructure.OverlayGraph, metrics *metrics.Metric) *PathUnpacker {
	return &PathUnpacker{
		graph:        graph,
		overlayGraph: overlayGraph,
		metrics:      metrics,

		info:      make(map[datastructure.Index]VertexInfo),
		pq:        datastructure.NewMinHeap[datastructure.CRPQueryKey](),
		overlayPq: datastructure.NewMinHeap[datastructure.Index](),
	}
}

/*
unpackPath. unpack a level-i shortcut (v, w) by running Dijkstra between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
*/
func (pu *PathUnpacker) unpackPath(packedPath []vertexEdgePair, sCellNumber, tCellNumber datastructure.Pv, unpackOverlayOffset datastructure.Index) ([]datastructure.Coordinate, []datastructure.OutEdge, float64) {
	unpackedPath := make([]datastructure.Coordinate, 0)
	unpackedEdgePath := make([]datastructure.OutEdge, 0, 50)
	totalDistance := 0.0
	for i := 0; i < len(packedPath)-1; {
		cur := packedPath[i]
		if cur.getEdge() < unpackOverlayOffset {
			// original vertex (non-overlay vertex)
			edgeGeometry := pu.graph.GetEdgeGeometry(cur.getEdge())
			unpackedPath = append(unpackedPath, edgeGeometry...)

			unpackedEdgePath = append(unpackedEdgePath, *pu.graph.GetOutEdge(cur.getEdge()))
			if cur.isOut() {
				totalDistance += pu.graph.GetOutEdge(cur.getEdge()).GetLength()
			} else {
				totalDistance += pu.graph.GetInEdge(cur.getEdge()).GetLength()
			}
			i++
		} else {
			// overlay vertex
			entryVertex := cur.getEdge() - unpackOverlayOffset

			entryCellNumber := pu.overlayGraph.GetVertex(entryVertex).GetCellNumber()
			queryLevel := pu.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			maxBoundaryVeticeLevel := pu.overlayGraph.GetVertex(entryVertex).GetEntryPointSize()
			if maxBoundaryVeticeLevel < queryLevel {
				queryLevel = maxBoundaryVeticeLevel
			}

			exitVertex := packedPath[i+1].getEdge() - unpackOverlayOffset

			pu.unpackInLevelCell(entryVertex, exitVertex, int(queryLevel), &unpackedPath, &unpackedEdgePath, &totalDistance)
			i += 2
		}
	}

	return unpackedPath, unpackedEdgePath, totalDistance
}

func (pu *PathUnpacker) unpackInLevelCell(sourceOverlayId, targetOverlayId datastructure.Index, level int, unpackedPath *[]datastructure.Coordinate,
	unpackedEdgePath *[]datastructure.OutEdge, distance *float64) {
	pu.info = make(map[datastructure.Index]VertexInfo)
	if level == 1 {
		sourceEntryPoint := pu.overlayGraph.GetVertex(sourceOverlayId).GetOriginalEdge()
		neighborOfTarget := pu.overlayGraph.GetVertex(targetOverlayId).GetNeighborOverlayVertex()
		targetEntryPoint := pu.overlayGraph.GetVertex(neighborOfTarget).GetOriginalEdge()
		pu.unpackInLowestLevelCell(sourceEntryPoint, targetEntryPoint, unpackedPath, unpackedEdgePath, distance)
		return
	}

	sourceCellNumber := pu.overlayGraph.GetVertex(sourceOverlayId).GetCellNumber()
	truncatedSourceCellNumber := pu.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, uint8(level))

	pu.info[sourceOverlayId] = NewVertexInfo(0, newVertexEdgePair(sourceOverlayId, sourceOverlayId, false))
	pu.overlayPq.Insert(datastructure.NewPriorityQueueNode(0, sourceOverlayId))

	for pu.overlayPq.Size() != 0 {
		u, _ := pu.overlayPq.ExtractMin()

		uOverlayId := u.GetItem()

		uTravelTime := u.GetRank()
		if uOverlayId == targetOverlayId {
			break
		}

		// traverse all out neighbor of u in level l in the same cell as u
		pu.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId datastructure.Index, wOffset datastructure.Index) {

			newTravelTime := uTravelTime + pu.metrics.GetShortcutWeight(wOffset)
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

			// visit next cell neighbor in the same level l-1
			wNeighborId := pu.overlayGraph.GetVertex(vOverlayId).GetNeighborOverlayVertex()
			wCellNumber := pu.overlayGraph.GetVertex(wNeighborId).GetCellNumber()
			truncatedWCellNumber := pu.overlayGraph.GetLevelInfo().TruncateToLevel(wCellNumber, uint8(level))
			if truncatedWCellNumber != truncatedSourceCellNumber {
				// if w is not in the same cell as sourceOverlayId in level l-1, dont visit w
				return
			}

			// get out edge that point to wEntryVertex from vOverlayId
			vOverlayVertex := pu.overlayGraph.GetVertex(vOverlayId)
			vOutEdge := pu.graph.GetOutEdge(vOverlayVertex.GetOriginalEdge())
			newTravelTime += pu.metrics.GetWeight(vOutEdge)

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
	curOverlayId := targetOverlayId
	overlayPath = append(overlayPath, curOverlayId)
	for curOverlayId != sourceOverlayId {

		parent := pu.info[curOverlayId].GetParent()
		curOverlayId = parent.edge
		overlayPath = append(overlayPath, curOverlayId)
	}

	pu.overlayPq.Clear()

	// reverse
	overlayPath = util.ReverseG[datastructure.Index](overlayPath)

	for i := 0; i < len(overlayPath)-1; i += 2 {

		pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, unpackedPath, unpackedEdgePath, distance)
	}
}

func (pu *PathUnpacker) unpackInLowestLevelCell(sourceEntryPoint, targetEntryPoint datastructure.Index,
	unpackedPath *[]datastructure.Coordinate, unpackedEdgePath *[]datastructure.OutEdge, distance *float64) {
	// sourceEntryPoint inEdge that point to source vertex
	pu.info = make(map[datastructure.Index]VertexInfo)
	// get source vertex
	sourceVertex := pu.graph.GetVertex(pu.graph.GetHeadFromInEdge(sourceEntryPoint))
	_, outEdge := pu.graph.GetHeadOfInedgeWithOutEdge(sourceEntryPoint)

	sourceCellNumber := pu.graph.GetCellNumber(sourceVertex.GetID())

	pu.info[sourceEntryPoint] = NewVertexInfo(0, newVertexEdgePairWithOutEdgeId(sourceVertex.GetID(), sourceEntryPoint,
		outEdge.GetEdgeId(), false))
	pu.pq.Insert(datastructure.NewPriorityQueueNode(0, datastructure.NewCRPQueryKeyWithOutEdgeId(sourceVertex.GetID(),
		sourceEntryPoint, outEdge.GetEdgeId())))

	for pu.pq.Size() != 0 {
		queryKey, _ := pu.pq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryPoint := uItem.GetEntryExitPoint()
		uOutEdgeId := uItem.GetOutEdgeId()

		if uEntryPoint == targetEntryPoint {
			break
		}

		// relax all out edges of u
		pu.graph.ForOutEdgesOf(uId, pu.graph.GetEntryOrder(uId, uEntryPoint), func(e *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
			vId := e.GetHead()

			vEntryPoint := pu.graph.GetEntryOffset(vId) + datastructure.Index(e.GetEntryPoint())
			newTravelTime := queryKey.GetRank() + pu.metrics.GetWeight(e) + pu.metrics.GetTurnCost(turnType)

			if pu.graph.GetCellNumber(vId) != sourceCellNumber && vEntryPoint != targetEntryPoint {
				// do not cross cell boundary except to the target entry point
				return
			}

			if newTravelTime >= pkg.INF_WEIGHT {
				return
			}

			_, vAlreadyVisited := pu.info[vEntryPoint]

			if vAlreadyVisited && newTravelTime >= pu.info[vEntryPoint].GetTravelTime() {
				return
			}

			pu.info[vEntryPoint] = NewVertexInfo(newTravelTime, newVertexEdgePairWithOutEdgeId(uId, uEntryPoint, uOutEdgeId, false))
			if !vAlreadyVisited {
				pu.pq.Insert(datastructure.NewPriorityQueueNode(newTravelTime,
					datastructure.NewCRPQueryKeyWithOutEdgeId(vId, vEntryPoint, e.GetEdgeId())))
			} else {
				pu.pq.DecreaseKey(datastructure.NewPriorityQueueNode(newTravelTime,
					datastructure.NewCRPQueryKeyWithOutEdgeId(vId, vEntryPoint, e.GetEdgeId())))
			}
		})
	}

	path := make([]datastructure.Coordinate, 0, 10)
	uId := targetEntryPoint
	currEdgePath := make([]datastructure.OutEdge, 0)
	first := true
	for pu.info[uId].GetParent().edge != sourceEntryPoint {
		prevEdgeId := pu.info[uId].GetParent().outEdgeId

		prevOutEdge := *pu.graph.GetOutEdge(prevEdgeId)
		currEdgePath = append(currEdgePath, prevOutEdge)
		edgeGeometry := pu.graph.GetEdgeGeometry(prevEdgeId)
		revEdgeGeometry := util.ReverseG[datastructure.Coordinate](edgeGeometry)
		if first {
			path = append(path, revEdgeGeometry...)
			first = false
		} else {
			path = append(path, revEdgeGeometry[1:]...)
		}

		*distance += prevOutEdge.GetLength()

		uId = pu.info[uId].GetParent().edge
	}

	lastEdgeGeometry := pu.graph.GetEdgeGeometry(pu.info[uId].GetParent().outEdgeId)
	revLastEdgeGeometry := util.ReverseG[datastructure.Coordinate](lastEdgeGeometry)
	path = append(path, revLastEdgeGeometry[1:]...)

	lastOutEdge := *pu.graph.GetOutEdge(pu.info[uId].GetParent().outEdgeId)
	currEdgePath = append(currEdgePath, lastOutEdge)
	*distance += lastOutEdge.GetLength()
	revCurrEdgePath := util.ReverseG(currEdgePath)
	*unpackedEdgePath = append(*unpackedEdgePath, revCurrEdgePath...)

	reversedPath := util.ReverseG[datastructure.Coordinate](path)
	*unpackedPath = append(*unpackedPath, reversedPath...)
	pu.pq.Clear()

}

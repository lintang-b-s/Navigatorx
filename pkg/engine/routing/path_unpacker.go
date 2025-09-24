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
	currentRound int
	round        map[datastructure.Index]int

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
		round:     make(map[datastructure.Index]int),
	}
}

/*
unpackPath. unpack a level-i shortcut (v, w) by running Dijkstra between v and w on level i − 1, restricted to subcells of the level-i cell containing the shortcut.
*/
func (pu *PathUnpacker) unpackPath(packedPath []vertexEdgePair, sCellNumber, tCellNumber datastructure.Pv) ([]datastructure.Index, float64) {
	unpackedPath := make([]datastructure.Index, 0)
	unpackOverlayOffset := datastructure.Index(pu.graph.NumberOfEdges()) * 5
	totalDistance := 0.0
	for i := 0; i < len(packedPath)-1; {
		cur := packedPath[i]
		if cur.getEdge() < unpackOverlayOffset {
			// original vertex (non-overlay vertex)
			unpackedPath = append(unpackedPath, cur.getVertex())
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

			exitVertex := packedPath[i+1].getEdge() - unpackOverlayOffset

			pu.unpackInLevelCell(entryVertex, exitVertex, int(queryLevel), &unpackedPath, &totalDistance)
			i += 2
		}
	}

	return unpackedPath, totalDistance
}

func (pu *PathUnpacker) unpackInLevelCell(sourceOverlayId, targetOverlayId datastructure.Index, level int, unpackedPath *[]datastructure.Index,
	distance *float64) {
	pu.info = make(map[datastructure.Index]VertexInfo)
	if level == 1 {
		sourceEntryPoint := pu.overlayGraph.GetVertex(sourceOverlayId).GetOriginalEdge()
		neighborOfTarget := pu.overlayGraph.GetVertex(targetOverlayId).GetNeighborOverlayVertex()
		targetEntryPoint := pu.overlayGraph.GetVertex(neighborOfTarget).GetOriginalEdge()
		pu.unpackInLowestLevelCell(sourceEntryPoint, targetEntryPoint, unpackedPath, distance)
		return
	}
	pu.currentRound++
	pu.round[sourceOverlayId] = pu.currentRound

	sourceCellNumber := pu.overlayGraph.GetVertex(sourceOverlayId).GetCellNumber()
	truncatedSourceCellNumber := pu.overlayGraph.GetLevelInfo().TruncateToLevel(sourceCellNumber, uint8(level))

	pu.info[sourceOverlayId] = NewVertexInfo(0, newVertexEdgePair(sourceOverlayId, sourceOverlayId, true))
	pu.overlayPq.Insert(datastructure.NewPriorityQueueNode(0, sourceOverlayId))

	for pu.overlayPq.Size() != 0 {
		u, _ := pu.overlayPq.ExtractMin()

		uOverlayId := u.GetItem()

		uEta := u.GetRank()
		if uOverlayId == targetOverlayId {
			break
		}

		// traverse all out neighbor of u in level l in the same cell as u
		pu.overlayGraph.ForOutNeighborsOf(uOverlayId, int(level-1), func(vOverlayId datastructure.Index, wOffset datastructure.Index) {

			newEta := uEta + pu.metrics.GetShortcutWeight(wOffset)
			_, vAlreadyVisited := pu.info[vOverlayId]

			if pu.round[vOverlayId] == pu.currentRound && newEta >= pu.info[vOverlayId].GetEta() {
				return
			}

			uOverlayVertex := pu.overlayGraph.GetVertex(uOverlayId)
			pu.info[vOverlayId] = NewVertexInfo(newEta, newVertexEdgePair(uOverlayVertex.GetOriginalVertex(),
				uOverlayId, true))
			pu.round[vOverlayId] = pu.currentRound

			if vOverlayId == targetOverlayId {
				// if v is the target overlay vertex, update the pq
				if !vAlreadyVisited {
					pu.overlayPq.Insert(datastructure.NewPriorityQueueNode(newEta, vOverlayId))
				} else {
					pu.overlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(newEta, vOverlayId))
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
			newEta += pu.metrics.GetWeight(vOutEdge)

			_, wAlreadyVisited := pu.info[wNeighborId]
			pu.info[wNeighborId] = NewVertexInfo(newEta, newVertexEdgePair(vOverlayVertex.GetOriginalVertex(),
				vOverlayId, true))
			pu.round[wNeighborId] = pu.currentRound
			if !wAlreadyVisited {
				pu.overlayPq.Insert(datastructure.NewPriorityQueueNode(newEta, wNeighborId))
			} else {
				pu.overlayPq.DecreaseKey(datastructure.NewPriorityQueueNode(newEta, wNeighborId))
			}
		})
	}

	overlayPath := make([]datastructure.Index, 0, 8)
	curOverlayId := targetOverlayId
	overlayPath = append(overlayPath, curOverlayId)
	for curOverlayId != sourceOverlayId {

		parent := pu.info[curOverlayId].GetParent()
		prevEdge := parent.getEdge()
		isOut := parent.isOut()
		if isOut {
			*distance += pu.graph.GetOutEdge(prevEdge).GetLength()
		} else {
			*distance += pu.graph.GetInEdge(prevEdge).GetLength()
		}
		curOverlayId = parent.edge
		overlayPath = append(overlayPath, curOverlayId)
	}

	pu.overlayPq.Clear()

	// reverse
	overlayPath = util.ReverseG[datastructure.Index](overlayPath)

	for i := 0; i < len(overlayPath)-1; i += 2 {

		pu.unpackInLevelCell(overlayPath[i], overlayPath[i+1], level-1, unpackedPath, distance)
	}
}

func (pu *PathUnpacker) unpackInLowestLevelCell(sourceEntryPoint, targetEntryPoint datastructure.Index,
	unpackedPath *[]datastructure.Index, distance *float64) {
	// sourceEntryPoint inEdge that point to source vertex

	// get source vertex
	sourceVertex := pu.graph.GetVertex(pu.graph.GetHeadFromInEdge(sourceEntryPoint))

	sourceCellNumber := pu.graph.GetCellNumber(sourceVertex.GetID())

	pu.currentRound++
	pu.round[sourceEntryPoint] = pu.currentRound
	pu.info[sourceEntryPoint] = NewVertexInfo(0, newVertexEdgePair(sourceVertex.GetID(), sourceEntryPoint, true))
	pu.pq.Insert(datastructure.NewPriorityQueueNode(0, datastructure.NewCRPQueryKey(sourceVertex.GetID(), sourceEntryPoint)))

	for pu.pq.Size() != 0 {
		queryKey, _ := pu.pq.ExtractMin()

		uItem := queryKey.GetItem()
		uId := uItem.GetNode()
		uEntryPoint := uItem.GetEntryExitPoint()

		if uEntryPoint == targetEntryPoint {
			break
		}

		// relax all out edges of u
		pu.graph.ForOutEdgesOf(uId, pu.graph.GetEntryOrder(uId, uEntryPoint), func(e *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
			vId := e.GetHead()

			vEntryPoint := pu.graph.GetEntryOffset(vId) + datastructure.Index(e.GetEntryPoint())
			newEta := queryKey.GetRank() + pu.metrics.GetWeight(e) + pu.metrics.GetTurnCost(turnType)

			if pu.graph.GetCellNumber(vId) != sourceCellNumber && vEntryPoint != targetEntryPoint {
				// do not cross cell boundary except to the target entry point
				return
			}

			if newEta > pkg.INF_WEIGHT {
				return
			}

			_, vAlreadyVisited := pu.info[vEntryPoint]

			if pu.round[vEntryPoint] == pu.currentRound && newEta >= pu.info[vEntryPoint].GetEta() {
				return
			}

			pu.info[vEntryPoint] = NewVertexInfo(newEta, newVertexEdgePair(uId, uEntryPoint, true))
			pu.round[vEntryPoint] = pu.currentRound
			if !vAlreadyVisited {
				pu.pq.Insert(datastructure.NewPriorityQueueNode(newEta, datastructure.NewCRPQueryKey(vId, vEntryPoint)))
			} else {
				pu.pq.DecreaseKey(datastructure.NewPriorityQueueNode(newEta, datastructure.NewCRPQueryKey(vId, vEntryPoint)))
			}
		})
	}

	path := make([]datastructure.Index, 0, 10)
	uId := targetEntryPoint
	for pu.info[uId].GetParent().edge != sourceEntryPoint {
		prevEdge := pu.info[uId].GetParent().edge
		isOut := pu.info[uId].GetParent().isOutEdge
		if isOut {
			*distance += pu.graph.GetOutEdge(prevEdge).GetLength()
		} else {
			*distance += pu.graph.GetInEdge(prevEdge).GetLength()
		}

		path = append(path, pu.info[uId].GetParent().vertex)
		uId = pu.info[uId].GetParent().edge
	}

	path = append(path, pu.info[uId].GetParent().vertex)

	pu.pq.Clear()

	*unpackedPath = append(*unpackedPath, util.ReverseG[datastructure.Index](path)...)
}

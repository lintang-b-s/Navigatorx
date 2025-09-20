package datastructure

import (
	"sync"

	"github.com/lintang-b-s/navigatorx-crp/pkg"
	"github.com/lintang-b-s/navigatorx-crp/pkg/costfunction"
)

type OverlayWeights struct {
	weights []float64
	lock    sync.Mutex
}

func (ow *OverlayWeights) GetWeight(i Index) float64 {

	shortcutWeight := ow.weights[i]

	return shortcutWeight
}

func (ow *OverlayWeights) GetWeights() []float64 {
	return ow.weights
}

func (ow *OverlayWeights) SetWeights(weights []float64) {

	copy(ow.weights, weights)
}

func NewOverlayWeights(weightVectorSize uint32) *OverlayWeights {
	return &OverlayWeights{weights: make([]float64, weightVectorSize)}
}

type customizerCell struct {
	cell       *Cell
	cellNumber Pv
}

func newCustomizerCell(cell *Cell, cellNumber Pv) customizerCell {
	return customizerCell{cell: cell, cellNumber: cellNumber}
}

/*
Build. Customizable Route Planning in Road Networks, Daniel Delling, et al. Page 11:

The customization phase has access to the actual cost function that must be optimized during queries.
Because we have the metric-independent data structures in place, all we need to do is compute the entries
of the above-mentioned array W , which represents the costs of all shortcuts between entry and exit vertices
within cells.
We compute these distances in a bottom-up fashion, one cell at a time. Consider a cell C in H1 (the
first overlay level). For each entry (overlay) vertex v in C, we run Dijkstra’s algorithm in G (restricted to
C) until the priority queue is empty. This computes the distances to all reachable exit vertices of C. Since
we work on the underlying graph, we must use the turn-aware implementation of Dijkstra, as explained in
Section 4.1.
A cell C at a higher level Hi (for i > 1) can be processed similarly, with one major difference. Instead of
working on the original graph, we can work on the subgraph of Hi−1 (the overlay level immediately below)
corresponding to subcells of C. This subgraph is much smaller than the corresponding subgraph of G. In
addition, since overlay graphs have no (explicit) turns, we can just apply the standard version of Dijkstra’s
algorithm, which tends to be faster.
*/
func (ow *OverlayWeights) Build(graph *Graph, overlayGraph *OverlayGraph,
	costFunction costfunction.CostFunction) {
	ow.buildLowestLevel(graph, overlayGraph, costFunction)
	for level := 2; level <= overlayGraph.GetLevelInfo().GetLevelCount(); level++ {
		ow.buildLevel(graph, overlayGraph, costFunction, level)
	}
}

// buildLowestLevel. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
func (ow *OverlayWeights) buildLowestLevel(graph *Graph, overlayGraph *OverlayGraph,
	costFunction costfunction.CostFunction) {
	// salah

	cellMapInLevelOne := overlayGraph.GetAllCellsInLevel(1)
	for cellNumber, cell := range cellMapInLevelOne {

		for i := Index(0); i < cell.numEntryPoints; i++ {
			startOverlayVertexId := overlayGraph.GetEntryPoint(cell, i)
			overlayVertex := overlayGraph.GetVertex(startOverlayVertexId)
			start := overlayVertex.originalVertex
			pq := NewMinHeap[CRPQueryKey]()
			eta := make(map[Index]float64)
			overlayEta := make(map[Index]float64)
			forwardCellOffset := graph.GetInEdgeCellOffset(start)
			startInEdgeOffset := overlayVertex.originalEdge - forwardCellOffset

			eta[startInEdgeOffset] = 0

			pq.Insert(NewPriorityQueueNode(0, NewCRPQueryKey(start, startInEdgeOffset)))

			for !pq.isEmpty() {
				pqNode, _ := pq.ExtractMin()
				uKey := pqNode.GetItem()
				uId := uKey.GetNode()
				uEntryPoint := uKey.GetEntryExitPoint()
				uEta := pqNode.GetRank()

				graph.ForOutEdgesOf(uId, graph.GetEntryOrder(uId, uEntryPoint+forwardCellOffset), func(outArc *OutEdge, exitPoint Index, turnType pkg.TurnType) {
					// traverse all out edges
					v := outArc.GetHead()

					exitPointEta := uEta + costFunction.GetTurnCost(turnType)
					outArcCost := costFunction.GetWeight(outArc)

					newETA := exitPointEta + outArcCost

					if newETA >= pkg.INF_WEIGHT {
						return
					}

					if graph.GetCellNumber(v) == cellNumber {
						vEntryPoint := graph.GetEntryOffset(v) + Index(outArc.GetEntryPoint()) - forwardCellOffset

						if _, ok := eta[vEntryPoint]; !ok || newETA < eta[vEntryPoint] {
							eta[vEntryPoint] = newETA
							if ok {
								pq.DecreaseKey(NewPriorityQueueNode(newETA, NewCRPQueryKey(v, vEntryPoint)))
							} else {
								pq.Insert(NewPriorityQueueNode(newETA, NewCRPQueryKey(v, vEntryPoint)))
							}
						}
					} else {
						// found an exit point of the cell
						// save this shortcut eta
						exitOverlay, _ := graph.GetOverlayVertex(uId, uint8(exitPoint), true)
						if _, ok := overlayEta[exitOverlay]; !ok || exitPointEta < overlayEta[exitOverlay] {
							overlayEta[exitOverlay] = exitPointEta
						}
					}
				})
			}

			// stores all eta of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
			for j := Index(0); j < cell.numExitPoints; j++ {
				exitPoint := overlayGraph.GetExitPoint(cell, j)
				ow.lock.Lock()
				_, exists := overlayEta[exitPoint]
				if !exists {
					ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = pkg.INF_WEIGHT
				} else {

					ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = overlayEta[exitPoint]
				}
				ow.lock.Unlock()
			}
		}
	}

}

// buildLevel. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
// this function uses overlay graph from the previous level to compute the weights
// basically: use shortcut edges from the previous level as edges in this current level overlay graph
func (ow *OverlayWeights) buildLevel(graph *Graph, overlayGraph *OverlayGraph,
	costFunction costfunction.CostFunction, level int) {

	levelInfo := overlayGraph.GetLevelInfo()

	buildCellClique := func(job customizerCell) any {

		cell := job.cell
		cellNumber := job.cellNumber

		for i := Index(0); i < cell.numEntryPoints; i++ {
			pq := NewMinHeap[Index]()
			eta := make(map[Index]float64)
			startOverlayVertexId := overlayGraph.GetEntryPoint(cell, i)

			eta[startOverlayVertexId] = 0

			pq.Insert(NewPriorityQueueNode(0, startOverlayVertexId))

			for !pq.isEmpty() {
				pqNode, _ := pq.ExtractMin()
				uOverlayId := pqNode.GetItem()
				uEta := pqNode.GetRank()

				overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exit Index, wOffset Index) {

					shorcutWeight := ow.weights[wOffset]
					newEta := uEta + shorcutWeight

					if newEta >= pkg.INF_WEIGHT {
						return
					}

					_, exitAlreadyVisited := eta[exit]
					if newEta < eta[exit] || !exitAlreadyVisited {
						eta[exit] = newEta
						exitOverlayVertex := overlayGraph.GetVertex(exit)
						neighborVertex := exitOverlayVertex.neighborOverlayVertex
						neighborOverlayVertex := overlayGraph.GetVertex(neighborVertex)

						if levelInfo.TruncateToLevel(neighborOverlayVertex.cellNumber, uint8(level)) == cellNumber {
							boundaryArcWeight := costFunction.GetWeight(graph.GetOutEdge(exitOverlayVertex.originalEdge))
							eta[neighborVertex] = newEta + boundaryArcWeight

							if _, neighborVertexAlreadyVisited := eta[neighborVertex]; !neighborVertexAlreadyVisited {
								pq.Insert(NewPriorityQueueNode(eta[neighborVertex], neighborVertex))
							} else {
								pq.DecreaseKey(NewPriorityQueueNode(eta[neighborVertex], neighborVertex))
							}
						}
					}
				})

			}

			// stores all eta of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
			for j := Index(0); j < cell.numExitPoints; j++ {
				exitPoint := overlayGraph.GetExitPoint(cell, j)
				_, exists := eta[exitPoint]
				if !exists {
					ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = pkg.INF_WEIGHT
				} else {
					ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = eta[exitPoint]
				}
			}
		}
		return nil
	}

	cellMapInLevelOne := overlayGraph.GetAllCellsInLevel(level)
	for pv, cell := range cellMapInLevelOne {
		buildCellClique(newCustomizerCell(cell, pv))
	}

}

package datastructure

import (
	"math"

	"github.com/lintang-b-s/navigatorx-crp/pkg/concurrent"
	"github.com/lintang-b-s/navigatorx-crp/pkg/costfunction"
)

type OverlayWeights struct {
	weights []float64
}

func (ow *OverlayWeights) GetWeight(i uint8) float64 {
	return ow.weights[i]
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
	cell       Cell
	cellNumber Pv
}

func newCustomizerCell(cell Cell, cellNumber Pv) customizerCell {
	return customizerCell{cell: cell, cellNumber: cellNumber}
}

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

	workers := concurrent.NewWorkerPool[customizerCell, any](workersNum,
		overlayGraph.numberOfCellsInLevel(1))

	buildCellClique := func(job customizerCell) any {
		pq := NewMinHeap[CRPQueryKey]()
		eta := make(map[Index]float64)
		overlayEta := make([]float64, graph.NumberOfVertices())

		cell := job.cell
		cellNumber := job.cellNumber

		for i := Index(0); i < cell.numEntryPoints; i++ {
			startOverlayVertexId := overlayGraph.GetEntryPoint(cell, i)
			overlayVertex := overlayGraph.GetVertex(startOverlayVertexId)
			start := overlayVertex.originalVertex

			eta[start] = 0

			startInEdgeOffset := overlayVertex.originalEdge
			pq.Insert(NewPriorityQueueNode(0, NewCRPQueryKey(start, startInEdgeOffset)))

			for !pq.isEmpty() {
				pqNode, _ := pq.ExtractMin()
				uKey := pqNode.item
				uId := uKey.node
				uEntryPoint := uKey.entryPoint
				uEta := pqNode.rank

				u := graph.GetVertex(uId)

				exitPoint := Index(0)
				for e := u.GetFirstOut(); e < graph.GetVertexFirstOut(start+1); e++ {
					// traverse all out edges
					outArc := graph.GetOutEdge(e)
					v := outArc.GetHead()

					turnType := graph.GetTurnType(u.GetID(), uEntryPoint, exitPoint)

					exitPointEta := uEta + costFunction.GetTurnCost(turnType)
					newETA := exitPointEta + costFunction.GetWeight(outArc)

					if newETA >= math.MaxFloat64 {
						continue
					}

					if graph.GetCellNumber(v) == cellNumber {
						vEntryPoint := graph.GetEntryOffset(v) + Index(outArc.GetEntryPoint())

						_, exists := eta[v]
						if !exists {
							eta[v] = newETA
							pq.Insert(NewPriorityQueueNode(newETA, NewCRPQueryKey(v, vEntryPoint)))
						} else if newETA < eta[v] {
							eta[v] = newETA
							pq.DecreaseKey(NewPriorityQueueNode(newETA, NewCRPQueryKey(v, vEntryPoint)))
						}
					} else {
						// found an exit point of the cell
						// save this shortcut eta
						exitOverlay, ok := graph.GetOverlayVertex(uId, uint8(exitPoint), true)
						if !ok {
							panic("overlay vertex not found") // for debugging (dev)
						}

						if exitPointEta < overlayEta[exitOverlay] {
							overlayEta[exitOverlay] = exitPointEta
						}
					}
					exitPoint++
				}
			}

			// stores all eta of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
			for j := Index(0); j < cell.numExitPoints; j++ {
				exitPoint := overlayGraph.GetExitPoint(cell, j)
				ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = overlayEta[exitPoint]
			}
		}
		return nil
	}

	cellMapInLevelOne := overlayGraph.GetAllCellsInLevel(1)
	for pv, cell := range cellMapInLevelOne {
		workers.AddJob(newCustomizerCell(cell, pv))
	}

	workers.Close()
	workers.Start(buildCellClique)
	workers.Wait()
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
	workers := concurrent.NewWorkerPool[customizerCell, any](workersNum,
		overlayGraph.numberOfCellsInLevel(level))

	buildCellClique := func(job customizerCell) any {
		pq := NewMinHeap[Index]()
		eta := make(map[Index]float64)
		overlayEta := make([]float64, graph.NumberOfVertices())

		cell := job.cell
		cellNumber := job.cellNumber

		for i := Index(0); i < cell.numEntryPoints; i++ {
			startOverlayVertexId := overlayGraph.GetEntryPoint(cell, i)

			eta[startOverlayVertexId] = 0

			pq.Insert(NewPriorityQueueNode(0, startOverlayVertexId))

			for !pq.isEmpty() {
				pqNode, _ := pq.ExtractMin()
				uOverlayId := pqNode.item
				uEta := pqNode.rank

				uOverlayVertex := overlayGraph.GetVertex(uOverlayId)
				entryPoint := uOverlayVertex.entryExitPoint[level-2] // -2 because level starts from 1, and slice starts from 0

				cell := overlayGraph.GetCell(uOverlayVertex.cellNumber, level-1)

				weightOffset := cell.cellOffset + entryPoint*cell.numExitPoints

				for j := Index(0); j < cell.numExitPoints; j++ {
					exit := overlayGraph.GetExitPoint(cell, j)
					newEta := uEta + ow.weights[weightOffset+j]

					if newEta >= math.MaxFloat64 {
						continue
					}

					if newEta < overlayEta[exit] {
						eta[exit] = newEta
						exitOverlayVertex := overlayGraph.GetVertex(exit)
						neighborVertex := exitOverlayVertex.neighborOverlayVertex
						neighborOverlayVertex := overlayGraph.GetVertex(neighborVertex)

						if levelInfo.TruncateToLevel(neighborOverlayVertex.cellNumber, uint8(level)) == cellNumber {
							boundaryArcWeight := costFunction.GetWeight(graph.GetOutEdge(exitOverlayVertex.originalEdge))
							eta[neighborVertex] = newEta + boundaryArcWeight

							if _, exists := eta[neighborVertex]; !exists {
								pq.Insert(NewPriorityQueueNode(eta[neighborVertex], neighborVertex))
							} else {
								pq.DecreaseKey(NewPriorityQueueNode(eta[neighborVertex], neighborVertex))
							}
						}

					}

				}
			}

			// stores all eta of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
			for j := Index(0); j < cell.numExitPoints; j++ {
				exitPoint := overlayGraph.GetExitPoint(cell, j)
				ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = overlayEta[exitPoint]
			}
		}
		return nil
	}

	cellMapInLevelOne := overlayGraph.GetAllCellsInLevel(level)
	for pv, cell := range cellMapInLevelOne {
		workers.AddJob(newCustomizerCell(cell, pv))
	}

	workers.Close()
	workers.Start(buildCellClique)
	workers.Wait()
}

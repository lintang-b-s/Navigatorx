package customizer

import (
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type cellCustomizationResTD struct {
	travelTime *da.PWL
	index      int
}

func NewCellCustomizationResultTD(travelTime *da.PWL, index int) cellCustomizationResTD {
	return cellCustomizationResTD{travelTime, index}
}

func (cc cellCustomizationResTD) getTravelTime() *da.PWL {
	return cc.travelTime
}

func (cc cellCustomizationResTD) getIndex() int {
	return cc.index
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
func (c *Customizer) BuildTD(
	costFunction costfunction.CostFunction) {
	fmt.Printf("total level: %v\n", c.overlayGraph.GetLevelInfo().GetLevelCount())
	c.buildLowestLevelTD(costFunction)
	for level := 2; level <= c.overlayGraph.GetLevelInfo().GetLevelCount(); level++ {
		c.buildLevelTD(costFunction, level)
	}
}

// buildLowestLevel. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
func (c *Customizer) buildLowestLevelTD(
	costFunction costfunction.CostFunction) {

	cellMapInLevelOne := c.overlayGraph.GetAllCellsInLevel(1)

	buildCellClique := func(job customizerCell) []cellCustomizationResTD {
		cellNumber, cell := job.cellNumber, job.cell

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationResTD, cellWeightSize)

		dijkstra := func(entries <-chan da.Index) {
			for i := range entries {
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)
				overlayVertex := c.overlayGraph.GetVertex(startOverlayVertexId)
				start := overlayVertex.GetOriginalVertex()
				pq := da.NewFourAryHeap[da.CRPQueryKey]()
				travelTime := make(map[da.Index]*da.PWL)
				forwardCellOffset := c.graph.GetInEdgeCellOffset(start)
				startInEdgeOffset := overlayVertex.GetOriginalEdge() - forwardCellOffset
				overlayTravelTime := make(map[da.Index]*da.PWL)

				startPs := make([]*da.Point, 1)
				startPs[0] = da.NewPoint(0, 0)
				travelTime[startInEdgeOffset] = da.NewPWL(startPs)

				pq.Insert(da.NewPriorityQueueNode(0,
					da.NewCRPQueryKey(start, startInEdgeOffset)))

				for !pq.IsEmpty() {
					pqNode, _ := pq.ExtractMin()
					uKey := pqNode.GetItem()
					uId := uKey.GetNode()
					uEntryPoint := uKey.GetEntryExitPoint()
					// uTravelTime := pqNode.GetRank()

					c.graph.ForOutEdgesOf(uId, c.graph.GetEntryOrder(uId, uEntryPoint+forwardCellOffset), func(
						outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
						// traverse all out edges
						v := outArc.GetHead()

						exitPointTravelTime := da.LinkConstOne(travelTime[uEntryPoint], costFunction.GetTurnCost(turnType))
						outArcCost := costFunction.GetWeightPWL(outArc)

						newTT := da.Link(exitPointTravelTime, outArcCost)

						if newTT.GetMin() >= pkg.INF_WEIGHT {
							return
						}

						vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
						if vTruncatedCellNumber == cellNumber {
							vEntryPoint := c.graph.GetEntryOffset(v) + da.Index(outArc.GetEntryPoint()) - forwardCellOffset

							_, ok := travelTime[vEntryPoint]

							if ok {
								if da.Ge(newTT.GetMin(), travelTime[vEntryPoint].GetMax()) {
									return
								}
								travelTime[vEntryPoint] = da.Merge(travelTime[vEntryPoint], newTT)

								pq.DecreaseKey(da.NewPriorityQueueNode(travelTime[vEntryPoint].GetMin(), da.NewCRPQueryKey(v, vEntryPoint)))
							} else {
								travelTime[vEntryPoint] = newTT
								pq.Insert(da.NewPriorityQueueNode(travelTime[vEntryPoint].GetMin(), da.NewCRPQueryKey(v, vEntryPoint)))
							}

						} else {
							// found an exit point of the cell
							// save this shortcut travelTime
							exitOverlay, _ := c.graph.GetOverlayVertex(uId, uint8(exitPoint), true) // overlay vetex id of exit vertex c_1(u).

							if _, ok := overlayTravelTime[exitOverlay]; !ok {
								overlayTravelTime[exitOverlay] = exitPointTravelTime
							} else {
								if da.Ge(exitPointTravelTime.GetMin(), overlayTravelTime[exitOverlay].GetMax()) {
									return
								}
								overlayTravelTime[exitOverlay] = da.Merge(overlayTravelTime[exitOverlay], exitPointTravelTime)
							}
						}
					})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitId := c.overlayGraph.GetExitId(cell, j)
					_, exists := overlayTravelTime[exitId]

					if !exists {
						dijkstraResChan <- NewCellCustomizationResultTD(da.NewPWL([]*da.Point{da.NewPoint(0, math.MaxFloat64)}), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResultTD(overlayTravelTime[exitId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}
			}
		}

		entries := make(chan da.Index, cell.GetNumEntryPoints())
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
			entries <- i
		}

		close(entries)

		cellWeights := make([]cellCustomizationResTD, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		for i := da.Index(0); i < cellWeightSize; i++ {
			res := <-dijkstraResChan
			cellWeights[i] = res
		}

		close(dijkstraResChan)

		return cellWeights
	}

	workers := concurrent.NewWorkerPool[customizerCell, []cellCustomizationResTD](8, len(cellMapInLevelOne))

	for cellNumber, cell := range cellMapInLevelOne {
		workers.AddJob(newCustomizerCell(cell, cellNumber))
	}

	workers.Close()
	workers.Start(buildCellClique)
	workers.Wait()

	for cellWeights := range workers.CollectResults() {
		for _, w := range cellWeights {
			c.owtd.SetWeight(w.getIndex(), w.getTravelTime())
		}
	}

}

// buildLevel. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
// this function uses overlay graph from the previous level to compute the weights
// basically: use shortcut edges from the previous level as edges in this current level overlay graph
func (c *Customizer) buildLevelTD(
	costFunction costfunction.CostFunction, level int) {

	levelInfo := c.overlayGraph.GetLevelInfo()

	buildCellClique := func(job customizerCell) []cellCustomizationResTD {

		cell := job.cell
		cellNumber := job.cellNumber

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationResTD, cellWeightSize)

		dijkstra := func(entries <-chan da.Index) {
			for i := range entries {
				pq := da.NewFourAryHeap[da.Index]()
				travelTime := make(map[da.Index]*da.PWL)
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)

				startPs := make([]*da.Point, 1)
				startPs[0] = da.NewPoint(0, 0)
				travelTime[startOverlayVertexId] = da.NewPWL(startPs)

				pq.Insert(da.NewPriorityQueueNode(0, startOverlayVertexId))

				for !pq.IsEmpty() {
					pqNode, _ := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					// uTravelTime := pqNode.GetRank()

					uTruncatedLevel := c.overlayGraph.TruncateToLevel(c.overlayGraph.GetVertex(uOverlayId).GetCellNumber(), uint8(level))
					util.AssertPanic(uTruncatedLevel == cellNumber, "current truncated cell number and boundary vertex truncated cell number must be the same!")

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exit da.Index, wOffset da.Index) {

						shortcutWeight := c.owtd.GetWeightPWL(wOffset)
						newTravelTime := da.Link(travelTime[uOverlayId], shortcutWeight)

						if newTravelTime.GetMin() >= pkg.INF_WEIGHT {
							return
						}

						oldExit, exitAlreadyVisited := travelTime[exit]

						if !exitAlreadyVisited {
							travelTime[exit] = newTravelTime
						} else {
							if da.Ge(newTravelTime.GetMin(), oldExit.GetMax()) {
								return
							}
							travelTime[exit] = da.Merge(travelTime[exit], newTravelTime)
						}

						exitOverlayVertex := c.overlayGraph.GetVertex(exit)
						neighborVertex := exitOverlayVertex.GetNeighborOverlayVertex()
						neighborOverlayVertex := c.overlayGraph.GetVertex(neighborVertex)
						exitOriEdge := exitOverlayVertex.GetOriginalEdge()

						if levelInfo.TruncateToLevel(neighborOverlayVertex.GetCellNumber(), uint8(level)) == cellNumber {
							boundaryArcWeight := costFunction.GetWeightPWL(c.graph.GetOutEdge(exitOriEdge))

							newNeighborTravelTime := da.Link(newTravelTime, boundaryArcWeight)
							oldNTravelTime, neighborVertexAlreadyVisited := travelTime[neighborVertex]

							if !neighborVertexAlreadyVisited {
								travelTime[neighborVertex] = newNeighborTravelTime

								pq.Insert(da.NewPriorityQueueNode(travelTime[neighborVertex].GetMin(), neighborVertex))
							} else {
								if da.Ge(newNeighborTravelTime.GetMin(), oldNTravelTime.GetMax()) {
									return
								}
								travelTime[neighborVertex] = da.Merge(travelTime[neighborVertex], newNeighborTravelTime)

								pq.DecreaseKey(da.NewPriorityQueueNode(travelTime[neighborVertex].GetMin(), neighborVertex))
							}
						}

					})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitId := c.overlayGraph.GetExitId(cell, j)

					_, exists := travelTime[exitId]
					if !exists {
						dijkstraResChan <- NewCellCustomizationResultTD(da.NewPWL([]*da.Point{da.NewPoint(0, math.MaxFloat64)}), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResultTD(travelTime[exitId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}
			}
		}

		entries := make(chan da.Index, cell.GetNumEntryPoints())
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
			entries <- i
		}

		close(entries)

		cellWeights := make([]cellCustomizationResTD, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		for i := da.Index(0); i < cellWeightSize; i++ {
			res := <-dijkstraResChan
			cellWeights[i] = res
		}

		close(dijkstraResChan)

		return cellWeights
	}

	cellMapInLevel := c.overlayGraph.GetAllCellsInLevel(level)

	workers := concurrent.NewWorkerPool[customizerCell, []cellCustomizationResTD](2, len(cellMapInLevel))

	for pv, cell := range cellMapInLevel {
		workers.AddJob(newCustomizerCell(cell, pv))
	}

	workers.Close()
	workers.Start(buildCellClique)
	workers.Wait()

	for cellWeights := range workers.CollectResults() {
		for _, w := range cellWeights {
			c.owtd.SetWeight(w.getIndex(), w.getTravelTime())
		}
	}

}

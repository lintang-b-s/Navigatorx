package customizer

import (
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
ref:

1. [subsection 4.3 Fast Computation] Kobitzsch, M., Radermacher, M. and Schieferdecker, D. (2013) “Evolution and Evaluation of the Penalty Method for Alternative Graphs,” OASIcs, Volume 33, ATMOS 2013. Edited by D. Frigioni and S. Stiller, 33, pp. 94–107. Available at: https://doi.org/10.4230/OASICS.ATMOS.2013.94.
*/

func (c *Customizer) UpdateDirtyCells(costFunction costfunction.CostFunction, dirtyCells []datastructure.DirtyCell,
	penaltyEdgeCost map[datastructure.PenaltiedEdge]float64, maxLevel int) map[datastructure.Index]float64 {
	dirtyCells = c.removeDirtyCellDuplicates(dirtyCells)
	updatedShortcutWeightMap := make(map[datastructure.Index]float64)

	updateResChan := make(chan map[datastructure.Index]float64)

	cellUpdater := func(dirtyCellChan <-chan datastructure.DirtyCell) {
		for dirtyCell := range dirtyCellChan {
			updateResChan <- c.UpdateCell(costFunction, dirtyCell.GetCellId(), dirtyCell.GetUsedLevel(), penaltyEdgeCost, maxLevel)
		}
	}

	dirtyCellChan := make(chan datastructure.DirtyCell, len(dirtyCells))

	for i := 0; i < CELL_UPDATER_WORKER; i++ {
		go cellUpdater(dirtyCellChan)
	}

	for _, dirtyCell := range dirtyCells {
		dirtyCellChan <- dirtyCell
	}
	close(dirtyCellChan)

	for i := 0; i < len(dirtyCells); i++ {
		cellWeightMap := <-updateResChan
		for wOfffset, shortcutWeight := range cellWeightMap {
			updatedShortcutWeightMap[wOfffset] = shortcutWeight
		}
	}

	close(updateResChan)

	return updatedShortcutWeightMap
}

func (c *Customizer) UpdateCell(costFunction costfunction.CostFunction, cellId datastructure.Pv, topLevelToUpdate int,
	penaltyEdgeCost map[datastructure.PenaltiedEdge]float64, maxLevel int) map[datastructure.Index]float64 {
	// update cellId level 1
	cellLevelOne := c.overlayGraph.GetCell(cellId, 1)

	truncatedCellId := c.overlayGraph.TruncateToLevel(cellId, 1)

	cellWeightMap := make(map[datastructure.Index]float64, cellLevelOne.GetNumEntryPoints()*cellLevelOne.GetNumExitPoints())

	cellWeightSize := cellLevelOne.GetNumEntryPoints() * cellLevelOne.GetNumExitPoints()
	dijkstraResChan := make(chan cellCustomizationRes, cellWeightSize)

	dijkstra := func(entries <-chan datastructure.Index) {
		for i := range entries {
			startOverlayVertexId := c.overlayGraph.GetEntryId(cellLevelOne, i)
			overlayVertex := c.overlayGraph.GetVertex(startOverlayVertexId)
			start := overlayVertex.GetOriginalVertex()
			pq := datastructure.NewFourAryHeap[datastructure.CRPQueryKey]()
			travelTime := make(map[datastructure.Index]float64)
			forwardCellOffset := c.graph.GetInEdgeCellOffset(start)
			startInEdgeOffset := overlayVertex.GetOriginalEdge() - forwardCellOffset
			overlayTravelTime := make(map[datastructure.Index]float64)
			travelTime[startInEdgeOffset] = 0

			pq.Insert(datastructure.NewPriorityQueueNode(0,
				datastructure.NewCRPQueryKey(start, startInEdgeOffset)))

			for !pq.IsEmpty() {
				pqNode, _ := pq.ExtractMin()
				uKey := pqNode.GetItem()
				uId := uKey.GetNode()
				uEntryPoint := uKey.GetEntryExitPoint()
				uTravelTime := pqNode.GetRank()

				c.graph.ForOutEdgesOf(uId, c.graph.GetEntryOrder(uId, uEntryPoint+forwardCellOffset), func(
					outArc *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
					// traverse all out edges
					v := outArc.GetHead()

					exitPointTravelTime := uTravelTime + costFunction.GetTurnCost(turnType)
					outArcCost := costFunction.GetWeight(outArc)

					if penaltyCost, penalized := penaltyEdgeCost[datastructure.NewPenaltiedEdge(outArc.GetEdgeId(), true)]; penalized {
						// apply penalty
						outArcCost = penaltyCost
					}

					newETA := exitPointTravelTime + outArcCost

					if newETA >= pkg.INF_WEIGHT {
						return
					}

					vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
					if vTruncatedCellNumber == truncatedCellId {
						vEntryPoint := c.graph.GetEntryOffset(v) + datastructure.Index(outArc.GetEntryPoint()) - forwardCellOffset

						if _, ok := travelTime[vEntryPoint]; !ok || newETA < travelTime[vEntryPoint] {
							travelTime[vEntryPoint] = newETA
							if ok {
								pq.DecreaseKey(datastructure.NewPriorityQueueNode(newETA, datastructure.NewCRPQueryKey(v, vEntryPoint)))
							} else {
								pq.Insert(datastructure.NewPriorityQueueNode(newETA, datastructure.NewCRPQueryKey(v, vEntryPoint)))
							}
						}
					} else {
						// found an exit point of the cell
						// save this shortcut travelTime
						exitOverlay, _ := c.graph.GetOverlayVertex(uId, uint8(exitPoint), true) // overlay vetex id of exit vertex c_1(u).
						if _, ok := overlayTravelTime[exitOverlay]; !ok || exitPointTravelTime < overlayTravelTime[exitOverlay] {
							overlayTravelTime[exitOverlay] = exitPointTravelTime
						}
					}
				})
			}

			// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
			for j := datastructure.Index(0); j < cellLevelOne.GetNumExitPoints(); j++ {
				exitId := c.overlayGraph.GetExitId(cellLevelOne, j)
				_, exists := overlayTravelTime[exitId]

				if !exists {
					dijkstraResChan <- NewCellCustomizationResult(pkg.INF_WEIGHT, int(cellLevelOne.GetCellOffset()+i*cellLevelOne.GetNumExitPoints()+j))
				} else {
					dijkstraResChan <- NewCellCustomizationResult(overlayTravelTime[exitId], int(cellLevelOne.GetCellOffset()+i*cellLevelOne.GetNumExitPoints()+j))
				}
			}
		}
	}

	entries := make(chan datastructure.Index, cellLevelOne.GetNumEntryPoints())
	for worker := 1; worker <= CELL_UPDATER_PAIR_WORKER; worker++ {
		go dijkstra(entries)
	}

	for i := datastructure.Index(0); i < cellLevelOne.GetNumEntryPoints(); i++ {
		entries <- i
	}

	close(entries)
	lock := sync.RWMutex{}

	for i := datastructure.Index(0); i < cellWeightSize; i++ {
		res := <-dijkstraResChan
		lock.Lock()
		cellWeightMap[datastructure.Index(res.getIndex())] = res.getTravelTime()
		lock.Unlock()
	}

	close(dijkstraResChan)

	levelInfo := c.overlayGraph.GetLevelInfo()

	// update level 2->topLevelToUpdate (1-indexed)
	for level := 2; level <= util.MinInt(topLevelToUpdate, maxLevel); level++ {
		cell := c.overlayGraph.GetCell(cellId, level)

		truncatedCellId = c.overlayGraph.TruncateToLevel(cellId, uint8(level))

		cellWeightSize = cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan = make(chan cellCustomizationRes, cellWeightSize)

		dijkstra := func(entries <-chan datastructure.Index) {
			for i := range entries {
				pq := datastructure.NewFourAryHeap[datastructure.Index]()
				travelTime := make(map[datastructure.Index]float64)
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)

				travelTime[startOverlayVertexId] = 0

				pq.Insert(datastructure.NewPriorityQueueNode(0, startOverlayVertexId))

				for !pq.IsEmpty() {
					pqNode, _ := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					uTravelTime := pqNode.GetRank()

					uTruncatedLevel := c.overlayGraph.TruncateToLevel(c.overlayGraph.GetVertex(uOverlayId).GetCellNumber(), uint8(level))
					util.AssertPanic(uTruncatedLevel == truncatedCellId, "current truncated cell number and boundary vertex truncated cell number must be the same!")

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exit datastructure.Index, wOffset datastructure.Index) {

						shortcutWeight := c.ow.GetWeight(wOffset)
						lock.RLock()
						if penalizedShortcutWeight, penalized := cellWeightMap[wOffset]; penalized {
							shortcutWeight = penalizedShortcutWeight
						}
						lock.RUnlock()
						newTravelTime := uTravelTime + shortcutWeight

						if newTravelTime >= pkg.INF_WEIGHT {
							return
						}

						oldExit, exitAlreadyVisited := travelTime[exit]
						if !exitAlreadyVisited || newTravelTime < oldExit {
							travelTime[exit] = newTravelTime
							exitOverlayVertex := c.overlayGraph.GetVertex(exit)
							neighborVertex := exitOverlayVertex.GetNeighborOverlayVertex()
							neighborOverlayVertex := c.overlayGraph.GetVertex(neighborVertex)

							if levelInfo.TruncateToLevel(neighborOverlayVertex.GetCellNumber(), uint8(level)) == truncatedCellId {
								boundaryArcWeight := costFunction.GetWeight(c.graph.GetOutEdge(exitOverlayVertex.GetOriginalEdge()))
								newNeighborTravelTime := newTravelTime + boundaryArcWeight
								oldNTravelTime, neighborVertexAlreadyVisited := travelTime[neighborVertex]

								if !neighborVertexAlreadyVisited || newNeighborTravelTime < oldNTravelTime {
									travelTime[neighborVertex] = newTravelTime + boundaryArcWeight

									if !neighborVertexAlreadyVisited {
										pq.Insert(datastructure.NewPriorityQueueNode(travelTime[neighborVertex], neighborVertex))
									} else {
										pq.DecreaseKey(datastructure.NewPriorityQueueNode(travelTime[neighborVertex], neighborVertex))
									}
								}
							}
						}
					})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := datastructure.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitId := c.overlayGraph.GetExitId(cell, j)

					_, exists := travelTime[exitId]
					if !exists {
						dijkstraResChan <- NewCellCustomizationResult(pkg.INF_WEIGHT, int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(travelTime[exitId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}
			}
		}

		entries := make(chan datastructure.Index, cell.GetNumEntryPoints())
		for worker := 1; worker <= CELL_UPDATER_PAIR_WORKER; worker++ {
			go dijkstra(entries)
		}

		for i := datastructure.Index(0); i < cell.GetNumEntryPoints(); i++ {
			entries <- i
		}

		close(entries)

		for i := datastructure.Index(0); i < cellWeightSize; i++ {
			res := <-dijkstraResChan
			lock.Lock()
			cellWeightMap[datastructure.Index(res.getIndex())] = res.getTravelTime()
			lock.Unlock()
		}

		close(dijkstraResChan)
	}

	return cellWeightMap
}

func (c *Customizer) removeDirtyCellDuplicates(dirtyCells []datastructure.DirtyCell) []datastructure.DirtyCell {
	set := make(map[datastructure.DirtyCell]struct{})

	res := make([]datastructure.DirtyCell, 0, len(dirtyCells))
	for _, dirtyCell := range dirtyCells {
		if _, exists := set[dirtyCell]; !exists {

			res = append(res, dirtyCell)
			set[dirtyCell] = struct{}{}
		}
	}

	return res
}

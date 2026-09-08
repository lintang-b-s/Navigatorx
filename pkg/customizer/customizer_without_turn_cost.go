package customizer

import (
	"context"
	"sync"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// penjelasan fase kustomisasi (tanpa turn cost) dari Customizable Route Planning ada di section 3.5:  https://drive.google.com/file/d/16X4_D82-dBz5CEKTLBWPb8DtG52eVMl5/view
// pdf password: <my-github-username>-<my-birth-year>-<my gdrive email without @gmail.com>

/*
// buildLowestLevelWithoutTurnCost. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm (restricted to cell C) from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// restricted to cell C: menggunakan only vertices dan edges yang terletak pada cell C.
// this function is parallelized using goroutines worker pool
*/
func (c *Customizer[W]) buildLowestLevelWithoutTurnCost(costFunction *costfunction.TimeFunction[W]) {

	cellMapInLevelOne := c.overlayGraph.GetAllCellsInLevel(1)

	cellCliqueOutChan := make(chan []cellCustomizationRes[W], cellCliqueOutChanSize)

	wg := sync.WaitGroup{}

	buildCellClique := func(job customizerCell) {

		cell := job.cell
		cellNumber := job.cellNumber

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationRes[W], dijkstraResChanSize)

		dijkstra := func(entries <-chan da.Index) {
			/*
				let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary vertices, and shortucts within any cell
				let n,m,k denote the number vertices,edges, and number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), respectively.


				pq contains at most all edges in a cell level 1
				extractMin at most n_p
				decreaseKey and insert at most m_p
				we do dijkstra for all entries in the cell, num of entries is at most n_op
				worst case: O( n_op * ((m_p + n_p)* log(n_p)) )

			*/
			for i := range entries {
				startOverlayVertexId := c.overlayGraph.GetInId(cell, i)
				overlayVertex := c.overlayGraph.GetVertex(startOverlayVertexId)
				start := overlayVertex.GetOrigVId()
				maxSearchSize := c.graph.GetMaxEdgesInCell()

				pq := c.lowestHeapNoTurnCostPool.Get().(*da.QueryHeap[da.CRPQueryKey, W])
				pq.Clear()
				done := func() {
					c.lowestHeapNoTurnCostPool.Put(pq)
				}

				cost := make(map[da.Index]W, maxSearchSize)
				overlayCost := make(map[da.Index]W, da.OVERLAY_CELL_SIZE)

				cost[start] = 0
				noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)

				sVertexData := da.NewVertexData(W(0), noPar)
				pq.Insert(start, 0, sVertexData, da.NewDijkstraKeyNoTurnCost(start))

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uKey := pqNode.GetItem()
					uId := uKey.GetNode()
					uCost := pqNode.GetRank()

					c.graph.ForOutEdgesOfNoTurnCost(uId, func(eId, head, entryPoint da.Index) {
						// traverse all out edges

						v := head

						uCostWithTurnCost := uCost
						outArcCost := costFunction.GetWeight(eId)

						newVCost := uCostWithTurnCost + outArcCost

						if util.Ge(newVCost, util.Infinity[W]()) {
							return
						}

						vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
						if vTruncatedCellNumber == cellNumber {

							_, ok := cost[v]
							if oldvCost := cost[v]; !ok || (ok && util.Lt(newVCost, oldvCost)) {
								cost[v] = newVCost
								if ok {
									pq.DecreaseKey(v, newVCost, newVCost, noPar)
								} else {
									vVertexData := da.NewVertexData(newVCost, noPar)
									pq.Insert(v, newVCost, vVertexData, da.NewDijkstraKeyNoTurnCost(v))
								}
							}
						} else {
							// found an exit vertex of the cell
							// save this shortcut cost
							// v is in another cell
							exitVertexCost := uCostWithTurnCost
							exitPoint := c.graph.GetExitOrder(uId, eId)
							exitOverlayVId, _ := c.graph.GetOverlayVertex(uId, exitPoint, true) // overlay vetex id of exit vertex c_1(u).
							_, ok := overlayCost[exitOverlayVId]
							if !ok || (ok && util.Lt(exitVertexCost, overlayCost[exitOverlayVId])) {
								overlayCost[exitOverlayVId] = exitVertexCost
							}
						}
					})
				}

				// stores all cost of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitOverlayVId := c.overlayGraph.GetOutId(cell, j)
					_, ok := overlayCost[exitOverlayVId]
					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(util.Infinity[W](), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(overlayCost[exitOverlayVId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}

				done()
			}
		}

		entries := make(chan da.Index, CELL_ENTRIES_CHAN_SIZE)
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		cellWeights := make([]cellCustomizationRes[W], cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		wg := sync.WaitGroup{}
		wg.Add(1)

		go func() {
			defer wg.Done()
			for i := da.Index(0); i < cellWeightSize; i++ {
				res := <-dijkstraResChan
				cellWeights[i] = res
			}
		}()

		for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
			entries <- i
		}

		close(entries)

		wg.Wait()
		close(dijkstraResChan)

		cellCliqueOutChan <- cellWeights
	}

	go func() {
		for cellWeights := range cellCliqueOutChan {
			for _, w := range cellWeights {
				c.ow.SetWeight(w.getIndex(), w.getCost())
			}
			wg.Done()
		}
	}()

	numberOfShortcuts := da.Index(0)
	for cellNumber, cell := range cellMapInLevelOne {
		wg.Add(1)
		numberOfShortcuts += cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		gopool.CtxGo(context.Background(), func() { buildCellClique(newCustomizerCell(cell, cellNumber)) })
	}

	// let c_1 be the number of cells in level 1
	// worst case buildLowestLevel: O( c_1 * n_op * ((m_p + n_p)* log(n_p)))

	wg.Wait()
	close(cellCliqueOutChan)
	c.logger.Sugar().Infof("number of shortcuts overlay graph level %v: %v ", 1, numberOfShortcuts)

}

// buildLevelWithoutTurnCost. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm (menggunakan shortcut edges pada subcells of the level-i cell) from each entry vertices of the cell to all exit vertices of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
func (c *Customizer[W]) buildLevelWithoutTurnCost(costFunction *costfunction.TimeFunction[W], level int) {

	levelData := c.overlayGraph.GetLevelData()
	cellMapInLevel := c.overlayGraph.GetAllCellsInLevel(level)

	cellCliqueOutChan := make(chan []cellCustomizationRes[W], cellCliqueOutChanSize)

	wg := sync.WaitGroup{}

	buildCellClique := func(job customizerCell) {

		cell := job.cell
		cellNumber := job.cellNumber

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationRes[W], dijkstraResChanSize)

		dijkstra := func(entries <-chan da.Index) {
			/*
				let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary vertices, and shortucts within any cell
				let n,m,k denote the number vertices,edges, and number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), respectively.


				pq contains at most all overlay vertices in all subcells of this cell in level-1
				extractMin at most n_op
				decreaseKey and insert at most \hat{m_p}

				we do dijkstra for all entries in the cell, num of entries is at most n_op
				worst case: O( n_op * (n_op + \hat{m_p})* log(n_op) )
			*/
			for i := range entries {

				pq := c.levelHeapNoTurnCostPool.Get().(*da.QueryHeap[da.Index, W])
				pq.Clear()
				done := func() {
					c.levelHeapNoTurnCostPool.Put(pq)
				}

				cost := make(map[da.Index]W, da.OVERLAY_CELL_SIZE)

				startOverlayVertexId := c.overlayGraph.GetInId(cell, i)

				noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)
				sVertexData := da.NewVertexData(W(0), noPar)

				pq.Insert(startOverlayVertexId, 0, sVertexData, startOverlayVertexId)

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					uCost := pqNode.GetRank()

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exitOverlayVertex da.Index, wOffset da.Index) {
						// iterate all shortcuts (u, \cdot)

						shortcutWeight := c.ow.GetWeight(wOffset)

						newVCost := uCost + shortcutWeight

						if util.Ge(newVCost, util.Infinity[W]()) {
							return
						}

						oldExitCost := cost[exitOverlayVertex]
						_, exitAlreadyLabelled := cost[exitOverlayVertex]
						if !exitAlreadyLabelled || (exitAlreadyLabelled && util.Lt(newVCost, oldExitCost)) {
							cost[exitOverlayVertex] = newVCost
							// visit neighbor of exitOverlayVertex
							//
							exitOverlayVertex := c.overlayGraph.GetVertex(exitOverlayVertex)
							neighborVertex := exitOverlayVertex.GetNeighborOverlayVertex()
							neighborOverlayVertex := c.overlayGraph.GetVertex(neighborVertex)
							// cut edge (exitOverlayVertex, neighborOverlayVertex)
							cutOutEdgeId := exitOverlayVertex.GetCutEdge()

							if levelData.TruncateToLevel(neighborOverlayVertex.GetCellNumber(), uint8(level)) == cellNumber {
								boundaryArcWeight := costFunction.GetWeight(cutOutEdgeId)

								newNeighborCost := newVCost + boundaryArcWeight
								oldNCost := cost[neighborVertex]
								_, nAlreadyLabelled := cost[neighborVertex]

								if !nAlreadyLabelled ||
									(nAlreadyLabelled && util.Lt(newNeighborCost, oldNCost)) {
									cost[neighborVertex] = newNeighborCost

									if !nAlreadyLabelled {
										vVertexData := da.NewVertexData(newVCost, noPar)
										pq.Insert(neighborVertex, newNeighborCost, vVertexData, neighborVertex)
									} else {
										pq.DecreaseKey(neighborVertex, newNeighborCost,
											newNeighborCost, noPar)
									}
								}
							}
						}
					})
				}

				// stores all cost of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitOverlayVId := c.overlayGraph.GetOutId(cell, j)

					_, ok := cost[exitOverlayVId]
					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(util.Infinity[W](), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(cost[exitOverlayVId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}

				done()
			}
		}

		entries := make(chan da.Index, CELL_ENTRIES_CHAN_SIZE)
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		cellWeights := make([]cellCustomizationRes[W], cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		wg := sync.WaitGroup{}
		wg.Add(1)

		go func() {
			defer wg.Done()
			for i := da.Index(0); i < cellWeightSize; i++ {
				res := <-dijkstraResChan
				cellWeights[i] = res
			}
		}()

		for i := da.Index(0); i < cell.GetNumEntryPoints(); i++ {
			entries <- i
		}

		close(entries)

		wg.Wait()
		close(dijkstraResChan)

		cellCliqueOutChan <- cellWeights
	}

	go func() {
		for cellWeights := range cellCliqueOutChan {
			for _, w := range cellWeights {
				c.ow.SetWeight(w.getIndex(), w.getCost())
			}
			wg.Done()
		}
	}()
	numberOfShortcuts := da.Index(0)

	for pv, cell := range cellMapInLevel {
		wg.Add(1)
		numberOfShortcuts += cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		gopool.CtxGo(context.Background(), func() {
			buildCellClique(newCustomizerCell(cell, pv))
		})
	}

	// let c_l be the number of cells in level l
	// worst case buildLevel:  O( c_l * n_op * (n_op + \hat{m_p})* log(n_op) )

	wg.Wait()
	close(cellCliqueOutChan)
	c.logger.Sugar().Infof("number of shortcuts overlay graph level %v: %v ", level, numberOfShortcuts)
}

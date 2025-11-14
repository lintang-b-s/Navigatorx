package customizer

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Customizer struct {
	logger               *zap.Logger
	graphFilePath        string
	overlayGraphFilePath string
	metricOutputFilePath string
	ow                   *datastructure.OverlayWeights
	graph                *datastructure.Graph
	overlayGraph         *datastructure.OverlayGraph
}

func NewCustomizer(graphFilePath, overlayGraphFilePath, metricOutputFilePath string,
	logger *zap.Logger) *Customizer {
	return &Customizer{
		graphFilePath:        graphFilePath,
		overlayGraphFilePath: overlayGraphFilePath,
		metricOutputFilePath: metricOutputFilePath,
		logger:               logger,
	}
}

func (c *Customizer) Customize() error {
	c.logger.Sugar().Infof("Starting customization step of Customizable Route Planning...")
	var err error
	c.logger.Sugar().Infof("Reading graph from %s", c.graphFilePath)
	c.graph, err = datastructure.ReadGraph(c.graphFilePath)
	if err != nil {
		return err
	}

	c.logger.Sugar().Infof("Reading overlay graph from %s", c.overlayGraphFilePath)
	c.overlayGraph, err = datastructure.ReadOverlayGraph(c.overlayGraphFilePath)
	if err != nil {
		return err
	}

	costFunction := costfunction.NewTimeCostFunction()
	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = datastructure.NewOverlayWeights(c.overlayGraph.GetWeightVectorSize())
	c.Build(costFunction)

	c.logger.Sugar().Infof("Building stalling tables...")
	metrics := metrics.NewMetric(c.graph, costFunction, c.ow)
	metrics.BuildStallingTables(c.overlayGraph, c.graph)
	err = metrics.WriteToFile(c.metricOutputFilePath)
	if err != nil {
		return err
	}
	c.logger.Sugar().Infof("Customization step completed successfully.")
	return nil
}

type customizerCell struct {
	cell       *datastructure.Cell
	cellNumber datastructure.Pv
}

func newCustomizerCell(cell *datastructure.Cell, cellNumber datastructure.Pv) customizerCell {
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
func (c *Customizer) Build(
	costFunction costfunction.CostFunction) {
	c.buildLowestLevel(costFunction)
	for level := 2; level <= c.overlayGraph.GetLevelInfo().GetLevelCount(); level++ {
		c.buildLevel(costFunction, level)
	}
}

type cellCustomizationRes struct {
	travelTime float64
	index      int
}

func NewCellCustomizationResult(travelTime float64, index int) cellCustomizationRes {
	return cellCustomizationRes{travelTime, index}
}

func (cc cellCustomizationRes) getTravelTime() float64 {
	return cc.travelTime
}

func (cc cellCustomizationRes) getIndex() int {
	return cc.index
}

// buildLowestLevel. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
func (c *Customizer) buildLowestLevel(
	costFunction costfunction.CostFunction) {

	cellMapInLevelOne := c.overlayGraph.GetAllCellsInLevel(1)

	buildCellClique := func(job customizerCell) []cellCustomizationRes {
		cellNumber, cell := job.cellNumber, job.cell

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationRes, cellWeightSize)

		dijkstra := func(entries <-chan datastructure.Index) {
			for i := range entries {
				startOverlayVertexId := c.overlayGraph.GetEntryPoint(cell, i)
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

						newETA := exitPointTravelTime + outArcCost

						if newETA >= pkg.INF_WEIGHT {
							return
						}

						vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
						if vTruncatedCellNumber == cellNumber {
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
				for j := datastructure.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitId := c.overlayGraph.GetExitId(cell, j)
					_, exists := overlayTravelTime[exitId]

					if !exists {
						dijkstraResChan <- NewCellCustomizationResult(pkg.INF_WEIGHT, int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(overlayTravelTime[exitId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}
			}
		}

		entries := make(chan datastructure.Index, cell.GetNumEntryPoints())
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		for i := datastructure.Index(0); i < cell.GetNumEntryPoints(); i++ {
			entries <- i
		}

		close(entries)

		cellWeights := make([]cellCustomizationRes, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		for i := datastructure.Index(0); i < cellWeightSize; i++ {
			res := <-dijkstraResChan
			cellWeights[i] = res
		}

		close(dijkstraResChan)

		return cellWeights
	}

	workers := concurrent.NewWorkerPool[customizerCell, []cellCustomizationRes](8, len(cellMapInLevelOne))

	for cellNumber, cell := range cellMapInLevelOne {
		workers.AddJob(newCustomizerCell(cell, cellNumber))
	}

	workers.Close()
	workers.Start(buildCellClique)
	workers.Wait()

	for cellWeights := range workers.CollectResults() {
		for _, w := range cellWeights {
			c.ow.SetWeight(w.getIndex(), w.getTravelTime())
		}
	}
}

// buildLevel. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
// this function uses overlay graph from the previous level to compute the weights
// basically: use shortcut edges from the previous level as edges in this current level overlay graph
func (c *Customizer) buildLevel(
	costFunction costfunction.CostFunction, level int) {

	levelInfo := c.overlayGraph.GetLevelInfo()

	buildCellClique := func(job customizerCell) []cellCustomizationRes {

		cell := job.cell
		cellNumber := job.cellNumber

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationRes, cellWeightSize)

		dijkstra := func(entries <-chan datastructure.Index) {
			for i := range entries {
				pq := datastructure.NewFourAryHeap[datastructure.Index]()
				travelTime := make(map[datastructure.Index]float64)
				startOverlayVertexId := c.overlayGraph.GetEntryPoint(cell, i)

				travelTime[startOverlayVertexId] = 0

				pq.Insert(datastructure.NewPriorityQueueNode(0, startOverlayVertexId))

				for !pq.IsEmpty() {
					pqNode, _ := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					uTravelTime := pqNode.GetRank()

					uTruncatedLevel := c.overlayGraph.TruncateToLevel(c.overlayGraph.GetVertex(uOverlayId).GetCellNumber(), uint8(level))
					util.AssertPanic(uTruncatedLevel == cellNumber, "current truncated cell number and boundary vertex truncated cell number must be the same!")

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exit datastructure.Index, wOffset datastructure.Index) {

						shortcutWeight := c.ow.GetWeight(wOffset)
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

							if levelInfo.TruncateToLevel(neighborOverlayVertex.GetCellNumber(), uint8(level)) == cellNumber {
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
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		for i := datastructure.Index(0); i < cell.GetNumEntryPoints(); i++ {
			entries <- i
		}

		close(entries)

		cellWeights := make([]cellCustomizationRes, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		for i := datastructure.Index(0); i < cellWeightSize; i++ {
			res := <-dijkstraResChan
			cellWeights[i] = res
		}

		close(dijkstraResChan)

		return cellWeights
	}

	cellMapInLevel := c.overlayGraph.GetAllCellsInLevel(level)

	workers := concurrent.NewWorkerPool[customizerCell, []cellCustomizationRes](20, len(cellMapInLevel))

	for pv, cell := range cellMapInLevel {
		workers.AddJob(newCustomizerCell(cell, pv))
	}

	workers.Close()
	workers.Start(buildCellClique)
	workers.Wait()

	for cellWeights := range workers.CollectResults() {
		for _, w := range cellWeights {
			c.ow.SetWeight(w.getIndex(), w.getTravelTime())
		}
	}
}

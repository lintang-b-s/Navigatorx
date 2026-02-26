package customizer

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Customizer struct {
	logger               *zap.Logger
	graphFilePath        string
	overlayGraphFilePath string
	metricOutputFilePath string
	ow                   *da.OverlayWeights
	graph                *da.Graph
	overlayGraph         *da.OverlayGraph
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

func NewCustomizerDirect(graph *da.Graph, overlayGraph *da.OverlayGraph, logger *zap.Logger) *Customizer {
	return &Customizer{
		graph:        graph,
		overlayGraph: overlayGraph,
		logger:       logger,
	}
}

func (c *Customizer) Customize() (*metrics.Metric, error) {

	c.logger.Sugar().Infof("Starting customization step of Customizable Route Planning...")
	var err error
	c.logger.Sugar().Infof("Reading graph from %s", c.graphFilePath)
	c.graph, err = da.ReadGraph(c.graphFilePath)
	if err != nil {
		return nil, err
	}

	c.logger.Sugar().Infof("Reading overlay graph from %s", c.overlayGraphFilePath)
	c.overlayGraph, err = da.ReadOverlayGraph(c.overlayGraphFilePath)
	if err != nil {
		return nil, err
	}

	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = da.NewOverlayWeights(c.overlayGraph.GetWeightVectorSize())
	c.logger.Info(fmt.Sprintf("number of shortcuts: %v", c.ow.GetNumberOfShortcuts()))
	var m *metrics.Metric
	costFunction := costfunction.NewTimeCostFunction()

	c.Build(costFunction)
	c.logger.Sugar().Infof("Building stalling tables...")
	m = metrics.NewMetric(c.graph, costFunction, c.ow)

	m.BuildStallingTables(c.overlayGraph, c.graph)
	err = m.WriteToFile(c.metricOutputFilePath)
	if err != nil {
		return nil, err
	}
	c.logger.Sugar().Infof("Customization step completed successfully.")

	return m, nil
}

// just for shortest path test
func (c *Customizer) CustomizeDirect() (*metrics.Metric, error) {

	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = da.NewOverlayWeights(c.overlayGraph.GetWeightVectorSize())
	c.logger.Info(fmt.Sprintf("number of shortcuts: %v", c.ow.GetNumberOfShortcuts()))

	var m *metrics.Metric
	costFunction := costfunction.NewTimeCostFunctionEmpty()

	c.Build(costFunction)
	c.logger.Sugar().Infof("Building stalling tables...")
	m = metrics.NewMetric(c.graph, costFunction, c.ow)
	m.BuildStallingTables(c.overlayGraph, c.graph)
	c.logger.Sugar().Infof("Customization step completed successfully.")

	return m, nil
}

type customizerCell struct {
	cell       *da.Cell
	cellNumber da.Pv
}

func newCustomizerCell(cell *da.Cell, cellNumber da.Pv) customizerCell {
	return customizerCell{cell: cell, cellNumber: cellNumber}
}

/*
Build. Customizable Route Planning in Road Networks, Delling et al., et al. Page 11:

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

time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortucts within any partition
let n,m,k denote the number vertices,edges, and partitioning depth, respectively.
let n_b, \hat{m} denote the total number of boundary nodes and shortcuts

time complexity of CRP Customization is: O(n_b * m_p * log n)
*/
func (c *Customizer) Build(
	costFunction costfunction.CostFunction) {
	c.buildLowestLevel(costFunction)
	c.logger.Info("finished crp customization level 1")
	for level := 2; level <= c.overlayGraph.GetLevelInfo().GetLevelCount(); level++ {
		c.buildLevel(costFunction, level)
		c.logger.Sugar().Infof("finished crp customization level %v", level)

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

		dijkstra := func(entries <-chan da.Index) {
			for i := range entries {
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)
				overlayVertex := c.overlayGraph.GetVertex(startOverlayVertexId)
				start := overlayVertex.GetOriginalVertex()
				pq := da.NewFourAryHeap[da.CRPQueryKey]()
				travelTime := make([]float64, c.graph.GetMaxEdgesInCell())
				overlayTravelTime := make([]float64, c.overlayGraph.NumberOfOverlayVertices())
				hnodes := make([]*da.PriorityQueueNode[da.CRPQueryKey], c.graph.GetMaxEdgesInCell())
				for q := 0; q < len(travelTime); q++ {
					travelTime[q] = pkg.INF_WEIGHT
				}
				for q := 0; q < c.overlayGraph.NumberOfOverlayVertices(); q++ {
					overlayTravelTime[q] = pkg.INF_WEIGHT
				}
				forwardCellOffset := c.graph.GetInEdgeCellOffset(start)
				startInEdgeOffset := overlayVertex.GetOriginalEdge() - forwardCellOffset

				travelTime[startInEdgeOffset] = 0

				pq.Insert(da.NewPriorityQueueNode(0,
					da.NewDijkstraKey(start, startInEdgeOffset)))

				for !pq.IsEmpty() {
					pqNode, _ := pq.ExtractMin()
					uKey := pqNode.GetItem()
					uId := uKey.GetNode()
					uEntryPoint := uKey.GetEntryExitPoint()
					uTravelTime := pqNode.GetRank()

					c.graph.ForOutEdgesOf(uId, c.graph.GetEntryOrder(uId, uEntryPoint+forwardCellOffset), func(
						outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
						// traverse all out edges
						v := outArc.GetHead()

						exitPointTravelTime := uTravelTime + costFunction.GetTurnCost(turnType)
						outArcCost := costFunction.GetWeight(outArc)

						newTravelTime := exitPointTravelTime + outArcCost

						if newTravelTime >= pkg.INF_WEIGHT {
							return
						}

						vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
						if vTruncatedCellNumber == cellNumber {
							vEntryPoint := c.graph.GetEntryOffset(v) + da.Index(outArc.GetEntryPoint()) - forwardCellOffset

							ok := da.Lt(travelTime[vEntryPoint], pkg.INF_WEIGHT)
							if oldvTT := travelTime[vEntryPoint]; !ok || (ok && newTravelTime < oldvTT) {
								travelTime[vEntryPoint] = newTravelTime
								if ok {
									pq.DecreaseKey(hnodes[vEntryPoint], newTravelTime)
								} else {
									vhNode := da.NewPriorityQueueNode(newTravelTime, da.NewDijkstraKey(v, vEntryPoint))
									hnodes[vEntryPoint] = vhNode
									pq.Insert(vhNode)
								}
							}
						} else {
							// found an exit point of the cell
							// save this shortcut travelTime
							exitOverlay, _ := c.graph.GetOverlayVertex(uId, int(exitPoint), true) // overlay vetex id of exit vertex c_1(u).
							ok := da.Lt(overlayTravelTime[exitOverlay], pkg.INF_WEIGHT)
							if !ok || (ok && exitPointTravelTime < overlayTravelTime[exitOverlay]) {
								overlayTravelTime[exitOverlay] = exitPointTravelTime
							}
						}
					})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitId := c.overlayGraph.GetExitId(cell, j)
					ok := da.Lt(overlayTravelTime[exitId], pkg.INF_WEIGHT)

					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(pkg.INF_WEIGHT, int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(overlayTravelTime[exitId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
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

		cellWeights := make([]cellCustomizationRes, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		for i := da.Index(0); i < cellWeightSize; i++ {
			res := <-dijkstraResChan
			cellWeights[i] = res
		}

		close(dijkstraResChan)

		return cellWeights
	}

	workers := concurrent.NewWorkerPool[customizerCell, []cellCustomizationRes](CUSTOMIZER_WORKER, len(cellMapInLevelOne))

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

		dijkstra := func(entries <-chan da.Index) {
			for i := range entries {
				pq := da.NewFourAryHeap[da.Index]()
				travelTime := make([]float64, c.overlayGraph.NumberOfOverlayVertices())
				hnodes := make([]*da.PriorityQueueNode[da.Index], c.overlayGraph.NumberOfOverlayVertices())
				for v := 0; v < c.overlayGraph.NumberOfOverlayVertices(); v++ {
					travelTime[v] = pkg.INF_WEIGHT
				}
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)

				travelTime[startOverlayVertexId] = 0

				pq.Insert(da.NewPriorityQueueNode(0, startOverlayVertexId))

				for !pq.IsEmpty() {
					pqNode, _ := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					uTravelTime := pqNode.GetRank()

					uTruncatedLevel := c.overlayGraph.TruncateToLevel(c.overlayGraph.GetVertex(uOverlayId).GetCellNumber(), uint8(level))
					util.AssertPanic(uTruncatedLevel == cellNumber, "current truncated cell number and boundary vertex truncated cell number must be the same!")

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exit da.Index, wOffset da.Index) {

						shortcutWeight := c.ow.GetWeight(wOffset)

						newTravelTime := uTravelTime + shortcutWeight

						if newTravelTime >= pkg.INF_WEIGHT {
							return
						}

						oldExit := travelTime[exit]
						exitAlreadyLabelled := da.Lt(travelTime[exit], pkg.INF_WEIGHT)
						if !exitAlreadyLabelled || (exitAlreadyLabelled && newTravelTime < oldExit) {
							travelTime[exit] = newTravelTime
							exitOverlayVertex := c.overlayGraph.GetVertex(exit)
							neighborVertex := exitOverlayVertex.GetNeighborOverlayVertex()
							neighborOverlayVertex := c.overlayGraph.GetVertex(neighborVertex)
							exitOriEdge := exitOverlayVertex.GetOriginalEdge()

							if levelInfo.TruncateToLevel(neighborOverlayVertex.GetCellNumber(), uint8(level)) == cellNumber {
								boundaryArcWeight := costFunction.GetWeight(c.graph.GetOutEdge(exitOriEdge))

								newNeighborTravelTime := newTravelTime + boundaryArcWeight
								oldNTravelTime := travelTime[neighborVertex]
								nAlreadyLabelled := da.Lt(travelTime[neighborVertex], pkg.INF_WEIGHT)

								if !nAlreadyLabelled ||
									(nAlreadyLabelled && newNeighborTravelTime < oldNTravelTime) {
									travelTime[neighborVertex] = newNeighborTravelTime

									if !nAlreadyLabelled {
										vhNode := da.NewPriorityQueueNode(travelTime[neighborVertex], neighborVertex)
										hnodes[neighborVertex] = vhNode
										pq.Insert(vhNode)
									} else {
										pq.DecreaseKey(hnodes[neighborVertex], newNeighborTravelTime)
									}
								}
							}
						}
					})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitId := c.overlayGraph.GetExitId(cell, j)

					ok := da.Lt(travelTime[exitId], pkg.INF_WEIGHT)
					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(pkg.INF_WEIGHT, int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(travelTime[exitId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
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

		cellWeights := make([]cellCustomizationRes, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

		for i := da.Index(0); i < cellWeightSize; i++ {
			res := <-dijkstraResChan
			cellWeights[i] = res
		}

		close(dijkstraResChan)

		return cellWeights
	}

	cellMapInLevel := c.overlayGraph.GetAllCellsInLevel(level)

	workers := concurrent.NewWorkerPool[customizerCell, []cellCustomizationRes](CUSTOMIZER_WORKER, len(cellMapInLevel))

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

func (c *Customizer) SetGraph(graph *da.Graph) {
	c.graph = graph
}

func (c *Customizer) SetOverlayGraph(overlayGraph *da.OverlayGraph) {
	c.overlayGraph = overlayGraph
}

func (c *Customizer) SetOverlayWeight(ow *da.OverlayWeights) {
	c.ow = ow
}

func (c *Customizer) GetGraph() *da.Graph {
	return c.graph
}

func (c *Customizer) GetOverlayGraph() *da.OverlayGraph {
	return c.overlayGraph
}

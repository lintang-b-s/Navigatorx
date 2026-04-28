package customizer

import (
	"context"
	"fmt"
	"sync"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

type Customizer struct {
	ow                   *da.OverlayWeights
	graph                *da.Graph
	overlayGraph         *da.OverlayGraph
	lowestHeapPool       sync.Pool
	levelHeapPool        sync.Pool
	verticesLookupTable  *LookupTable[uint64]
	logger               *zap.Logger
	edgeSpeedsFilePath   []string
	graphFilePath        string
	overlayGraphFilePath string
	metricOutputFilePath string
	timefunctionFilePath string
	landmarkFile         string
}

func NewCustomizer(graphFilePath, overlayGraphFilePath, metricOutputFilePath, timefunctionFilePath, landmarkFile string,
	logger *zap.Logger) *Customizer {
	cst := &Customizer{
		graphFilePath:        graphFilePath,
		overlayGraphFilePath: overlayGraphFilePath,
		metricOutputFilePath: metricOutputFilePath,
		logger:               logger,
		timefunctionFilePath: timefunctionFilePath,
		landmarkFile:         landmarkFile,
	}

	return cst
}

func (c *Customizer) SetEdgeSpeedsFilePath(filePath []string) {
	c.edgeSpeedsFilePath = filePath
}

func NewCustomizerDirect(graph *da.Graph, overlayGraph *da.OverlayGraph, logger *zap.Logger) *Customizer {
	return &Customizer{
		graph:        graph,
		overlayGraph: overlayGraph,
		logger:       logger,
	}
}

func (c *Customizer) Customize() (*metrics.Metric, error) {

	var err error

	c.logger.Sugar().Infof("Starting customization step of Customizable Route Planning...")
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

	vertexOsmIds := c.graph.GetVertexOsmIds()
	c.verticesLookupTable = NewLookupTable(vertexOsmIds, func(a, b uint64) bool {
		return a < b
	})

	roadNetwork := c.graph.IsRoadNetworkGraph()

	updatedEdgeIds := make([]da.Index, 0)
	updatedEdgeMaxSpeeds := make([]float64, 0)

	if len(c.edgeSpeedsFilePath) != 0 {
		for _, currFilePath := range c.edgeSpeedsFilePath {
			currEdgeIds, currEdgeMaxSpeeds, err := c.readEdgeSpeedsFromFile(currFilePath)
			if err != nil {
				return nil, err
			}
			updatedEdgeIds = append(updatedEdgeIds, currEdgeIds...)
			updatedEdgeMaxSpeeds = append(updatedEdgeMaxSpeeds, currEdgeMaxSpeeds...)
		}
	}

	edgeMaxSpeeds := c.makeEdgeMaxSpeeds(updatedEdgeIds, updatedEdgeMaxSpeeds)
	costFunction := costfunction.NewTimeCostFunction(roadNetwork, edgeMaxSpeeds)
	err = costFunction.WriteToFile(c.timefunctionFilePath)
	if err != nil {
		return nil, err
	}

	maxEdgesInCell := c.graph.GetMaxEdgesInCell()

	c.lowestHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](uint32(maxEdgesInCell), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	c.levelHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index](uint32(da.OVERLAY_INFO_SIZE), uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	c.Build(costFunction)
	c.logger.Sugar().Infof("Building stalling tables...")
	m = metrics.NewMetric(c.graph.NumberOfVertices(), c.timefunctionFilePath, c.ow, c.metricOutputFilePath)

	m.BuildStallingTables(c.overlayGraph, c.graph)

	c.logger.Sugar().Infof("Customization step completed successfully.")

	lm := landmark.NewLandmark()
	numberOfLandmarks := viper.GetInt("landmarks")
	err = lm.PreprocessALT(numberOfLandmarks, m, c.graph, c.logger)
	if err != nil {
		panic(err)
	}
	err = lm.WriteLandmark(c.landmarkFile, c.graph.NumberOfVertices())
	if err != nil {
		panic(err)
	}

	// ini write metrics harus terakhir karena bakal di update background worker
	err = m.WriteToFile(c.metricOutputFilePath)
	if err != nil {
		return nil, err
	}
	return m, nil
}

// just for shortest path test
func (c *Customizer) CustomizeDirect() (*metrics.Metric, error) {

	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = da.NewOverlayWeights(c.overlayGraph.GetWeightVectorSize())
	c.logger.Info(fmt.Sprintf("number of shortcuts: %v", c.ow.GetNumberOfShortcuts()))

	var m *metrics.Metric
	emptyCf := costfunction.NewTimeCostFunctionEmpty()
	maxEdgesInCell := c.graph.GetMaxEdgesInCell()

	c.lowestHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey](uint32(maxEdgesInCell), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	c.levelHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index](uint32(da.OVERLAY_INFO_SIZE), uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	c.Build(emptyCf)
	c.logger.Sugar().Infof("Building stalling tables...")
	m = metrics.NewMetric(c.graph.NumberOfVertices(), c.timefunctionFilePath, c.ow, "")
	m.BuildStallingTables(c.overlayGraph, c.graph)
	c.logger.Sugar().Infof("Customization step completed successfully.")

	return m, nil
}

// makeEdgeMaxSpeeds. bikin map dari edgeId -> edge max speed (in m/s)
// updatedEdgeMaxSpeeds in m/s.
func (c *Customizer) makeEdgeMaxSpeeds(updatedEdgeIds []da.Index, updatedEdgeMaxSpeeds []float64) []float64 {

	numOfEdges := c.graph.NumberOfOutEdges()

	edgeMaxSpeeds := make([]float64, numOfEdges)
	oldEdgeSpeeds := c.graph.GetEdgeSpeeds()
	for eId := 0; eId < numOfEdges; eId++ {
		edgeMaxSpeeds[eId] = oldEdgeSpeeds[eId]
	}

	for i := 0; i < len(updatedEdgeIds); i++ {
		uEId := updatedEdgeIds[i]
		edgeMaxSpeeds[uEId] = updatedEdgeMaxSpeeds[i]
	}
	return edgeMaxSpeeds
}

type customizerCell struct {
	cell       da.Cell
	cellNumber da.Pv
}

func newCustomizerCell(cell da.Cell, cellNumber da.Pv) customizerCell {
	return customizerCell{cell: cell, cellNumber: cellNumber}
}

/*
Customization Phase of Customizable Route Planning (CRP) by delling et al. see section 5.2 Customization: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary vertices, and shortucts within any partition

worst case buildLowestLevel: O( c_1 * n_op * (m_p* log(m_p)) )
worst case buildLevel in level l:  O( c_l * n_op * (n_op + \hat{m_p})* log(n_op) )

worst case crp customization: O(  c_1 * n_op * (m_p* log(m_p)) + c_l * n_op * (n_op + \hat{m_p}) * log(n_op)  )
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

/*
// buildLowestLevel. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm (restricted to cell C) from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// restricted to cell C: menggunakan only vertices dan edges yang terletak pada cell C.
// this function is parallelized using goroutines worker pool
*/
func (c *Customizer) buildLowestLevel(
	costFunction costfunction.CostFunction) {

	cellMapInLevelOne := c.overlayGraph.GetAllCellsInLevel(1)

	cellCliqueOutChan := make(chan []cellCustomizationRes, 50)

	wg := sync.WaitGroup{}

	buildCellClique := func(job customizerCell) {

		cell := job.cell
		cellNumber := job.cellNumber

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationRes, cellWeightSize)

		dijkstra := func(entries <-chan da.Index) {
			/*
				let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary vertices, and shortucts within any partition
				let n,m,k denote the number vertices,edges, and number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), respectively.


				pq contains at most all edges in a cell level 1
				extractMin at most m_p
				decreaseKey and insert at most m_p
				we do dijkstra for all entries in the cell, num of entries is at most n_op
				worst case: O( n_op * (m_p* log(m_p)) )

			*/
			for i := range entries {
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)
				overlayVertex := c.overlayGraph.GetVertex(startOverlayVertexId)
				start := overlayVertex.GetOriginalVertex()
				maxSearchSize := c.graph.GetMaxEdgesInCell()

				pq := c.lowestHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
				pq.Clear()
				done := func() {
					c.lowestHeapPool.Put(pq)
				}

				travelTime := make([]float64, maxSearchSize)
				overlayTravelTime := make([]float64, c.overlayGraph.NumberOfOverlayVertices())
				for q := 0; q < len(travelTime); q++ {
					travelTime[q] = pkg.INF_WEIGHT
				}
				for q := 0; q < len(overlayTravelTime); q++ {
					overlayTravelTime[q] = pkg.INF_WEIGHT
				}
				forwardCellOffset := c.graph.GetInEdgeCellOffset(start)
				startInEdgeOffset := overlayVertex.GetOriginalEdge() - forwardCellOffset

				travelTime[startInEdgeOffset] = 0
				noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)

				sVertexInfo := da.NewVertexInfo(0, noPar)
				pq.Insert(startInEdgeOffset, 0, sVertexInfo, da.NewDijkstraKey(start, startInEdgeOffset))

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uKey := pqNode.GetItem()
					uId := uKey.GetNode()
					uEntryId := uKey.GetEntryExitPoint()
					uTravelTime := pqNode.GetRank()

					c.graph.ForOutEdgesOf(uId, c.graph.GetEntryOrder(uId, uEntryId+forwardCellOffset),
						func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType,
							hwType pkg.OsmHighwayType) {
							// traverse all out edges

							v := head

							turnCost := costFunction.GetTurnCost(turnType)

							uTravelTimeWithTurnCost := uTravelTime + turnCost
							outArcCost := costFunction.GetWeight(eId, weight, length)

							newTravelTime := uTravelTimeWithTurnCost + outArcCost

							if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
								return
							}

							vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
							if vTruncatedCellNumber == cellNumber {
								vEntryId := c.graph.GetEntryOffset(v) + da.Index(entryPoint) - forwardCellOffset

								ok := util.Lt(travelTime[vEntryId], pkg.INF_WEIGHT)
								if oldvTT := travelTime[vEntryId]; !ok || (ok && newTravelTime < oldvTT) {
									travelTime[vEntryId] = newTravelTime
									if ok {
										pq.DecreaseKey(vEntryId, newTravelTime, newTravelTime, noPar)
									} else {
										vVertexInfo := da.NewVertexInfo(newTravelTime, noPar)
										pq.Insert(vEntryId, newTravelTime, vVertexInfo, da.NewDijkstraKey(v, vEntryId))
									}
								}
							} else {
								// found an exit vertex of the cell
								// save this shortcut travelTime
								exitVertexTravelTime := uTravelTimeWithTurnCost
								exitOverlayVId, _ := c.graph.GetOverlayVertex(uId, exitPoint, true) // overlay vetex id of exit vertex c_1(u).
								ok := util.Lt(overlayTravelTime[exitOverlayVId], pkg.INF_WEIGHT)
								if !ok || (ok && exitVertexTravelTime < overlayTravelTime[exitOverlayVId]) {
									overlayTravelTime[exitOverlayVId] = exitVertexTravelTime
								}
							}
						})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitOverlayVId := c.overlayGraph.GetExitId(cell, j)
					ok := util.Lt(overlayTravelTime[exitOverlayVId], pkg.INF_WEIGHT)

					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(pkg.INF_WEIGHT, int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(overlayTravelTime[exitOverlayVId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}

				done()
			}
		}

		entries := make(chan da.Index, CELL_ENTRIES_CHAN_SIZE)
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		cellWeights := make([]cellCustomizationRes, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

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
				c.ow.SetWeight(w.getIndex(), w.getTravelTime())
			}
			wg.Done()
		}
	}()

	for cellNumber, cell := range cellMapInLevelOne {
		wg.Add(1)
		gopool.CtxGo(context.Background(), func() { buildCellClique(newCustomizerCell(cell, cellNumber)) })
	}

	// let c_1 be the number of cells in level 1
	// worst case buildLowestLevel: O( c_1 * n_op * (m_p* log(m_p)) )

	wg.Wait()
	close(cellCliqueOutChan)
}

// buildLevel. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm (menggunakan shortcut edges pada subcells of the level-i cell) from each entry vertices of the cell to all exit vertices of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
func (c *Customizer) buildLevel(
	costFunction costfunction.CostFunction, level int) {

	levelInfo := c.overlayGraph.GetLevelInfo()
	cellMapInLevel := c.overlayGraph.GetAllCellsInLevel(level)

	cellCliqueOutChan := make(chan []cellCustomizationRes, 50)

	wg := sync.WaitGroup{}

	buildCellClique := func(job customizerCell) {

		cell := job.cell
		cellNumber := job.cellNumber

		cellWeightSize := cell.GetNumEntryPoints() * cell.GetNumExitPoints()
		dijkstraResChan := make(chan cellCustomizationRes, cellWeightSize)

		dijkstra := func(entries <-chan da.Index) {
			/*
				let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary vertices, and shortucts within any partition
				let n,m,k denote the number vertices,edges, and number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), respectively.


				pq contains at most all overlay vertices in all subcells of this cell in level-1
				extractMin at most n_op
				decreaseKey and insert at most \hat{m_p}

				we do dijkstra for all entries in the cell, num of entries is at most n_op
				worst case: O( n_op * (n_op + \hat{m_p})* log(n_op) )
			*/
			for i := range entries {

				pq := c.levelHeapPool.Get().(*da.QueryHeap[da.Index])
				pq.Clear()
				done := func() {
					c.levelHeapPool.Put(pq)
				}

				travelTime := make([]float64, c.overlayGraph.NumberOfOverlayVertices())
				for v := 0; v < c.overlayGraph.NumberOfOverlayVertices(); v++ {
					travelTime[v] = pkg.INF_WEIGHT
				}
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)

				noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)
				sVertexInfo := da.NewVertexInfo(0, noPar)

				pq.Insert(startOverlayVertexId, 0, sVertexInfo, startOverlayVertexId)

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					uTravelTime := pqNode.GetRank()

					uOverlayVertex := c.overlayGraph.GetVertex(uOverlayId)
					uTruncatedLevel := c.overlayGraph.TruncateToLevel(uOverlayVertex.GetCellNumber(), uint8(level))
					util.AssertPanic(uTruncatedLevel == cellNumber, "current truncated cell number and boundary vertex truncated cell number must be the same!")

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exitOverlayVertex da.Index, wOffset da.Index) {
						// iterate all shortcuts (u, \cdot)

						shortcutWeight := c.ow.GetWeight(wOffset)

						newTravelTime := uTravelTime + shortcutWeight

						if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
							return
						}

						oldExitTravelTime := travelTime[exitOverlayVertex]
						exitAlreadyLabelled := util.Lt(travelTime[exitOverlayVertex], pkg.INF_WEIGHT)
						if !exitAlreadyLabelled || (exitAlreadyLabelled && newTravelTime < oldExitTravelTime) {
							travelTime[exitOverlayVertex] = newTravelTime
							// visit neighbor of exitOverlayVertex
							//
							exitOverlayVertex := c.overlayGraph.GetVertex(exitOverlayVertex)
							neighborVertex := exitOverlayVertex.GetNeighborOverlayVertex()
							neighborOverlayVertex := c.overlayGraph.GetVertex(neighborVertex)
							// cut edge (exitOverlayVertex, neighborOverlayVertex)
							exitOriEdgeId := exitOverlayVertex.GetOriginalEdge()

							if levelInfo.TruncateToLevel(neighborOverlayVertex.GetCellNumber(), uint8(level)) == cellNumber {
								exOriEdge := c.graph.GetOutEdge(exitOriEdgeId)
								boundaryArcWeight := costFunction.GetWeight(exOriEdge.GetEdgeId(), exOriEdge.GetWeight(), exOriEdge.GetLength())

								newNeighborTravelTime := newTravelTime + boundaryArcWeight
								oldNTravelTime := travelTime[neighborVertex]
								nAlreadyLabelled := util.Lt(travelTime[neighborVertex], pkg.INF_WEIGHT)

								if !nAlreadyLabelled ||
									(nAlreadyLabelled && newNeighborTravelTime < oldNTravelTime) {
									travelTime[neighborVertex] = newNeighborTravelTime

									if !nAlreadyLabelled {
										vVertexInfo := da.NewVertexInfo(newTravelTime, noPar)
										pq.Insert(neighborVertex, newNeighborTravelTime, vVertexInfo, neighborVertex)
									} else {
										pq.DecreaseKey(neighborVertex, newNeighborTravelTime,
											newNeighborTravelTime, noPar)
									}
								}
							}
						}
					})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitOverlayVId := c.overlayGraph.GetExitId(cell, j)

					ok := util.Lt(travelTime[exitOverlayVId], pkg.INF_WEIGHT)
					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(pkg.INF_WEIGHT, int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(travelTime[exitOverlayVId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					}
				}

				done()
			}
		}

		entries := make(chan da.Index, CELL_ENTRIES_CHAN_SIZE)
		for worker := 1; worker <= CELL_WORKER; worker++ {
			go dijkstra(entries)
		}

		cellWeights := make([]cellCustomizationRes, cell.GetNumEntryPoints()*cell.GetNumExitPoints())

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
				c.ow.SetWeight(w.getIndex(), w.getTravelTime())
			}
			wg.Done()
		}
	}()

	for pv, cell := range cellMapInLevel {
		wg.Add(1)
		gopool.CtxGo(context.Background(), func() {
			buildCellClique(newCustomizerCell(cell, pv))
		})
	}

	// let c_l be the number of cells in level l
	// worst case buildLevel:  O( c_l * n_op * (n_op + \hat{m_p})* log(n_op) )

	wg.Wait()
	close(cellCliqueOutChan)
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

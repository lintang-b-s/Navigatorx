package customizer

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

type Customizer[W util.RoutingNumber] struct {
	ow           *da.OverlayWeights[W]
	graph        *da.Graph
	overlayGraph *da.OverlayGraph

	levelOneHeapPool   sync.Pool
	upperLevelHeapPool sync.Pool
	segmentLookupTable *da.LookupTable[*da.SegmentKV]
	turnLookupTable    *da.LookupTable[*da.TurnKV]

	logger                              *zap.Logger
	edgeSpeedsFilePath, turnPenFilePath []string
	graphFilePath                       string
	overlayGraphFilePath                string
	metricOutputFilePath                string
	timefunctionFilePath                string
	prepWeightFunctionFilePath          string
	prepWeightFunction                  *met.TimeFunction[W]
	landmarkFile                        string
}

func getCustomizerFilePath(fileType pkg.FILE_TYPE) (
	graph, overlayGraph, landmark, metrics, tf string,
) {
	root := config.ProfilesRoot()
	base := fmt.Sprintf("%s/%s/%s", root, pkg.ProfileName, pkg.RegionName)
	return base + ".ngraph",
		base + "_overlay_graph.ngraph",
		base + "_landmark.nlm",
		base + "_metrics.nmt",
		base + ".ntf"
}

func NewCustomizer[W util.RoutingNumber](
	logger *zap.Logger) *Customizer[W] {
	util.ActivateMode[W]()
	gf, ogf, lmf, metf, tff := getCustomizerFilePath(pkg.TIPE)
	cst := &Customizer[W]{
		graphFilePath:              gf,
		overlayGraphFilePath:       ogf,
		metricOutputFilePath:       metf,
		logger:                     logger,
		timefunctionFilePath:       tff,
		prepWeightFunctionFilePath: met.PrepTimeFunctionPath(),
		landmarkFile:               lmf,
	}

	return cst
}

func (c *Customizer[W]) SetEdgeSpeedsFilePath(filePath []string) {
	c.edgeSpeedsFilePath = filePath
}

func (c *Customizer[W]) SetTurnPenaltiesFilePath(filePath []string) {
	c.turnPenFilePath = filePath
}

func NewCustomizerDirect[W util.RoutingNumber](
	graph *da.Graph,
	overlayGraph *da.OverlayGraph,
	prepWeightFunction *met.TimeFunction[W],
	logger *zap.Logger,
) *Customizer[W] {
	util.ActivateMode[W]()
	return &Customizer[W]{
		graph:              graph,
		overlayGraph:       overlayGraph,
		prepWeightFunction: prepWeightFunction,
		logger:             logger,
	}
}

func (c *Customizer[W]) Customize() (*met.Metric[W], error) {

	var err error
	if pkg.TIPE == pkg.ROUTER || pkg.TIPE == pkg.TEST {
		// only for osm routiing engine
		rf := config.ProfilesRoot()
		seglkFilename := fmt.Sprintf("%s/%s/%s_segment.nlk", rf, pkg.ProfileName, pkg.RegionName)
		turnlkFilename := fmt.Sprintf("%s/%s/%s_turn.nlk", rf, pkg.ProfileName, pkg.RegionName)
		c.segmentLookupTable, err = da.ReadSegmentTable(seglkFilename)
		if err != nil {
			return nil, fmt.Errorf("Customize: failed to read segmentLookupTable from %s: %w", seglkFilename, err)
		}
		c.turnLookupTable, err = da.ReadTurnTable(turnlkFilename)
		if err != nil {
			return nil, fmt.Errorf("Customize: failed to read turnLookupTable from %s: %w", turnlkFilename, err)
		}
	}

	c.logger.Sugar().Infof("Starting customization step of Customizable Route Planning...")
	c.logger.Sugar().Infof("Reading graph from %s", c.graphFilePath)
	c.graph, err = da.ReadGraph(c.graphFilePath)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to read graph from %s: %w", c.graphFilePath, err)
	}

	c.logger.Sugar().Infof("Reading overlay graph from %s", c.overlayGraphFilePath)
	c.overlayGraph, err = da.ReadOverlayGraph(c.overlayGraphFilePath)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to read overlay graph from %s: %w", c.overlayGraphFilePath, err)
	}
	c.prepWeightFunction, err = met.ReadCostFunctionFromFile[W](c.prepWeightFunctionFilePath)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to read preprocessing time function from %s: %w", c.prepWeightFunctionFilePath, err)
	}

	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = da.NewOverlayWeights[W](c.overlayGraph.GetWeightVectorSize())
	c.logger.Info(fmt.Sprintf("number of shortcuts: %v", c.ow.GetNumberOfShortcuts()))
	var m *met.Metric[W]

	upSegmentIds := make([]da.Index, 0)
	upSegmentSpeedLimits := make([]float64, 0)

	lastSegmentSpeedFiles := make([]string, 0)
	if len(c.edgeSpeedsFilePath) != 0 {
		for _, currSpeedFilePath := range c.edgeSpeedsFilePath {
			currSegmentIds, currSegmentSpeedLimits, err := c.readSegmentSpeedsFromFile(currSpeedFilePath)
			if err != nil {
				return nil, fmt.Errorf("Customize: failed to read edge speeds from %s: %w", currSpeedFilePath, err)
			}
			upSegmentIds = append(upSegmentIds, currSegmentIds...)
			upSegmentSpeedLimits = append(upSegmentSpeedLimits, currSegmentSpeedLimits...)
			lastSegmentSpeedFiles = append(lastSegmentSpeedFiles, currSpeedFilePath)
		}
	}

	upTurnIds := make([]da.Index, 0)
	upTurnPenalties := make([]float64, 0)
	lastTurnPenaltiesFiles := make([]string, 0)
	if len(c.turnPenFilePath) != 0 {
		for _, turnPenFilePath := range c.turnPenFilePath {
			currturnTableIds, currTurnPenalties, err := c.readTurnPenaltiesFromFile(turnPenFilePath)
			if err != nil {
				return nil, fmt.Errorf("Customize: failed to read turn penalties from %s: %w", turnPenFilePath, err)
			}
			upTurnIds = append(upTurnIds, currturnTableIds...)
			upTurnPenalties = append(upTurnPenalties, currTurnPenalties...)
			lastTurnPenaltiesFiles = append(lastTurnPenaltiesFiles, turnPenFilePath)
		}
	}

	wf := c.update(upSegmentIds, upSegmentSpeedLimits, upTurnIds, upTurnPenalties)
	maxVerticesIncell := c.graph.GetMaxVerticesInCell()

	c.levelOneHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.QueryKey, W](uint32(maxVerticesIncell), uint32(maxVerticesIncell), da.MAP_STORAGE, true)
		},
	}

	c.upperLevelHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](uint32(da.OVERLAY_VERTICES_SIZE), uint32(maxVerticesIncell), da.MAP_STORAGE, true)
		},
	}

	lm := landmark.NewLandmark[W]()

	viper.SetDefault("landmarks", 8)
	wg := sync.WaitGroup{}
	wg.Go(func() {
		numberOfLandmarks := viper.GetInt("landmarks")
		err = lm.PreprocessALT(numberOfLandmarks, wf, c.graph, c.logger)
		if err != nil {
			panic(err)
		}
	})

	c.Build(wf)
	c.logger.Sugar().Infof("Writing metrics data...")
	m = met.NewMetric(c.graph.NumberOfVertices(), c.timefunctionFilePath, c.ow, c.metricOutputFilePath)

	wg.Wait()
	err = lm.WriteLandmark(c.landmarkFile, c.graph.NumberOfVertices())
	if err != nil {
		panic(err)
	}

	m.SetLastSegmentSpeedFiles(lastSegmentSpeedFiles)
	m.SetLastTurnPenaltyFiles(lastTurnPenaltiesFiles)
	// ini write metrics harus terakhir karena bakal di update background worker
	err = m.WriteToFile(c.metricOutputFilePath)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to write metric output to %s: %w", c.metricOutputFilePath, err)
	}
	err = wf.WriteToFile(c.timefunctionFilePath)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to write updated time function output to %s: %w", c.timefunctionFilePath, err)
	}

	c.logger.Sugar().Infof("Customization step completed successfully.")

	return m, nil
}

// just for shortest path test
func (c *Customizer[W]) CustomizeDirect() (*met.Metric[W], error) {

	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = da.NewOverlayWeights[W](c.overlayGraph.GetWeightVectorSize())
	c.logger.Info(fmt.Sprintf("number of shortcuts: %v", c.ow.GetNumberOfShortcuts()))

	var m *met.Metric[W]

	maxVerticesIncell := c.graph.GetMaxVerticesInCell()

	c.levelOneHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.QueryKey, W](uint32(maxVerticesIncell), uint32(maxVerticesIncell), da.MAP_STORAGE, true)
		},
	}

	c.upperLevelHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](uint32(da.OVERLAY_VERTICES_SIZE), uint32(maxVerticesIncell), da.MAP_STORAGE, true)
		},
	}

	wf := c.prepWeightFunction
	c.Build(wf)
	m = met.NewMetric(c.graph.NumberOfVertices(), c.timefunctionFilePath, c.ow, "")
	m.SetTimeFunction(wf)
	c.logger.Sugar().Infof("Customization step completed successfully.")

	return m, nil
}

// update.
func (c *Customizer[W]) update(
	upVIds []da.Index,
	upSegmentSpeedLimits []float64,
	upTurnEdgeIds []da.Index,
	upTurnPenalties []float64,
) *met.TimeFunction[W] {

	upEbgNodeIds := make([]da.Index, 0, len(upSegmentSpeedLimits))

	upEbgEdgeIds := make([]da.Index, 0, len(upSegmentSpeedLimits))
	upSpLimits := make([]uint32, 0, len(upSegmentSpeedLimits))
	for i := 0; i < len(upVIds); i++ {
		speed := util.SpeedFromKilometerPerHour(upSegmentSpeedLimits[i])
		u := upVIds[i]

		c.graph.ForOutEdgeIdsOf(u, func(eId da.Index) {
			// update list of updated weight of edge ids
			upEbgEdgeIds = append(upEbgEdgeIds, eId)
			upSpLimits = append(upSpLimits, speed)
			upEbgNodeIds = append(upEbgNodeIds, u)
		})

	}

	upTurnCosts := make([]uint16, len(upTurnEdgeIds))
	for i := 0; i < len(upTurnEdgeIds); i++ {
		tc := util.QuantizeTurnCost(upTurnPenalties[i], math.IsInf(upTurnPenalties[i], 1))
		upTurnCosts[i] = tc
	}

	wf := c.prepWeightFunction.Update(upEbgNodeIds, upEbgEdgeIds, upSpLimits, upTurnEdgeIds, upTurnCosts)
	return wf
}

type customizerCell struct {
	cell       da.Cell
	cellNumber da.Pv
}

func newCustomizerCell(cell da.Cell, cellNumber da.Pv) customizerCell {
	return customizerCell{cell: cell, cellNumber: cellNumber}
}

/*
Customization Phase of Customizable Route Planning (CRP) by delling et al. read section 5.2 Customization: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary vertices, and shortcuts within any cell
let c_1, c_l be the number of cells in level 1 and the number of cells in level l.

worst case buildLowestLevel: O( c_1 * n_op * (m_p* log(m_p)) )
worst case buildLevel in level l:  O( c_l * n_op * (n_op + \hat{m_p})* log(n_op) )

worst case crp customization: O(  c_1 * n_op * (m_p* log(m_p)) + c_l * n_op * (n_op + \hat{m_p}) * log(n_op)  )
*/
func (c *Customizer[W]) Build(wf *met.TimeFunction[W]) {
	c.buildLowestLevel(wf)

	c.logger.Info("finished crp customization level 1")
	totLevel := c.overlayGraph.GetLevelData().GetLevelCount()
	for level := 2; level <= totLevel; level++ {
		c.buildLevel(wf, level)
		c.logger.Sugar().Infof("finished crp customization level %v", level)
	}
}

type cellCustomizationRes[W util.RoutingNumber] struct {
	cost  W
	index int
}

func NewCellCustomizationResult[W util.RoutingNumber](cost W, index int) cellCustomizationRes[W] {
	return cellCustomizationRes[W]{cost, index}
}

func (cc cellCustomizationRes[W]) getCost() W {
	return cc.cost
}

func (cc cellCustomizationRes[W]) getIndex() int {
	return cc.index
}

// (acknowledgment) inspired by crp customization code implementation by michael wegner: https://github.com/michaelwegner/CRP/blob/master/datastructures/OverlayWeights.cpp

/*
// buildLowestLevel. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm (restricted to cell C) from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// restricted to cell C: menggunakan only vertices dan edges yang terletak pada cell C.
// this function is parallelized using goroutines worker pool
// read section 5.2 Customization: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
(acknowledgment) inspired by crp customization code implementation by michael wegner: https://github.com/michaelwegner/CRP/blob/master/datastructures/OverlayWeights.cpp
*/
func (c *Customizer[W]) buildLowestLevel(wf *met.TimeFunction[W]) {

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
				let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary/overlay vertices, and shortcuts within any cell
				let n,m,k denote the number vertices,edges, and number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), respectively.


				pq contains at most all edges in a cell level 1
				extractMin at most n_p
				decreaseKey and insert at most m_p
				we do dijkstra for all entries in the cell, num of entries is at most n_op
				worst case: O( n_op * ((m_p + n_p)* log(n_p)) )

			*/
			for i := range entries {
				sOvId := c.overlayGraph.GetCellEntry(cell, i)
				overlayVertex := c.overlayGraph.GetVertex(sOvId)
				start := overlayVertex.GetOrigVId()

				pq := c.levelOneHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
				pq.Clear()
				done := func() {
					c.levelOneHeapPool.Put(pq)
				}

				overlayCost := make(map[da.Index]W, da.OVERLAY_CELL_SIZE)

				noPar := da.NewParentVertex(da.INVALID_VERTEX_ID)

				sVertexData := da.NewVData(W(0), noPar)
				pq.Insert(start, 0, sVertexData, da.NewDijkstraKey(start))

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uKey := pqNode.GetItem()
					uId := uKey.GetNode()
					uCost := pqNode.GetRank()

					c.graph.ForOutEdgesOf(uId, func(eId, head, entryPoint da.Index) {
						// traverse all out edges
						v := head
						eCost := wf.GetWeight(eId)
						newVCost := uCost + eCost
						if util.Ge(newVCost, util.Infinity[W]()) {
							return
						}

						vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
						if vTruncatedCellNumber == cellNumber {

							oldvCost := pq.GetCost(v)
							ok := util.Lt(oldvCost, util.Infinity[W]())
							if !ok || (ok && util.Lt(newVCost, oldvCost)) {

								if ok {
									pq.DecreaseKey(v, newVCost, newVCost, noPar)
								} else {
									vVertexData := da.NewVData(newVCost, noPar)
									pq.Insert(v, newVCost, vVertexData, da.NewDijkstraKey(v))
								}
							}
						} else {
							// found an exit vertex of the cell
							// save this shortcut cost
							// v is in another cell
							exitVertexCost := uCost
							exitPoint := c.graph.GetExitOrder(uId, eId)
							exOvId, _ := c.graph.GetOverlayVertex(uId, exitPoint, true) // overlay vetex id of exit vertex c_1(u).
							_, ok := overlayCost[exOvId]
							if !ok || (ok && util.Lt(exitVertexCost, overlayCost[exOvId])) {
								overlayCost[exOvId] = exitVertexCost
							}
						}
					})
				}

				// stores all cost of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exOvId := c.overlayGraph.GetCellExit(cell, j)
					_, ok := overlayCost[exOvId]
					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(util.Infinity[W](), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(overlayCost[exOvId], int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
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

// buildLevel. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm (menggunakan shortcut edges & cut edges pada subcells of the level-i cell) from each entry boundary/overlay vertices of the cell to all exit boundary/overlay vertices of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
// read section 5.2 Customization: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
// (acknowledgment) inspired by crp customization code implementation by michael wegner: https://github.com/michaelwegner/CRP/blob/master/datastructures/OverlayWeights.cpp
func (c *Customizer[W]) buildLevel(wf *met.TimeFunction[W], level int) {

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
				let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary/overlay vertices, and shortcuts within any cell
				let n,m,k denote the number vertices,edges, and number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), respectively.


				pq contains at most all overlay vertices in all subcells of this cell in level-1
				extractMin at most n_op
				decreaseKey and insert at most \hat{m_p}

				we do dijkstra for all entries in the cell, num of entries is at most n_op
				worst case: O( n_op * (n_op + \hat{m_p})* log(n_op) )
			*/
			for i := range entries {

				pq := c.upperLevelHeapPool.Get().(*da.QueryHeap[da.Index, W])
				pq.Clear()
				done := func() {
					c.upperLevelHeapPool.Put(pq)
				}

				sOvId := c.overlayGraph.GetCellEntry(cell, i)

				noPar := da.NewParentVertex(da.INVALID_VERTEX_ID)
				sVertexData := da.NewVData(W(0), noPar)

				pq.Insert(sOvId, 0, sVertexData, sOvId)

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					uCost := pqNode.GetRank()

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exOvId da.Index, wOffset da.Index) {
						// iterate all shortcuts (u, \cdot)
						shortcutWeight := c.ow.GetWeight(wOffset)
						newVCost := uCost + shortcutWeight
						if util.Ge(newVCost, util.Infinity[W]()) {
							return
						}

						oldVCost := pq.GetCost(exOvId)
						vLabelled := util.Lt(oldVCost, util.Infinity[W]())
						if !vLabelled || (vLabelled && util.Lt(newVCost, oldVCost)) {
							vvData := da.NewVData(newVCost, noPar)
							pq.Set(exOvId, vvData, exOvId)

							// visit neighbor of exit overlay vertex exOvId
							exOverlayVertex := c.overlayGraph.GetVertex(exOvId)
							nOvId := exOverlayVertex.GetNeighborOverlayVertex()
							nOverlayVertex := c.overlayGraph.GetVertex(nOvId)
							// cut edge (exOverlayVertex, nOverlayVertex)
							cutOutEdgeId := exOverlayVertex.GetCutEdge()

							nTruncatedCellNumber := levelData.TruncateToLevel(nOverlayVertex.GetCellNumber(), uint8(level))
							if nTruncatedCellNumber == cellNumber {
								cutEdgeWeight := wf.GetWeight(cutOutEdgeId)
								nnCost := newVCost + cutEdgeWeight
								oldNCost := pq.GetCost(nOvId)
								nLabelled := util.Lt(oldNCost, util.Infinity[W]())
								if util.Ge(nnCost, util.Infinity[W]()) {
									return
								}

								if !nLabelled || (nLabelled && util.Lt(nnCost, oldNCost)) {

									if !nLabelled {
										nvData := da.NewVData(nnCost, noPar)
										pq.Insert(nOvId, nnCost, nvData, nOvId)
									} else {
										pq.DecreaseKey(nOvId, nnCost,
											nnCost, noPar)
									}
								}
							}
						}
					})
				}

				// stores all cost of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exOvId := c.overlayGraph.GetCellExit(cell, j)

					oldExitVertexCost := pq.GetCost(exOvId)
					ok := util.Lt(oldExitVertexCost, util.Infinity[W]())
					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(util.Infinity[W](), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
					} else {
						dijkstraResChan <- NewCellCustomizationResult(oldExitVertexCost, int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
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

func (c *Customizer[W]) SetGraph(graph *da.Graph) {
	c.graph = graph
}

func (c *Customizer[W]) SetOverlayGraph(overlayGraph *da.OverlayGraph) {
	c.overlayGraph = overlayGraph
}

func (c *Customizer[W]) SetOverlayWeight(ow *da.OverlayWeights[W]) {
	c.ow = ow
}

func (c *Customizer[W]) GetGraph() *da.Graph {
	return c.graph
}

func (c *Customizer[W]) GetOverlayGraph() *da.OverlayGraph {
	return c.overlayGraph
}

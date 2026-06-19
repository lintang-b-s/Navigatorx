package customizer

import (
	"bufio"
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

type Customizer[W util.RoutingNumber] struct {
	ow                                        *da.OverlayWeights[W]
	graph                                     *da.Graph
	overlayGraph                              *da.OverlayGraph
	lowestHeapPool                            sync.Pool
	levelHeapPool                             sync.Pool
	verticesLookupTable                       *LookupTable[uint64]
	logger                                    *zap.Logger
	edgeSpeedsFilePath, turnPenaltiesFilePath []string
	graphFilePath                             string
	overlayGraphFilePath                      string
	metricOutputFilePath                      string
	timefunctionFilePath                      string
	preprocessingTimeFunctionFilePath         string
	preprocessingTimeFunction                 *costfunction.TimeFunction[W]
	landmarkFile                              string
}

func NewCustomizer(graphFilePath, overlayGraphFilePath, metricOutputFilePath, timefunctionFilePath, landmarkFile string,
	logger *zap.Logger) *Customizer[int32] {
	util.USE_INT32 = true
	cst := &Customizer[int32]{
		graphFilePath:                     graphFilePath,
		overlayGraphFilePath:              overlayGraphFilePath,
		metricOutputFilePath:              metricOutputFilePath,
		logger:                            logger,
		timefunctionFilePath:              timefunctionFilePath,
		preprocessingTimeFunctionFilePath: costfunction.PreprocessingTimeFunctionPath(graphFilePath),
		landmarkFile:                      landmarkFile,
	}

	return cst
}

func (c *Customizer[W]) SetEdgeSpeedsFilePath(filePath []string) {
	c.edgeSpeedsFilePath = filePath
}

func (c *Customizer[W]) SetTurnPenaltiesFilePath(filePath []string) {
	c.turnPenaltiesFilePath = filePath
}

func NewCustomizerDirect[W util.RoutingNumber](
	graph *da.Graph,
	overlayGraph *da.OverlayGraph,
	preprocessingTimeFunction *costfunction.TimeFunction[W],
	logger *zap.Logger,
) *Customizer[W] {
	var zero W
	switch any(zero).(type) {
	case float64:
		util.USE_INT32 = false
	default:
		util.USE_INT32 = true
	}
	return &Customizer[W]{
		graph:                     graph,
		overlayGraph:              overlayGraph,
		preprocessingTimeFunction: preprocessingTimeFunction,
		logger:                    logger,
	}
}

func (c *Customizer[W]) Customize() (*metrics.Metric[W], error) {

	var err error
	readBuf := bufio.NewReaderSize(nil, util.BUFIO_SIZE)

	c.logger.Sugar().Infof("Starting customization step of Customizable Route Planning...")
	c.logger.Sugar().Infof("Reading graph from %s", c.graphFilePath)
	c.graph, err = da.ReadGraph(c.graphFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to read graph from %s: %w", c.graphFilePath, err)
	}

	c.logger.Sugar().Infof("Reading overlay graph from %s", c.overlayGraphFilePath)
	c.overlayGraph, err = da.ReadOverlayGraph(c.overlayGraphFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to read overlay graph from %s: %w", c.overlayGraphFilePath, err)
	}
	c.preprocessingTimeFunction, err = costfunction.ReadPreprocessingFromFile[W](c.preprocessingTimeFunctionFilePath)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to read preprocessing time function from %s: %w", c.preprocessingTimeFunctionFilePath, err)
	}

	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = da.NewOverlayWeights[W](c.overlayGraph.GetWeightVectorSize())
	c.logger.Info(fmt.Sprintf("number of shortcuts: %v", c.ow.GetNumberOfShortcuts()))
	var m *metrics.Metric[W]

	vertexOsmIds := c.graph.GetVertexOsmIds()
	c.verticesLookupTable = NewLookupTable(vertexOsmIds, func(a, b uint64) bool {
		return a < b
	})

	updatedEdgeIds := make([]da.Index, 0)
	updatedEdgeMaxSpeeds := make([]float64, 0)

	lastSegmentSpeedFiles := make([]string, 0)
	if len(c.edgeSpeedsFilePath) != 0 {
		for _, currSpeedFilePath := range c.edgeSpeedsFilePath {
			currEdgeIds, currEdgeMaxSpeeds, err := c.readEdgeSpeedsFromFile(currSpeedFilePath)
			if err != nil {
				return nil, fmt.Errorf("Customize: failed to read edge speeds from %s: %w", currSpeedFilePath, err)
			}
			updatedEdgeIds = append(updatedEdgeIds, currEdgeIds...)
			updatedEdgeMaxSpeeds = append(updatedEdgeMaxSpeeds, currEdgeMaxSpeeds...)
			lastSegmentSpeedFiles = append(lastSegmentSpeedFiles, currSpeedFilePath)
		}
	}

	updatedTurnTableIds := make([]da.Index, 0)
	updatedTurnPenalties := make([]float64, 0)
	lastTurnPenaltiesFiles := make([]string, 0)
	if len(c.turnPenaltiesFilePath) != 0 {
		for _, turnPenaltiesFilePath := range c.turnPenaltiesFilePath {
			currturnTableIds, currTurnPenalties, err := c.readTurnPenaltiesFromFile(turnPenaltiesFilePath)
			if err != nil {
				return nil, fmt.Errorf("Customize: failed to read turn penalties from %s: %w", turnPenaltiesFilePath, err)
			}
			updatedTurnTableIds = append(updatedTurnTableIds, currturnTableIds...)
			updatedTurnPenalties = append(updatedTurnPenalties, currTurnPenalties...)
			lastTurnPenaltiesFiles = append(lastTurnPenaltiesFiles, turnPenaltiesFilePath)
		}
	}

	edgeMaxSpeeds := c.makeEdgeMaxSpeeds(updatedEdgeIds, updatedEdgeMaxSpeeds)

	turnTypes := c.graph.GetTurnTypes()
	turnTable := c.makeTurnTable(turnTypes, updatedTurnTableIds, updatedTurnPenalties, edgeMaxSpeeds)
	costFunction := c.preprocessingTimeFunction.WithCustomization(edgeMaxSpeeds, turnTable)
	err = costFunction.WriteToFile(c.timefunctionFilePath)
	if err != nil {
		return nil, fmt.Errorf("Customize: failed to write time cost function to %s: %w", c.timefunctionFilePath, err)
	}

	maxEdgesInCell := c.graph.GetMaxEdgesInCell()

	c.lowestHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxEdgesInCell), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	c.levelHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](uint32(da.OVERLAY_INFO_SIZE), uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	c.Build(costFunction)
	c.logger.Sugar().Infof("Building stalling tables...")
	m = metrics.NewMetric(c.graph.NumberOfVertices(), c.timefunctionFilePath, c.ow, c.metricOutputFilePath, readBuf)

	m.BuildStallingTables(c.overlayGraph, c.graph)

	c.logger.Sugar().Infof("Customization step completed successfully.")

	lm := landmark.NewLandmark[W]()
	numberOfLandmarks := viper.GetInt("landmarks")
	err = lm.PreprocessALT(numberOfLandmarks, m, c.graph, c.logger)
	if err != nil {
		panic(err)
	}

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
	return m, nil
}

// just for shortest path test
func (c *Customizer[W]) CustomizeDirect() (*metrics.Metric[W], error) {

	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	c.ow = da.NewOverlayWeights[W](c.overlayGraph.GetWeightVectorSize())
	c.logger.Info(fmt.Sprintf("number of shortcuts: %v", c.ow.GetNumberOfShortcuts()))

	var m *metrics.Metric[W]
	readBuf := bufio.NewReaderSize(nil, util.BUFIO_SIZE)
	cf := c.preprocessingTimeFunction.WithCustomization(c.preprocessingTimeFunction.GetEdgeMaxSpeeds(), nil)
	maxEdgesInCell := c.graph.GetMaxEdgesInCell()

	c.lowestHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxEdgesInCell), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
		},
	}

	c.levelHeapPool = sync.Pool{
		New: func() any {
			return da.NewQueryHeap[da.Index, W](uint32(da.OVERLAY_INFO_SIZE), uint32(maxEdgesInCell), da.MAP_STORAGE, true)
		},
	}

	c.Build(cf)
	c.logger.Sugar().Infof("Building stalling tables...")
	m = metrics.NewMetric(c.graph.NumberOfVertices(), c.timefunctionFilePath, c.ow, "", readBuf)
	m.SetTimeFunction(cf)
	m.BuildStallingTables(c.overlayGraph, c.graph)
	c.logger.Sugar().Infof("Customization step completed successfully.")

	return m, nil
}

// makeEdgeMaxSpeeds merge old road segment speed limits & updated road segment speed limits.
func (c *Customizer[W]) makeEdgeMaxSpeeds(
	updatedEdgeIds []da.Index,
	updatedEdgeMaxSpeeds []float64,
) []uint32 {

	numOfEdges := c.graph.NumberOfOutEdges()

	edgeMaxSpeeds := make([]uint32, numOfEdges)
	oldEdgeSpeeds := c.preprocessingTimeFunction.GetEdgeMaxSpeeds()
	for eId := 0; eId < numOfEdges; eId++ {
		edgeMaxSpeeds[eId] = oldEdgeSpeeds[eId]
	}

	for i := 0; i < len(updatedEdgeIds); i++ {
		uEId := updatedEdgeIds[i]
		speed := c.preprocessingTimeFunction.SpeedFromKilometerPerHour(updatedEdgeMaxSpeeds[i])
		edgeMaxSpeeds[uEId] = speed
	}
	return edgeMaxSpeeds
}

// makeTurnTable builds turn costs using edge speeds converted to meters per second.
func (c *Customizer[W]) makeTurnTable(
	turnTypeTable []pkg.TurnType,
	updatedTurnTableIds []da.Index,
	updatedTurnPenalties []float64,
	edgeMaxSpeeds []uint32,
) []uint16 {
	mapTurnCosts := viper.GetStringMap("turncosts")
	turnTypesCost := make([]float64, 6)
	for turnTypeStr, cost := range mapTurnCosts {
		if turnTypeStr == "traffic_light" || turnTypeStr == "max_turn_cost_based_on_angle_between_edges" {
			continue
		}
		turnType := getTurnTableId(turnTypeStr)
		switch v := cost.(type) {
		case int:
			turnTypesCost[turnType] = float64(v)
		case float64:
			turnTypesCost[turnType] = float64(v)
		default:
			panic("unsupported type")
		}
	}

	trafficLightPenalty := viper.GetFloat64("turncosts.traffic_light") // in seconds
	turnCostByAngleThreshold := viper.GetFloat64("turncosts.max_turn_cost_based_on_angle_between_edges")

	turnTypesCost[pkg.NONE] = 0
	turnTypesCost[pkg.NO_ENTRY] = math.Inf(1)

	n := len(turnTypeTable)
	turnTableSeconds := make([]float64, n)
	for id := 0; id < n; id++ {
		turnType := turnTypeTable[id]
		turnTableSeconds[id] += turnTypesCost[turnType]
	}

	minResolution := c.graph.GetMinResolution()

	c.graph.ForOutEdges(func(exitPoint, head, tail, entryId, entryPoint da.Index, percentage float64, eIdFrom da.Index) {
		if c.graph.IsDummyOutEdge(eIdFrom) {
			return
		}
		vLimitFrom := c.preprocessingTimeFunction.SpeedToMetersPerSecond(edgeMaxSpeeds[eIdFrom])
		c.graph.ForOutEdgesOf(head, entryPoint, func(eIdTo, headTo da.Index, exitPoint, entryPoint, turnTableId da.Index, turnType pkg.TurnType, hwType pkg.OsmHighwayType) {
			_, fromInEdgeId := c.graph.GetTailOfOutedgeWithInEdge(eIdFrom)
			fromInEdge := c.graph.GetInEdge(fromInEdgeId)
			toOutEdge := c.graph.GetOutEdge(eIdTo)

			containsTrafficLight := fromInEdge.ContainsTrafficLight() || toOutEdge.ContainsTrafficLight()
			if containsTrafficLight {
				turnTableSeconds[turnTableId] += trafficLightPenalty
			} else {
				// only add turning cost di persimpangan bangjo
				return
			}

			fromSegmentStreetName := c.graph.GetStreetName(eIdFrom)
			toSegmentStreetName := c.graph.GetStreetName(eIdTo)

			if !isTurnCostByAngleBetweenEdgesAllowed(turnType) || isSameName(fromSegmentStreetName, toSegmentStreetName) {
				// skip kalau turnType bukan LEFT_TURN dan bukan RIGHT_TURN. skip juga kalau gak pindah jalan.
				// soale kalau di jalan tol (example: https://www.openstreetmap.org/way/1301675709#map=15/-7.63705/110.66151)
				// sering dipisah jadi beberaapa osm ways -> yang mana jadi beberapa graph edges. padahal masih bisa ngebut dan gak perlu turn costs di jalan tol??...
				return
			}

			vLimitTo := c.preprocessingTimeFunction.SpeedToMetersPerSecond(edgeMaxSpeeds[eIdTo])
			currentTurnCost := turnTableSeconds[turnTableId]

			prevVertex := c.graph.GetVertex(tail)
			tailVertex := c.graph.GetVertex(head)
			headVertex := c.graph.GetVertex(headTo)

			prevInitialBearing := geo.ComputeInitialBearing(prevVertex.GetLat(), prevVertex.GetLon(), tailVertex.GetLat(),
				tailVertex.GetLon())
			relativeBearing := geo.ComputeRelativeBearing(tailVertex.GetLat(), tailVertex.GetLon(), headVertex.GetLat(),
				headVertex.GetLon(), prevInitialBearing)
			absRelativeBearing := math.Abs(relativeBearing)
			turnAngleDeg := util.RadiansToDegree(absRelativeBearing)

			fromOutEdgeId := c.graph.GetExitIdOfInEdge(fromInEdgeId)
			l := c.preprocessingTimeFunction.DistanceToMeters(c.preprocessingTimeFunction.GetSegmentLength(fromOutEdgeId))
			lPrime := c.preprocessingTimeFunction.DistanceToMeters(c.preprocessingTimeFunction.GetSegmentLength(eIdTo))
			turningSpeed := pkg.CalcTurningSpeed(l, lPrime, minResolution, turnAngleDeg)

			if util.Eq(turningSpeed, 0) || turnType == pkg.NO_ENTRY || math.IsInf(currentTurnCost, 1) || c.graph.IsDummyOutEdge(eIdTo) {
				// gak ada turn penalty (pkg.NewTurnRest())
				return
			}

			turnCostByAngleBetweenEdges := pkg.CalcTurningCost(turningSpeed, vLimitFrom, vLimitTo)

			turnCostByAngleBetweenEdges = util.MinFloat(turnCostByAngleBetweenEdges, turnCostByAngleThreshold)

			turnTableSeconds[turnTableId] += turnCostByAngleBetweenEdges
		})
	})

	m := len(updatedTurnTableIds)
	for i := 0; i < m; i++ {
		turnTableId := updatedTurnTableIds[i]
		turnTableSeconds[turnTableId] += updatedTurnPenalties[i]
	}
	turnTable := make([]uint16, n)
	for i, seconds := range turnTableSeconds {
		turnTable[i] = util.QuantizeTurnCost(seconds, math.IsInf(seconds, 1))
	}
	return turnTable
}

func isTurnCostByAngleBetweenEdgesAllowed(turnType pkg.TurnType) bool {
	return turnType == pkg.LEFT_TURN || turnType == pkg.RIGHT_TURN
}

func isSameName(name1, name2 string) bool {
	if name1 == "" || name2 == "" {
		// seringkali di osm, nama street kosong "" (terutama di residential/living street/tertiary osm ways), better dianggap false
		// biar kalo belok masih ada turn instructionnya
		// contoh tertiary osm way yang gak ada namanya:  https://www.openstreetmap.org/way/332233207#map=17/-7.555473/110.769728
		return false
	}
	return name1 == name2
}

func getTurnTableId(turnTypeStr string) pkg.TurnType {
	var turnType pkg.TurnType
	switch turnTypeStr {
	case "left_turn":
		turnType = pkg.LEFT_TURN
	case "right_turn":
		turnType = pkg.RIGHT_TURN
	case "straight_on":
		turnType = pkg.STRAIGHT_ON
	case "u_turn":
		turnType = pkg.U_TURN
	case "no_entry":
		turnType = pkg.NO_ENTRY
	case "none":
		turnType = pkg.NONE
	default:
		panic("unsupported turn type")
	}
	return turnType
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

let n_p,m_p, n_op,and \hat{m_p} denote the maximum number of nodes, edges, boundary vertices, and shortucts within any cell
let c_1, c_l be the number of cells in level 1 and the number of cells in level l.

worst case buildLowestLevel: O( c_1 * n_op * (m_p* log(m_p)) )
worst case buildLevel in level l:  O( c_l * n_op * (n_op + \hat{m_p})* log(n_op) )

worst case crp customization: O(  c_1 * n_op * (m_p* log(m_p)) + c_l * n_op * (n_op + \hat{m_p}) * log(n_op)  )
*/
func (c *Customizer[W]) Build(costFunction *costfunction.TimeFunction[W]) {
	c.buildLowestLevel(costFunction)
	c.logger.Info("finished crp customization level 1")
	for level := 2; level <= c.overlayGraph.GetLevelInfo().GetLevelCount(); level++ {
		c.buildLevel(costFunction, level)
		c.logger.Sugar().Infof("finished crp customization level %v", level)

	}
}

type cellCustomizationRes[W util.RoutingNumber] struct {
	travelTime W
	index      int
}

func NewCellCustomizationResult[W util.RoutingNumber](travelTime W, index int) cellCustomizationRes[W] {
	return cellCustomizationRes[W]{travelTime, index}
}

func (cc cellCustomizationRes[W]) getTravelTime() W {
	return cc.travelTime
}

func (cc cellCustomizationRes[W]) getIndex() int {
	return cc.index
}

/*
// buildLowestLevel. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm (restricted to cell C) from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// restricted to cell C: menggunakan only vertices dan edges yang terletak pada cell C.
// this function is parallelized using goroutines worker pool
*/
func (c *Customizer[W]) buildLowestLevel(costFunction *costfunction.TimeFunction[W]) {

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

				pq := c.lowestHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey, W])
				pq.Clear()
				done := func() {
					c.lowestHeapPool.Put(pq)
				}

				travelTime := make([]W, maxSearchSize)
				overlayTravelTime := make([]W, c.overlayGraph.NumberOfOverlayVertices())
				for q := 0; q < len(travelTime); q++ {
					travelTime[q] = util.Infinity[W]()
				}
				for q := 0; q < len(overlayTravelTime); q++ {
					overlayTravelTime[q] = util.Infinity[W]()
				}
				forwardCellOffset := c.graph.GetInEdgeCellOffset(start)
				startInEdgeOffset := overlayVertex.GetOriginalEdge() - forwardCellOffset

				travelTime[startInEdgeOffset] = 0
				noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)

				sVertexInfo := da.NewVertexInfo(W(0), noPar)
				pq.Insert(startInEdgeOffset, 0, sVertexInfo, da.NewDijkstraKey(start, startInEdgeOffset))

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uKey := pqNode.GetItem()
					uId := uKey.GetNode()
					uEntryId := uKey.GetEntryExitPoint()
					uTravelTime := pqNode.GetRank()

					c.graph.ForOutEdgesOf(uId, c.graph.GetEntryOrder(uId, uEntryId+forwardCellOffset),
						func(eId, head da.Index, exitPoint, entryPoint, turnTableId da.Index, turnType pkg.TurnType,
							hwType pkg.OsmHighwayType) {
							// traverse all out edges

							v := head

							turnCost := costFunction.GetTurnCost(turnTableId)

							uTravelTimeWithTurnCost := uTravelTime + turnCost
							outArcCost := costFunction.GetWeight(eId)

							newTravelTime := uTravelTimeWithTurnCost + outArcCost

							if util.Ge(newTravelTime, util.Infinity[W]()) {
								return
							}

							vTruncatedCellNumber := c.overlayGraph.TruncateToLevel(c.graph.GetCellNumber(v), 1)
							if vTruncatedCellNumber == cellNumber {
								vEntryId := c.graph.GetEntryOffset(v) + da.Index(entryPoint) - forwardCellOffset

								ok := util.Lt(travelTime[vEntryId], util.Infinity[W]())
								if oldvTT := travelTime[vEntryId]; !ok || (ok && util.Lt(newTravelTime, oldvTT)) {
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
								ok := util.Lt(overlayTravelTime[exitOverlayVId], util.Infinity[W]())
								if !ok || (ok && util.Lt(exitVertexTravelTime, overlayTravelTime[exitOverlayVId])) {
									overlayTravelTime[exitOverlayVId] = exitVertexTravelTime
								}
							}
						})
				}

				// stores all travelTime of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
				for j := da.Index(0); j < cell.GetNumExitPoints(); j++ {
					exitOverlayVId := c.overlayGraph.GetExitId(cell, j)
					ok := util.Lt(overlayTravelTime[exitOverlayVId], util.Infinity[W]())

					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(util.Infinity[W](), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
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
				c.ow.SetWeight(w.getIndex(), w.getTravelTime())
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
	// worst case buildLowestLevel: O( c_1 * n_op * (m_p* log(m_p)) )

	wg.Wait()
	close(cellCliqueOutChan)
	c.logger.Sugar().Infof("number of shortcuts overlay graph level %v: %v ", 1, numberOfShortcuts)

}

// buildLevel. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm (menggunakan shortcut edges pada subcells of the level-i cell) from each entry vertices of the cell to all exit vertices of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
func (c *Customizer[W]) buildLevel(costFunction *costfunction.TimeFunction[W], level int) {

	levelInfo := c.overlayGraph.GetLevelInfo()
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

				pq := c.levelHeapPool.Get().(*da.QueryHeap[da.Index, W])
				pq.Clear()
				done := func() {
					c.levelHeapPool.Put(pq)
				}

				travelTime := make([]W, c.overlayGraph.NumberOfOverlayVertices())
				for v := 0; v < c.overlayGraph.NumberOfOverlayVertices(); v++ {
					travelTime[v] = util.Infinity[W]()
				}
				startOverlayVertexId := c.overlayGraph.GetEntryId(cell, i)

				noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)
				sVertexInfo := da.NewVertexInfo(W(0), noPar)

				pq.Insert(startOverlayVertexId, 0, sVertexInfo, startOverlayVertexId)

				for !pq.IsEmpty() {
					pqNode := pq.ExtractMin()
					uOverlayId := pqNode.GetItem()
					uTravelTime := pqNode.GetRank()

					c.overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exitOverlayVertex da.Index, wOffset da.Index) {
						// iterate all shortcuts (u, \cdot)

						shortcutWeight := c.ow.GetWeight(wOffset)

						newTravelTime := uTravelTime + shortcutWeight

						if util.Ge(newTravelTime, util.Infinity[W]()) {
							return
						}

						oldExitTravelTime := travelTime[exitOverlayVertex]
						exitAlreadyLabelled := util.Lt(travelTime[exitOverlayVertex], util.Infinity[W]())
						if !exitAlreadyLabelled || (exitAlreadyLabelled && util.Lt(newTravelTime, oldExitTravelTime)) {
							travelTime[exitOverlayVertex] = newTravelTime
							// visit neighbor of exitOverlayVertex
							//
							exitOverlayVertex := c.overlayGraph.GetVertex(exitOverlayVertex)
							neighborVertex := exitOverlayVertex.GetNeighborOverlayVertex()
							neighborOverlayVertex := c.overlayGraph.GetVertex(neighborVertex)
							// cut edge (exitOverlayVertex, neighborOverlayVertex)
							cutOutEdgeId := exitOverlayVertex.GetOriginalEdge()

							if levelInfo.TruncateToLevel(neighborOverlayVertex.GetCellNumber(), uint8(level)) == cellNumber {
								boundaryArcWeight := costFunction.GetWeight(cutOutEdgeId)

								newNeighborTravelTime := newTravelTime + boundaryArcWeight
								oldNTravelTime := travelTime[neighborVertex]
								nAlreadyLabelled := util.Lt(travelTime[neighborVertex], util.Infinity[W]())

								if !nAlreadyLabelled ||
									(nAlreadyLabelled && util.Lt(newNeighborTravelTime, oldNTravelTime)) {
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

					ok := util.Lt(travelTime[exitOverlayVId], util.Infinity[W]())
					if !ok {
						dijkstraResChan <- NewCellCustomizationResult(util.Infinity[W](), int(cell.GetCellOffset()+i*cell.GetNumExitPoints()+j))
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
				c.ow.SetWeight(w.getIndex(), w.getTravelTime())
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

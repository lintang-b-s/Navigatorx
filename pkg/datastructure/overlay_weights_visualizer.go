package datastructure

import (
	"encoding/json"
	"fmt"
	"os"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// to produce visualizations of the customization results

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
func (ow *OverlayWeights) VisualizeBuild(graph *Graph, overlayGraph *OverlayGraph,
	costFunction costfunction.CostFunction) {
	shortcutsMap := ow.visualizeBuildLowestLevel(graph, overlayGraph, costFunction)
	for level := 2; level <= overlayGraph.GetLevelInfo().GetLevelCount(); level++ {
		shortcutPaths := make([][]Coordinate, 0) // for each shortcut edges in level=level, save its shortest path
		shortcutsMap = ow.visualizeBuildLevel(graph, overlayGraph, costFunction, level, &shortcutPaths, shortcutsMap)
		writeShortcutsToJsonFile(shortcutPaths, level)
	}
}

// buildLowestLevel. build clique of each cell in the lowest level (level 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
func (ow *OverlayWeights) visualizeBuildLowestLevel(graph *Graph, overlayGraph *OverlayGraph,
	costFunction costfunction.CostFunction) map[Index]map[Index][]Coordinate {

	shortcutPaths := make([][]Coordinate, 0) // for each shortcut edges in level 1, save its shortest path
	shortcutsMap := make(map[Index]map[Index][]Coordinate, 0)
	cellMapInLevelOne := overlayGraph.GetAllCellsInLevel(1)
	for cellNumber, cell := range cellMapInLevelOne {

		for i := Index(0); i < cell.numEntryPoints; i++ {
			startOverlayVertexId := overlayGraph.GetEntryPoint(cell, i)
			overlayVertex := overlayGraph.GetVertex(startOverlayVertexId)
			start := overlayVertex.originalVertex
			pq := NewMinHeap[CRPQueryKey]()
			eta := make(map[Index]float64)
			overlayEta := make(map[Index]float64)

			parentState := make(map[CRPQueryKey]CRPQueryKey)
			bestExitState := make(map[Index]CRPQueryKey)

			forwardCellOffset := graph.GetInEdgeCellOffset(start)
			startInEdgeOffset := overlayVertex.originalEdge - forwardCellOffset

			startKey := NewCRPQueryKey(start, startInEdgeOffset)
			parentState[startKey] = NewCRPQueryKey(INVALID_VERTEX_ID, 0)

			eta[startInEdgeOffset] = 0
			pq.Insert(NewPriorityQueueNode(0, startKey))

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
						vKey := NewCRPQueryKey(v, vEntryPoint)

						if old, ok := eta[vEntryPoint]; !ok || newETA < old {
							eta[vEntryPoint] = newETA
							parentState[vKey] = uKey
							if ok {
								pq.DecreaseKey(NewPriorityQueueNode(newETA, vKey))
							} else {
								pq.Insert(NewPriorityQueueNode(newETA, vKey))
							}
						}
					} else {
						// found an exit point of the cell
						exitOverlay, _ := graph.GetOverlayVertex(uId, uint8(exitPoint), true)
						if best, ok := overlayEta[exitOverlay]; !ok || exitPointEta < best {
							overlayEta[exitOverlay] = exitPointEta
							bestExitState[exitOverlay] = uKey
						}
					}
				})
			}

			for j := Index(0); j < cell.numExitPoints; j++ {
				exitPoint := overlayGraph.GetExitPoint(cell, j)
				_, exists := overlayEta[exitPoint]
				if !exists {
					ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = pkg.INF_WEIGHT
					continue
				}

				ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = overlayEta[exitPoint]

				curKey, ok := bestExitState[exitPoint]
				if !ok {
					continue
				}

				pathCoords := make([]Coordinate, 0)
				for curKey.GetNode() != INVALID_VERTEX_ID {
					curLat, curLon := graph.GetVertexCoordinates(curKey.GetNode())
					pathCoords = append(pathCoords, NewCoordinate(curLat, curLon))

					nextKey, ok := parentState[curKey]
					if !ok {
						break
					}
					curKey = nextKey
				}
				shortcutPaths = append(shortcutPaths, pathCoords)

				if _, ok := shortcutsMap[start]; !ok {
					shortcutsMap[start] = make(map[Index][]Coordinate)
				}
				exitKey := bestExitState[exitPoint]
				shortcutsMap[start][exitKey.GetNode()] = pathCoords
			}
		}
	}

	writeShortcutsToJsonFile(shortcutPaths, 1)
	return shortcutsMap
}

// buildLevel. build clique of each cell in the level (level > 1)
// using Dijkstra algorithm from each entry point of the cell to all exit points of the cell
// and store the result in ow.weights
// this function is parallelized using goroutines worker pool
// this function uses overlay graph from the previous level to compute the weights
// basically: use shortcut edges from the previous level as edges in this current level overlay graph
func (ow *OverlayWeights) visualizeBuildLevel(graph *Graph, overlayGraph *OverlayGraph,
	costFunction costfunction.CostFunction, level int, shortcutPaths *[][]Coordinate,
	shortcutsMap map[Index]map[Index][]Coordinate) map[Index]map[Index][]Coordinate {

	levelInfo := overlayGraph.GetLevelInfo()

	buildCellClique := func(job customizerCell) any {

		cell := job.cell
		cellNumber := job.cellNumber

		for i := Index(0); i < cell.numEntryPoints; i++ {
			pq := NewMinHeap[Index]()
			eta := make(map[Index]float64)
			startOverlayVertexId := overlayGraph.GetEntryPoint(cell, i)
			eta[startOverlayVertexId] = 0

			overlayVertex := overlayGraph.GetVertex(startOverlayVertexId)
			parentState := make(map[CRPQueryKey]CRPQueryKey)

			startKey := NewCRPQueryKey(overlayVertex.originalVertex, startOverlayVertexId)
			parentState[startKey] = NewCRPQueryKey(INVALID_VERTEX_ID, 0)

			pq.Insert(NewPriorityQueueNode(0, startOverlayVertexId))

			for !pq.isEmpty() {
				pqNode, _ := pq.ExtractMin()
				uOverlayId := pqNode.GetItem()
				uEta := pqNode.GetRank()

				uTruncatedLevel := overlayGraph.levelInfo.TruncateToLevel(overlayGraph.GetVertex(uOverlayId).cellNumber, uint8(level))
				util.AssertPanic(uTruncatedLevel == cellNumber, "current truncated cell number and boundary vertex truncated cell number must be the same!")

				overlayGraph.ForOutNeighborsOf(uOverlayId, level-1, func(exit Index, wOffset Index) {

					shorcutWeight := ow.weights[wOffset]
					newEta := uEta + shorcutWeight

					if newEta >= pkg.INF_WEIGHT {
						return
					}

					oldExit, exitAlreadyVisited := eta[exit]
					if !exitAlreadyVisited || newEta < oldExit {
						eta[exit] = newEta
						exitOverlayVertex := overlayGraph.GetVertex(exit)
						neighborVertex := exitOverlayVertex.neighborOverlayVertex
						neighborOverlayVertex := overlayGraph.GetVertex(neighborVertex)

						uOverlayVertex := overlayGraph.GetVertex(uOverlayId)

						vKey := NewCRPQueryKey(exitOverlayVertex.originalVertex, exit)
						parentState[vKey] = NewCRPQueryKey(uOverlayVertex.originalVertex, uOverlayId)

						if levelInfo.TruncateToLevel(neighborOverlayVertex.cellNumber, uint8(level)) == cellNumber {
							boundaryArcWeight := costFunction.GetWeight(graph.GetOutEdge(exitOverlayVertex.originalEdge))
							newNeighborEta := newEta + boundaryArcWeight
							oldNEta, neighborVertexAlreadyVisited := eta[neighborVertex]

							if !neighborVertexAlreadyVisited || newNeighborEta < oldNEta {
								eta[neighborVertex] = newEta + boundaryArcWeight

								neighborKey := NewCRPQueryKey(neighborOverlayVertex.originalVertex, neighborVertex)
								parentState[neighborKey] = vKey

								if !neighborVertexAlreadyVisited {
									pq.Insert(NewPriorityQueueNode(eta[neighborVertex], neighborVertex))
								} else {
									pq.DecreaseKey(NewPriorityQueueNode(eta[neighborVertex], neighborVertex))
								}
							}
						}
					}
				})
			}

			// stores all eta of cell shortcut edges (shortest path from this entry point to each exit point of the cell)
			for j := Index(0); j < cell.numExitPoints; j++ {
				exitPoint := overlayGraph.GetExitPoint(cell, j)
				_, exists := eta[exitPoint]
				exitOverlayVertex := overlayGraph.GetVertex(exitPoint)
				if !exists {
					ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = pkg.INF_WEIGHT
				} else {
					ow.weights[cell.cellOffset+i*cell.numExitPoints+j] = eta[exitPoint]

					currentVertexKey := NewCRPQueryKey(exitOverlayVertex.originalVertex, exitPoint)
					pathCoords := make([]Coordinate, 0)
					path := make([]Index, 0)
					for currentVertexKey.GetNode() != INVALID_VERTEX_ID {
						// currentVertexLat, currentVertexLon := graph.GetVertexCoordinates(currentVertexKey.GetNode())
						// pathCoords = append(pathCoords, NewCoordinate(currentVertexLat, currentVertexLon))
						path = append(path, currentVertexKey.GetNode())
						currentVertexKey = parentState[currentVertexKey]
					}
					for i := 0; i < len(path)-1; i++ {
						currentPathCoords := shortcutsMap[path[i]][path[i+1]]
						pathCoords = append(pathCoords, currentPathCoords...)
					}

					shortcutsMap[startKey.GetNode()][exitOverlayVertex.originalVertex] = pathCoords
					*shortcutPaths = append(*shortcutPaths, pathCoords)

				}
			}
		}
		return nil
	}

	cellMapInLevelOne := overlayGraph.GetAllCellsInLevel(level)
	for pv, cell := range cellMapInLevelOne {
		buildCellClique(newCustomizerCell(cell, pv))
	}
	return shortcutsMap
}

func writeShortcutsToJsonFile(shortcuts [][]Coordinate, level int) error {
	// perkecil
	shortcuts = shortcuts[:int(float64(len(shortcuts))*0.2)]

	filename := fmt.Sprintf("shortcuts_level_%d.json", level-1)

	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	enc := json.NewEncoder(f)
	enc.SetIndent("", "  ")
	return enc.Encode(shortcuts)
}

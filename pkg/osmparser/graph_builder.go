package osmparser

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

// BuildGraph. build graph data structure from list of edges.
// roadNetwork = flag if the graph is a road network graph.
// test shortestpath ada beberapa yang gak pakai road network graph, diambil dari test cases soal-soal kontes pemrograman.
// jika roadNetwork=false, kita harus tambahkan dummy edge (v,v) untuk setiap vertex v di graph.
// hal ini karena Customizable Route Planning (CRP) Query phase (support turn costs) mengasumsikan setiap vertices memiliki setidaknya satu edge.
func (p *OsmParser) BuildGraph(scannedEdges []Edge, graphStorage *da.GraphStorage, numV uint32, skipUTurn bool, roadNetwork bool) (*da.Graph, [][]da.Index) {
	var (
		outEdges    [][]da.OutEdge = make([][]da.OutEdge, numV)
		inEdges     [][]da.InEdge  = make([][]da.InEdge, numV)
		edgeInfoIds [][]da.Index   = make([][]da.Index, numV)
		inDegree    []int          = make([]int, numV)
		outDegree   []int          = make([]int, numV)
		vertices    []da.Vertex    = make([]da.Vertex, numV+1)
	)

	for v := 0; v < int(numV)+1; v++ {
		vertices[v] = da.NewVertex(0, 0, da.Index(v))
	}

	vertexOsmIds := make([]uint64, numV)

	for eId, e := range scannedEdges {
		u := da.Index(e.from)
		v := da.Index(e.to)
		uOsmId := e.GetFromOsmId()
		vOsmId := e.GetToOsmId()

		vEntryPoint := da.Index(len(inEdges[v]))
		outEdge := da.NewOutEdge(0,
			v, e.GetWeight(), e.GetDistance(), vEntryPoint, e.GetHighwayType())
		outEdges[u] = append(outEdges[u], outEdge)

		edgeInfoIds[u] = append(edgeInfoIds[u], da.Index(eId))

		outDegree[u]++

		uExitPoint := da.Index(len(outEdges[u]) - 1)
		inEdge := da.NewInEdge(0,
			u, e.GetWeight(), e.GetDistance(), uExitPoint, e.GetHighwayType())
		inEdges[v] = append(inEdges[v], inEdge)
		inDegree[v]++

		uData := p.wayNodeMap[p.nodeToOsmId[da.Index(u)]].coord
		vertices[u] = da.NewVertex(uData.lat, uData.lon, u)

		vData := p.wayNodeMap[p.nodeToOsmId[da.Index(v)]].coord
		vertices[v] = da.NewVertex(vData.lat, vData.lon, v)

		vertexOsmIds[u] = uOsmId
		vertexOsmIds[v] = vOsmId
	}

	for v := 0; v < len(vertices)-1; v++ {
		// we need to do this because Customizable Route Planning (with turn costs) query assume all vertex have at least one outEdge (at for target as source)
		if !roadNetwork || (roadNetwork && (outDegree[v] == 0 || inDegree[v] == 0)) {
			dummyOut := da.NewOutEdge(da.INVALID_EDGE_ID, da.Index(v),
				pkg.INF_WEIGHT, pkg.INF_WEIGHT, da.Index(len(inEdges[v])), pkg.INVALID_HIGHWAY)
			outEdges[v] = append(outEdges[v], dummyOut)
			edgeInfoIds[v] = append(edgeInfoIds[v], da.INVALID_EDGE_INFO_ID)
			outDegree[v]++

			dummyIn := da.NewInEdge(da.INVALID_EDGE_ID, da.Index(v),
				pkg.INF_WEIGHT, pkg.INF_WEIGHT, da.Index(len(outEdges[v])-1), pkg.INVALID_HIGHWAY)
			inEdges[v] = append(inEdges[v], dummyIn)
			inDegree[v]++
			graphStorage.AppendEdgeMetadata(
				-1,
				1, 1,
				p.tagStringIdMap.GetID(""),
				pkg.INVALID_HIGHWAY,
				pkg.INVALID_HIGHWAY,
				uint8(0),
			)

		}
	}

	turnMatrices := make([][]pkg.TurnType, len(vertices)-1)
	// T_u[i*outDegree[u]+j] = turn type from inEdge i to outEdge j at vertex u

	// init turn matrices
	for via := 0; via < len(turnMatrices); via++ {
		turnMatrices[via] = make([]pkg.TurnType, outDegree[via]*inDegree[via])

		for j := 0; j < len(turnMatrices[via]); j++ {
			turnMatrices[via][j] = pkg.NONE
		}

		// tambahin turn type buat turn left/ turn right

		for entryId := 0; entryId < len(inEdges[via]); entryId++ {
			inEdge := inEdges[via][entryId]
			rowOffset := entryId * outDegree[via]

			for exitId := 0; exitId < len(outEdges[via]); exitId++ {
				outEdge := outEdges[via][exitId]

				prevPoint := vertices[inEdge.GetTail()].GetCoordinate()
				tail := vertices[via].GetCoordinate()
				headPoint := vertices[outEdge.GetHead()].GetCoordinate()

				prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
					tail.GetLon())
				turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
					headPoint.GetLon(), prevInitialBearing)
				if turn == da.TURN_SLIGHT_LEFT || turn == da.TURN_LEFT || turn == da.TURN_SHARP_LEFT {
					turnMatrices[via][rowOffset+exitId] = pkg.LEFT_TURN
				} else if turn == da.TURN_SLIGHT_RIGHT || turn == da.TURN_RIGHT || turn == da.TURN_SHARP_RIGHT {
					turnMatrices[via][rowOffset+exitId] = pkg.RIGHT_TURN
				}
			}
		}

	}

	for wayID, way := range p.ways {

		/*
			misal osm way (twoway):
				u1<->u2<->u3<->u4

				kita harus store semua u_turn dari edges yanng menyusun twoway osm way tsb:
				list u_turn:
				u1->u2->u1

				u2->u3->u2
				u2->u1->u2

				u3->u4->u3
				u3->u2->u3

				u4->u3->u4
		*/
		if !way.oneWay {

			for i, via := range way.graphNodes {

				if len(way.graphNodes) <= 1 {
					continue
				}

				if i == 0 {
					// store u_turn restrictions
					// dont allow u_turn at (to, via)->(via, to)
					to := way.graphNodes[1]
					if to == via {
						continue
					}

					entryId := -1
					exitId := -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k].GetHead() == to {
							exitId = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k].GetTail() == to {
							entryId = k
							break
						}
					}

					if entryId == -1 || exitId == -1 {
						continue
					}

					turnMatrices[via][entryId*int(outDegree[via])+exitId] = pkg.U_TURN
				} else if i < len(way.graphNodes)-1 {

					// backward
					to := way.graphNodes[i-1]
					if to == via {
						continue
					}

					entryId := -1
					exitId := -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k].GetHead() == to {
							exitId = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k].GetTail() == to {
							entryId = k
							break
						}
					}

					if entryId != -1 && exitId != -1 {
						turnMatrices[via][entryId*int(outDegree[via])+exitId] = pkg.U_TURN
					}

					// forward
					to = way.graphNodes[i+1]
					if to == via {
						continue
					}

					entryId = -1
					exitId = -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k].GetHead() == to {
							exitId = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k].GetTail() == to {
							entryId = k
							break
						}
					}

					if entryId != -1 && exitId != -1 {
						turnMatrices[via][entryId*int(outDegree[via])+exitId] = pkg.U_TURN
					}
				} else {
					// last node in way.graphNodes
					to := way.graphNodes[i-1]

					if to == via {
						continue
					}

					entryId := -1
					exitId := -1
					for k := 0; k < len(outEdges[via]); k++ {
						if outEdges[via][k].GetHead() == to {
							exitId = k
							break
						}
					}

					for k := 0; k < len(inEdges[via]); k++ {
						if inEdges[via][k].GetTail() == to {
							entryId = k
							break
						}
					}

					if entryId == -1 || exitId == -1 {
						continue
					}

					turnMatrices[via][entryId*int(outDegree[via])+exitId] = pkg.U_TURN
				}
			}
		}

		// store turn restrictions https://wiki.openstreetmap.org/wiki/Relation:restriction
		/*
				turn restriction berbentuk: {from-way, via-node, to-way}
				di kode ini:
				wayId/way: from-way
				restriction.to: to-way

				via-node berada di nodes nya from-way

				contoh: https://www.openstreetmap.org/relation/19474168#map=19/-7.782550/110.375438
				https://www.openstreetmap.org/api/0.6/relation/19474168
				https://www.openstreetmap.org/relation/5710500

			jadi kita pertama harus cari way.graphNodes yang jadi via-node

			note that from-way ke to-way bisa terhubung karena ada via-node yang jadi node di kedua way
			misal:
			u1 -> u2->via -> w1 ->w2
			from-way      to-way
			nah via ini jadi node di from-way.graphNodes dan to-way.graphNodes

			langkah kedua kita harus cari to-way.graphNodes yang == restriction.via

			kita store turnTables as:
			key (entryId, viaNode, exitId) -> tipe dari turn restrictionnya
			entryId adalah inEdge yang headnya ke viaNode
			exitId adalah outEdge yang tailnya dari viaNode

		*/
		fromNodes := way.graphNodes
		fromRestrictions := p.restrictions[wayID]
		for _, restriction := range fromRestrictions {

			if wayID == int64(restriction.to) { // ignore restrictions from wayId == restriction.to
				continue
			}

			_, acceptedWay := p.ways[int64(restriction.to)]
			if !acceptedWay {
				continue
			}

			for i := 0; i < len(fromNodes); i++ {
				if fromNodes[i] == restriction.via {
					if i == 0 && way.oneWay {
						// no predecessor
						continue
					}

					var predecessor da.Index // predecessor dari via nya turn restriction
					if i == 0 {
						predecessor = fromNodes[i+1]
					} else {
						predecessor = fromNodes[i-1]
					}

					if predecessor == restriction.via {
						continue
					}

					successor := da.Index(math.MaxUint32) // successor dari via nya turn restriction
					toNodes := p.ways[int64(restriction.to)].graphNodes
					for j := 0; j < len(toNodes)-1; j++ {
						if toNodes[j] == restriction.via {
							if j == len(toNodes)-1 {
								successor = toNodes[j-1]
							} else {
								successor = toNodes[j+1]
							}
							break
						}
					}

					if successor != da.Index(math.MaxUint32) && successor != restriction.via {

						// (from, via, to) nodes dari turn restriction
						from := da.Index(predecessor)
						via := da.Index(restriction.via)
						to := da.Index(successor)

						entryID := da.Index(math.MaxUint32)
						exitID := da.Index(math.MaxUint32)

						inEdge := da.InEdge{}
						for k := 0; k < len(inEdges[via]); k++ {
							if inEdges[via][k].GetTail() == from {
								entryID = da.Index(k)
								inEdge = inEdges[via][k]
								break
							}
						}

						if entryID == da.Index(math.MaxUint32) {
							continue
						}

						rowOffset := entryID * da.Index(outDegree[via])
						for k := 0; k < len(outEdges[via]); k++ {
							outEdge := outEdges[via][k]
							if outEdge.GetHead() == to {
								exitID = da.Index(k)
							}

							prevPoint := vertices[inEdge.GetTail()].GetCoordinate()
							tail := vertices[via].GetCoordinate()
							headPoint := vertices[outEdge.GetHead()].GetCoordinate()

							if restriction.turnRestriction == ONLY_LEFT_TURN {
								/*
										. = restriction.via node

									--------.--------- restriction.to way (two-way)
											|
											|
											|
											|
											|
											restriction.from  way

										misal ONLY_LEFT_TURN:
										berarti kita harus dissalow semua turn right...
										cara taunya cuma bisa dari relative bearing dari restriction.from ke restriction.to....

								*/

								prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
									tail.GetLon())
								turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
									headPoint.GetLon(), prevInitialBearing)
								if turn == da.TURN_SLIGHT_RIGHT || turn == da.TURN_RIGHT || turn == da.TURN_SHARP_RIGHT {
									// dissallow semua turn right...

									turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
								}

							} else if restriction.turnRestriction == ONLY_RIGHT_TURN {

								prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
									tail.GetLon())
								turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
									headPoint.GetLon(), prevInitialBearing)
								if turn == da.TURN_SLIGHT_LEFT || turn == da.TURN_LEFT || turn == da.TURN_SHARP_LEFT {
									// dissallow semua turn left...
									turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
								}

							} else if restriction.turnRestriction == ONLY_STRAIGHT_ON {
								prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
									tail.GetLon())
								turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
									headPoint.GetLon(), prevInitialBearing)
								if turn == da.TURN_SLIGHT_LEFT || turn == da.TURN_LEFT || turn == da.TURN_SHARP_LEFT ||
									turn == da.TURN_SLIGHT_RIGHT || turn == da.TURN_RIGHT || turn == da.TURN_SHARP_RIGHT {
									// dissallow semua turn right dan turn left...
									// contoh: http://openstreetmap.org/relation/19474168
									turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
								}

							}
						}

						if exitID == da.Index(math.MaxUint32) {
							continue
						}

						if rowOffset+exitID >= da.Index(len(turnMatrices[via])) {
							continue
						}

						switch restriction.turnRestriction {
						case NO_LEFT_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_RIGHT_TURN: // contoh: https://www.openstreetmap.org/relation/5710505
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_STRAIGHT_ON:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_U_TURN: // harus NO_ENTRY karena gak boleh u-turn: example: https://www.openstreetmap.org/relation/10732316#map=19/-7.566370/110.775455
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_ENTRY:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case ONLY_LEFT_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.LEFT_TURN

						case ONLY_RIGHT_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.RIGHT_TURN

						case ONLY_STRAIGHT_ON: // udah kita dissalow semua right & left turn di loc diatas
							turnMatrices[via][rowOffset+exitID] = pkg.NONE

						default:
							turnMatrices[via][rowOffset+exitID] = pkg.NONE

						}
					}
					break
				}
			}
		}
	}

	matrices := make([]pkg.TurnType, 0)
	matrixOffset := 0

	for v := 0; v < len(vertices)-1; v++ {
		// set the turnTablePtr of vertex v to the current matrixOffset
		// matrix offset is index of the first element of turnMatrices[v] in the flattened matrices array
		vertices[v].SetTurnTablePtr(da.Index(matrixOffset))
		// flatten the turnMatrices
		for i := 0; i < len(turnMatrices[v]); i++ {
			matrices = append(matrices, turnMatrices[v][i])
		}

		matrixOffset += len(turnMatrices[v])
	}

	outEdgeOffset := da.Index(0)
	inEdgeOffset := da.Index(0)

	for i := 0; i < len(vertices)-1; i++ {
		vertices[i].SetFirstOut(outEdgeOffset) // index of the first outEdge of vertex i in the flattened outEdges array
		vertices[i].SetFirstIn(inEdgeOffset)
		outEdgeOffset += da.Index(len(outEdges[i]))
		inEdgeOffset += da.Index(len(inEdges[i]))
	}

	// dummy update vertex
	vertices[len(vertices)-1] = da.NewVertex(0, 0, da.Index(len(vertices)-1))
	vertices[len(vertices)-1].SetFirstOut(outEdgeOffset)
	vertices[len(vertices)-1].SetFirstIn(inEdgeOffset)

	flattenOutEdges := flatten(outEdges)
	for i := 0; i < len(flattenOutEdges); i++ {
		outEdgeId := da.Index(i)
		flattenOutEdges[i].SetEdgeId(outEdgeId)
	}

	flattenInEdges := flatten(inEdges)
	for i := 0; i < len(flattenInEdges); i++ {
		flattenInEdges[i].SetEdgeId(da.Index(i))
	}

	verticesOsmIdsPs := da.NewPackedSlice(da.BIT_SIZE_OSM_NODE_ID, uint64(numV)+1)

	for _, osmId := range vertexOsmIds {
		verticesOsmIdsPs.Append(osmId)
	}

	graph := da.NewGraph(vertices, flattenOutEdges, flattenInEdges, matrices, roadNetwork, verticesOsmIdsPs)

	return graph, edgeInfoIds
}

func flatten[T any](container [][]T) []T {
	finalSize := 0
	for _, part := range container {
		finalSize += len(part)
	}

	result := make([]T, finalSize)
	idx := 0
	for _, part := range container {
		for _, elem := range part {
			result[idx] = elem
			idx++
		}
	}
	return result
}

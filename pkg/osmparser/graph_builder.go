package osmparser

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func (p *OsmParser) BuildGraph(scannedEdges []Edge, graphStorage *datastructure.GraphStorage, numV uint32, skipUTurn bool) *datastructure.Graph {
	var (
		outEdges  [][]*datastructure.OutEdge = make([][]*datastructure.OutEdge, numV)
		inEdges   [][]*datastructure.InEdge  = make([][]*datastructure.InEdge, numV)
		inDegree  []int                      = make([]int, numV)
		outDegree []int                      = make([]int, numV)
		vertices  []*datastructure.Vertex    = make([]*datastructure.Vertex, numV+1)
	)

	for v := 0; v < int(numV)+1; v++ {
		vertices[v] = datastructure.NewVertex(0, 0, datastructure.Index(v))
	}

	lastEdgeId := uint32(0)
	edgeId := datastructure.Index(0)
	for _, e := range scannedEdges {
		u := datastructure.Index(e.from)
		v := datastructure.Index(e.to)

		outEdges[u] = append(outEdges[u], datastructure.NewOutEdge(edgeId,
			v, e.weight, e.distance, len(inEdges[v])))
		outDegree[u]++

		inEdges[v] = append(inEdges[v], datastructure.NewInEdge(edgeId,
			u, e.weight, e.distance, len(outEdges[u])-1))
		inDegree[v]++

		uData := p.acceptedNodeMap[p.nodeToOsmId[datastructure.Index(u)]]
		vertices[u] = datastructure.NewVertex(uData.lat, uData.lon, u)

		vData := p.acceptedNodeMap[p.nodeToOsmId[datastructure.Index(v)]]
		vertices[v] = datastructure.NewVertex(vData.lat, vData.lon, v)
		edgeId++
		if e.edgeID >= lastEdgeId {
			lastEdgeId = e.edgeID + 1
		}
	}

	for v := 0; v < len(vertices)-1; v++ {
		// we need to do this because crp query assume all vertex have at least one outEdge (at for target as source)

		dummyID := datastructure.Index(lastEdgeId)
		dummyOut := datastructure.NewOutEdge(dummyID, datastructure.Index(v),
			0, 0, len(inEdges[v]))
		outEdges[v] = append(outEdges[v], dummyOut)
		outDegree[v]++

		dummyIn := datastructure.NewInEdge(dummyID, datastructure.Index(v),
			0, 0, len(outEdges[v])-1)
		inEdges[v] = append(inEdges[v], dummyIn)
		inDegree[v]++
		lastEdgeId++
		graphStorage.AppendMapEdgeInfo(
			datastructure.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(""),
				uint8(p.tagStringIdMap.GetID("")),
				uint8(0),
				uint8(p.tagStringIdMap.GetID("")),
				datastructure.Index(uint32(math.Pow(1, 30))), datastructure.Index(uint32(math.Pow(1, 30))),
				-1,
			),
		)

	}

	turnMatrices := make([][]pkg.TurnType, len(vertices)-1)
	// T_u[i*outDegree[u]+j] = turn type from inEdge i to outEdge j at vertex u

	// init turn matrices
	for i := 0; i < len(turnMatrices); i++ {
		turnMatrices[i] = make([]pkg.TurnType, outDegree[i]*inDegree[i])

		for j := 0; j < len(turnMatrices[i]); j++ {
			turnMatrices[i][j] = pkg.NONE
		}
	}

	// store u_turn restrictions
	if !skipUTurn {
		for _, e := range scannedEdges {
			// dont allow u_turns at (u,v) -> (v,u)
			via := datastructure.Index(e.from)
			if inDegree[via] != 1 || outDegree[via] != 1 {
				to := datastructure.Index(e.to)

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

			// to
			via = datastructure.Index(e.to)
			if inDegree[via] != 1 || outDegree[via] != 1 {
				to := datastructure.Index(e.from)

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
	// store turn restrictions
	for wayID, way := range p.ways {
		fromNodes := way.nodes
		fromRestrictions := p.restrictions[wayID]
		for _, restriction := range fromRestrictions {
			_, isAcceptedNode := p.nodeIDMap[int64(restriction.via)]
			if wayID == int64(restriction.to) || !isAcceptedNode {
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

					var predecessor datastructure.Index
					if i == 0 {
						predecessor = fromNodes[i+1]
					} else {
						predecessor = fromNodes[i-1]
					}

					if predecessor == restriction.via {
						continue
					}
					successor := datastructure.Index(math.MaxUint32)
					toNodes := p.ways[int64(restriction.to)].nodes
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

					if successor != datastructure.Index(math.MaxUint32) && successor != restriction.via {

						from := datastructure.Index(predecessor)
						via := datastructure.Index(restriction.via)
						to := datastructure.Index(successor)

						entryID := datastructure.Index(math.MaxUint32)
						exitID := datastructure.Index(math.MaxUint32)

						for k := 0; k < len(inEdges[via]); k++ {
							if inEdges[via][k].GetTail() == from {
								entryID = datastructure.Index(k)
								break
							}
						}

						if entryID == datastructure.Index(math.MaxUint32) {
							continue
						}

						rowOffset := entryID * datastructure.Index(outDegree[via])
						for k := 0; k < len(outEdges[via]); k++ {
							if outEdges[via][k].GetHead() == to {
								exitID = datastructure.Index(k)
							}

							if restriction.turnRestriction == ONLY_LEFT_TURN || restriction.turnRestriction == ONLY_RIGHT_TURN ||
								restriction.turnRestriction == ONLY_STRAIGHT_ON {
								turnMatrices[via][rowOffset+datastructure.Index(k)] = pkg.NO_ENTRY
							}
						}

						if exitID == datastructure.Index(math.MaxUint32) {
							continue
						}

						if rowOffset+exitID >= datastructure.Index(len(turnMatrices[via])) {
							continue
						}

						switch restriction.turnRestriction {
						case NO_LEFT_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_RIGHT_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_STRAIGHT_ON:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_U_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case NO_ENTRY:
							turnMatrices[via][rowOffset+exitID] = pkg.NO_ENTRY

						case ONLY_LEFT_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.LEFT_TURN

						case ONLY_RIGHT_TURN:
							turnMatrices[via][rowOffset+exitID] = pkg.RIGHT_TURN

						case ONLY_STRAIGHT_ON:
							turnMatrices[via][rowOffset+exitID] = pkg.STRAIGHT_ON

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
		vertices[v].SetTurnTablePtr(datastructure.Index(matrixOffset))
		// flatten the turnMatrices
		for i := 0; i < len(turnMatrices[v]); i++ {
			matrices = append(matrices, turnMatrices[v][i])
		}

		matrixOffset += len(turnMatrices[v])
	}

	outEdgeOffset := datastructure.Index(0)
	inEdgeOffset := datastructure.Index(0)

	for i := 0; i < len(vertices)-1; i++ {
		vertices[i].SetTurnTablePtr(vertices[i].GetTurnTablePtr())
		vertices[i].SetFirstOut(outEdgeOffset) // index of the first outEdge of vertex i in the flattened outEdges array
		vertices[i].SetFirstIn(inEdgeOffset)
		outEdgeOffset += datastructure.Index(len(outEdges[i]))
		inEdgeOffset += datastructure.Index(len(inEdges[i]))
	}

	vertices[len(vertices)-1] = datastructure.NewVertex(0, 0, datastructure.Index(len(vertices)-1))
	vertices[len(vertices)-1].SetFirstOut(outEdgeOffset)
	vertices[len(vertices)-1].SetFirstIn(inEdgeOffset)

	flattenOutEdges := flatten(outEdges)
	flattenInEdges := flatten(inEdges)
	graph := datastructure.NewGraph(vertices, flattenOutEdges, flattenInEdges, matrices)

	return graph
}

func flatten[T any](container [][]*T) []*T {
	finalSize := 0
	for _, part := range container {
		finalSize += len(part)
	}

	result := make([]*T, finalSize)
	idx := 0
	for _, part := range container {
		for _, elem := range part {
			result[idx] = elem
			idx++
		}
	}
	return result
}

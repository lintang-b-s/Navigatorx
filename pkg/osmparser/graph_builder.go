package osmparser

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func (p *OsmParser) BuildGraph(scannedEdges []Edge, graphStorage *da.GraphStorage, numV uint32, skipUTurn bool) (*da.Graph, [][]da.Index) {
	var (
		outEdges    [][]da.OutEdge = make([][]da.OutEdge, numV)
		inEdges     [][]da.InEdge  = make([][]da.InEdge, numV)
		edgeInfoIds [][]da.Index   = make([][]da.Index, numV)
		inDegree    []int          = make([]int, numV)
		outDegree   []int          = make([]int, numV)
		vertices    []da.Vertex    = make([]da.Vertex, numV+1)
	)

	for v := 0; v < int(numV)+1; v++ {
		vertices[v] = da.NewVertex(0, 0, da.Index(v), 0)
	}

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

		uData := p.acceptedNodeMap[p.nodeToOsmId[da.Index(u)]]
		vertices[u] = da.NewVertex(uData.lat, uData.lon, u, uOsmId)

		vData := p.acceptedNodeMap[p.nodeToOsmId[da.Index(v)]]
		vertices[v] = da.NewVertex(vData.lat, vData.lon, v, vOsmId)
	}

	for v := 0; v < len(vertices)-1; v++ {
		// we need to do this because crp query assume all vertex have at least one outEdge (at for target as source)

		dummyOut := da.NewOutEdge(da.INVALID_EDGE_ID, da.Index(v),
			0, 0, da.Index(len(inEdges[v])), pkg.UNKNOWN)
		outEdges[v] = append(outEdges[v], dummyOut)
		edgeInfoIds[v] = append(edgeInfoIds[v], da.INVALID_EDGE_INFO_ID)
		outDegree[v]++

		dummyIn := da.NewInEdge(da.INVALID_EDGE_ID, da.Index(v),
			0, 0, da.Index(len(outEdges[v])-1), pkg.UNKNOWN)
		inEdges[v] = append(inEdges[v], dummyIn)
		inDegree[v]++
		graphStorage.AppendEdgeInfos(
			da.NewEdgeExtraInfo(
				p.tagStringIdMap.GetID(""),
				p.tagStringIdMap.GetID(""),
				p.tagStringIdMap.GetID(""),
				uint8(0),

				da.Index(uint32(math.Pow(1, 30))), da.Index(uint32(math.Pow(1, 30))),
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
			via := da.Index(e.from)
			if inDegree[via] != 1 || outDegree[via] != 1 {
				to := da.Index(e.to)

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
			via = da.Index(e.to)
			if inDegree[via] != 1 || outDegree[via] != 1 {
				to := da.Index(e.from)

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

					var predecessor da.Index
					if i == 0 {
						predecessor = fromNodes[i+1]
					} else {
						predecessor = fromNodes[i-1]
					}

					if predecessor == restriction.via {
						continue
					}
					successor := da.Index(math.MaxUint32)
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

					if successor != da.Index(math.MaxUint32) && successor != restriction.via {

						from := da.Index(predecessor)
						via := da.Index(restriction.via)
						to := da.Index(successor)

						entryID := da.Index(math.MaxUint32)
						exitID := da.Index(math.MaxUint32)

						for k := 0; k < len(inEdges[via]); k++ {
							if inEdges[via][k].GetTail() == from {
								entryID = da.Index(k)
								break
							}
						}

						if entryID == da.Index(math.MaxUint32) {
							continue
						}

						rowOffset := entryID * da.Index(outDegree[via])
						for k := 0; k < len(outEdges[via]); k++ {
							if outEdges[via][k].GetHead() == to {
								exitID = da.Index(k)
							}

							if restriction.turnRestriction == ONLY_LEFT_TURN || restriction.turnRestriction == ONLY_RIGHT_TURN ||
								restriction.turnRestriction == ONLY_STRAIGHT_ON {
								turnMatrices[via][rowOffset+da.Index(k)] = pkg.NO_ENTRY
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
		vertices[i].SetTurnTablePtr(vertices[i].GetTurnTablePtr())
		vertices[i].SetFirstOut(outEdgeOffset) // index of the first outEdge of vertex i in the flattened outEdges array
		vertices[i].SetFirstIn(inEdgeOffset)
		outEdgeOffset += da.Index(len(outEdges[i]))
		inEdgeOffset += da.Index(len(inEdges[i]))
	}

	// dummy update vertex
	vertices[len(vertices)-1] = da.NewVertex(0, 0, da.Index(len(vertices)-1), 0)
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

	graph := da.NewGraph(vertices, flattenOutEdges, flattenInEdges, matrices)

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

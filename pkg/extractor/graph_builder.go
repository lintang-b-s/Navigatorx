package extractor

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// BuildGraph build graph data structure from list of edges.
// roadNetwork = flag if the graph is a road network graph.
// test shortestpath ada beberapa yang gak pakai road network graph, diambil dari test cases soal-soal kontes pemrograman.
// untuk roadNetwork=true, inputnya file OpenStreetMap pbf, kita support hampir semua tipe osm turn restrictions.
// terinspirasi dari https://github.com/michaelwegner/CRP/blob/master/io/OSMParser.cpp
func (p *Extractor[W]) BuildGraph(edges []Edge[W], rn *da.RoadNetworkDataContainer, numV uint32, roadNetwork bool) (*da.Graph,
	*met.TimeFunction[W], []da.Index, []pkg.TurnType) {
	util.ActivateMode[W]()

	var (
		outEdges       = make([][]da.Index, numV)
		inEdges        = make([][]da.Index, numV)
		outWeights     = make([][]W, numV)
		outLengths     = make([][]uint32, numV)
		inLengths      = make([][]uint32, numV)
		segmentDataIds = make([][]da.Index, numV)
		inDegree       = make([]int, numV)
		outDegree      = make([]int, numV)
		vertices       = make([]da.Vertex, numV+1)

		entryPointsAdjList = make([][]da.Index, numV) // entryPointsAdjList[u][i]: indeks i dari incoming edge (u,v) di inEdges[v]
		exitPointsAdjList  = make([][]da.Index, numV) // exitPointsAdjList[v][i]: indeks i dari outgoing edge (u,v) di outEdges[u]

		isParallelOutEdge = make([][]bool, numV+1)
		isParallelInEdge  = make([][]bool, numV+1)
	)

	fmt.Printf("0%%...")
	vertexOsmIds := make([]uint64, numV)
	for eID, e := range edges {
		u := da.Index(e.from)
		v := da.Index(e.to)

		entryPoint := da.Index(len(inEdges[v]))
		exitPoint := da.Index(len(outEdges[u]))
		entryPointsAdjList[u] = append(entryPointsAdjList[u], entryPoint)
		exitPointsAdjList[v] = append(exitPointsAdjList[v], exitPoint)

		outEdges[u] = append(outEdges[u], v)
		outWeights[u] = append(outWeights[u], e.GetWeight())
		outLengths[u] = append(outLengths[u], e.GetDistance())

		inEdges[v] = append(inEdges[v], u)
		inLengths[v] = append(inLengths[v], e.GetDistance())

		uData := p.wayNodeMap[p.nodeToOsmId[u]].coord
		vData := p.wayNodeMap[p.nodeToOsmId[v]].coord
		vertices[u] = da.NewVertex(uData.lat, uData.lon, u)
		vertices[v] = da.NewVertex(vData.lat, vData.lon, v)
		vertexOsmIds[u] = e.GetFromOsmId()
		vertexOsmIds[v] = e.GetToOsmId()
		segmentDataIds[u] = append(segmentDataIds[u], da.Index(eID))
	}

	for v := range numV {
		outDegree[v] = len(outEdges[v])
		inDegree[v] = len(inEdges[v])

		if roadNetwork {
			for q := 0; q < (outDegree[v]); q++ {
				isParallelOutEdge[v] = append(isParallelOutEdge[v], false)
			}

			for q := 0; q < (inDegree[v]); q++ {
				isParallelInEdge[v] = append(isParallelInEdge[v], false)
			}
		}
	}

	fmt.Printf("10%%...")
	newEDataId := len(edges)

	// tambahin parallel edges dulu buat via-way turn restrictions
	if roadNetwork {
		for wayId, way := range p.ways {
			newEDataId = addParallelViaEdges(p, wayId, way, newEDataId, outEdges, inEdges, entryPointsAdjList, exitPointsAdjList, rn, segmentDataIds, outWeights, outLengths,
				inLengths, outDegree, inDegree, isParallelOutEdge, isParallelInEdge)
		}
	}

	fmt.Printf("25%%...")

	// T[u][i*outDegree[u]+j] = turn type from entryPoint i (inEdge ke-i dari vertex u) to exitPoint j  (outEdge ke-j dari vertex u)  at vertex u.
	// buat via yang tipe nya osm node: https://wiki.openstreetmap.org/wiki/Relation:restriction .
	turnMatrices := make([][]pkg.TurnType, len(vertices)-1)

	minResolution := pkg.INF_WEIGHT

	// init turn matrices. (only for road network OpenStreetMap input file)
	// T(n) = \sum_{v in V} outDeg(v)*inDeg(v) <= \sum_{v in V} c^2 =O(n). if we let inDeg(v)=outDeg(v)=O(1) (for any vertex v) like in road networks.
	if roadNetwork {
		for v := 0; v < len(turnMatrices); v++ {
			turnMatrices[v] = make([]pkg.TurnType, outDegree[v]*inDegree[v])

			for j := 0; j < len(turnMatrices[v]); j++ {
				turnMatrices[v][j] = pkg.NONE
			}

			// tambahin turn type buat turn left/ turn right
			for entryPoint := 0; entryPoint < len(inEdges[v]); entryPoint++ {
				u := inEdges[v][entryPoint]
				rowOffset := entryPoint * outDegree[v]

				for exitPoint := 0; exitPoint < len(outEdges[v]); exitPoint++ {
					w := outEdges[v][exitPoint]

					prevPoint := vertices[u].GetCoordinate()
					tail := vertices[v].GetCoordinate()
					headPoint := vertices[w].GetCoordinate()

					prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
						tail.GetLon())
					turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
						headPoint.GetLon(), prevInitialBearing)

					l := util.MetersFromCentimeters(inLengths[v][entryPoint])
					lPrime := util.MetersFromCentimeters(outLengths[v][exitPoint])

					if !util.Eq(l, 0) && !util.Eq(lPrime, 0) {
						delta := pkg.CalcResolution(l, lPrime, pkg.INF_WEIGHT)
						minResolution = min(minResolution, delta)
					}

					switch turn {
					case da.TURN_SLIGHT_LEFT, da.TURN_LEFT, da.TURN_SHARP_LEFT:
						turnMatrices[v][rowOffset+exitPoint] = pkg.LEFT_TURN
					case da.TURN_SLIGHT_RIGHT, da.TURN_RIGHT, da.TURN_SHARP_RIGHT:
						turnMatrices[v][rowOffset+exitPoint] = pkg.RIGHT_TURN
					}
				}
			}
		}
	}

	fmt.Printf("45%%...")

	conditionalTurnRestrictions := make([]da.ConditionalTurnRestriction, 0)

	// let w=number of osm ways , q = max number of nodes of any osm ways, r = max number of restrictions of any osm ways
	// O(w*r*q^2)
	if roadNetwork {
		for wayId, way := range p.ways {
			addTwoWayTurnCost(p, wayId, way, outEdges, inEdges, turnMatrices, outDegree)

			// store turn restrictions https://wiki.openstreetmap.org/wiki/Relation:restriction

			fromNodes := way.graphNodes
			fromRestrictions := p.restrictions[wayId]
			for fromResId, restriction := range fromRestrictions {

				if wayId == int64(restriction.to) { // ignore restrictions from wayId == restriction.to
					continue
				}

				_, acceptedWay := p.ways[int64(restriction.to)]
				if !acceptedWay {
					continue
				}

				if !restriction.isWay {
					addViaNodeTurnRestriction(p, wayId, way, fromNodes, restriction, fromResId, outEdges, inEdges,
						outDegree, vertices, turnMatrices, &conditionalTurnRestrictions, isParallelOutEdge, isParallelInEdge)
				} else if restriction.isWay {
					addViaWayTurnRestriction(p, wayId, way, fromNodes, restriction, fromResId, outEdges, inEdges,
						rn, segmentDataIds, turnMatrices, outDegree, inDegree, isParallelOutEdge, isParallelInEdge)
				}
			}
		}
	}

	fmt.Printf("85%%...")

	flattenTurnMatrices := make([]pkg.TurnType, 0)
	matrixOffset := 0
	vertexTurnTablePtr := make([]da.Index, len(vertices)-1)

	if roadNetwork {
		// O(n)
		for u := 0; u < len(vertices)-1; u++ {
			// set the turnTablePtr of vertex v to the current matrixOffset
			// matrix offset is index of the first element of turnMatrices[v] in the flattened matrices array
			vertexTurnTablePtr[u] = da.Index(matrixOffset)
			// flatten the turnMatrices
			for i := 0; i < len(turnMatrices[u]); i++ {
				flattenTurnMatrices = append(flattenTurnMatrices, turnMatrices[u][i])
			}

			matrixOffset += len(turnMatrices[u])
		}
	}

	outEdgeOffset := da.Index(0)
	inEdgeOffset := da.Index(0)

	for u := 0; u < len(vertices)-1; u++ {
		vertices[u].SetFirstOut(outEdgeOffset) // index of the first outEdge of vertex i in the flattened outEdges array
		vertices[u].SetFirstIn(inEdgeOffset)
		outEdgeOffset += da.Index(len(outEdges[u]))
		inEdgeOffset += da.Index(len(inEdges[u]))
	}

	// dummy vertex
	vertices[len(vertices)-1] = da.NewVertex(0, 0, da.Index(len(vertices)-1))
	vertices[len(vertices)-1].SetFirstOut(outEdgeOffset)
	vertices[len(vertices)-1].SetFirstIn(inEdgeOffset)

	// flatten the edges data
	heads := util.Flatten(outEdges)
	weights := util.Flatten(outWeights)
	segmentLengths := util.Flatten(outLengths)
	tails := util.Flatten(inEdges)
	entryPoints := util.Flatten(entryPointsAdjList)
	exitPoints := util.Flatten(exitPointsAdjList)
	if roadNetwork {
		flDataIds := util.Flatten(segmentDataIds)
		nPerm := make([]int, len(flDataIds))
		for i := 0; i < len(flDataIds); i++ {
			nPerm[i] = int(flDataIds[i])
		}

		rn.ApplySegmentsPermutation(nPerm)
	}

	graph := da.NewGraph(vertices, heads, tails, roadNetwork, entryPoints, exitPoints)
	rn.BuildNameTable(p.tagStringIdMap.GetIdToStr())

	setConditionalRestrictions(p, roadNetwork, graph, rn, conditionalTurnRestrictions)

	segmentDurations := make([]uint32, len(weights))
	for i := 0; i < len(weights); i++ {
		segmentDurations[i] = uint32(weights[i])
	}
	if roadNetwork {
		graph.SetMinResolution(minResolution)
	}

	timeFunction := met.NewTimeCostFunction(
		roadNetwork, weights, segmentLengths, segmentDurations,
	)

	fmt.Printf("100%%\n")
	return graph, timeFunction, vertexTurnTablePtr, flattenTurnMatrices
}

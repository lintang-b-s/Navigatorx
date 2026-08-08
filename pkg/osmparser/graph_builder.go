package osmparser

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// BuildGraph build graph data structure from list of edges.
// roadNetwork = flag if the graph is a road network graph.
// test shortestpath ada beberapa yang gak pakai road network graph, diambil dari test cases soal-soal kontes pemrograman.
// jika roadNetwork=false, kita harus tambahkan dummy edge (v,v) untuk setiap vertex v di graph, untuk correctness test.
// karena Customizable Route Planning (CRP) Query phase (with turn costs) mengasumsikan setiap vertices memiliki setidaknya 1 incoming edge & 1 outgoing edge biar bisa point-to-point shortest path (p2p) with turn costs ke/dari setiap vertices.
// untuk roadNetwork=true, inputnya file OpenStreetMap pbf, kita support hampir semua tipe osm turn restrictions.
func (p *OsmParser[W]) BuildGraph(scannedEdges []Edge[W], graphStorage *da.GraphStorage, numV uint32, roadNetwork bool) (*da.Graph, *costfunction.TimeFunction[W], [][]da.Index) {
	util.ActivateMode[W]()

	var (
		outEdges    = make([][]da.OutEdge, numV)
		inEdges     = make([][]da.InEdge, numV)
		outWeights  = make([][]W, numV)
		outLengths  = make([][]uint32, numV)
		inLengths   = make([][]uint32, numV)
		edgeInfoIds = make([][]da.Index, numV)
		inDegree    = make([]int, numV)
		outDegree   = make([]int, numV)
		vertices    = make([]da.Vertex, numV+1)
	)

	fmt.Printf("0%%...")
	vertexOsmIds := make([]uint64, numV)
	for eID, e := range scannedEdges {
		u := da.Index(e.from)
		v := da.Index(e.to)

		vEntryPoint := da.Index(len(inEdges[v]))
		outEdge := da.NewOutEdge(da.INVALID_EDGE_ID, v, vEntryPoint, e.GetHighwayType())
		if e.IsJunctionHead() {
			outEdge.SetJunctionHead()
		}
		if e.IsJunctionTail() {
			outEdge.SetJunctionTail()
		}
		if e.ContainsTrafficLight() {
			outEdge.SetContainsTrafficLight()
		}
		outEdges[u] = append(outEdges[u], outEdge)
		outWeights[u] = append(outWeights[u], e.GetWeight())
		outLengths[u] = append(outLengths[u], e.GetDistance())

		uExitPoint := da.Index(len(outEdges[u]) - 1)
		inEdge := da.NewInEdge(da.INVALID_EDGE_ID, u, uExitPoint, e.GetHighwayType())
		if e.IsJunctionHead() {
			inEdge.SetJunctionHead()
		}
		if e.IsJunctionTail() {
			inEdge.SetJunctionTail()
		}
		if e.ContainsTrafficLight() {
			inEdge.SetContainsTrafficLight()
		}
		inEdges[v] = append(inEdges[v], inEdge)
		inLengths[v] = append(inLengths[v], e.GetDistance())

		uData := p.wayNodeMap[p.nodeToOsmId[u]].coord
		vData := p.wayNodeMap[p.nodeToOsmId[v]].coord
		vertices[u] = da.NewVertex(uData.lat, uData.lon, u)
		vertices[v] = da.NewVertex(vData.lat, vData.lon, v)
		vertexOsmIds[u] = e.GetFromOsmId()
		vertexOsmIds[v] = e.GetToOsmId()
		edgeInfoIds[u] = append(edgeInfoIds[u], da.Index(eID))

	}

	for v := range numV {
		outDegree[v] = len(outEdges[v])
		inDegree[v] = len(inEdges[v])
	}

	fmt.Printf("10%%...")
	newEInfoId := len(scannedEdges)
	// tambahin parallel edges dulu buat via-way turn restrictions
	for wayId, way := range p.ways {
		newEInfoId = addParallelViaEdges(p, wayId, way, newEInfoId, outEdges, inEdges, graphStorage, edgeInfoIds, outWeights, outLengths,
			inLengths, outDegree, inDegree)
	}

	// O(V)
	for v := 0; v < len(vertices)-1; v++ {
		// we need to do this because Customizable Route Planning (with turn costs) query assume all vertex have at least one outEdge (at for target as source)
		if !roadNetwork || (roadNetwork && (outDegree[v] == 0 || inDegree[v] == 0)) {
			dummyOut := da.NewOutEdge(da.INVALID_EDGE_ID, da.Index(v),
				da.Index(len(inEdges[v])), pkg.INVALID_HIGHWAY)
			dummyOut.SetDummyEdge()
			outEdges[v] = append(outEdges[v], dummyOut)
			outWeights[v] = append(outWeights[v], util.Infinity[W]())
			outLengths[v] = append(outLengths[v], 1)
			edgeInfoIds[v] = append(edgeInfoIds[v], da.Index(newEInfoId))
			outDegree[v]++

			dummyIn := da.NewInEdge(da.INVALID_EDGE_ID, da.Index(v),
				da.Index(len(outEdges[v])-1), pkg.INVALID_HIGHWAY)
			dummyIn.SetDummyEdge()
			inEdges[v] = append(inEdges[v], dummyIn)
			inLengths[v] = append(inLengths[v], 1)
			inDegree[v]++

			graphStorage.AppendEdgeMetadata(
				-1,
				1, 1,
				p.tagStringIdMap.GetID(""),
				pkg.INVALID_HIGHWAY,
				pkg.INVALID_HIGHWAY,
				uint8(0),
			)
			newEInfoId++
		}
	}

	fmt.Printf("25%%...")

	// T[u][i*outDegree[u]+j] = turn type from entryPoint i (inEdge ke-i dari vertex u) to exitPoint j  (outEdge ke-j dari vertex u)  at vertex u.
	// buat via yang tipe nya osm node: https://wiki.openstreetmap.org/wiki/Relation:restriction .
	turnMatrices := make([][]pkg.TurnType, len(vertices)-1)

	minResolution := pkg.INF_WEIGHT

	// init turn matrices
	for via := 0; via < len(turnMatrices); via++ {
		turnMatrices[via] = make([]pkg.TurnType, outDegree[via]*inDegree[via])

		for j := 0; j < len(turnMatrices[via]); j++ {
			turnMatrices[via][j] = pkg.NONE
		}

		if !roadNetwork {
			continue
		}

		// tambahin turn type buat turn left/ turn right
		for entryPoint := 0; entryPoint < len(inEdges[via]); entryPoint++ {
			inEdge := inEdges[via][entryPoint]
			rowOffset := entryPoint * outDegree[via]

			for exitPoint := 0; exitPoint < len(outEdges[via]); exitPoint++ {
				outEdge := outEdges[via][exitPoint]

				prevPoint := vertices[inEdge.GetTail()].GetCoordinate()
				tail := vertices[via].GetCoordinate()
				headPoint := vertices[outEdge.GetHead()].GetCoordinate()

				prevInitialBearing := geo.ComputeInitialBearing(prevPoint.GetLat(), prevPoint.GetLon(), tail.GetLat(),
					tail.GetLon())
				turn := geo.GetTurnDirection(tail.GetLat(), tail.GetLon(), headPoint.GetLat(),
					headPoint.GetLon(), prevInitialBearing)

				l := util.MetersFromCentimeters(inLengths[via][entryPoint])
				lPrime := util.MetersFromCentimeters(outLengths[via][exitPoint])

				if !util.Eq(l, 0) && !util.Eq(lPrime, 0) {
					delta := pkg.CalcResolution(l, lPrime, pkg.INF_WEIGHT)
					minResolution = min(minResolution, delta)
				}

				switch turn {
				case da.TURN_SLIGHT_LEFT, da.TURN_LEFT, da.TURN_SHARP_LEFT:
					turnMatrices[via][rowOffset+exitPoint] = pkg.LEFT_TURN
				case da.TURN_SLIGHT_RIGHT, da.TURN_RIGHT, da.TURN_SHARP_RIGHT:
					turnMatrices[via][rowOffset+exitPoint] = pkg.RIGHT_TURN
				}
			}
		}
	}

	fmt.Printf("45%%...")

	conditionalTurnRestrictions := make([]da.ConditionalTurnRestriction, 0)

	// let m=number of ways , q = max number of nodes of any osm ways, r = max number of restrictions of any osm ways
	// O(m*r*q^2)
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
				addViaNodeTurnRestriction(p, wayId, way, fromNodes, restriction, fromResId, outEdges, inEdges, outDegree, vertices, turnMatrices, &conditionalTurnRestrictions)
			} else if restriction.isWay {
				addViaWayTurnRestriction(p, wayId, way, fromNodes, restriction, fromResId, outEdges, inEdges, graphStorage, edgeInfoIds, turnMatrices, outDegree, inDegree)
			}
		}
	}

	fmt.Printf("85%%...")

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

	// dummy vertex
	vertices[len(vertices)-1] = da.NewVertex(0, 0, da.Index(len(vertices)-1))
	vertices[len(vertices)-1].SetFirstOut(outEdgeOffset)
	vertices[len(vertices)-1].SetFirstIn(inEdgeOffset)

	flattenOutEdges := flatten(outEdges)
	defaultWeights := flatten(outWeights)
	segmentLengths := flatten(outLengths)
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
	graphStorage.BuildNameTable(p.tagStringIdMap.GetIdToStr())

	setConditionalRestrictions(p, roadNetwork, graph, graphStorage, edgeInfoIds, conditionalTurnRestrictions)

	graph.SetGraphStorage(graphStorage)

	if roadNetwork {
		graph.SetMinResolution(minResolution)
	}

	timeFunction := costfunction.NewPreprocessingTimeFunction(
		roadNetwork, defaultWeights, segmentLengths,
	)

	fmt.Printf("100%%...\n")
	return graph, timeFunction, edgeInfoIds
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

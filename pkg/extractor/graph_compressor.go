package extractor

import (
	"maps"

	"github.com/bits-and-blooms/bitset"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
compress incoming edge & outgouing edge dari vertices yang punya inDegree=1 & outDegree=1 & beberapa kriteria lainnya menjadi satu edge

example:
v0-------e1----->v1-----e2----->v2
								|
								|
								e3
								|
								|
								\/
					...<---e5----v3-----e4---->...

v1,v2 punya inDegree=1 & outDegree=1 & beberapa kriteria lainnya (spt. highway key & osm way id kedua edge nya sama, speed limit kedua edge sama,dll)
kita bisa compress e1,e2 jadi satu edge e6:
v0----e6--->v2

step buat compress nya:
1. tandain vertices yang bisa di compress
2. iterate semua vertices yang bisa di compress:
3. di setiap vertex yang bisa di compress & belum discovered:
4. traverse/dfs forward pakai outgouing edges, selama traverse add compressible edges ke array && tandain discovered vertices, stop ketika current vertex non compressible / discovered.
5. traverse/dfs backward pakai incoming edges, sama kaya diatas...
6. merge semua compressible edges.

inspired by: https://github.com/Project-OSRM/osrm-backend/blob/master/src/extractor/graph_compressor.cpp
*/

// compressOSMGraph compress osm node-based graph by removing vertices with only outdegree & indegree of 1.
func (p *Extractor[W]) compressOSMGraph(
	edges []Edge[W],
	rn *da.RoadNetworkDataContainer,
	streetDirection map[int64][2]bool,
) ([]Edge[W], *da.RoadNetworkDataContainer, uint32) {
	numVertices := len(p.nodeToOsmId)

	inEdges := make([][]int, numVertices)
	outEdges := make([][]int, numVertices)
	for edgeId := range edges {
		edge := &edges[edgeId]
		outEdges[edge.from] = append(outEdges[edge.from], edgeId)
		inEdges[edge.to] = append(inEdges[edge.to], edgeId)
	}

	dfsState := make([]int, numVertices)

	protected := p.compressionProtectedVertices(edges, rn)
	contractible := make([]bool, numVertices) // if contractible[vertex] == true, incomingEdge->vertex->outgouingEdge can be compressed as one edge
	for vertex := range contractible {
		// O(n).n=number of vertices
		dfsState[vertex] = unvisited
		if protected[vertex] || len(inEdges[vertex]) != 1 || len(outEdges[vertex]) != 1 {
			// only compress vertices that only have inDegree=1 & outDegree=1
			continue
		}

		inID := inEdges[vertex][0]
		outID := outEdges[vertex][0]
		if inID == outID {
			// cycle edge
			continue
		}
		contractible[vertex] = canCompress(
			&edges[inID], &edges[outID], rn, da.Index(inID), da.Index(outID),
			streetDirection, p.osmWayDefaultSpeed,
		)
	}

	// tandain satu vertex di cycle of contractible vertices sebagai non-contractible
	// O(n+m) dfs
	for u := range contractible {
		if dfsState[u] == unvisited && contractible[u] {
			cycle, v := cycleCheck(uint32(u), dfsState, outEdges, edges, contractible)
			if cycle {
				contractible[v] = false
			}
		}
	}

	// Removed vertices keep INVALID_VERTEX_ID.
	// only store uncontractible vertices
	// remap vertex ids
	oldToNew := make([]da.Index, numVertices)
	for i := range oldToNew {
		oldToNew[i] = da.INVALID_VERTEX_ID
	}
	newNodeToOSMID := make(map[da.Index]int64, numVertices)
	newNodeIDMap := make(map[int64]da.Index, numVertices)
	var nVID da.Index
	// O(n)
	for oVId := range contractible {
		if contractible[oVId] {
			// karena kita remove compressible vertices
			continue
		}
		oldToNew[oVId] = nVID

		osmID := p.nodeToOsmId[da.Index(oVId)]
		newNodeToOSMID[nVID] = osmID
		newNodeIDMap[osmID] = nVID
		nVID++
	}

	m := len(edges)
	compEdges := make([]Edge[W], 0, m)
	crn := da.NewRoadNetworkDataContainer(rn.GetOsmwayBitSize())
	discovered := make([]bool, numVertices)
	compEdgesSet := bitset.New(uint(m))

	for v := da.Index(0); v < da.Index(numVertices); v++ {
		if !contractible[v] || discovered[v] {
			continue
		}
		discovered[v] = true
		p.merge(inEdges, outEdges, compEdgesSet, v, contractible, discovered, edges, &compEdges, crn, rn, oldToNew)
	}

	// sisa edges yang non-compressible
	// O(m)
	for eId := da.Index(0); eId < da.Index(m); eId++ {
		if compEdgesSet.Test(uint(eId)) {
			continue
		}
		e := edges[eId]
		e.from = uint32(oldToNew[e.from])
		e.to = uint32(oldToNew[e.to])
		neId := da.Index(len(compEdges))
		geometry := rn.GetSegmentGeometry(eId)
		nIds := rn.GetSegmentOsmNodeIds(eId)
		curved := rn.IsCurved(eId)
		p.appendCompSegments(eId, neId, e, geometry, nIds, curved, crn, rn, &compEdges)
	}

	p.nodeToOsmId = newNodeToOSMID
	p.nodeIDMap = newNodeIDMap
	p.applyOsmWayGraphNodesPermutation(oldToNew)
	return compEdges, crn, uint32(nVID)
}

func (p *Extractor[W]) appendCompSegments(sId, nSegId da.Index, merged Edge[W], geometry []da.Coordinate, osmNodeIds []uint64, curved bool,
	crn *da.RoadNetworkDataContainer, rn *da.RoadNetworkDataContainer, compEdges *[]Edge[W]) {
	start := da.Index(crn.GetOsmNodePointsCount())
	crn.AppendOsmNodePoints(geometry, osmNodeIds)
	end := da.Index(crn.GetOsmNodePointsCount())
	crn.AppendSegmentData(
		merged.osmwayId,
		start,
		end,
		rn.GetStreetNameId(sId),
		rn.GetRoadClass(sId),
		rn.GetRoadClassLink(sId),
		rn.GetRoadLanes(sId),
		da.NewEmptyTurnLanesData(),
	)

	flag := rn.GetSegmentFlag(sId)
	if curved {
		flag |= da.FlagIsCurved
	} else {
		flag &= ^da.FlagIsCurved
	}

	crn.SetSegmentFlag(nSegId, flag)
	*compEdges = append(*compEdges, merged)
}

// merge. form path that all of it inner vertices (not the endpoint vertices of the path) only have indegree 1 and outdegree 1. and merge this path.
func (p *Extractor[W]) merge(inEdges, outEdges [][]int, compEdgesSet *bitset.BitSet, v da.Index, contractible, discovered []bool, edges []Edge[W],
	compEdges *[]Edge[W], crn, rn *da.RoadNetworkDataContainer, oldToNew []da.Index) {

	compressibleEdges := make([]int, 0, 4)

	edgeId := inEdges[v][0]
	for {
		compressibleEdges = append(compressibleEdges, edgeId)
		compEdgesSet.Set(uint(edgeId))
		tail := edges[edgeId].from
		if !contractible[tail] || discovered[tail] {
			break
		}
		discovered[tail] = true
		edgeId = inEdges[tail][0]
	}
	util.ReverseG(compressibleEdges)

	edgeId = outEdges[v][0]
	for {
		compressibleEdges = append(compressibleEdges, edgeId)
		compEdgesSet.Set(uint(edgeId))
		head := edges[edgeId].to
		if !contractible[head] || discovered[head] {
			break
		}
		discovered[head] = true
		edgeId = outEdges[head][0]
	}

	merged, geometry, osmNodeIds, curved := mergeOsmSegments(edges, rn, compressibleEdges, oldToNew)

	sId := da.Index(compressibleEdges[0])
	neId := da.Index(len(*compEdges))
	p.appendCompSegments(sId, neId, merged, geometry, osmNodeIds, curved, crn, rn, compEdges)
}

// mergeOsmSegments combines  edges into a single edge.
func mergeOsmSegments[W util.RoutingNumber](
	edges []Edge[W],
	rn *da.RoadNetworkDataContainer,
	edgeIds []int,
	oldToNew []da.Index,
) (Edge[W], []da.Coordinate, []uint64, bool) {
	first := edges[edgeIds[0]]
	last := edges[edgeIds[len(edgeIds)-1]]
	var totalWeight W
	var totalLength uint32
	geometry := make([]da.Coordinate, 0, len(edgeIds)*2)
	osmNodeIds := make([]uint64, 0, len(edgeIds)*2)
	curved := false

	for _, edgeId := range edgeIds {
		edge := &edges[edgeId]
		totalWeight += edge.weight
		totalLength += edge.distance
		points := rn.GetSegmentGeometry(da.Index(edgeId))
		nIds := rn.GetSegmentOsmNodeIds(da.Index(edgeId))

		if len(nIds) > 0 && len(osmNodeIds) > 0 &&
			osmNodeIds[len(osmNodeIds)-1] == nIds[0] {
			points = points[1:]
			nIds = nIds[1:]
		}
		geometry = append(geometry, points...)
		osmNodeIds = append(osmNodeIds, nIds...)
		curved = curved || rn.IsCurved(da.Index(edgeId))
	}
	merged := first
	merged.from = uint32(oldToNew[first.from])
	merged.to = uint32(oldToNew[last.to])
	merged.weight = W(totalWeight)
	merged.distance = uint32(totalLength)
	merged.toOsmId = last.toOsmId

	return merged, geometry, osmNodeIds, curved
}

// compressionProtectedVertices marks vertices that carry semantics an edge
// merge cannot represent safely.
// This includes barriers, traffic lights, turn-restriction participants, and
// endpoints of conditionally restricted ways.
func (p *Extractor[W]) compressionProtectedVertices(
	edges []Edge[W],
	rn *da.RoadNetworkDataContainer,
) []bool {
	protected := make([]bool, len(p.nodeToOsmId))
	// O(n). n=number of vertices
	for vertex, osmID := range p.nodeToOsmId {

		if p.barrierNodes[osmID] {
			// protect barrier nodes
			protected[vertex] = true
		}
	}

	for _, barrier := range p.conditionalBarrierNodes {
		// protect barrier nodes
		if vertex, ok := p.nodeIDMap[barrier.GetOsmNodeId()]; ok {
			protected[vertex] = true
		}
	}
	for fromWay, restrictions := range p.restrictions {
		// protect vertices that along a turn restrictions (fromWay, viaNode, toWay) or (fromWay, viaWays, toWay)
		protectWayGraphNodes(protected, p.ways[fromWay])
		for _, restriction := range restrictions {
			protectWayGraphNodes(protected, p.ways[restriction.to])
			if !restriction.isWay {
				protected[restriction.via] = true
				continue
			}
			for _, viaWay := range restriction.viaWays {
				protectWayGraphNodes(protected, p.ways[viaWay])
			}
		}
	}
	// O(m). m = number of edges
	for edgeId := range edges {
		// nodes contained in conditional restriction
		edge := &edges[edgeId]
		_, conditionalSpeed := p.conditionalSpeedLimits[edge.osmwayId]
		_, conditionalDirection := p.conditionalReversibleWayVals[edge.osmwayId]
		_, conditionalAccess := p.conditionalTrafficModesVal[edge.osmwayId]
		if conditionalSpeed || conditionalDirection || conditionalAccess {
			protected[edge.from] = true
			protected[edge.to] = true
		}
	}
	return protected
}

// protectWayGraphNodes protect graph nodes of this osm way so that it not contracted later.
func protectWayGraphNodes(protected []bool, way osmWay) {
	for _, vertex := range way.graphNodes {
		protected[vertex] = true
	}
}

const (
	unvisited int = iota
	explored      // visisted but not yet completed
	visited       // visited and completed
)

// canCompress reports whether removing the shared vertex preserves
// routing, access, and guidance semantics.
// OSM way IDs intentionally do not need to match. Adjacent ways may be merged
// when their road metadata, direction, speed, and geometry describe the same
// continuous road.
func canCompress[W util.RoutingNumber](
	inEdge, outEdge *Edge[W],
	rn *da.RoadNetworkDataContainer,
	inID, outID da.Index,
	streetDirection map[int64][2]bool,
	waySpeeds map[int64]float64,
) bool {

	if rn.IsRoundabout(inID) || rn.IsRoundabout(outID) {
		return false
	}

	if rn.GetStreetNameId(inID) != rn.GetStreetNameId(outID) ||
		rn.GetRoadLanes(inID) != rn.GetRoadLanes(outID) ||
		rn.GetRoadClass(inID) != rn.GetRoadClass(outID) ||
		rn.GetRoadClassLink(inID) != rn.GetRoadClassLink(outID) ||
		streetDirection[inEdge.osmwayId] != streetDirection[outEdge.osmwayId] {
		return false
	}

	return isSameSpeed(inEdge, outEdge, waySpeeds)
}

func isSameSpeed[W util.RoutingNumber](inEdge *Edge[W], outEdge *Edge[W], waySpeeds map[int64]float64) bool {
	inSpeed, hasInSpeed := waySpeeds[inEdge.osmwayId]
	outSpeed, hasOutSpeed := waySpeeds[outEdge.osmwayId]

	return hasInSpeed && hasOutSpeed &&
		inSpeed == outSpeed
}

// cycleCheck. find cycle of contractible vertices using dfs.
func cycleCheck[W util.RoutingNumber](u uint32, dfsState []int, outEdges [][]int, edges []Edge[W], contractible []bool) (bool, uint32) {
	dfsState[u] = explored
	for _, eId := range outEdges[u] {
		v := edges[eId].to
		if !contractible[v] {
			continue
		}
		if dfsState[v] == explored || dfsState[v] == visited {
			return true, v
		}
		if found, w := cycleCheck[W](v, dfsState, outEdges, edges, contractible); found {
			return true, w
		}
	}

	dfsState[u] = visited
	return false, 0
}

// applyOsmWayGraphNodesPermutation updates osmparser fields after compression.
func (p *Extractor[W]) applyOsmWayGraphNodesPermutation(oldToNew []da.Index) {
	newWayMap := make(map[int64]osmWay, len(p.ways))
	for wayID, way := range p.ways {
		graphNodes := make([]da.Index, 0)
		for _, oVId := range way.graphNodes {
			newVertex := oldToNew[oVId]
			if newVertex == da.INVALID_VERTEX_ID {
				continue
			}
			graphNodes = append(graphNodes, newVertex)
		}
		way.graphNodes = graphNodes
		newWayMap[wayID] = way
	}
	p.ways = newWayMap

	newRestricions := make(map[int64][]restriction, len(p.restrictions))
	maps.Copy(newRestricions, p.restrictions)
	for fromWay, restrictions := range p.restrictions {
		for i := range restrictions {
			if !restrictions[i].isWay { // via-node turn restrictions
				newRestricions[fromWay][i].via = oldToNew[restrictions[i].via]
			}
		}
	}
	p.restrictions = newRestricions
}

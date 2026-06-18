package osmparser

import (
	"fmt"

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
1. traverse/dfs forward pakai outgouing edges, selama traverse add compressible edges ke array && tandain discovered vertices, stop ketika current vertex non compressible / discovered.
2. traverse/dfs backward pakai incoming edges, sama kaya diatas...
3. merge semua compressible edges.
*/

// compressOSMGraph replaces compatible directed edge chains with single edges.
//
// Compression is limited to OSM input because it depends on OSM metadata and
// parser bookkeeping. A vertex may be removed only when it has exactly one
// incoming and one outgoing edge, is not protected by routing semantics, and
// both adjacent edges describe the same continuous road.
//
// The pass has five stages:
//  1. Build incoming and outgoing edge indexes.
//  2. Mark protected and contractible vertices.
//  3. Assign compact IDs to the vertices that remain.
//  4. Merge each maximal edge chain and rebuild edge metadata.
//  5. Remap OSM-node, way, and restriction references to compact IDs.
func (p *OsmParser) compressOSMGraph(
	edges []Edge[int32],
	storage *da.GraphStorage,
	streetDirection map[int64][2]bool,
) ([]Edge[int32], *da.GraphStorage, uint32) {
	numVertices := len(p.nodeToOsmId)

	// Edge indexes make degree checks and chain traversal constant time.
	inEdges := make([][]int, numVertices)
	outEdges := make([][]int, numVertices)
	for edgeId := range edges {
		edge := &edges[edgeId]
		outEdges[edge.from] = append(outEdges[edge.from], edgeId)
		inEdges[edge.to] = append(inEdges[edge.to], edgeId)
	}
	dfsState := make([]int, numVertices)

	// A 1-in/1-out vertex is only a candidate. Metadata compatibility below
	// decides whether removing it preserves routing and guidance behavior.
	protected := p.compressionProtectedVertices(edges, storage)
	contractible := make([]bool, numVertices) // if contractible[vertex] == true, incomingEdge->vertex->outgouingEdge can be compressed as one edge
	for vertex := range contractible {
		dfsState[vertex] = unvisited
		if protected[vertex] || len(inEdges[vertex]) != 1 || len(outEdges[vertex]) != 1 {
			// only compress vertices that only have inDegree=1 & outDegree=1
			continue
		}
		inID := inEdges[vertex][0]
		outID := outEdges[vertex][0]
		if inID == outID {
			continue
		}
		contractible[vertex] = canCompress(
			&edges[inID], &edges[outID], storage, da.Index(inID), da.Index(outID),
			streetDirection, p.osmWayDefaultSpeed,
		)
	}

	for u := range contractible {
		if dfsState[u] == unvisited && contractible[u] {
			cycle, v := cycleCheck(uint32(u), dfsState, outEdges, edges, contractible)
			if cycle {
				contractible[v] = false
			}
		}
	}

	// Removed vertices keep INVALID_VERTEX_ID. Every surviving vertex receives
	// a dense ID so the compressed graph does not need placeholder vertices.
	oldToNew := make([]da.Index, numVertices)
	for i := range oldToNew {
		oldToNew[i] = da.INVALID_VERTEX_ID
	}
	newNodeToOSMID := make(map[da.Index]int64, numVertices)
	newNodeIDMap := make(map[int64]da.Index, numVertices)
	var newVertexID da.Index
	for oldVertex := range contractible {
		if contractible[oldVertex] {
			// karena kita remove compressible vertices
			continue
		}
		oldToNew[oldVertex] = newVertexID
		osmID := p.nodeToOsmId[da.Index(oldVertex)]
		newNodeToOSMID[newVertexID] = osmID
		newNodeIDMap[osmID] = newVertexID
		newVertexID++
	}

	m := len(edges)
	compressed := make([]Edge[int32], 0, m)
	compressedStorage := da.NewGraphStorage(storage.GetOsmwayBitSize())
	discovered := make([]bool, numVertices)
	compressedEdgesSet := bitset.New(uint(m))

	appendEdgeToStorage := func(sourceID, newEdgeId da.Index, mergedEdge Edge[int32], geometry []da.Coordinate, curved bool) {
		start := da.Index(compressedStorage.GetOsmNodePointsCount())
		compressedStorage.AppendOsmNodePoints(geometry)
		end := da.Index(compressedStorage.GetOsmNodePointsCount())
		compressedStorage.AppendEdgeMetadata(
			mergedEdge.osmwayId,
			start,
			end,
			storage.GetStreetNameId(sourceID),
			storage.GetRoadClass(sourceID),
			storage.GetRoadClassLink(sourceID),
			storage.GetRoadLanes(sourceID),
		)
		compressedStorage.SetRoundabout(newEdgeId, storage.IsRoundabout(sourceID))
		compressedStorage.SetIsCurved(newEdgeId, curved)
		compressed = append(compressed, mergedEdge)
	}

	emit := func(v da.Index) {
		compressibleEdges := make([]int, 0, 4)

		edgeId := inEdges[v][0]
		for {
			compressibleEdges = append(compressibleEdges, edgeId)
			compressedEdgesSet.Set(uint(edgeId))
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
			compressedEdgesSet.Set(uint(edgeId))
			head := edges[edgeId].to
			if !contractible[head] || discovered[head] {
				break
			}
			discovered[head] = true
			edgeId = outEdges[head][0]
		}

		mergedEdge, geometry, curved := mergeOSMEdgeChain(edges, storage, compressibleEdges, oldToNew)

		sourceID := da.Index(compressibleEdges[0])
		newEId := da.Index(len(compressed))
		appendEdgeToStorage(sourceID, newEId, mergedEdge, geometry, curved)
	}

	for v := da.Index(0); v < da.Index(numVertices); v++ {
		if !contractible[v] || discovered[v] {
			continue
		}
		discovered[v] = true
		emit(v)
	}

	// sisa edges yang non-compressible
	for eId := da.Index(0); eId < da.Index(m); eId++ {
		if compressedEdgesSet.Test(uint(eId)) {
			continue
		}
		e := edges[eId]
		e.from = uint32(oldToNew[e.from])
		e.to = uint32(oldToNew[e.to])
		newEId := da.Index(len(compressed))
		geometry := storage.GetEdgeGeometry(eId)
		curved := storage.IsCurved(eId)
		appendEdgeToStorage(eId, newEId, e, geometry, curved)
	}

	// Traffic-light vertices are protected, so their compact IDs are always
	// valid and their vertex-level metadata can be copied directly.
	for oldVertex, newID := range oldToNew {
		if newID == da.INVALID_VERTEX_ID {
			continue
		}
		if storage.GetTrafficLight(da.Index(oldVertex)) {
			compressedStorage.SetTrafficLight(newID, true)
		}
	}
	p.nodeToOsmId = newNodeToOSMID
	p.nodeIDMap = newNodeIDMap
	p.remapCompressedGraphNodes(oldToNew)
	return compressed, compressedStorage, uint32(newVertexID)
}

// compressionProtectedVertices marks vertices that carry semantics an edge
// merge cannot represent safely.
//
// This includes barriers, traffic lights, turn-restriction participants, and
// endpoints of conditionally restricted ways. Protecting both endpoints of a
// conditional edge keeps later time-dependent customization attached to the
// same graph locations.
func (p *OsmParser) compressionProtectedVertices(
	edges []Edge[int32],
	storage *da.GraphStorage,
) []bool {
	protected := make([]bool, len(p.nodeToOsmId))
	for vertex, osmID := range p.nodeToOsmId {
		if p.barrierNodes[osmID] || storage.GetTrafficLight(vertex) {
			protected[vertex] = true
		}
	}
	for _, barrier := range p.conditionalBarrierNodes {
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
				if int(restriction.via) < len(protected) {
					protected[restriction.via] = true
				}
				continue
			}
			for _, viaWay := range restriction.viaWays {
				protectWayGraphNodes(protected, p.ways[viaWay])
			}
		}
	}
	for edgeId := range edges {
		edge := &edges[edgeId]
		_, conditionalSpeed := p.conditionalSpeedLimits[edge.osmwayId]
		_, conditionalDirection := p.conditionalReversibleWayVals[edge.osmwayId]
		_, conditionalAccess := p.conditionalTrafficModesVal[edge.osmwayId]
		if edge.containsTrafficLight || conditionalSpeed || conditionalDirection || conditionalAccess {
			protected[edge.from] = true
			protected[edge.to] = true
		}
	}
	return protected
}

// protectWayGraphNodes preserves every graph vertex referenced by a restricted
// way because a turn restriction may depend on any position along that way.
func protectWayGraphNodes(protected []bool, way osmWay) {
	for _, vertex := range way.graphNodes {
		if int(vertex) < len(protected) {
			protected[vertex] = true
		}
	}
}

const (
	unvisited int = iota
	explored      // visisted but not yet completed
	visited       // visited and completed
)

// canCompress reports whether removing the shared vertex preserves
// routing, access, and guidance semantics.
//
// OSM way IDs intentionally do not need to match. Adjacent ways may be merged
// when their road metadata, direction, speed, and geometry describe the same
// continuous road. If explicit way speeds are unavailable, equal
// weight-to-distance ratios provide the equivalent speed check without
// floating-point division.
func canCompress(
	inEdge, outEdge *Edge[int32],
	storage *da.GraphStorage,
	inID, outID da.Index,
	streetDirection map[int64][2]bool,
	waySpeeds map[int64]float64,
) bool {
	if inEdge.containsTrafficLight || outEdge.containsTrafficLight {
		return false
	}

	if storage.IsRoundabout(inID) || storage.IsRoundabout(outID) {
		return false
	}

	if inEdge.hwType != outEdge.hwType ||
		storage.GetStreetNameId(inID) != storage.GetStreetNameId(outID) ||
		storage.GetRoadLanes(inID) != storage.GetRoadLanes(outID) ||
		storage.GetRoadClass(inID) != storage.GetRoadClass(outID) ||
		storage.GetRoadClassLink(inID) != storage.GetRoadClassLink(outID) ||
		streetDirection[inEdge.osmwayId] != streetDirection[outEdge.osmwayId] {
		return false
	}

	inSpeed, hasInSpeed := waySpeeds[inEdge.osmwayId]
	outSpeed, hasOutSpeed := waySpeeds[outEdge.osmwayId]
	if hasInSpeed && hasOutSpeed {
		return inSpeed == outSpeed
	}
	return inEdge.distance > 0 && outEdge.distance > 0 &&
		inEdge.weight > 0 && outEdge.weight > 0 &&
		int64(inEdge.weight)*int64(outEdge.distance) ==
			int64(outEdge.weight)*int64(inEdge.distance)
}

// cycleCheck. find cycle of contractible vertices.
func cycleCheck(u uint32, dfsState []int, outEdges [][]int, edges []Edge[int32], contractible []bool) (bool, uint32) {
	dfsState[u] = explored
	for _, eId := range outEdges[u] {
		v := edges[eId].to
		if !contractible[v] {
			continue
		}
		if dfsState[v] == explored || dfsState[v] == visited {
			return true, v
		}
		if found, w := cycleCheck(v, dfsState, outEdges, edges, contractible); found {
			return true, w
		}
	}

	dfsState[u] = visited
	return false, 0
}

// mergeOSMEdgeChain combines one maximal chain into a single edge.
//
// Weight and distance are accumulated in int64 so overflow is detected before
// narrowing to their stored int32 and uint32 types. Geometry is appended into
// new storage and duplicate coordinates at edge boundaries are omitted. The
// first edge supplies compatible road metadata, while the last edge supplies
// destination-specific OSM and junction fields.
func mergeOSMEdgeChain(
	edges []Edge[int32],
	storage *da.GraphStorage,
	edgeIds []int,
	oldToNew []da.Index,
) (Edge[int32], []da.Coordinate, bool) {
	first := edges[edgeIds[0]]
	last := edges[edgeIds[len(edgeIds)-1]]
	var totalWeight int64
	var totalLength int64
	geometry := make([]da.Coordinate, 0, len(edgeIds)*2)
	curved := false
	for _, edgeId := range edgeIds {
		edge := &edges[edgeId]
		totalWeight += int64(edge.weight)
		totalLength += int64(edge.distance)

		points := storage.GetEdgeGeometry(da.Index(edgeId))
		// Adjacent edge geometries normally share their boundary coordinate.
		// Store it once so distance consumers and guidance see a continuous path.
		if len(geometry) > 0 && len(points) > 0 &&
			da.IsSameCoordinate(geometry[len(geometry)-1], points[0]) {
			points = points[1:]
		}
		geometry = append(geometry, points...)
		curved = curved || storage.IsCurved(da.Index(edgeId))
	}
	merged := first
	merged.from = uint32(oldToNew[first.from])
	merged.to = uint32(oldToNew[last.to])
	merged.weight = int32(totalWeight)
	if merged.to == uint32(da.INVALID_VERTEX_ID) {
		fmt.Printf("debug")
	}
	merged.distance = uint32(totalLength)
	merged.toOsmId = last.toOsmId
	merged.junctionHead = last.junctionHead
	merged.containsTrafficLight = false
	return merged, geometry, curved
}

// remapCompressedGraphNodes updates parser-owned references after compaction.
//
// Removed vertices disappear from way.graphNodes, consecutive duplicate IDs
// are collapsed, and node-based restriction via vertices are translated to
// their compact IDs. Restriction participants were protected earlier, so a
// node-based via vertex must always have a valid mapping here.
func (p *OsmParser) remapCompressedGraphNodes(oldToNew []da.Index) {
	for wayID, way := range p.ways {
		graphNodes := way.graphNodes[:0]
		for _, oldVertex := range way.graphNodes {
			newVertex := oldToNew[oldVertex]
			if newVertex == da.INVALID_VERTEX_ID {
				continue
			}
			if len(graphNodes) == 0 || graphNodes[len(graphNodes)-1] != newVertex {
				graphNodes = append(graphNodes, newVertex)
			}
		}
		way.graphNodes = graphNodes
		p.ways[wayID] = way
	}
	for fromWay, restrictions := range p.restrictions {
		for i := range restrictions {
			if !restrictions[i].isWay { // via-node turn restrictions
				restrictions[i].via = oldToNew[restrictions[i].via]
			}
		}
		p.restrictions[fromWay] = restrictions
	}
}

package datastructure

import (
	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// SubVertex   map dari (vId, entryExitPoint, exitFlag) to overlay vertex.
type SubVertex struct {
	vId            Index // original vertex id
	exitEntryOrder Index // entry/exit point order (from 0 to outDegree-1/inDegree-1)
	exit           bool  // is exit point
}

// Pv is a type representing a cell number.
type Pv uint64

// Graph represents the main Customizable Route Planning (CRP) compact graph.
// uses adjacency arrays / Compressed Sparse Row (CSR).
// CSR graph nya C++ Boost library: https://www.boost.org/doc/libs/1_61_0/libs/graph/doc/compressed_sparse_row.html
// kode ini terinspirasi dari implementasi CRP yang dibuat oleh Michael Wegner: https://github.com/michaelwegner/CRP/blob/master/datastructures/Graph.h
// untuk adjacency array  juga terinspirasi dari: https://github.com/RoutingKit/RoutingKit/blob/54d49bb0cdea56dde182357522e4e86a03c57852/include/routingkit/osm_graph_builder.h
// See section 4.1 & 4.3: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
type Graph struct {
	vertices    []Vertex // map from vertex v id to vertex v data structure
	heads       []Index  // head v dari edge (u,v) sorted by tail u
	tails       []Index  // tail v reversed edges (v,u). setiap in edge (v,u) punya bobot yang sama dengan out edge (u,v). sorted by head u.
	entryPoints []Index  // map from outgoing edge id (u,v) to index of this edge in the list of incoming edges of v.
	exitPoints  []Index  // map from incoming (reversed) edge id (v,u) to index of this edge in the of outgoing edges of v.

	// overlay graph related
	overlayVertices   map[SubVertex]Index // graph vertices -> overlay vertices
	cellNumbers       []Pv                // cellNumbers contains all unique bitpacked cell numbers from level 0->L for each vertex.
	outEdgeCellOffset []Index             // offset of first outEdge for each cellNumber
	inEdgeCellOffset  []Index             // offset of first inEdge for each cellNumber

	// strongly connected components related
	sccs               []Index // verticeId -> sccId
	sccCondensationAdj [][]Index
	// sccCondensationAdj [][]Index // condensation graph connection of scc of u -> scc of v
	sccReach []*bitset.BitSet // sccId v -> bitset dari list dari other sccIds u yang dapat reach sccId v

	boundingBox       *BoundingBox
	minResolution     float64
	maxVerticesInCell Index // maximum number of vertices in any level 1 cell
	roadNetwork       bool
}

func NewGraph(vertices []Vertex, heads []Index, tails []Index, roadNetwork bool, entryPoints []Index, exitPoints []Index) *Graph {
	return &Graph{vertices: vertices, heads: heads, tails: tails, maxVerticesInCell: 0, roadNetwork: roadNetwork,
		entryPoints: entryPoints, exitPoints: exitPoints}
}

// ---- graph data stucture related ----

func (g *Graph) NumberOfVertices() int {
	return len(g.vertices) - 1
}

func (g *Graph) NumberOfEdges() int {
	return len(g.heads)
}

func (g *Graph) IsRoadNetworkGraph() bool {
	return g.roadNetwork
}

func (g *Graph) SetMinResolution(minResolution float64) {
	g.minResolution = minResolution
}

func (g *Graph) GetMinResolution() float64 {
	return g.minResolution
}

func (g *Graph) GetOutDegree(u Index) Index {
	// must return index for uint32 (lot usage of outDegree used as big slice size)
	return g.vertices[u+1].firstOut - g.vertices[u].firstOut
}

func (g *Graph) GetInDegree(u Index) Index {
	return g.vertices[u+1].firstIn - g.vertices[u].firstIn
}

func (g *Graph) GetExitOffset(u Index) Index {
	return g.vertices[u].firstOut
}

func (g *Graph) GetEntryOffset(u Index) Index {
	return g.vertices[u].firstIn
}

func (g *Graph) GetInEdgeId(u Index, enPoint Index) Index {
	return g.vertices[u].firstIn + enPoint
}

func (g *Graph) GetOutEdgeId(u Index, exPoint Index) Index {
	return g.vertices[u].firstOut + exPoint
}

func (g *Graph) GetHead(e Index) Index {
	return g.heads[e]
}

func (g *Graph) GetTail(e Index) Index {
	return g.tails[e]
}

func (g *Graph) GetCellNumbers() []Pv {
	return g.cellNumbers
}

func (g *Graph) GetHeadOfInedge(e Index) Index {
	v := g.tails[e]
	tail := g.vertices[v]
	exitPoint := Index(g.exitPoints[e])
	return g.heads[tail.firstOut+exitPoint]
}

func (g *Graph) GetTailOfOutedge(e Index) Index {
	v := g.heads[e]
	head := g.vertices[v]
	entryPoint := Index(g.entryPoints[e])
	return g.tails[head.firstIn+Index(entryPoint)]
}

// get inEdgeId of outEdgeId e
func (g *Graph) GetRevId(e Index) Index {
	head := g.vertices[g.heads[e]]
	entryPoint := Index(g.entryPoints[e])
	return head.firstIn + Index(entryPoint)
}

// get outgoing edge Id of incoming edge Id e
func (g *Graph) GetOutId(e Index) Index {
	tail := g.vertices[g.tails[e]]
	exitPoint := Index(g.exitPoints[e])
	return tail.firstOut + Index(exitPoint)
}

// GetExitOrder. return Index of exit point of a out edge (u,v) at vertex u.
func (g *Graph) GetExitOrder(u, outEdgeId Index) Index {
	exitPoint := outEdgeId - g.vertices[u].firstOut
	return exitPoint
}

// GetEntryOrder. return Index of entry point of a in edge (u,v) at vertex v.
func (g *Graph) GetEntryOrder(v, inEdgeId Index) Index {
	return inEdgeId - g.vertices[v].firstIn
}

// GetEntryPoint. get entryPoint of outgoing edge eId
func (g *Graph) GetEntryPoint(eId Index) Index {
	return g.entryPoints[eId]
}

func (g *Graph) SetCellNumbers(cellNumbers []Pv) {
	g.cellNumbers = cellNumbers
}

func (g *Graph) SetOverlayMapping(overlayVertices map[SubVertex]Index) {
	g.overlayVertices = overlayVertices
}

// ForOutEdgesOf. iterates all outgoing edges of vertex u.
func (g *Graph) ForOutEdgesOf(u Index, handle func(eId, head Index, entryPoint Index)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		entryPoint := g.entryPoints[e]
		handle(e, g.heads[e], entryPoint)
	}
}

// ForOutEdgesOf. iterates all incoming (reversed) edges of vertex u.
func (g *Graph) ForInEdgesOf(v Index, handle func(eId, tail Index, exitPoint Index)) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {
		exitPoint := g.exitPoints[e]
		handle(e, g.tails[e], exitPoint)
	}
}

// ForOutEdgesOfWithTurn. iterate outgoing edges of vertex u from i-th incoming edge of u. return turn table id & turn type of its turn, highway type, head vertex, edge id, etc.
func (g *Graph) ForOutEdgesOfWithTurn(u Index, i Index, handle func(eId, head, exitPoint, entryPoint Index)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		handle(e, g.heads[e], g.GetExitOrder(u, e), g.entryPoints[e])
	}
}

func (g *Graph) GetNumberOfEdges(u Index) Index {
	return g.vertices[u+1].firstOut - g.vertices[u].firstOut
}

func (g *Graph) ForOutEdgeIdsOf(u Index, handle func(eId Index)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		handle(e)
	}
}

// GetOutEdgeBounds exposes the contiguous outgoing edge range for allocation-free iteration.
func (g *Graph) GetOutEdgeBounds(u Index) (Index, Index) {
	return g.vertices[u].firstOut, g.vertices[u+1].firstOut
}

func (g *Graph) ForInEdgeIdsOf(v Index, handle func(id Index)) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {
		handle(e)
	}
}

// GetOverlayVertex. return overlay vertex id
func (g *Graph) GetOverlayVertex(u Index, exitEntryOrder Index, exit bool) (Index, bool) {
	subV := SubVertex{
		vId:            u,
		exitEntryOrder: exitEntryOrder,
		exit:           exit,
	}
	id, exists := g.overlayVertices[subV]
	return id, exists
}

func (g *Graph) GetCellNumber(u Index) Pv {
	return g.cellNumbers[g.vertices[u].pvPtr]
}

func (g *Graph) GetNumberOfCellsNumbers() int {
	return len(g.cellNumbers)
}

func (g *Graph) ForOutEdges(handle func(exitPoint, head, tail, entryPoint Index, percentage float64, eId Index)) {
	for eId, v := range g.heads {

		tail := g.GetTailOfOutedge(Index(eId))

		percentage := float64(eId) / float64(len(g.heads)) * 100
		handle(g.GetExitOrder(tail, Index(eId)), v, tail, Index(g.entryPoints[eId]), percentage, Index(eId))
	}
}

func (g *Graph) ForVertices(handle func(v Vertex, id Index)) {
	for i, v := range g.vertices[:g.NumberOfVertices()] { // skip dummy vertex di  g.vertices[:len(g.vertices)-1]
		handle(v, Index(i))
	}
}

func (g *Graph) SetVertexPvPtr(id Index, pvPtr Index) {
	g.vertices[id].SetPvPtr(pvPtr)
}

func (g *Graph) SetFirstOut(id Index, firstOut Index) {
	g.vertices[id].SetFirstOut(firstOut)
}

func (g *Graph) SetFirstIn(id Index, firstIn Index) {
	g.vertices[id].SetFirstIn(firstIn)
}

func (g *Graph) SetVId(id Index, vId Index) {
	g.vertices[id].SetId(vId)
}

func (g *Graph) GetVertices() []Vertex {
	return g.vertices[:g.NumberOfVertices()]
}

func (g *Graph) GetNumberOfOverlayVertexMapping() int {
	return len(g.overlayVertices)
}

func (g *Graph) GetVertexCoordinates(u Index) (float64, float64) {
	v := g.vertices[u]
	return v.GetLat(), v.GetLon()
}

func (g *Graph) GetMaxVerticesInCell() Index {
	return g.maxVerticesInCell
}

func (g *Graph) SetMaxVerticesInCell(maxVerticesInCell Index) {
	g.maxVerticesInCell = maxVerticesInCell
}

func (g *Graph) GetOutEdgeCellOffset(v Index) Index {
	return g.outEdgeCellOffset[g.vertices[v].pvPtr]
}

func (g *Graph) SetHeadCellOffset(i Index, outOffset Index) {
	g.outEdgeCellOffset[i] = outOffset
}

func (g *Graph) MakeOutEdgeCellOffset(cellNumbers int) {
	g.outEdgeCellOffset = make([]Index, cellNumbers)
}

func (g *Graph) MakeInEdgeCellOffset(cellNumbers int) {
	g.inEdgeCellOffset = make([]Index, cellNumbers)
}

func (g *Graph) GetInEdgeCellOffset(v Index) Index {
	return g.inEdgeCellOffset[g.vertices[v].pvPtr]
}
func (g *Graph) SetTailCellOffset(i Index, inOffset Index) {
	g.inEdgeCellOffset[i] = inOffset
}

func (g *Graph) GetVertex(u Index) Vertex {
	return g.vertices[u]
}

func (g *Graph) GetVertexCoordinate(u Index) Coordinate {
	return g.vertices[u].GetCoordinate()
}

func (g *Graph) GetVertexPvPtr(u Index) Index {
	return g.vertices[u].GetPvPtr()
}

func (g *Graph) GetVertexFirstOut(u Index) Index {
	return g.vertices[u].GetFirstOut()
}

func (g *Graph) GetVertexFirstIn(u Index) Index {
	return g.vertices[u].GetFirstIn()
}

func (g *Graph) SetHead(eId Index, v Index) {
	g.heads[eId] = v
}

func (g *Graph) SetTail(eId Index, v Index) {
	g.tails[eId] = v
}

func (g *Graph) GetVerticeIds() []Index {
	nodeIds := make([]Index, 0, g.NumberOfVertices())
	for i := 0; i < g.NumberOfVertices(); i++ {
		nodeIds = append(nodeIds, Index(i))
	}
	return nodeIds
}

// ---- road network data related ----

func (g *Graph) SetBoundingBox(bb *BoundingBox) {
	g.boundingBox = bb
}

func (g *Graph) GetBoundingBox() *BoundingBox {
	return g.boundingBox
}

// ---- nodes & edges permutation related ----

// ApplyGraphPermutation. apply vertices & edges permutation
// perm=permutation slice that maps new vertex id to old vertex id
// ePerm=permutation slice that maps new edge id to old edge id
func (g *Graph) ApplyGraphPermutation(nPerm, ePerm, eRevPerm []int) {
	g.vertices = util.ApplyPermutation(g.vertices, nPerm)
	g.entryPoints = util.ApplyPermutation(g.entryPoints, ePerm)
	g.exitPoints = util.ApplyPermutation(g.exitPoints, eRevPerm)
}

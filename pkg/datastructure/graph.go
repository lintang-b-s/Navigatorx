package datastructure

import (
	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// SubVertex maps from (vId, entryExitPoint, exitFlag) to overlay vertex.
// todo: ini bisa dihapus, langsung simpan entry/exit boundary/overlay vertices di Cell struct{}
type SubVertex struct {
	vId            Index // original vertex id
	exitEntryOrder uint8 // entry/exit point order (from 0 to outDegree-1/inDegree-1)
	exit           bool  // is exit point
}

// Pv is a type representing a cell number.
type Pv uint64

// Graph represents the main Customizable Route Planning (CRP) compact graph.
// uses adjacency arrays / Compressed Sparse Row (CSR). https://ocw.mit.edu/courses/6-172-performance-engineering-of-software-systems-fall-2018/f58df934e205b870e473c2af3fe5e6f7_MIT6_172F18_lec22.pdf
// CSR graph nya C++ Boost library: https://www.boost.org/doc/libs/1_61_0/libs/graph/doc/compressed_sparse_row.html
// kode ini terinspirasi dari implementasi CRP yang dibuat oleh Michael Wegner: https://github.com/michaelwegner/CRP/blob/master/datastructures/Graph.h
// untuk adjacency array  juga terinspirasi dari: https://github.com/RoutingKit/RoutingKit/blob/54d49bb0cdea56dde182357522e4e86a03c57852/include/routingkit/osm_graph_builder.h
type Graph struct {
	vertices    []Vertex // map from vertex v id to vertex v data structure
	heads       []Index  // map from outgoing edge id to head v dari edges (u,v) sorted by tail u
	tails       []Index  // map from incoming (reversed) edge id to tail v reversed edges (v,u). setiap in edge (v,u) punya bobot yang sama dengan out edge (u,v). sorted by head u.
	entryPoints []uint8  // map from outgoing edge id (u,v) to index of this edge in the list of incoming edges of v.
	exitPoints  []uint8  // map from incoming (reversed) edge id (v,u) to index of this edge in the of outgoing edges of v.

	// overlay graph related. adapted from https://github.com/michaelwegner/CRP/blob/master/datastructures/Graph.h
	overlayVertices  map[SubVertex]Index // graph vertices -> overlay vertices
	cellNumbers      []Pv                // cellNumbers contains all unique bitpacked cell numbers from level 0->L for each vertex.
	vertexCellOffset []Index             // offset of first vertex for each cellNumber
	// vertices is sorted by its vertex cell id.... in such a way that all vertices within a cell have sequential IDs within the vertices slice.
	// vertexCellOffset is a map from cellNumber to first vertex id in vertices that contained in this cell with id=cellNumber.
	// code for sorting vertices by its vertex cell id is in SortByCellNumber() in preprocessor.go

	// strongly connected components related
	sccs               []Index // verticeId -> sccId
	sccCondensationAdj [][]Index
	// sccCondensationAdj [][]Index // condensation graph connection of scc of u -> scc of v
	sccReach []*bitset.BitSet // sccId v -> bitset dari list dari other sccIds u yang dapat reach sccId v

	minResolution     float64
	maxVerticesInCell Index // maximum number of vertices in any level 1 cell
	roadNetwork       bool
}

func NewGraph(vertices []Vertex, heads []Index, tails []Index, roadNetwork bool, entryPoints []uint8, exitPoints []uint8) *Graph {
	return &Graph{vertices: vertices, heads: heads, tails: tails, maxVerticesInCell: 0, roadNetwork: roadNetwork,
		entryPoints: entryPoints, exitPoints: exitPoints}
}

// ---- graph data stucture related ----

// NumberOfVertices return number of graph vertices excluding last dummy vertex
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

// NumOutEdges returns the edge offset and number for outgoing edges of vertex u.
//
//go:inline
func (g *Graph) NumOutEdges(u Index) (Index, Index) {
	firstOut := g.vertices[u].firstOut
	num := g.vertices[u+1].firstOut - firstOut
	return firstOut, num
}

// NumInEdges returns the edge offset and number for incoming (reversed) edges of vertex v.
//
//go:inline
func (g *Graph) NumEdges(v Index, out bool) (Index, Index) {
	if out {
		return g.NumOutEdges(v)
	}
	firstIn := g.vertices[v].firstIn
	num := g.vertices[v+1].firstIn - firstIn
	return firstIn, num
}

//go:inline
func (g *Graph) GetHead(e Index) Index {
	return g.heads[e]
}

//go:inline
func (g *Graph) GetTail(e Index) Index {
	return g.tails[e]
}

func (g *Graph) GetCellNumbers() []Pv {
	return g.cellNumbers
}

func (g *Graph) GetHeadOfInedge(e Index) Index {
	tail := g.vertices[g.tails[e]]
	exitPoint := Index(g.exitPoints[e])
	return g.heads[tail.firstOut+exitPoint]
}

func (g *Graph) GetTailOfOutedge(e Index) Index {
	head := g.vertices[g.heads[e]]
	entryPoint := Index(g.entryPoints[e])
	return g.tails[head.firstIn+Index(entryPoint)]
}

// get incoming edgeId of outgoing edgeId e
func (g *Graph) GetRevId(e Index) Index {
	entryPoint := Index(g.entryPoints[e])
	return g.vertices[g.heads[e]].firstIn + entryPoint
}

// get outgoing edge Id of incoming (or reversed) edge Id e
//
//go:inline
func (g *Graph) GetOutId(e Index) Index {
	exitPoint := Index(g.exitPoints[e])
	return g.vertices[g.tails[e]].firstOut + exitPoint
}

// GetExitOrder. return Index of exit point of a out edge (u,v) at vertex u.
func (g *Graph) GetExitOrder(u, outEdgeId Index) uint8 {
	exitPoint := outEdgeId - g.vertices[u].firstOut
	return uint8(exitPoint)
}

// GetEntryOrder. return Index of entry point of a in edge (u,v) at vertex v.
func (g *Graph) GetEntryOrder(v, inEdgeId Index) uint8 {
	return uint8(inEdgeId - g.vertices[v].firstIn)
}

func (g *Graph) SetCellNumbers(cellNumbers []Pv) {
	g.cellNumbers = cellNumbers
}

func (g *Graph) SetOverlayMapping(overlayVertices map[SubVertex]Index) {
	g.overlayVertices = overlayVertices
}

// ForOutEdgesOf. iterates all outgoing edges of vertex u.
func (g *Graph) ForOutEdgesOf(u Index, handle func(eId, head Index, entryPoint uint8)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		entryPoint := g.entryPoints[e]
		handle(e, g.heads[e], entryPoint)
	}
}

// ForOutEdgesOf. iterates all incoming (reversed) edges of vertex u.
func (g *Graph) ForInEdgesOf(u Index, handle func(eId, tail Index, exitPoint uint8)) {
	for e := g.vertices[u].firstIn; e < g.vertices[u+1].firstIn; e++ {
		exitPoint := g.exitPoints[e]
		handle(e, g.tails[e], exitPoint)
	}
}

// ForOutEdgesOfWithTurn. iterate outgoing edges of vertex u from i-th incoming edge of u. return turn table id & turn type of its turn, highway type, head vertex, edge id, etc.
func (g *Graph) ForOutEdgesOfWithTurn(u Index, i uint8, handle func(eId, head Index, exitPoint, entryPoint uint8)) {
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

func (g *Graph) GetEdgesOf(u Index, out bool) []Index {
	if out {
		return g.heads[g.vertices[u].firstOut:g.vertices[u+1].firstOut]
	}
	return g.tails[g.vertices[u].firstIn:g.vertices[u+1].firstIn]
}

func (g *Graph) ForInEdgeIdsOf(v Index, handle func(id Index)) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {
		handle(e)
	}
}

// GetOverlayVertex. return overlay vertex id of vertex u that have cut (outgoing/incoming) edge eId.
func (g *Graph) GetOverlayVertex(u Index, eId Index, out bool) Index {
	exitEntryOrder := g.entryPoints[eId]
	exit := false
	if !out {
		exitEntryOrder = g.exitPoints[eId]
		exit = true
	}
	subV := SubVertex{
		vId:            u,
		exitEntryOrder: exitEntryOrder,
		exit:           exit,
	}
	id := g.overlayVertices[subV]
	return id
}

func (g *Graph) GetCellNumber(u Index) Pv {
	return g.cellNumbers[g.vertices[u].pvPtr]
}

func (g *Graph) GetNumberOfCellsNumbers() int {
	return len(g.cellNumbers)
}

func (g *Graph) ForOutEdges(handle func(exitPoint uint8, head, tail Index, entryPoint uint8, percentage float64, eId Index)) {
	for eId, v := range g.heads {
		tail := g.GetTailOfOutedge(Index(eId))
		percentage := float64(eId) / float64(len(g.heads)) * 100
		handle(g.GetExitOrder(tail, Index(eId)), v, tail, g.entryPoints[eId], percentage, Index(eId))
	}
}

func (g *Graph) ForVertices(handle func(v Vertex, id Index)) {
	for i, v := range g.vertices[:g.NumberOfVertices()] { // skip dummy vertex di  g.vertices[:len(g.vertices)-1]
		handle(v, Index(i))
	}
}

func (g *Graph) SetVertexPvPtr(id Index, pvPtr Index) {
	g.vertices[id].pvPtr = pvPtr
}

func (g *Graph) SetFirstOut(id Index, firstOut Index) {
	g.vertices[id].SetFirstOut(firstOut)
}

func (g *Graph) SetFirstIn(id Index, firstIn Index) {
	g.vertices[id].SetFirstIn(firstIn)
}

func (g *Graph) SetVId(id Index, vId Index) {
	g.vertices[id].id = vId
}

func (g *Graph) GetVertices() []Vertex {
	return g.vertices[:g.NumberOfVertices()]
}

func (g *Graph) GetNumberOfOverlayVertexMapping() int {
	return len(g.overlayVertices)
}

func (g *Graph) GetMaxVerticesInCell() Index {
	return g.maxVerticesInCell
}

func (g *Graph) SetMaxVerticesInCell(maxVerticesInCell Index) {
	g.maxVerticesInCell = maxVerticesInCell
}

func (g *Graph) GetVertex(u Index) Vertex {
	return g.vertices[u]
}

func (g *Graph) GetVertexCoordinate(u Index) Coordinate {
	return g.vertices[u].GetCoordinate()
}

func (g *Graph) GetVertexPvPtr(u Index) Index {
	return g.vertices[u].pvPtr
}

func (g *Graph) GetVertexFirstOut(u Index) Index {
	return g.vertices[u].firstOut
}

func (g *Graph) GetVertexFirstIn(u Index) Index {
	return g.vertices[u].firstIn
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

// BuildVertexCellOffsets constructs the vertex offset for each cell number.
func (g *Graph) BuildVertexCellOffsets() {
	numCells := len(g.cellNumbers)
	g.vertexCellOffset = make([]Index, numCells+1)

	currentCell := g.vertices[0].pvPtr
	g.vertexCellOffset[currentCell] = 0
	for v := 1; v < g.NumberOfVertices(); v++ {
		cell := g.vertices[v].pvPtr
		if cell != currentCell {
			for c := currentCell + 1; c <= cell; c++ {
				// v not inside currentCell
				// handle empty cells after currentCell
				// because graph.vertices sorted by its cellNumber, and in this outer loop we iterate vertices by increasing vertex ids
				// all values in vertexCellOffset is monotonically increasing..
				g.vertexCellOffset[c] = Index(v)
			}
			currentCell = cell
		}
	}

	for c := currentCell + 1; c <= Index(numCells); c++ {

		g.vertexCellOffset[c] = Index(g.NumberOfVertices())
	}
	var maxCellSize Index
	for c := 0; c < int(numCells); c++ {
		size := g.vertexCellOffset[c+1] - g.vertexCellOffset[c]
		if size > maxCellSize {
			maxCellSize = size
		}
	}

	g.maxVerticesInCell = maxCellSize
}

// GetVertexCellOffset returns the first vertex Id inside a cell that contain vertex u and cell size (number of vertices inside this cell) for the cell containing vertex u.
func (g *Graph) GetVertexCellOffset(u Index) (offset Index, num Index) {
	pvPtr := g.vertices[u].pvPtr
	offset = g.vertexCellOffset[pvPtr]
	num = g.vertexCellOffset[pvPtr+1] - offset
	return offset, num
}

// ---- nodes & edges permutation related ----

// ApplyGraphPermutation. apply vertices & edges permutation
// nPerm=permutation slice that maps new vertex id to old vertex id
// ePerm=permutation slice that maps new edge id to old edge id
// eRevPerm=permutation slice that maps new reversed edge id to old reversed edge id
func (g *Graph) ApplyGraphPermutation(nPerm, ePerm, eRevPerm []int) {
	g.vertices = util.ApplyPermutation(g.vertices, nPerm)
	g.entryPoints = util.ApplyPermutation(g.entryPoints, ePerm)
	g.exitPoints = util.ApplyPermutation(g.exitPoints, eRevPerm)
	g.BuildVertexCellOffsets()
}

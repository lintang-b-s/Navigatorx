package datastructure

import (
	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg"
)

type Index uint32

type Vertex struct {
	lat          float64
	lon          float64
	pvPtr        Index // pointer index to cellNumbers slice
	turnTablePtr Index // index of the first element of turnMatrices[v] in the flattened graph.turnTypeTable array
	// turnMatrices[v][i][j] -> turnMatrices = flattened 1-D indexed array index of i-th incoming edge and j-th outgoing edge  = i*outDegree + j
	firstOut Index // index of the first outEdge of this vertex in the flattened graph.outEdges array (see CSR Graph nya C++ Boost Library: https://www.boost.org/doc/libs/latest/libs/graph/doc/compressed_sparse_row.html)
	firstIn  Index // index of the first inEdge of this vertex in the flattened graph.inEdges array
	id       Index
}

func NewVertex(lat, lon float64, id Index) Vertex {
	return Vertex{
		lat: lat,
		lon: lon,
		id:  id,
	}
}

func NewEmptyVertex() Vertex {
	return Vertex{
		id: INVALID_VERTEX_ID,
	}
}

func (v *Vertex) SetFirstOut(firstOut Index) {
	v.firstOut = firstOut
}

func (v *Vertex) SetFirstIn(firstIn Index) {
	v.firstIn = firstIn
}

func (v *Vertex) SetId(id Index) {
	v.id = id
}
func (v *Vertex) SetPvPtr(pvPtr Index) {
	v.pvPtr = pvPtr
}

func (v *Vertex) SetTurnTablePtr(turnTablePtr Index) {
	v.turnTablePtr = turnTablePtr
}

func (v *Vertex) GetID() Index {
	return v.id
}

func (v *Vertex) GetLat() float64 {
	return v.lat
}

func (v *Vertex) GetLon() float64 {
	return v.lon
}

func (v *Vertex) GetCoordinate() Coordinate {
	return NewCoordinate(v.GetLat(), v.GetLon())
}

func (v *Vertex) GetFirstOut() Index {
	return v.firstOut
}

func (v *Vertex) GetFirstIn() Index {
	return v.firstIn
}

func (v *Vertex) GetPvPtr() Index {
	return v.pvPtr
}

func (v *Vertex) GetTurnTablePtr() Index {
	return v.turnTablePtr
}

// OutEdge represents an outgoing edge that enters vertex head at entryPoint.
type OutEdge struct {
	weight     float64 // minute
	dist       float64 // meter
	edgeId     Index   // edgeId = edgeId di graph.outEdges.
	head       Index
	entryPoint Index
	hwType     pkg.OsmHighwayType
	flag       uint8 // dummy edge (for phantom node) or parallel edge (for OSM turn restriction via-way)
}

// InEdge represents an incoming edge that exits vertex tail at exitPoint.
type InEdge struct {
	weight    float64 // minute
	dist      float64 // meter
	edgeId    Index   //edgeId = edgeId di graph.inEdges.
	tail      Index
	exitPoint Index
	hwType    pkg.OsmHighwayType
	flag      uint8 // dummy edge (for phantom node)  or parallel edge (for OSM turn restriction via-way)
}

func NewOutEdge(edgeId, head Index, weight, dist float64, entryPoint Index, hwType pkg.OsmHighwayType) OutEdge {
	return OutEdge{
		edgeId:     edgeId,
		head:       head,
		weight:     weight,
		dist:       dist,
		entryPoint: entryPoint,
		hwType:     hwType,
		flag:       0,
	}
}

func NewInEdge(edgeId, tail Index, weight, dist float64, exitPoint Index, hwType pkg.OsmHighwayType) InEdge {
	return InEdge{
		edgeId:    edgeId,
		tail:      tail,
		weight:    weight,
		dist:      dist,
		exitPoint: exitPoint,
		hwType:    hwType,
		flag:      0,
	}
}

func (e *OutEdge) SetFlag(flag uint8) {
	e.flag = flag
}

func (e *OutEdge) SetDummyEdge() {
	e.flag |= FlagDummy
}

func (e *OutEdge) SetJunctionHead() {
	e.flag |= FlagJunctionHead
}

func (e *OutEdge) SetJunctionTail() {
	e.flag |= FlagJunctionTail
}

func (e *OutEdge) SetContainsTrafficLight() {
	e.flag |= FlagContainsTrafficLight
}

func (e *OutEdge) IsJunctionHead() bool {
	return e.flag&FlagJunctionHead != 0
}

func (e *OutEdge) IsJunctionTail() bool {
	return e.flag&FlagJunctionTail != 0
}

func (e *OutEdge) ContainsTrafficLight() bool {
	return e.flag&FlagContainsTrafficLight != 0
}

func (e *InEdge) SetDummyEdge() {
	e.flag |= FlagDummy
}

func (e *OutEdge) SetParallelEdge() {
	e.flag |= FlagParallel
}

func (e *InEdge) SetParallelEdge() {
	e.flag |= FlagParallel
}

func (e *InEdge) SetJunctionHead() {
	e.flag |= FlagJunctionHead
}

func (e *InEdge) SetJunctionTail() {
	e.flag |= FlagJunctionTail
}

func (e *InEdge) SetContainsTrafficLight() {
	e.flag |= FlagContainsTrafficLight
}

func (e *InEdge) IsJunctionHead() bool {
	return e.flag&FlagJunctionHead != 0
}

func (e *InEdge) IsJunctionTail() bool {
	return e.flag&FlagJunctionTail != 0
}

func (e *InEdge) ContainsTrafficLight() bool {
	return e.flag&FlagContainsTrafficLight != 0
}

func (e *InEdge) SetFlag(flag uint8) {
	e.flag = flag
}

func (e *OutEdge) GetFlag() uint8 {
	return e.flag
}

func (e *InEdge) GetFlag() uint8 {
	return e.flag
}

func (e *OutEdge) GetWeight() float64 {
	return e.weight
}

// GetEdgeSpeed. get edge speed in meter/minute
func (e *OutEdge) GetEdgeSpeed() float64 {
	if e.weight == 0 {
		return 0
	}
	return e.dist / e.weight
}

func (e *OutEdge) SetWeight(travelTime float64) {
	e.weight = travelTime
}

func (e *OutEdge) SetEdgeId(edgeId Index) {
	e.edgeId = edgeId
}

func (e *OutEdge) SetHead(headId Index) {
	e.head = headId
}

func (e *OutEdge) GetLength() float64 {
	return e.dist
}

func (e *OutEdge) GetHead() Index {
	return e.head
}

func (e *OutEdge) GetEntryPoint() Index {
	return e.entryPoint
}

func (e *OutEdge) GetHighwayType() pkg.OsmHighwayType {
	return e.hwType
}

func (e *OutEdge) SetEntryPoint(p Index) {
	e.entryPoint = p
}

func (e *OutEdge) GetEdgeId() Index {
	return e.edgeId
}

func (e *InEdge) GetWeight() float64 {
	return e.weight
}

func (e *InEdge) GetHighwayType() pkg.OsmHighwayType {
	return e.hwType
}

func (e *InEdge) SetWeight(travelTime float64) {
	e.weight = travelTime
}

func (e *InEdge) GetEdgeSpeed() float64 {
	if e.weight == 0 {
		return 0
	}
	return e.dist / e.weight
}

func (e *InEdge) GetLength() float64 {
	return e.dist
}

func (e *InEdge) GetTail() Index {
	return e.tail
}

func (e *InEdge) GetExitPoint() Index {
	return e.exitPoint
}

func (e *InEdge) SetExitPoint(p Index) {
	e.exitPoint = p
}

func (e *InEdge) GetEdgeId() Index {
	return e.edgeId
}

func (e *InEdge) SetTailId(tailId Index) {
	e.tail = tailId
}

func (e *InEdge) SetEdgeId(edgeId Index) {
	e.edgeId = edgeId
}

// SubVertex   map dari (vId, entryExitPoint, exitFlag) to overlay vertex.
type SubVertex struct {
	originalID     Index // original vertex id
	exitEntryOrder Index // entry/exit point order (from 0 to outDegree-1/inDegree-1)
	exit           bool  // is exit point
}

// Pv is a type representing a cell number.
type Pv uint64

// Graph represents the main Customizable Route Planning (CRP) compact graph.
// uses adjacency arrays / Compressed Sparse Row (CSR) and a compact graph representation.
// See section 4.1 & 4.3: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
// CSR graph nya C++ Boost library: https://www.boost.org/doc/libs/latest/libs/graph/doc/compressed_sparse_row.html
// ini terinspirasi dari implementasi CRP yang dibuat oleh Michael Wegner: https://github.com/michaelwegner/CRP
type Graph struct {
	graphStorage      *GraphStorage
	vertices          []Vertex
	outEdges          []OutEdge
	inEdges           []InEdge            // reversed edges. setiap in edge (v,u) punya bobot yang sama dengan out edge (u,v)
	overlayVertices   map[SubVertex]Index // graph vertices -> overlay vertices
	verticesOsmIds    *PackedSlice
	cellNumbers       []Pv       // cellNumbers contains all unique bitpacked cell numbers from level 0->L for each vertex.
	outEdgeCellOffset []Index    // offset of first outEdge for each cellNumber
	inEdgeCellOffset  []Index    // offset of first inEdge for each cellNumber
	turnTypeTable     []pkg.Turn // [1-D indexed array index from 2D turnMatrices] over all vertices and flattened into graph.turnTypeTable. 1D-TurnMatrices[v][i][j] = i*outDegree + j

	// strongly connected components
	sccs               []Index   // verticeId -> sccId
	sccCondensationAdj [][]Index // condensation graph connection of scc of u -> scc of v

	boundingBox    *BoundingBox
	maxEdgesInCell Index // maximum number of inEdges/outEdges in any cell

	roadNetwork bool
}

func NewGraph(vertices []Vertex, outEdges []OutEdge, inEdges []InEdge, turnTypeTable []pkg.Turn, roadNetwork bool, verticesOsmIds *PackedSlice) *Graph {
	return &Graph{vertices: vertices, outEdges: outEdges, inEdges: inEdges, turnTypeTable: turnTypeTable, maxEdgesInCell: 0, roadNetwork: roadNetwork,
		verticesOsmIds: verticesOsmIds}
}

func (g *Graph) GetGraphStorage() *GraphStorage {
	return g.graphStorage
}

func (g *Graph) NumberOfVertices() int {
	return len(g.vertices) - 1
}

func (g *Graph) NumberOfEdges() int {
	return len(g.outEdges)
}

func (g *Graph) NumberOfOutEdges() int {
	return len(g.outEdges)
}

func (g *Graph) NumberOfInEdges() int {
	return len(g.inEdges)
}
func (g *Graph) IsRoadNetworkGraph() bool {
	return g.roadNetwork
}

func (g *Graph) GetEdgeSpeeds() []float64 {
	m := Index(g.NumberOfEdges())
	eSpeeds := make([]float64, m)
	for eId := Index(0); eId < m; eId++ {
		eSpeeds[eId] = g.outEdges[eId].GetLength() / g.outEdges[eId].GetWeight() // in m/s

	}

	return eSpeeds
}

func (g *Graph) GetTurnTypes() []pkg.Turn {
	ttp := make([]pkg.Turn, len(g.turnTypeTable))
	copy(ttp, g.turnTypeTable)
	return ttp
}

func (g *Graph) GetVertexOsmId(vId Index) uint64 {
	return g.verticesOsmIds.Get(uint64(vId))
}

func (g *Graph) GetVertexOsmIds() []uint64 {
	osmNodeIds := make([]uint64, len(g.vertices))
	for v := 0; v < g.NumberOfVertices(); v++ {
		osmNodeIds[v] = g.verticesOsmIds.Get(uint64(v))
	}
	return osmNodeIds
}

func (g *Graph) SetVertexOsmIds(verticesOsmIds *PackedSlice) {
	g.verticesOsmIds = verticesOsmIds
}

func (g *Graph) SetNewTurnTypeTable(turnTypeTable []pkg.Turn) {
	g.turnTypeTable = turnTypeTable
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

func (g *Graph) GetOutEdge(e Index) *OutEdge {
	return &g.outEdges[e]
}

func (g *Graph) GetHeadOfOutEdge(e Index) Index {
	return g.outEdges[e].GetHead()
}

func (g *Graph) GetTailOfInedge(e Index) Index {
	return g.inEdges[e].tail
}

func (g *Graph) GetInEdge(e Index) *InEdge {
	return &g.inEdges[e]
}

func (g *Graph) GetOutEdgeLength(e Index) float64 {
	return g.outEdges[e].GetLength()
}

func (g *Graph) GetInEdgeLength(e Index) float64 {
	return g.inEdges[e].GetLength()
}

func (g *Graph) GetOutEdgeWeight(e Index) float64 {
	return g.outEdges[e].GetWeight()
}

func (g *Graph) GetOutEdgeHighwayType(e Index) pkg.OsmHighwayType {
	return g.outEdges[e].hwType
}

// GetOutEdgeTripleWeightKey. return default weight, length, and speed of outEdge e
func (g *Graph) GetOutEdgeTripleWeightKey(e Index) (float64, float64, float64) {
	speed := g.outEdges[e].GetWeight() / g.outEdges[e].GetLength()

	return g.outEdges[e].GetWeight(), g.outEdges[e].GetLength(), speed
}

// GetInEdgeTripleWeightKey. return default weight, length, and speed of inEdge e
func (g *Graph) GetInEdgeTripleWeightKey(e Index) (float64, float64, float64) {
	speed := g.inEdges[e].GetWeight() / g.inEdges[e].GetLength()
	return g.inEdges[e].GetWeight(), g.inEdges[e].GetLength(), speed
}

func (g *Graph) GetCellNumbers() []Pv {
	return g.cellNumbers
}

func (g *Graph) GetHeadOfInedge(e Index) Index {
	inEdge := g.inEdges[e]
	tail := g.vertices[inEdge.tail]
	return g.outEdges[tail.firstOut+Index(inEdge.exitPoint)].head
}

func (g *Graph) GetHeadOfInedgeWithOutEdge(e Index) (Index, Index) {
	inEdge := g.inEdges[e]
	tail := g.vertices[inEdge.tail]
	return g.outEdges[tail.firstOut+Index(inEdge.exitPoint)].head, tail.firstOut + Index(inEdge.exitPoint)
}

func (g *Graph) GetTailOfOutedge(e Index) Index {
	outEdge := g.outEdges[e]
	head := g.vertices[outEdge.head]
	return g.inEdges[head.firstIn+Index(outEdge.entryPoint)].tail
}

func (g *Graph) GetTailOfOutedgeWithInEdge(e Index) (Index, Index) {
	outEdge := g.outEdges[e]
	head := g.vertices[outEdge.head]
	return g.inEdges[head.firstIn+Index(outEdge.entryPoint)].tail, head.firstIn + Index(outEdge.entryPoint)
}

// get inEdgeId of outEdgeId e
func (g *Graph) GetEntryIdOfOutEdge(e Index) Index {
	head := g.vertices[g.outEdges[e].head]
	return head.firstIn + Index(g.outEdges[e].entryPoint)
}

// get outEdgeId of inEdgeId e
func (g *Graph) GetExitIdOfInEdge(e Index) Index {
	tail := g.vertices[g.inEdges[e].tail]
	return tail.firstOut + Index(g.inEdges[e].exitPoint)
}

func (g *Graph) GetEntryPointOfOutEdge(e Index) Index {
	return Index(g.outEdges[e].entryPoint)
}

func (g *Graph) GetExitPointOfInEdge(e Index) Index {
	return Index(g.inEdges[e].exitPoint)
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

// GetTurnType get turn type dari entryPoint->u->exitPoint
func (g *Graph) GetTurnType(u Index, entryPoint, exitPoint Index) pkg.TurnType {
	turnTableId := g.vertices[u].turnTablePtr + entryPoint*g.GetOutDegree(u) + exitPoint
	return g.turnTypeTable[turnTableId].GetTurnType()
}

// GetTurnType get turntableId dari entryPoint->u->exitPoint
func (g *Graph) GetTurnTableId(u Index, entryPoint, exitPoint Index) Index {
	turnTableId := g.vertices[u].turnTablePtr + entryPoint*g.GetOutDegree(u) + exitPoint
	return turnTableId
}

func (g *Graph) SetCellNumbers(cellNumbers []Pv) {
	g.cellNumbers = cellNumbers
}

func (g *Graph) SetOverlayMapping(overlayVertices map[SubVertex]Index) {
	g.overlayVertices = overlayVertices
}

// langsung return OutEge copy structnya jadi lebih gede allocation  B/op pas di benchmark
func (g *Graph) ForOutEdgesOf(u Index, entryPoint Index, handle func(eId, head Index, weight, length float64, exitPoint, entryPoint, turnTableId Index, turnType pkg.TurnType,
	hwType pkg.OsmHighwayType)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {

		handle(e, g.outEdges[e].head, g.outEdges[e].GetWeight(), g.outEdges[e].GetLength(), g.GetExitOrder(u, e), g.outEdges[e].GetEntryPoint(),
			g.GetTurnTableId(u, entryPoint, g.GetExitOrder(u, e)), g.GetTurnType(u, entryPoint, g.GetExitOrder(u, e)), g.outEdges[e].hwType)
	}
}

func (g *Graph) ForInEdgesOf(v Index, exitPoint Index, handle func(eId, tail Index, weight, length float64, exitPoint, entryPoint, turnTableId Index,
	turnType pkg.TurnType, hwType pkg.OsmHighwayType)) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {

		handle(e, g.inEdges[e].tail, g.inEdges[e].GetWeight(), g.inEdges[e].GetLength(), g.inEdges[e].GetExitPoint(), g.GetEntryOrder(v, e),
			g.GetTurnTableId(v, g.GetEntryOrder(v, e), exitPoint), g.GetTurnType(v, g.GetEntryOrder(v, e), exitPoint), g.inEdges[e].hwType)
	}
}

// GetDummyOutEdgeId. return dummy outEdge (u,u)
func (g *Graph) GetDummyOutEdgeId(u Index) Index {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		if g.outEdges[e].flag&FlagDummy != 0 {
			return e
		}
	}
	return g.vertices[u].firstOut

}

// GetDummyOutEdgeId. return dummy inEdge (u,u)
func (g *Graph) GetDummyInEdgeId(u Index) Index {
	for e := g.vertices[u].firstIn; e < g.vertices[u+1].firstIn; e++ {
		if g.inEdges[e].flag&FlagDummy != 0 {
			return e
		}
	}
	return g.vertices[u].firstIn
}

func (g *Graph) GetNumberOfOutEdges(u Index) Index {
	return g.vertices[u+1].firstOut - g.vertices[u].firstOut
}

func (g *Graph) ForOutEdgeIdsOf(u Index, handle func(eId Index)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		if g.IsDummyOutEdge(e) || g.IsParallelOutEdge(e) {
			continue
		}

		handle(e)
	}
}

func (g *Graph) IsDummyOutEdge(eId Index) bool {
	return g.outEdges[eId].flag&FlagDummy != 0
}

func (g *Graph) IsDummyInEdge(eId Index) bool {
	return g.inEdges[eId].flag&FlagDummy != 0
}

func (g *Graph) IsParallelOutEdge(eId Index) bool {
	return g.outEdges[eId].flag&FlagParallel != 0
}

func (g *Graph) IsParallelInlEdge(eId Index) bool {
	return g.inEdges[eId].flag&FlagParallel != 0
}

func (g *Graph) IsJunctionHead(eId Index) bool {
	return g.outEdges[eId].IsJunctionHead()
}

func (g *Graph) IsJunctionTail(eId Index) bool {
	return g.outEdges[eId].IsJunctionTail()
}

func (g *Graph) ForInEdgeIdsOf(v Index, handle func(id Index)) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {
		if g.IsDummyInEdge(e) || g.IsParallelInlEdge(e) {
			continue
		}
		handle(e)
	}
}

func (g *Graph) GetHeadFromInEdge(entryId Index) Index {
	InEdge := g.GetInEdge(entryId)
	tailAtInEdge := g.GetVertex(InEdge.GetTail())
	head := g.outEdges[tailAtInEdge.GetFirstOut()+Index(InEdge.GetExitPoint())].GetHead()
	return head
}

func (g *Graph) GetTailFromOutEdge(exitPoint Index) Index {
	outEdge := &g.outEdges[exitPoint]
	headAtOutEdge := g.GetVertex(outEdge.GetHead())
	return g.inEdges[headAtOutEdge.GetFirstIn()+Index(outEdge.GetEntryPoint())].GetTail()
}

// GetOverlayVertex. return overlay vertex id
func (g *Graph) GetOverlayVertex(u Index, exitEntryOrder Index, exit bool) (Index, bool) {
	subV := SubVertex{
		originalID:     u,
		exitEntryOrder: exitEntryOrder,
		exit:           exit,
	}
	id, exists := g.overlayVertices[subV]
	return id, exists
}

func (g *Graph) GetTurnTypetable() []pkg.Turn {
	return g.turnTypeTable
}

func (g *Graph) GetTurnTypeTableLength() int {
	return len(g.turnTypeTable)
}

func (g *Graph) GetCellNumber(u Index) Pv {
	return g.cellNumbers[g.vertices[u].pvPtr]
}

func (g *Graph) GetNumberOfCellsNumbers() int {
	return len(g.cellNumbers)
}

func (g *Graph) ForOutEdges(handle func(exitPoint, head Index, tail, entryId, entryPoint Index, percentage float64, idx Index)) {
	for idx, e := range g.outEdges {
		if g.IsDummyOutEdge(Index(idx)) { // jangan skip parallel edges disini, karena masih kepake di map matching
			continue
		}

		tail := g.GetTailOfOutedge(Index(idx))

		percentage := float64(idx) / float64(len(g.outEdges)) * 100

		entryId := g.vertices[e.head].GetFirstIn() + Index(e.GetEntryPoint())

		handle(g.GetExitOrder(tail, Index(idx)), e.head, tail, entryId, Index(e.GetEntryPoint()), percentage, Index(idx))
	}
}

func (g *Graph) ForInEdges(handle func(e InEdge, entryPoint, head Index, tail, exitId Index, percentage float64, idx Index)) {
	for idx, e := range g.inEdges {
		percentage := float64(idx) / float64(len(g.inEdges)) * 100
		head := g.GetHeadOfInedge(Index(idx))

		exitId := g.vertices[e.tail].GetFirstOut() + Index(e.GetExitPoint())

		handle(e, g.GetEntryOrder(e.tail, Index(idx)), head, e.tail, exitId, percentage, Index(idx))
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

func (g *Graph) GetVertices() []Vertex {
	return g.vertices[:g.NumberOfVertices()]
}

func (g *Graph) GetNumberOfOverlayVertexMapping() int {
	return len(g.overlayVertices)
}

func (g *Graph) GetVertexCoordinates(u Index) (float64, float64) {
	v := g.vertices[u]
	return v.lat, v.lon
}

func (g *Graph) SetVertices(vs []Vertex) {
	g.vertices = vs
}

func (g *Graph) GetMaxEdgesInCell() Index {
	return g.maxEdgesInCell
}

func (g *Graph) SetMaxEdgesInCell(maxEdgesInCell Index) {
	g.maxEdgesInCell = maxEdgesInCell
}

func (g *Graph) GetOutEdgeCellOffset(v Index) Index {
	return g.outEdgeCellOffset[g.vertices[v].pvPtr]
}

func (g *Graph) SetOutEdgeCellOffset(i Index, outOffset Index) {
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
func (g *Graph) SetInEdgeCellOffset(i Index, inOffset Index) {
	g.inEdgeCellOffset[i] = inOffset
}

func (g *Graph) GetOutEdgeCellOffsets() []Index {
	return g.outEdgeCellOffset
}

func (g *Graph) GetInEdgeCellOffsets() []Index {
	return g.inEdgeCellOffset
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

func (g *Graph) SetOutEdge(id Index, e OutEdge) {
	g.outEdges[id] = e
}

func (g *Graph) GetNumberOfVerticesWithDummyVertex() int {
	return len(g.vertices)
}

func (g *Graph) SetInEdge(id Index, e InEdge) {
	g.inEdges[id] = e
}

func (g *Graph) SetSCCs(sccs []Index) {
	g.sccs = sccs
}

func (g *Graph) SetSCCCondensationAdj(adj [][]Index) {
	g.sccCondensationAdj = adj
}

func (g *Graph) GetSCCOfAVertex(u Index) Index {
	return g.sccs[u]
}

func (g *Graph) GetSCCS() []Index {
	return g.sccs
}
func (g *Graph) GetSCCCondensationAdjList() [][]Index {
	return g.sccCondensationAdj
}

func (g *Graph) SetBoundingBox(bb *BoundingBox) {
	g.boundingBox = bb
}

func (g *Graph) GetBoundingBox() *BoundingBox {
	return g.boundingBox
}

// PathExistsFromUToVUsingCondensationGraph. cek apakah ada path (tanpa costs) dari u ke v
// worst case O(V_G + E_G), V_G=number of sccs of the graph/number of vertices in condensation graph, E_G=number of edges in condensation graph
func (g *Graph) PathExistsFromUToVUsingCondensationGraph(u, v Index) bool {
	sccOfU := g.sccs[u]
	sccOfV := g.sccs[v]

	uvPathExists := false
	discovered := make([]bool, len(g.sccCondensationAdj))
	g.dfsCondensationGraph(sccOfU, sccOfV, discovered, &uvPathExists)
	return uvPathExists
}

// dfsCondensationGraph. dfs di condesation graph
// O(V_G + E_G), V_G=number of sccs in graph/number of vertices in condensation graph, E_G=number of edges in condensation graph
func (g *Graph) dfsCondensationGraph(u Index, t Index, discovered []bool, uvPathExists *bool) {
	if u == t {
		*uvPathExists = true
		return // gak perlu discover out neighbor dari t. discover u = discover vertex u sebelum adjacency listnya examined
	}

	if discovered[u] {
		return
	}
	discovered[u] = true

	for _, v := range g.sccCondensationAdj[u] {
		if *uvPathExists {
			// kita bisa return early karena uvPathExists=true
			// gak perlu examine other out neighbor dari u
			return
		}
		g.dfsCondensationGraph(v, t, discovered, uvPathExists)
	}
}

func (g *Graph) GetRoundaboutFlag() *bitset.BitSet {
	return g.graphStorage.roundaboutFlag
}

func (g *Graph) GetIsCurvedFlag() *bitset.BitSet {
	return g.graphStorage.isCurvedFlag
}

func (g *Graph) SetRoundabout(edgeID Index, isRoundabout bool) {
	g.graphStorage.SetRoundabout(edgeID, isRoundabout)
}

func (g *Graph) IsCurved(edgeId Index) bool {
	return g.graphStorage.IsCurved(edgeId)
}

func (g *Graph) IsTrafficLight(vertexId Index) bool {
	return g.graphStorage.GetTrafficLight(vertexId)
}

// PathExists. cek apakah ada path (tanpa costs) dari u ke v .
// kalau u dan v terdapat dalam scc yang sama, then its strongly connected atau ada path dari u ke v dan sebaliknya
// kita sudah precompute condensation graph yang merupakan directed acyclic graph (DAG) dengan vertices nya adalah sccs dari graph
// dan terdapat edge dari scc c1 ke scc c2 jika pada graph terdapat simpul in c1 yang memiliki edge dengan head in c2.
// pas kita dfs di condensation graph dari c1, jika kita bisa reach/discover c2 maka terdapat path dari u ke v,
// hal ini karena all vertices in c2 strongly connected.
// O(V_G + E_G), V_G=number of sccs of the graph/number of vertices in condensation graph scc, E_G=number of edges in condensation graph
func (g *Graph) PathExists(u, v Index) bool {
	sccOfU := g.GetSCCOfAVertex(u)
	sccOfV := g.GetSCCOfAVertex(v)
	if sccOfU == sccOfV {
		return true
	}

	return g.PathExistsFromUToVUsingCondensationGraph(u, v)
}

func (g *Graph) GetNodeTrafficLight() *bitset.BitSet {
	return g.graphStorage.nodeTrafficLight
}

func (g *Graph) SetNodeTrafficLight(vId Index, yes bool) {
	g.graphStorage.SetTrafficLight(vId, yes)
}

func (g *Graph) SetGraphStorage(gs *GraphStorage) {
	g.graphStorage = gs
}

func (g *Graph) IsRoundabout(edgeId Index) bool {
	return g.graphStorage.IsRoundabout(edgeId)
}

func (g *Graph) GetStreetName(edgeId Index) string {
	stNameId := g.graphStorage.streetName[edgeId]

	return g.graphStorage.GetStr(stNameId)
}

func (g *Graph) GetStreetNameId(edgeId Index) uint32 {
	stNameId := g.graphStorage.streetName[edgeId]

	return stNameId
}

func (g *Graph) GetStrFromId(stNameId uint32) string {
	if stNameId == INVALID_STREET_NAME_ID {
		return ""
	}

	return g.graphStorage.GetStr(stNameId)
}

func (g *Graph) GetRoadClass(edgeId Index) string {
	roadClassId := g.graphStorage.roadClass[edgeId]
	return pkg.GetHighwayTypeString(roadClassId)
}

func (g *Graph) GetRoadClassLink(edgeId Index) string {
	roadClassLinkId := g.graphStorage.roadClassLink[edgeId]
	return pkg.GetHighwayTypeString(roadClassLinkId)
}

func (g *Graph) GetRoadLanes(edgeId Index) uint8 {
	return g.graphStorage.lanes[edgeId]
}

func (g *Graph) GetStreetDirection(edgeId Index) [2]bool {
	return g.graphStorage.GetStreetDirection(edgeId)
}

func (g *Graph) IsStreetBidirectional(edgeId Index) bool {
	edgeWayDirection := g.graphStorage.GetStreetDirection(edgeId) //
	return edgeWayDirection[0] && edgeWayDirection[1]
}

func (g *Graph) GetOsmWayId(edgeId Index) int64 {
	return int64(g.graphStorage.edgeOsmWayId.Get(uint64(edgeId)))
}

func (g *Graph) GetEdgeGeometry(edgeID Index) []Coordinate {
	return g.graphStorage.GetEdgeGeometry(edgeID)
}

func (g *Graph) AppendPathWithEdgeGeometry(path *Coordinates, edgeID Index) {
	g.graphStorage.AppendPathWithEdgeGeometry(path, edgeID)
}

func (g *Graph) GetEdgeGeometryLength(edgeID Index) int {
	return g.graphStorage.GetEdgeGeometryLength(edgeID)
}

func (g *Graph) GetOsmWayBitSize() uint8 {
	return g.graphStorage.osmwayBitSize
}

func (g *Graph) SetRoundaboutFlags(roundaboutFlags *bitset.BitSet) {
	g.graphStorage.roundaboutFlag = roundaboutFlags
}

func (g *Graph) SetIsCurvedFlags(isCurvedFlag *bitset.BitSet) {
	g.graphStorage.isCurvedFlag = isCurvedFlag
}

func (g *Graph) SetTrafficLightFlags(trafficLightFlags *bitset.BitSet) {
	g.graphStorage.nodeTrafficLight = trafficLightFlags
}

func (g *Graph) SetNewEdgeMetadatas(osmWayIds *PackedSlice,
	edgeStartPointsIndex, // edge geometry start index di gs.osmNodePoints
	edgeEndPointsIndex []Index,
	streetName []uint32,
	roadClass, roadClassLink []pkg.OsmHighwayType,
	lanes []uint8) {
	g.graphStorage.SetNewEdgeMetadata(osmWayIds, edgeStartPointsIndex, edgeEndPointsIndex,
		streetName, roadClass, roadClassLink, lanes)
}

func (g *Graph) GetEdgePointsIndices(edgeId Index) (Index, Index) {
	return g.graphStorage.edgeStartPointsIndex[edgeId], g.graphStorage.edgeEndPointsIndex[edgeId]
}

func (g *Graph) SetStreetDirection(streetDirectionForward, streetDirectionBackward *bitset.BitSet) {
	g.graphStorage.SetStreetDirection(streetDirectionForward, streetDirectionBackward)
}

func (g *Graph) ForOutEdgesOfVertex(u Index, handle func(head, exitPoint Index, weight float64)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		if g.IsDummyOutEdge(e) || g.IsParallelOutEdge(e) {
			continue
		}
		handle(g.outEdges[e].head, g.GetExitOrder(u, e), g.outEdges[e].weight)
	}
}

func (g *Graph) GetVerticeIds() []Index {
	nodeIds := make([]Index, 0, g.NumberOfVertices())
	for i := 0; i < g.NumberOfVertices(); i++ {
		nodeIds = append(nodeIds, Index(i))
	}
	return nodeIds
}

func NewEmptyOutEdge() OutEdge {
	return OutEdge{
		edgeId:     INVALID_EDGE_ID,
		head:       0,
		weight:     0,
		dist:       0,
		entryPoint: 0,
		hwType:     0,
	}
}

func NewEmptyInEdge() InEdge {
	return InEdge{
		edgeId:    INVALID_EDGE_ID,
		tail:      0,
		weight:    0,
		dist:      0,
		exitPoint: 0,
		hwType:    0,
	}
}

func (g *Graph) ForEachConditionalBarrierNode(handle func(id Index, res ConditionalBarrierNode)) {
	for idx, res := range g.graphStorage.GetConditionalBarrierNodes() {
		handle(Index(idx), res)
	}
}

func (g *Graph) ForEachConditionalReversibleEdge(handle func(id Index, res ConditionalReversibleEdge)) {
	for idx, res := range g.graphStorage.GetConditionalReversibleEdges() {
		handle(Index(idx), res)
	}
}

func (g *Graph) ForEachConditionalSpeedLimit(handle func(id Index, res ConditionalSpeedLimit)) {
	for idx, res := range g.graphStorage.GetConditionalSpeedLimits() {
		handle(Index(idx), res)
	}
}

func (g *Graph) ForEachConditionalTrafficMode(handle func(id Index, res ConditionalTrafficMode)) {
	for idx, res := range g.graphStorage.GetConditionalTrafficModes() {
		handle(Index(idx), res)
	}
}

func (g *Graph) ForEachConditionalTurnRestriction(handle func(id Index, res ConditionalTurnRestriction)) {
	for idx, res := range g.graphStorage.GetConditionalTurnRestrictions() {
		handle(Index(idx), res)
	}
}

func (g *Graph) SetEdgeGeohashes(edgeGeohashes []uint64) {
	g.graphStorage.SetEdgeGeohashes(edgeGeohashes)
}

// GetEdgeGeohash get  geohash (precision 6) dari edge edgeId
func (g *Graph) GetEdgeGeohash(edgeId Index) uint64 {
	return g.graphStorage.GetEdgeGeohash(edgeId)
}

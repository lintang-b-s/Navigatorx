package datastructure

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
)

type Index uint32

type Vertex struct {
	lat          float64
	lon          float64
	pvPtr        Index // pointer index to cellNumbers slice
	turnTablePtr Index // index of the first element of turnMatrices[v] in the flattened graph.turnTables array
	// turnMatrices[v][i][j] = 1-D indexed array index of i-th incoming edge and j-th outgoing edge  = i*outDegree + j
	firstOut Index // index of the first outEdge of this vertex in the flattened graph.outEdges array
	firstIn  Index // index of the first inEdge of this vertex in the flattened graph.inEdges array
	id       Index
}

func NewVertex(lat, lon float64, id Index) *Vertex {
	return &Vertex{
		lat: lat,
		lon: lon,
		id:  id,
	}
}

func NewVertexComplete(lat, lon float64, id, pvPtr, turnTablePtr, firstOut, firstIn Index) *Vertex {
	return &Vertex{
		lat: lat,
		lon: lon,
		id:  id,
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

// outedge enters vertex head at entryPoint
type OutEdge struct {
	weight            float64 // minute
	dist              float64 // meter
	edgeId, oriEdgeId Index
	head              Index
	entryPoint        int
}

// inedge exits vertex tail at exitPoint
type InEdge struct {
	weight            float64 // minute
	dist              float64 // meter
	edgeId, oriEdgeId Index
	tail              Index
	exitPoint         int
}

func NewOutEdge(edgeId, head Index, weight, dist float64, entryPoint int) *OutEdge {
	return &OutEdge{
		edgeId:     edgeId,
		head:       head,
		weight:     weight,
		dist:       dist,
		entryPoint: entryPoint,
	}
}

func NewInEdge(edgeId, tail Index, weight, dist float64, exitPoint int) *InEdge {
	return &InEdge{
		edgeId:    edgeId,
		tail:      tail,
		weight:    weight,
		dist:      dist,
		exitPoint: exitPoint,
	}
}

func (e *OutEdge) GetWeight() float64 {
	return e.weight
}

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

func (e *OutEdge) GetEntryPoint() int {
	return e.entryPoint
}

func (e *OutEdge) SetEntryPoint(p int) {
	e.entryPoint = p
}

func (e *OutEdge) SetOriginalEdgeId(oriEdgeId Index) {
	e.oriEdgeId = oriEdgeId
}

func (e *OutEdge) GetOriginalEdgeId() Index {
	return e.oriEdgeId
}

func (e *OutEdge) GetEdgeId() Index {
	return e.edgeId
}

func (e *InEdge) GetWeight() float64 {
	return e.weight
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

func (e *InEdge) GetExitPoint() int {
	return e.exitPoint
}

func (e *InEdge) SetExitPoint(p int) {
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

func (e *InEdge) SetOriginalEdgeId(oriEdgeId Index) {
	e.oriEdgeId = oriEdgeId
}

func (e *InEdge) GetOriginalEdgeId() Index {
	return e.oriEdgeId
}

type SubVertex struct {
	originalID     Index // original vertex id
	exitEntryOrder int   // entry/exit point order (from 0 to outDegree-1/inDegree-1)
	exit           bool  // is exit point
}

type VertexIDPair struct {
	originalVertexID Index // original vertex id
	id               Index
}

type Pv uint64

type ViaKey struct {
	level      int
	sourceCell Pv
	targetCell Pv
}

func NewViaKey(level int, sourceCell, targetCell Pv) ViaKey {
	return ViaKey{level, sourceCell, targetCell}
}

// main crp graph. static (i.e. can't add new edges)
type Graph struct {
	graphStorage      *GraphStorage
	vertices          []*Vertex
	outEdges          []*OutEdge
	inEdges           []*InEdge
	turnTables        []pkg.TurnType      // [1-D indexed array index from 2D turnMatrices] over all vertices and flattened into graph.turnTables. 1D-TurnMatrices[v][i][j] = i*outDegree + j
	cellNumbers       []Pv                // cellNumbers contains all unique bitpacked cell numbers from level 0->L for each vertex.
	maxEdgesInCell    Index               // maximum number of inEdges/outEdges in any cell
	outEdgeCellOffset []Index             // offset of first outEdge for each cellNumber
	inEdgeCellOffset  []Index             // offset of first inEdge for each cellNumber
	overlayVertices   map[SubVertex]Index // graph vertices -> overlay vertices

	// strongly connected components
	sccs               []Index   // verticeId -> sccId
	sccCondensationAdj [][]Index // condensation connection of scc of u -> scc of v

	boundingBox *BoundingBox
}

func NewGraph(vertices []*Vertex, forwardEdges []*OutEdge, inEdges []*InEdge, turnTables []pkg.TurnType) *Graph {
	return &Graph{vertices: vertices, outEdges: forwardEdges, inEdges: inEdges, turnTables: turnTables, maxEdgesInCell: 0}
}

func (g *Graph) NumberOfVertices() int {
	return len(g.vertices) - 1
}

func (g *Graph) NumberOfEdges() int {
	return len(g.outEdges)
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
	return g.outEdges[e]
}

func (g *Graph) GetInEdge(e Index) *InEdge {
	return g.inEdges[e]
}

func (g *Graph) GetCellNumbers() []Pv {
	return g.cellNumbers
}

func (g *Graph) FindInEdge(u, v Index) (Index, bool) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {
		if g.inEdges[e].tail == u {
			return e, true
		}
	}
	return 0, false
}

func (g *Graph) GetHeadOfInedge(e Index) Index {
	inEdge := g.GetInEdge(e)
	tail := g.vertices[inEdge.tail]
	outEdge := g.GetOutEdge(tail.firstOut + Index(inEdge.exitPoint))
	return outEdge.head
}

func (g *Graph) GetHeadOfInedgeWithOutEdge(e Index) (Index, *OutEdge) {
	inEdge := g.GetInEdge(e)
	tail := g.vertices[inEdge.tail]
	outEdge := g.GetOutEdge(tail.firstOut + Index(inEdge.exitPoint))

	return outEdge.head, outEdge
}

func (g *Graph) GetTailOfOutedge(e Index) Index {
	outEdge := g.GetOutEdge(e)
	head := g.vertices[outEdge.head]
	inEdge := g.GetInEdge(head.firstIn + Index(outEdge.entryPoint))
	return inEdge.tail
}

func (g *Graph) GetTailOfOutedgeWithInEdge(e Index) (Index, *InEdge) {
	outEdge := g.GetOutEdge(e)
	head := g.vertices[outEdge.head]
	inEdge := g.GetInEdge(head.firstIn + Index(outEdge.entryPoint))
	return inEdge.tail, inEdge
}

// GetExitOrder. return Index of exit point of a out edge (u,v) at vertex u.
func (g *Graph) GetExitOrder(u, outEdge Index) Index {
	exitPoint := outEdge - g.vertices[u].firstOut
	return exitPoint
}

// GetEntryOrder. return Index of entry point of a in edge (u,v) at vertex v.
func (g *Graph) GetEntryOrder(v, InEdge Index) Index {
	return InEdge - g.vertices[v].firstIn
}

func (g *Graph) GetTurnType(u Index, entryPoint, exitPoint Index) pkg.TurnType {
	turnTableOffset := g.vertices[u].turnTablePtr + Index(entryPoint)*Index(g.GetOutDegree(u)) + Index(exitPoint)
	return g.turnTables[turnTableOffset]
}

func (g *Graph) SetCellNumbers(cellNumbers []Pv) {
	g.cellNumbers = cellNumbers
}

func (g *Graph) SetOverlayMapping(overlayVertices map[SubVertex]Index) {
	g.overlayVertices = overlayVertices
}

func (g *Graph) ForOutEdgesOf(u Index, entryPoint Index, handle func(e *OutEdge, exitPoint Index, turnType pkg.TurnType)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {

		handle(g.outEdges[e], g.GetExitOrder(u, e), g.GetTurnType(u, entryPoint, g.GetExitOrder(u, e)))
	}
}

func (g *Graph) GetNumberOfOutEdges(u Index) Index {
	return g.vertices[u+1].firstOut - g.vertices[u].firstOut
}

func (g *Graph) ForOutEdgesOfWithId(u Index, handle func(e *OutEdge, id Index)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {

		handle(g.outEdges[e], e)
	}
}

func (g *Graph) ForInEdgesOf(v Index, exitPoint Index, handle func(e *InEdge, entryPoint Index, turnType pkg.TurnType)) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {

		handle(g.inEdges[e], g.GetEntryOrder(v, e), g.GetTurnType(v, g.GetEntryOrder(v, e), exitPoint))
	}
}

func (g *Graph) ForInEdgesOfWithId(v Index, handle func(e *InEdge, id Index)) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {
		if g.inEdges[e].GetTail() == v {
			continue
		}

		handle(g.inEdges[e], e)
	}
}

func (g *Graph) GetHeadFromInEdge(entryPoint Index) Index {
	sourceInEdge := g.GetInEdge(entryPoint)
	tailAtSourceInEdge := g.GetVertex(sourceInEdge.GetTail())
	outEdgeToSource := g.GetOutEdge(tailAtSourceInEdge.GetFirstOut() + Index(sourceInEdge.GetExitPoint()))
	head := outEdgeToSource.GetHead()
	return head
}

func (g *Graph) GetTailFromOutEdge(exitPoint Index) Index {
	sourceOutEdge := g.GetOutEdge(exitPoint)
	headAtSourceOutEdge := g.GetVertex(sourceOutEdge.GetHead())
	inEdgeFromSource := g.GetInEdge(headAtSourceOutEdge.GetFirstIn() + Index(sourceOutEdge.GetEntryPoint()))
	return inEdgeFromSource.GetTail()
}

// GetOverlayVertex. return overlay vertex id
func (g *Graph) GetOverlayVertex(u Index, exitEntryOrder int, exit bool) (Index, bool) {
	subV := SubVertex{
		originalID:     u,
		exitEntryOrder: exitEntryOrder,
		exit:           exit,
	}
	id, exists := g.overlayVertices[subV]
	return id, exists
}

func (g *Graph) GetTurntables() []pkg.TurnType {
	return g.turnTables
}

func (g *Graph) GetCellNumber(u Index) Pv {
	return g.cellNumbers[g.vertices[u].pvPtr]
}

func (g *Graph) GetNumberOfCellsNumbers() int {
	return len(g.cellNumbers)
}

func (g *Graph) ForOutEdges(handle func(e *OutEdge, exitPoint, head Index, tail, entryId Index, percentage float64, idx Index)) {
	for idx, e := range g.outEdges {
		percentage := float64(idx) / float64(len(g.outEdges)) * 100
		tail := g.GetTailOfOutedge(Index(idx))

		entryId := g.GetVertex(e.head).GetFirstIn() + Index(e.GetEntryPoint())

		handle(e, g.GetExitOrder(tail, Index(idx)), e.head, tail, entryId, percentage, Index(idx))
	}
}

func (g *Graph) ForVertices(handle func(v *Vertex)) {
	for _, v := range g.vertices {

		handle(v)
	}
}

func (g *Graph) GetNumberOfOverlayVertexMapping() int {
	return len(g.overlayVertices)
}

func (g *Graph) GetVertexCoordinates(u Index) (float64, float64) {
	v := g.vertices[u]
	return v.lat, v.lon
}

func (g *Graph) SetVertices(vs []*Vertex) {
	g.vertices = vs
}

// for source point in query
func (g *Graph) GetVertexCoordinatesFromOutEdge(u Index) (float64, float64) {
	v := g.GetOutEdge(u)
	vertex := g.GetVertex(v.GetHead())
	return vertex.lat, vertex.lon
}

// for target point in query
func (g *Graph) GetVertexCoordinatesFromInEdge(u Index) (float64, float64) {
	v := g.GetInEdge(u)
	vertex := g.GetVertex(v.GetTail())
	return vertex.lat, vertex.lon
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

func (g *Graph) GetVertices() []*Vertex {
	vertices := make([]*Vertex, 0, g.NumberOfVertices())
	for _, vertex := range g.vertices[:g.NumberOfVertices()] {
		vertices = append(vertices, vertex)
	}
	return vertices
}

func (g *Graph) GetVertex(u Index) *Vertex {
	return g.vertices[u]
}

func (g *Graph) GetVertexFirstOut(u Index) Index {
	return g.vertices[u].GetFirstOut()
}

func (g *Graph) GetVertexFirstIn(u Index) Index {
	return g.vertices[u].GetFirstIn()
}

func (g *Graph) SetOutEdge(id Index, e *OutEdge) {
	g.outEdges[id] = e
}

func (g *Graph) GetNumberOfVerticesWithDummyVertex() int {
	return len(g.vertices)
}

func (g *Graph) SetInEdge(id Index, e *InEdge) {
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

func (g *Graph) SetBoundingBox(bb *BoundingBox) {
	g.boundingBox = bb
}

func (g *Graph) GetBoundingBox() *BoundingBox {
	return g.boundingBox
}

func (g *Graph) CondensationGraphOrigintoDestinationConnected(u, v Index) bool {
	sccOfU := g.sccs[u]
	sccOfV := g.sccs[v]

	connected := false
	visited := make([]bool, len(g.sccs))
	g.dfsCondensationGraph(sccOfU, sccOfV, visited, &connected)
	return connected
}

// O(V_G + E_G), V_G=number of sccs/number of vertices in condensation graph^scc, E_G=number of edges in condensation graph^scc
func (g *Graph) dfsCondensationGraph(u Index, t Index, visited []bool, connected *bool) {
	if u == t {
		*connected = true
	}
	if visited[u] {
		return
	}
	visited[u] = true

	for _, v := range g.sccCondensationAdj[u] {
		g.dfsCondensationGraph(v, t, visited, connected)
	}
}

func (g *Graph) GetMapEdgeInfo() []EdgeExtraInfo {
	return g.graphStorage.mapEdgeInfo
}

func (g *Graph) SetEdgeInfo(id Index, edgeInfo EdgeExtraInfo) {
	g.graphStorage.mapEdgeInfo[id] = edgeInfo
}

func (g *Graph) GetRoundaboutFlag() []Index {
	return g.graphStorage.roundaboutFlag
}

func (g *Graph) SetRoundabout(edgeID Index, isRoundabout bool) {
	g.graphStorage.SetRoundabout(edgeID, isRoundabout)
}

func (g *Graph) IsTrafficLight(vertexId Index) bool {
	return g.graphStorage.GetTrafficLight(vertexId)
}

// O(V_G + E_G), V_G=number of sccs/number of vertices in condensation graph^scc, E_G=number of edges in condensation graph^scc
func (g *Graph) VerticeUandVAreConnected(u, v Index) bool {
	sccOfU := g.GetSCCOfAVertex(u)
	sccOfV := g.GetSCCOfAVertex(v)
	if sccOfU == sccOfV {
		return true
	}

	return g.CondensationGraphOrigintoDestinationConnected(u, v) // O(V_G + E_G), V_G=number of sccs/number of vertices in condensation graph^scc, E_G=number of edges in condensation graph^scc
}

func (g *Graph) GetNodeTrafficLight() []Index {
	return g.graphStorage.nodeTrafficLight
}

func (g *Graph) SetNodeTrafficLight(vId Index, yes bool) {
	g.graphStorage.SetTrafficLight(vId, yes)
}

func (g *Graph) GetHaversineDistanceFromUtoV(u, v Index) float64 {
	uvertex := g.GetVertex(u)
	vvertex := g.GetVertex(v)
	return geo.CalculateHaversineDistance(uvertex.lat, uvertex.lon, vvertex.lat, vvertex.lon)
}

func (g *Graph) SetGraphStorage(gs *GraphStorage) {
	g.graphStorage = gs
}

func (g *Graph) IsRoundabout(edgeId Index) bool {
	_, roundabout := g.graphStorage.GetEdgeExtraInfo(edgeId, false)
	return roundabout
}

func (g *Graph) GetStreetName(edgeId Index) string {
	edgeInfo, _ := g.graphStorage.GetEdgeExtraInfo(edgeId, false)
	return g.graphStorage.tagStringIDMap.GetStr(edgeInfo.streetName)
}

func (g *Graph) GetRoadClass(edgeId Index) string {
	edgeInfo, _ := g.graphStorage.GetEdgeExtraInfo(edgeId, false)
	return g.graphStorage.tagStringIDMap.GetStr(int(edgeInfo.roadClass))
}

func (g *Graph) GetRoadClassLink(edgeId Index) string {
	edgeInfo, _ := g.graphStorage.GetEdgeExtraInfo(edgeId, false)
	return g.graphStorage.tagStringIDMap.GetStr(int(edgeInfo.roadClassLink))
}

func (g *Graph) GetRoadLanes(edgeId Index) uint8 {
	edgeInfo, _ := g.graphStorage.GetEdgeExtraInfo(edgeId, false)
	return edgeInfo.lanes
}

func (g *Graph) GetStreetDirection(edgeId Index) [2]bool {
	edgeInfo, _ := g.graphStorage.GetEdgeExtraInfo(edgeId, false)
	return g.graphStorage.GetStreetDirection(edgeInfo.osmWayId)
}

func (g *Graph) GetOsmWayId(edgeId Index) int64 {
	edgeInfo, _ := g.graphStorage.GetEdgeExtraInfo(edgeId, false)
	return edgeInfo.osmWayId
}

func (g *Graph) GetEdgeGeometry(edgeID Index) []Coordinate {
	return g.graphStorage.GetEdgeGeometry(edgeID)
}

func (g *Graph) ForOutEdgesOfVertex(u Index, handle func(e *OutEdge, exitPoint Index)) {
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		if g.outEdges[e].GetHead() == u {
			continue
		}

		handle(g.outEdges[e], g.GetExitOrder(u, e))
	}
}

func (g *Graph) GetVerticeIds() []Index {
	nodeIds := make([]Index, 0, g.NumberOfVertices())
	for i := 0; i < g.NumberOfVertices(); i++ {
		nodeIds = append(nodeIds, Index(i))
	}
	return nodeIds
}

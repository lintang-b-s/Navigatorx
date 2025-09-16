package datastructure

type Index uint32

type Vertex struct {
	lat          float64
	lon          float64
	pvPtr        Index
	turnTablePtr Index
	firstOut     Index
	firstIn      Index
	id           Index
}

func NewVertex(lat, lon float64, id Index) Vertex {
	return Vertex{
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
	weight     float64 // minute
	dist       float64 // meter
	edgeID     Index
	head       Index
	entryPoint uint8
}

// inedge exits vertex tail at exitPoint
type InEdge struct {
	weight    float64 // minute
	dist      float64 // meter
	edgeID    Index
	tail      Index
	exitPoint uint8
}

func NewOutEdge(edgeID, head Index, weight, dist float64, entryPoint uint8) OutEdge {
	return OutEdge{
		edgeID:     edgeID,
		head:       head,
		weight:     weight,
		dist:       dist,
		entryPoint: entryPoint,
	}
}

func NewInEdge(edgeID, tail Index, weight, dist float64, exitPoint uint8) InEdge {
	return InEdge{
		edgeID:    edgeID,
		tail:      tail,
		weight:    weight,
		dist:      dist,
		exitPoint: exitPoint,
	}
}

func (e *OutEdge) GetEdgeSpeed() float64 {
	return e.dist / e.weight
}

func (e *OutEdge) GetHead() Index {
	return e.head
}

func (e *OutEdge) SetEntryPoint(p uint8) {
	e.entryPoint = p
}

func (e *InEdge) GetEdgeSpeed() float64 {
	return e.dist / e.weight
}

func (e *InEdge) GetTail() Index {
	return e.tail
}

func (e *InEdge) SetExitPoint(p uint8) {
	e.exitPoint = p
}

type SubVertex struct {
	originalID Index
	turnOrder  uint8
	exit       bool
}

type VertexIDPair struct {
	originalVertexID Index
	id               Index
}

// enum of turn_type
type TurnType uint8

const (
	LEFT_TURN TurnType = iota
	RIGHT_TURN
	STRAIGHT_ON
	U_TURN
	NO_ENTRY
	NONE
)

type pv uint64

type Graph struct {
	vertices          []Vertex
	outEdges          []OutEdge
	inEdges           []InEdge
	turnTables        []TurnType
	cellNumbers       []pv
	maxEdgesInCell    Index
	outEdgeCellOffset []Index
	inEdgeCellOffset  []Index
	overlayVertices   map[SubVertex]Index // graph vertices -> overlay vertices
}

func NewGraph(vertices []Vertex, forwardEdges []OutEdge, inEdges []InEdge, turnMatrices []TurnType) *Graph {
	return &Graph{vertices: vertices, outEdges: forwardEdges, inEdges: inEdges, turnTables: turnMatrices, maxEdgesInCell: 0}
}

func (g *Graph) NumberOfVertices() int {
	return len(g.vertices) - 1
}

func (g *Graph) NumberOfEdges() int {
	return len(g.outEdges)
}

func (g *Graph) GetOutDegree(u Index) Index {
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

func (g *Graph) GetOutEdge(e Index) OutEdge {
	return g.outEdges[e]
}

func (g *Graph) GetInEdge(e Index) InEdge {
	return g.inEdges[e]
}

func (g *Graph) GetHeadOfInedge(e Index) Index {
	inEdge := g.GetInEdge(e)
	tail := g.vertices[inEdge.tail]
	outEdge := g.GetOutEdge(tail.firstOut + Index(inEdge.exitPoint))
	return outEdge.head
}

func (g *Graph) GetTailOfOutedge(e Index) Index {
	outEdge := g.GetOutEdge(e)
	head := g.vertices[outEdge.head]
	inEdge := g.GetInEdge(head.firstIn + Index(outEdge.entryPoint))
	return inEdge.tail
}

// GetExitOrder. return Index of exit point of a out edge (u,v) at vertex u.
func (g *Graph) GetExitOrder(u, outEdge Index) Index {
	return outEdge - g.vertices[u].firstOut
}

// GetEntryOrder. return Index of entry point of a in edge (u,v) at vertex v.
func (g *Graph) GetEntryOrder(v, InEdge Index) Index {
	return InEdge - g.vertices[v].firstIn
}

func (g *Graph) GetTurnType(u Index, entryPoint, exitPoint uint8) TurnType {
	turnTableOffset := g.vertices[u].turnTablePtr + Index(entryPoint)*Index(g.GetOutDegree(u)) + Index(exitPoint)
	return g.turnTables[turnTableOffset]
}

func (g *Graph) SetCellNumbers(cellNumbers []pv) {
	g.cellNumbers = cellNumbers
}

func (g *Graph) SetOverlayMapping(overlayVertices map[SubVertex]Index) {
	g.overlayVertices = overlayVertices
}

func (g *Graph) GetOverlayVertex(u Index, turnOrder uint8, exit bool) (Index, bool) {
	subV := SubVertex{
		originalID: u,
		turnOrder:  turnOrder,
		exit:       exit,
	}
	id, exists := g.overlayVertices[subV]
	return id, exists
}

func (g *Graph) GetTUrntables() []TurnType {
	return g.turnTables
}

func (g *Graph) GetCellNumber(u Index) pv {
	return g.cellNumbers[g.vertices[u].pvPtr]
}

func (g *Graph) GetNumberOfCellsNumbers() int {
	return len(g.cellNumbers)
}

func (g *Graph) GetNumberOfOverlayVertexMapping() int {
	return len(g.overlayVertices)
}

func (g *Graph) GetVertexCoordinates(u Index) (float64, float64) {
	v := g.vertices[u]
	return v.lat, v.lon
}

func (g *Graph) GetMaxEdgesInCell() Index {
	return g.maxEdgesInCell
}

func (g *Graph) GetOutEdgeCellOffset(v Index) Index {
	return g.outEdgeCellOffset[g.vertices[v].pvPtr]
}

func (g *Graph) GetInEdgeCellOffset(v Index) Index {
	return g.inEdgeCellOffset[g.vertices[v].pvPtr]
}

func (g *Graph) GetOutEdgeCellOffsets() []Index {
	return g.outEdgeCellOffset
}

func (g *Graph) GetInEdgeCellOffsets() []Index {
	return g.inEdgeCellOffset
}

func (g *Graph) GetVertices() []Vertex {
	vertices := make([]Vertex, len(g.vertices))
	for _, vertex := range g.vertices {
		vertices = append(vertices, vertex)
	}
	return vertices
}

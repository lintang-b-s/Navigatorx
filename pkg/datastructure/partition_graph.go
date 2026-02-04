package datastructure

// we use capacity 1 for each edge in graph (see On Balanced Separators in Road Networks, Schild, et al.)
// maximum flow using Dinic’s algorithm (augmenting paths, in the unit-capacity case computed by breadth-first search)
type MaxFlowEdge struct {
	id       int
	u        Index
	v        Index
	capacity int
	flow     int
}

func NewMaxFlowEdge(id int, u, v Index, capacity int) *MaxFlowEdge {
	return &MaxFlowEdge{
		id:       id,
		u:        u,
		v:        v,
		capacity: capacity,
		flow:     0,
	}
}

func (e *MaxFlowEdge) GetID() int {
	return e.id
}

func (e *MaxFlowEdge) GetCapacity() int {
	return e.capacity
}

func (e *MaxFlowEdge) GetFlow() int {
	return e.flow
}

func (e *MaxFlowEdge) GetFrom() Index {
	return e.u
}

func (e *MaxFlowEdge) GetTo() Index {
	return e.v
}

func (e *MaxFlowEdge) AddFlow(f int) {
	e.flow += f
}

type PartitionVertex struct {
	id               Index
	originalVertexId Index
	lat, lon         float64
}

func NewPartitionVertex(id, originalVertexId Index, lat, lon float64) PartitionVertex {
	return PartitionVertex{
		id:               id,
		originalVertexId: originalVertexId,
		lat:              lat,
		lon:              lon,
	}
}

func (v *PartitionVertex) GetID() Index {
	return v.id
}

func (v *PartitionVertex) SetId(id Index) {
	v.id = id
}

func (v *PartitionVertex) GetOriginalVertexID() Index {
	return v.originalVertexId
}

func (v *PartitionVertex) GetVertexCoordinate() (float64, float64) {
	return v.lat, v.lon
}

type PartitionGraph struct {
	vertices      []PartitionVertex
	adjacencyList [][]int
	edgeList      []*MaxFlowEdge
	level         []int
	last          []int
	visited       []bool
	prev          []*MaxFlowEdge
}

func NewPartitionGraph(numberOfVertices int) *PartitionGraph {
	adjacencyList := make([][]int, numberOfVertices)
	for i := range adjacencyList {
		adjacencyList[i] = make([]int, 0)
	}
	return &PartitionGraph{
		vertices:      make([]PartitionVertex, 0),
		adjacencyList: adjacencyList,
		edgeList:      make([]*MaxFlowEdge, 0),
		level:         make([]int, numberOfVertices),
		last:          make([]int, numberOfVertices),
		visited:       make([]bool, numberOfVertices),
		prev:          make([]*MaxFlowEdge, numberOfVertices+2),
	}
}

func (g *PartitionGraph) GetVertices() []PartitionVertex {
	return g.vertices
}
func (g *PartitionGraph) NumberOfVertices() int {
	return len(g.vertices)
}

func (g *PartitionGraph) GetPrev(u Index) *MaxFlowEdge {
	return g.prev[u]
}
func (g *PartitionGraph) SetPrev(u Index, e *MaxFlowEdge) {
	g.prev[u] = e
}

func (g *PartitionGraph) ResetPrev() {
	for i := range g.prev {
		g.prev[i] = nil
	}
}

func (g *PartitionGraph) HandleVisited(handle func(u Index, visited bool)) {
	for u, visited := range g.visited {
		handle(Index(u), visited)
	}
}

func (g *PartitionGraph) AddVertex(v PartitionVertex) {
	g.vertices = append(g.vertices, v)
	if len(g.adjacencyList) < int(v.id)+1 {
		g.adjacencyList = append(g.adjacencyList, []int{})
	}
	if len(g.level) < int(v.id)+1 {
		g.level = append(g.level, 0)
	}
	if len(g.last) < int(v.id)+1 {
		g.last = append(g.last, 0)
	}
	if len(g.visited) < int(v.id)+1 {
		g.visited = append(g.visited, false)
	}
}

func (g *PartitionGraph) GetVertex(u Index) PartitionVertex {
	return g.vertices[u]
}

func (g *PartitionGraph) ResetGraph() {
	for _, edge := range g.edgeList {
		edge.flow = 0
		edge.capacity = 1
	}
	for i := range g.visited {
		g.visited[i] = false
		g.level[i] = 0
		g.last[i] = 0
	}
}

func (g *PartitionGraph) GetVertexLevel(u Index) int {
	return g.level[u]
}
func (g *PartitionGraph) SetVertexLevel(u Index, level int) {
	g.level[u] = level
}

func (g *PartitionGraph) GetLastEdgeIndex(u Index) int {
	return g.last[u]
}

func (g *PartitionGraph) SetLastEdgeIndex(u Index, idx int) {
	g.last[u] = idx
}

func (g *PartitionGraph) IncrementLastEdgeIndex(u Index) {
	g.last[u]++
}

func (g *PartitionGraph) GetVertexEdgesSize(u Index) int {
	return len(g.adjacencyList[u])
}

func (g *PartitionGraph) GetEdgeOfVertex(u Index, idx int) *MaxFlowEdge {
	edgeIndex := g.adjacencyList[u][idx]
	return g.edgeList[edgeIndex]
}

func (g *PartitionGraph) GetReversedEdgeOfVertex(u Index, idx int) *MaxFlowEdge {
	edgeIndex := g.adjacencyList[u][idx] ^ 1
	return g.edgeList[edgeIndex]
}

func (g *PartitionGraph) ForEachVertexEdges(u Index, handle func(e *MaxFlowEdge)) {
	for _, edgeIdx := range g.adjacencyList[u] {
		handle(g.edgeList[edgeIdx])
	}
}

func (g *PartitionGraph) ForEachVertices(handle func(v PartitionVertex)) {
	for _, v := range g.vertices {
		handle(v)
	}
}

func (g *PartitionGraph) GetEdgeById(eId int) *MaxFlowEdge {
	return g.edgeList[eId]
}

func (g *PartitionGraph) AddEdge(u, v Index, w int, directed bool) {
	if u == v {
		return
	}

	edge := NewMaxFlowEdge(len(g.edgeList), u, v, w)
	g.edgeList = append(g.edgeList, edge)
	g.adjacencyList[u] = append(g.adjacencyList[u], len(g.edgeList)-1)

	var reverseEdge *MaxFlowEdge = &MaxFlowEdge{}
	if directed {
		reverseEdge = NewMaxFlowEdge(len(g.edgeList), v, u, 0)
	} else {
		reverseEdge = NewMaxFlowEdge(len(g.edgeList), v, u, w)
	}

	g.edgeList = append(g.edgeList, reverseEdge)
	g.adjacencyList[v] = append(g.adjacencyList[v], len(g.edgeList)-1)
}

func (g *PartitionGraph) ForEdgeList(handle func(e *MaxFlowEdge, eId int)) {
	for eId, e := range g.edgeList {
		handle(e, eId)
	}
}

func (g *PartitionGraph) AddSingleEdge(e *MaxFlowEdge, eId int) {
	u := e.GetFrom()

	g.edgeList = append(g.edgeList, e)
	g.adjacencyList[u] = append(g.adjacencyList[u], len(g.edgeList)-1)
}

func (pg *PartitionGraph) Clone() *PartitionGraph {

	newPg := NewPartitionGraph(pg.NumberOfVertices())

	for i := 0; i < pg.NumberOfVertices(); i++ {
		v := pg.GetVertex(Index(i))
		lat, lon := v.GetVertexCoordinate()
		cv := NewPartitionVertex(v.GetID(), v.GetOriginalVertexID(), lat, lon)
		newPg.AddVertex(cv)
	}

	copy(newPg.level, pg.level)
	copy(newPg.last, pg.last)

	for i, adj := range pg.adjacencyList {
		newAdj := make([]int, len(adj))
		copy(newAdj, adj)
		newPg.adjacencyList[i] = newAdj
	}
	for _, e := range pg.edgeList {
		newPg.edgeList = append(newPg.edgeList, NewMaxFlowEdge(e.id, e.u, e.v, e.capacity))
	}

	return newPg
}

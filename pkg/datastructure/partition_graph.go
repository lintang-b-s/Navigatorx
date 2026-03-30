package datastructure

// we use capacity 1 for each edge in graph (see On Balanced Separators in Road Networks, Schild, et al.)
// maximum flow using Dinic’s algorithm (augmenting paths, in the unit-capacity case computed by breadth-first search)
type MaxFlowEdge struct {
	id       int
	u        Index
	v        Index
	capacity int64
	flow     int64
}

func NewMaxFlowEdge(id int, u, v Index, capacity int64) MaxFlowEdge {
	return MaxFlowEdge{
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

func (e *MaxFlowEdge) GetCapacity() int64 {
	return e.capacity
}

func (e *MaxFlowEdge) GetFlow() int64 {
	return e.flow
}

func (e *MaxFlowEdge) GetFrom() Index {
	return e.u
}

func (e *MaxFlowEdge) GetTo() Index {
	return e.v
}

func (e *MaxFlowEdge) AddFlow(f int64) {
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
	edgeList      []MaxFlowEdge
	level         []int
	last          []int
}

func NewPartitionGraph(numberOfVertices int) *PartitionGraph {
	adjacencyList := make([][]int, numberOfVertices)
	for i := range adjacencyList {
		adjacencyList[i] = make([]int, 0)
	}
	return &PartitionGraph{
		vertices:      make([]PartitionVertex, 0),
		adjacencyList: adjacencyList,
		edgeList:      make([]MaxFlowEdge, 0),
		level:         make([]int, numberOfVertices),
		last:          make([]int, numberOfVertices),
	}
}

func (g *PartitionGraph) GetVertices() []PartitionVertex {
	return g.vertices
}

func (g *PartitionGraph) SetVertices(vertices []PartitionVertex) {
	g.vertices = vertices
}

func (g *PartitionGraph) ForVertices(handle func(vertex PartitionVertex, idx int)) {
	for idx, vertex := range g.vertices {
		handle(vertex, idx)
	}
}

func (g *PartitionGraph) NumberOfVertices() int {
	return len(g.vertices)
}

func (g *PartitionGraph) GetLast() []int {
	return g.last
}

func (g *PartitionGraph) GetLevel() []int {
	return g.level
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
}

func (g *PartitionGraph) GetVertex(u Index) PartitionVertex {
	return g.vertices[u]
}

func (g *PartitionGraph) GetVertexEdgesSize(u Index) int {
	return len(g.adjacencyList[u])
}

func (g *PartitionGraph) GetEdgeOfVertex(u Index, idx int) MaxFlowEdge {
	edgeIndex := g.adjacencyList[u][idx]
	return g.edgeList[edgeIndex]
}

func (g *PartitionGraph) GetEdgeId(u Index, idx int) int {
	return g.adjacencyList[u][idx]
}

func (g *PartitionGraph) GetReversedEdgeId(u Index, idx int) int {
	return g.adjacencyList[u][idx] ^ 1
}

func (g *PartitionGraph) ForEachVertexEdges(u Index, handle func(e MaxFlowEdge, artificial bool)) {
	for _, edgeIdx := range g.adjacencyList[u] {
		handle(g.edgeList[edgeIdx], false)
	}
}

func (g *PartitionGraph) ForEachVertices(handle func(v PartitionVertex)) {
	for _, v := range g.vertices {
		handle(v)
	}
}

func (g *PartitionGraph) GetNumberOfEdges() int {
	return len(g.edgeList)
}

func (g *PartitionGraph) ForEdges(handle func(i int, e MaxFlowEdge)) {
	for i, e := range g.edgeList {
		handle(i, e)
	}
}

func (g *PartitionGraph) AddEdge(u, v Index, w int64, directed bool) {
	if u == v {
		return
	}

	edge := NewMaxFlowEdge(len(g.edgeList), u, v, w)
	g.edgeList = append(g.edgeList, edge)
	g.adjacencyList[u] = append(g.adjacencyList[u], len(g.edgeList)-1)

	var reversedEdge MaxFlowEdge = MaxFlowEdge{}
	if directed {
		reversedEdge = NewMaxFlowEdge(len(g.edgeList), v, u, 0)
	} else {
		reversedEdge = NewMaxFlowEdge(len(g.edgeList), v, u, w)
	}

	g.edgeList = append(g.edgeList, reversedEdge)
	g.adjacencyList[v] = append(g.adjacencyList[v], len(g.edgeList)-1)
}

func (g *PartitionGraph) ForEdgeList(handle func(e MaxFlowEdge, eId int)) {
	for eId, e := range g.edgeList {
		handle(e, eId)
	}
}

// GetDirectedEdges.
// return directed edges
func (g *PartitionGraph) GetDirectedEdges() []MaxFlowEdge {
	// karena partition graph bisa undirected
	// fungsi ini hanya return directed edges
	// di AddEdge() kita add direceted edge dengan id genap
	edges := make([]MaxFlowEdge, 0, len(g.edgeList)/2)
	for edgeId, edge := range g.edgeList {
		if edgeId%2 == 0 {
			edges = append(edges, edge)
		}
	}

	return edges
}

// GetDirectedEdges.
// return directed edges dari vertex u
func (g *PartitionGraph) ForEachDirectedEdgesOf(u Index, handle func(e MaxFlowEdge, eId int)) {
	// karena partition graph bisa undirected
	// fungsi ini hanya return directed edges
	// di AddEdge() kita add direceted edge dengan id genap
	g.ForEachVertexEdges(u, func(e MaxFlowEdge, artificial bool) {
		edgeId := e.GetID()
		if edgeId%2 == 0 {
			handle(e, edgeId)
		}
	})
}

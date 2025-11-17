package datastructure

// for generating alternative routes using penalty-method
type DynamicGraph struct {
	vertices []*Vertex
	outEdges [][]*OutEdge // adjacency list
	inEdges  [][]*InEdge
}

func NewDynamicGraph() *DynamicGraph {
	return &DynamicGraph{vertices: make([]*Vertex, 0), outEdges: make([][]*OutEdge, 0)}
}

func (g *DynamicGraph) AddVertex(v *Vertex) Index {
	g.vertices = append(g.vertices, NewVertex(v.GetLat(), v.GetLon(), Index(len(g.vertices))))
	g.outEdges = append(g.outEdges, make([]*OutEdge, 0))
	g.inEdges = append(g.inEdges, make([]*InEdge, 0))
	return Index(len(g.vertices)) - 1
}

func (g *DynamicGraph) AddEdge(u, v Index, e OutEdge) {
	g.outEdges[u] = append(g.outEdges[u], NewOutEdge(e.GetEdgeId(), v, e.GetWeight(), e.GetLength(), e.GetEntryPoint()))
	g.inEdges[v] = append(g.inEdges[v], NewInEdge(e.GetEdgeId(), u, e.GetWeight(), e.GetLength(), e.GetEntryPoint()))
}

func (g *DynamicGraph) ForOutEdgesOf(u Index, handle func(e *OutEdge)) {
	for _, e := range g.outEdges[u] {
		handle(e)
	}
}

func (g *DynamicGraph) ForInEdgesOf(u Index, handle func(e *InEdge)) {
	for _, e := range g.inEdges[u] {
		handle(e)
	}
}

func (g *DynamicGraph) GetVertex(v Index) *Vertex {
	return g.vertices[v]
}

func (g *DynamicGraph) ForVertices(handle func(v *Vertex)) {
	for _, v := range g.vertices {
		handle(v)
	}
}


func (g *DynamicGraph)  NumberOfVertices() int {
	return len(g.vertices)
}
package datastructure

// CellGraph is a cell containing all vertices and edges within this recursive bisection cell, resulting from the inertial flow graph partitioning algorithm performed in `pkg/partitioner`.
// there are two types of cell:
// 1. recursive bisection (RB) cell is the S or T cell resulting from minimum st-cut algorithm in recursive_bisection.go and inertial_flow.go.
// 2. MultilevelPartition (MLP) cell is the cell at current level-l resulting from multilevel_partitioner.go
// if current MLP level equal to MultilevelPartitioner.l, then this MLP cell equal to the full road network graph (equal to Graph in graph.go).
// note that in recursive_biscetion.go, everytime we applyBisection(cut, cg) (cg is the current CellGraph) the cellGraph.vIds is partitioned into two cells S and T, resulting from minimum st-cut algorithm..
// then the recursive bisection, recursively do the same minimum st-cut algorithm into cell S and T, resulting another new 4 cells, and repeat until each cell size < U.
// everytime a cell partitioned into two cell S and T, we use cg.Partition(pred) such that cg.vIds[c.begin:mid] is containing all vertices in cell S, and cg.vIds[mid:c.end] is containing all vertices inside cell T.
// we do the same cg.Partition(pred) recursively but using new offset (c.begin, mid, c.end) by recursive bisection algorithm...
// cg.gv[i] is the map from road network graph vertex id=i to recursive bisection cell vertex id, resulting from the recursive bisection algorithm, such that vIds[gv[i]+cg.begin] = i, for all i.
// so, cg.begin is like offset for getting the first recursive bisection cell RB cell vertex id (road network graph vertex id) from the MLP cell vertices in cg.vIds.
type CellGraph struct {
	// if current MLP level equal to MultilevelPartitioner.l, then this MLP cell equal to the full road network graph.
	vIds       []Index // road network Graph (graph.go) vertices ids that inside current MultiLevelPartition (MLP) cell. permuted such that vIds[begin:end] contains all vertices in this recursive bisection cell. [begin, end) (begin inclusive, end exclusive).
	begin, end Index   // begin and end index in vIds that contains vertices in this recursive bisection cell, such that vIds[begin:end] contains all vertices in this recursive bisection cell.
	gv         []Index // map from road network Graph (in graph.go) vertex id to recursive bisection cell vertex id. such that vIds[gv[i]+begin] = i, for all i.
	// Compressed Sparse Row Graph https://ocw.mit.edu/courses/6-172-performance-engineering-of-software-systems-fall-2018/f58df934e205b870e473c2af3fe5e6f7_MIT6_172F18_lec22.pdf
	heads    []Index // maps from outgoing edge (u,v) id to head vertex v sorted by tail u.
	tails    []Index // maps from reversed edge (v,u) id to tail vertex u sorted by head v.
	firstOut []Index // first offset of outgoing edge (u,v) of vertex u.
	firstIn  []Index //  first offset of reversed edge (v,u) of vertex v.

	// edge (u,v) is inside recursive bisection cell iff tail vertex u and head vertex v inside this recursive bisection cell.
	g *Graph // road network Graph (in graph.go).
}

// NewCellGraph create new cell graph.
// cellvIds is the graph vertices ids inside this recursive bisection cell.
// vIds is road network Graph (graph.go) vertices ids that inside current MultiLevelPartition (MLP) cell. permuted such that vIds[begin:end]=cellvIds contains all vertices in this recursive bisection cell. [begin, end) (begin inclusive, end exclusive).
// gv is map from road networkGraph (in graph.go) vertex id to cell vertex id. such that vIds[gv[i]+begin] = i, for all i.
// vCellMasks . bitset that bitset[i]=true iff road network graph vertex id=i contained in this cell. Used to iterate over the outgoing edges that inside this recursive bisection cell from any vertex of this recursive bisection cell.
// begin and end index in vIds that contains vertices in this recursive bisection cell, such that vIds[begin:end] contains all vertices in this recursive bisection cell.
func NewCellGraph(g *Graph, cellvIds []Index, vIds []Index, gv []Index, begin, end Index) *CellGraph {
	for k, gvId := range cellvIds {
		gv[gvId] = Index(k)
	}
	cg := &CellGraph{vIds: vIds, g: g, begin: begin, end: end, gv: gv}
	cg.heads, cg.firstOut = cg.uniqueHeads(cellvIds)
	cg.tails, cg.firstIn = cg.uniqueTails(cellvIds)

	return cg
}

// NumberOfCellVertices. get number of vetices that inside this cell.
func (c *CellGraph) NumberOfCellVertices() int {
	return int(c.end - c.begin)
}

// GetCellVIds get road network graph vertices ids that inside this recursive bisection cell.
func (c *CellGraph) GetCellVIds() []Index {
	return c.vIds[c.begin:c.end]
}

// ForEachCellVertices iterate all recursive bisection cell vertices. return its RB cell vertex id, road network graph vertex id, and its coordinate.
// vId is the recursive bisection cell vertex id (or RB cell vertex id).
// gvId is the road network grpah vertex id.
func (c *CellGraph) ForEachCellVertices(handle func(vId, gvId Index, coord Coordinate)) {
	for i, gvId := range c.vIds[c.begin:c.end] {
		handle(Index(i), gvId, c.g.GetVertexCoordinate(gvId))
	}
}

// ForOutEdgesOf iterate all edges (and reversed edges) that inside this recursive bisection cell that coming from RB cell vertex id=u (recursive bisection cell vertex id).
func (c *CellGraph) ForEdgesOf(u Index, handle func(v Index, out bool)) {
	for o := c.firstOut[u]; o < c.firstOut[u+1]; o++ {
		v := c.heads[o]
		handle(v, true)
	}
	for o := c.firstIn[u]; o < c.firstIn[u+1]; o++ {
		v := c.tails[o]
		handle(v, false)
	}
}

func (c *CellGraph) OutEdge(u, i Index) Index {
	e := c.firstOut[u] + i
	return c.heads[e]
}

func (c *CellGraph) InEdge(u, i Index) Index {
	e := c.firstIn[u] + i
	return c.tails[e]
}

func (c *CellGraph) OutDegree(u Index) Index {
	return c.firstOut[u+1] - c.firstOut[u]
}

func (c *CellGraph) InDegree(u Index) Index {
	return c.firstIn[u+1] - c.firstIn[u]
}

// Partition partition the c.vIds into two parts determined by predicate pred. similiar to c++ std::partition https://en.cppreference.com/cpp/algorithm/partition
func (c *CellGraph) Partition(pred func(vId Index) bool) (*CellGraph, *CellGraph) {
	first := c.end // first index which pred(c.vIds[index]) == false

	for i := c.begin; i < c.end; i++ {
		vId := i - c.begin
		if !pred(vId) {
			first = i
			break
		}
	}

	for i := first + 1; i < c.end; i++ {
		vId := i - c.begin
		// loop invariant: all elements before first have predicate value of true. pred(vId)=true means that that vId is inside cell s.
		if pred(vId) {
			// swap
			c.vIds[first], c.vIds[i] = c.vIds[i], c.vIds[first]
			first++
		}
	}

	sgvIds := c.vIds[c.begin:first] // road network graph vIds of cell s
	tgvIds := c.vIds[first:c.end]   // road network graph vIds of cell t

	sCell := NewCellGraph(c.g, sgvIds, c.vIds, c.gv, c.begin, first)
	tCell := NewCellGraph(c.g, tgvIds, c.vIds, c.gv, first, c.end)
	return sCell, tCell
}

func (c *CellGraph) headsOf(u Index) []Index {
	heads := c.g.heads[c.g.vertices[u].firstOut:c.g.vertices[u+1].firstOut]
	return heads
}

func (c *CellGraph) tailsOf(u Index) []Index {
	tails := c.g.tails[c.g.vertices[u].firstIn:c.g.vertices[u+1].firstIn]
	return tails
}

func (c *CellGraph) insideCell(v, vv, cellLen Index) bool {
	return v < cellLen && c.vIds[c.begin+v] == vv
}

func (c *CellGraph) uniqueNeighbors(us []Index, out bool) ([]Index, []Index) {
	n := len(us)
	neighbors := make([]Index, 0, n)
	offset := make([]Index, n+1)
	cellLen := c.end - c.begin
	// O(m), m=total number of edges of vertices us
	for k, uu := range us {
		offset[k] = Index(len(neighbors))
		var uNeighbors []Index
		if out {
			uNeighbors = c.headsOf(uu)
		} else {
			uNeighbors = c.tailsOf(uu)
		}
		set := make(map[Index]struct{}, 3)
		for _, vv := range uNeighbors {
			v := c.gv[vv]
			insideCell := c.insideCell(v, vv, cellLen)
			if _, exists := set[v]; !exists && insideCell {
				neighbors = append(neighbors, v)
				set[v] = struct{}{}
			}
		}
	}

	offset[n] = Index(len(neighbors))
	return neighbors, offset
}

// uniqueHeads return outgoing edges (only head vertices of edges) of each vertex u in us (removed parallel edges).
func (c *CellGraph) uniqueHeads(us []Index) ([]Index, []Index) {
	return c.uniqueNeighbors(us, true)
}

// uniqueTails return incoming (reversed) edges (only tail vertices of reversed edges) of each vertex u in us (removed parallel edges).
func (c *CellGraph) uniqueTails(us []Index) ([]Index, []Index) {
	return c.uniqueNeighbors(us, false)
}

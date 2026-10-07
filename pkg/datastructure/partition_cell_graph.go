package datastructure

import "github.com/bits-and-blooms/bitset"

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
	vIds []Index // road network Graph (graph.go) vertices ids that inside current MultiLevelPartition (MLP) cell. permuted such that vIds[begin:end] contains all vertices in this recursive bisection cell. [begin, end) (begin inclusive, end exclusive).
	// if current MLP level equal to MultilevelPartitioner.l, then this MLP cell equal to the full road network graph.
	begin, end Index   // begin and end index in vIds that contains vertices in this recursive bisection cell, such that vIds[begin:end] contains all vertices in this recursive bisection cell.
	gv         []Index // map from road network Graph (in graph.go) vertex id to recursive bisection cell vertex id. such that vIds[gv[i]+begin] = i, for all i.

	vCellMasks *bitset.BitSet // bitset that bitset[i]=true iff road network graph vertex id=i contained in this cell. Used to iterate over the outgoing edges that inside this recursive bisection cell from any vertex of this recursive bisection cell.
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
	n := g.NumberOfVertices()
	vCellMasks := bitset.New(uint(n))
	for _, v := range cellvIds {
		vCellMasks.Set(uint(v))
	}

	return &CellGraph{vIds: vIds, vCellMasks: vCellMasks, g: g, begin: begin, end: end, gv: gv}
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
// vId is the recursive bisection cell vertex id (or RB cell id).
// gvId is the road network grpah vertex id.
func (c *CellGraph) ForEachCellVertices(handle func(vId, gvId Index, coord Coordinate)) {
	for i, gvId := range c.vIds[c.begin:c.end] {
		handle(Index(i), gvId, c.g.GetVertexCoordinate(gvId))
	}
}

// Partition partition the c.vIds into two parts determined by predicate pred. similiar to c++ std::partition https://en.cppreference.com/cpp/algorithm/partition
func (c *CellGraph) Partition(pred func(vId Index) bool) (*CellGraph, *CellGraph) {
	first := c.begin // first index which pred(c.vIds[index]) == false

	for i := c.begin; i < c.end; i++ {
		vId := i - c.begin
		if !pred(vId) {
			first = i
			break
		}
	}

	for i := first; i < c.end; i++ {
		vId := i - c.begin
		// loop invariant: all elements before first have predicate value of true. pred(vId)=true means that that vId is inside cell one.
		if pred(vId) {
			// swap
			c.vIds[first], c.vIds[i] = c.vIds[i], c.vIds[first]
			first++
		}
	}

	ogvIds := c.vIds[c.begin:first] // road network graph vIds of cell one
	tgvIds := c.vIds[first:c.end]   // road network graph vIds of cell two

	for k, gvId := range ogvIds {
		c.gv[gvId] = Index(k) // c.gv for cell one
	}

	for k, gvId := range tgvIds {
		c.gv[gvId] = Index(k) // c.gv for cell two
	}

	one := NewCellGraph(c.g, ogvIds, c.vIds, c.gv, c.begin, first)
	two := NewCellGraph(c.g, tgvIds, c.vIds, c.gv, first, c.end)
	return one, two
}

// ForOutEdgesOf iterate all outgoing edges that inside this recursive bisection cell that coming from RB cell vertex id=u (recursive bisection cell vertex id).
func (c *CellGraph) ForOutEdgesOf(u Index, handle func(v Index)) {
	uu := c.vIds[c.begin+u]
	c.g.ForOutEdgesOf(uu, func(_, vv, _ Index) { // iterate all road network graph outgoing edges (uu,vv)
		if c.vCellMasks.Test(uint(vv)) {
			v := c.gv[vv] // map back from road network graph vertex id=vv to recursive bisection cell vertex id=v.
			handle(v)
		}
	})
}

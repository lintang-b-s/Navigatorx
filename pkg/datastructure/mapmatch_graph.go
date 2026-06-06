package datastructure

import (
	"errors"
	"fmt"
	"io"
	"os"
	"sort"

	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type MapMatchVertex struct {
	id       Index
	firstOut Index // firstOut index dari edge pertama dari vertex ini (edge yang tailnya vertex ini).
}

func NewMapMatchVertex(id Index, firstOut Index) MapMatchVertex {
	return MapMatchVertex{
		id:       id,
		firstOut: firstOut,
	}
}

func (v *MapMatchVertex) GetId() Index {
	return v.id
}

func (v *MapMatchVertex) SetId(id Index) {
	v.id = id
}

func (v *MapMatchVertex) GetFirstOut() Index {
	return v.firstOut
}

func (v *MapMatchVertex) SetFirstOut(firstOut Index) {
	v.firstOut = firstOut
}

type MapMatchEdge struct {
	id       Index //  edge Id di road network graph (graph.go)
	tail     Index
	head     Index
	length   float64
	geometry []Coordinate
}

func NewMapMatchEdge(id Index, tail Index, head Index, length float64, geometry []Coordinate) MapMatchEdge {
	return MapMatchEdge{
		id:       id,
		tail:     tail,
		head:     head,
		length:   length,
		geometry: geometry,
	}
}

func (e *MapMatchEdge) GetRoadNetworkEdgeId() Index {
	return e.id
}

func (e *MapMatchEdge) SetId(id Index) {
	e.id = id
}

func (e *MapMatchEdge) GetTail() Index {
	return e.tail
}

func (e *MapMatchEdge) SetTail(tail Index) {
	e.tail = tail
}

func (e *MapMatchEdge) GetHead() Index {
	return e.head
}

func (e *MapMatchEdge) SetHead(head Index) {
	e.head = head
}

func (e *MapMatchEdge) GetLength() float64 {
	return e.length
}

func (e *MapMatchEdge) SetLength(length float64) {
	e.length = length
}

func (e *MapMatchEdge) GetGeometry() []Coordinate {
	return e.geometry
}

func (e *MapMatchEdge) SetGeometry(geometry []Coordinate) {
	e.geometry = geometry
}

// MapMatchingGraph compressed sparse row graph terinspirasi dari CSR graphnya C++ Boost libary: https://www.boost.org/doc/libs/latest/libs/graph/doc/compressed_sparse_row.html
// https://www.usenix.org/system/files/login/articles/login_winter20_16_kelly.pdf
type MapMatchingGraph struct {
	vertices       []MapMatchVertex // vertices
	edges          []MapMatchEdge
	loadedEdgesSet map[Index]Index // map dari  original edge Id (edgeId di roadnetworkGraph graph.go) -> local edge Id (edgeId di mapmatchingGraph disini)
}

func NewMapMatchingGraph(vertices []MapMatchVertex, edges []MapMatchEdge) *MapMatchingGraph {
	return &MapMatchingGraph{
		vertices:       vertices,
		edges:          edges,
		loadedEdgesSet: make(map[Index]Index),
	}
}

func (g *MapMatchingGraph) GetMapMatchEdgeId(originalId Index) (Index, bool) {
	localId, ok := g.loadedEdgesSet[originalId]
	return localId, ok
}

func (g *MapMatchingGraph) Reset() {
	g.edges = g.edges[:0]
	g.loadedEdgesSet = make(map[Index]Index)
}

func InitializeMapMatchingGraph(numberOfVertices int) *MapMatchingGraph {
	vertices := make([]MapMatchVertex, numberOfVertices+1)
	return NewMapMatchingGraph(vertices, make([]MapMatchEdge, 0))
}

func (g *MapMatchingGraph) GetVertices() []MapMatchVertex {
	return g.vertices
}

func (g *MapMatchingGraph) SetVertices(vertices []MapMatchVertex) {
	g.vertices = vertices
}

func (g *MapMatchingGraph) GetEdges() []MapMatchEdge {
	return g.edges
}

func (g *MapMatchingGraph) NumofEdges() int {
	return len(g.edges)
}

func (g *MapMatchingGraph) SetEdges(edges []MapMatchEdge) {
	g.edges = edges
}

func (g *MapMatchingGraph) GetEdgeGeometry(eId Index) []Coordinate {
	return g.edges[eId].GetGeometry()
}

func (g *MapMatchingGraph) GetRoadnetworkEdgeId(eId Index) Index {
	return g.edges[eId].id
}

func (g *MapMatchingGraph) GetOutEdge(eId Index) MapMatchEdge {
	return g.edges[eId]
}

func (g *MapMatchingGraph) ForOutEdgesOf(u Index, handle func(eId, head Index, length float64, geometry []Coordinate)) {
	for eId := g.vertices[u].firstOut; eId < g.vertices[u+1].firstOut; eId++ {
		e := g.edges[eId]
		handle(eId, e.GetHead(), e.GetLength(), e.GetGeometry())
	}
}

// RebuildMapMatchGraph rebuild map matching road network graph dari file graph tile (see tiling_engine.go)
// terinspirasi dari MapDataManager nya Lyft: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
func (g *MapMatchingGraph) RebuildMapMatchGraph(graphTileFilePath string) error {

	f, err := os.Open(graphTileFilePath)
	if err != nil {
		return err
	}
	defer f.Close()

	return g.RebuildMapMatchGraphFromReader(f)
}

// RebuildMapMatchGraphFromReader rebuild map matching road network graph dari file graph tile (see tiling_engine.go)
// terinspirasi dari MapDataManager nya Lyft: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
func (g *MapMatchingGraph) RebuildMapMatchGraphFromReader(r io.Reader) error {
	g.Reset()
	return g.Rebuild(r)
}

func (g *MapMatchingGraph) Rebuild(r io.Reader) error {
	reader := util.NewBinaryReader(s2.NewReader(r))

	for {
		eId, err := reader.Uint32()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			return fmt.Errorf("read map-matching tile edge ID: %w", err)
		}

		tailVId, err := reader.Uint32()
		if err != nil {
			return fmt.Errorf("read map-matching tile edge %d tail: %w", eId, err)
		}
		headVId, err := reader.Uint32()
		if err != nil {
			return fmt.Errorf("read map-matching tile edge %d head: %w", eId, err)
		}
		length, err := reader.Float64()
		if err != nil {
			return fmt.Errorf("read map-matching tile edge %d length: %w", eId, err)
		}

		numGeom, err := reader.Uint32()
		if err != nil {
			return fmt.Errorf("read map-matching tile edge %d geometry count: %w", eId, err)
		}

		geom := make([]Coordinate, int(numGeom))
		for i := range geom {
			lat, err := reader.Int32()
			if err != nil {
				return fmt.Errorf("read map-matching tile edge %d geometry %d latitude: %w", eId, i, err)
			}
			lon, err := reader.Int32()
			if err != nil {
				return fmt.Errorf("read map-matching tile edge %d geometry %d longitude: %w", eId, i, err)
			}
			geom[i] = NewFixedCoordinate(lat, lon)
		}

		if _, ok := g.loadedEdgesSet[Index(eId)]; ok {
			continue
		}

		g.edges = append(g.edges, MapMatchEdge{
			id:       Index(eId),
			tail:     Index(tailVId),
			head:     Index(headVId),
			length:   length,
			geometry: geom,
		})

		g.loadedEdgesSet[Index(eId)] = 0 // dummy value, diupdate setelah sorting edges by its tail
	}

	// sort edges by tail
	sort.Slice(g.edges, func(i, j int) bool {
		return g.edges[i].tail < g.edges[j].tail
	})

	m := Index(len(g.edges))
	// rebuild Compressed Sparse Row (CSR) MapMatchingGraph
	currEId := Index(0)
	for vId := 0; vId < len(g.vertices); vId++ { // O(n+m), n= number of vertices in roadNetworkGraph (graph.go),m =number of edges in current MapMatchingGraph geohashes tile (mapmatch_graph.go)
		g.vertices[vId].firstOut = currEId
		for currEId < m && g.edges[currEId].tail == Index(vId) {
			roadNetworkEdgeId := g.edges[currEId].GetRoadNetworkEdgeId()
			g.loadedEdgesSet[roadNetworkEdgeId] = currEId
			currEId++
		}
	}

	return nil
}

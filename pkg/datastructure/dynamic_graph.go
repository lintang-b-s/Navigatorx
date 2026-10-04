package datastructure

import (
	"fmt"
	"io"
	"math"
	"sync/atomic"

	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/twpayne/go-polyline"
)

// ClientVertex  edge-based graph node. road segment. geometry is the geometry of the road segment
type ClientVertex struct {
	rnId     Index // id dari vertex (edge-based graph) di graph.go
	firstOut Index // firstOut index dari edge pertama dari vertex ini (edge yang tailnya vertex ini).

}

func NewClientVertex(rnId Index) ClientVertex {
	return ClientVertex{
		rnId:     rnId,
		firstOut: math.MaxUint32,
	}
}

// DynamicGraph read MapDataManager in this article: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type DynamicGraph struct {
	g atomic.Pointer[AdjacencyArray]
}

// AdjacencyArray store client road network graph
// DynamicGraph adjacency array graph terinspirasi dari CSR graphnya C++ Boost libary: https://www.boost.org/doc/libs/1_61_0/libs/graph/doc/compressed_sparse_row.html
// https://www.usenix.org/system/files/login/articles/login_winter20_16_kelly.pdf
// edge-based graph
type AdjacencyArray struct {
	vertices []ClientVertex // vertices.
	heads    []Index        // head v dari edge (u,v) sorted by tail u
	weights  []uint32       // weights of all edge-based graph edges. in centiseconds.

	speeds          []float64      // speed limit of  road segments in m/s
	lengths         []float64      // length of  road segments in meters
	segmentGeometry [][]Coordinate // geometry of road segments
}

func (dg *DynamicGraph) ForOutEdgesOf(u Index, handle func(eId, v Index, weight uint32)) {
	g := dg.g.Load()
	for e := g.vertices[u].firstOut; e < g.vertices[u+1].firstOut; e++ {
		handle(e, g.heads[e], g.weights[e])
	}
}

func (dg *DynamicGraph) NumVertices() Index {
	g := dg.g.Load()
	return Index(len(g.vertices))
}

func (dg *DynamicGraph) GetSegmentLength(u Index) float64 {
	g := dg.g.Load()
	return g.lengths[u]
}

func (dg *DynamicGraph) GetSegmentSpeed(u Index) float64 {
	g := dg.g.Load()
	return g.speeds[u]
}

func (dg *DynamicGraph) GetSegmentGeometry(u Index) []Coordinate {
	g := dg.g.Load()
	return g.segmentGeometry[u]
}

func (dg *DynamicGraph) GetRoadNetworkSegmentId(u Index) Index {
	g := dg.g.Load()
	return g.vertices[u].rnId
}

func (dg *DynamicGraph) Rebuild(r io.Reader) error {
	sr := util.NewBinaryReader(s2.NewReader(r))

	n, err := sr.Length()
	if err != nil {
		return fmt.Errorf("DynamicGraph.Rebuild: failed to read length of segments %v", err)
	}
	vertices := make([]ClientVertex, n)
	segmentGeometry := make([][]Coordinate, n)
	segmentSpeeds := make([]float64, n)
	segmentLengths := make([]float64, n)

	vm := make(map[Index]uint32, n)
	for i := uint32(0); i < n; i++ {
		rnId, err := sr.Uint32()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment road network id %v", err)
		}
		speed, err := sr.Float64()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment speed %v", err)
		}
		length, err := sr.Float64()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment length %v", err)
		}
		gpoly, err := sr.String()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment geometry polyline %v", err)
		}
		geometry, _, err := polyline.DecodeCoords([]byte(gpoly))
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to decode segment geometry polyline %v", err)
		}
		geometryCoords := NewCoordinates(geometry)
		vm[Index(rnId)] = i
		segmentGeometry[i] = geometryCoords
		segmentLengths[i] = length
		segmentSpeeds[i] = speed
		vertices[i] = NewClientVertex(Index(rnId))
	}

	nt, err := sr.Uint32()
	if err != nil {
		return fmt.Errorf("DynamicGraph.Rebuild: failed to read length of turns %v", err)
	}
	tails := make([]Index, nt)
	heads := make([]Index, nt)
	weights := make([]uint32, nt)
	for i := uint32(0); i < nt; i++ {
		weight, err := sr.Uint32()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read edge weight %v", err)
		}
		u, err := sr.Uint32()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read edge tail vertex %v", err)
		}
		v, err := sr.Uint32()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read edge head vertex %v", err)
		}
		vId := vm[Index(u)]
		if vertices[vId].firstOut == math.MaxUint32 {
			vertices[vId].firstOut = Index(i)
		}
		tails[i] = Index(u)
		heads[i] = Index(v)
		weights[i] = weight
	}

	g := &AdjacencyArray{}
	g.weights = weights
	g.heads = heads
	g.vertices = vertices
	g.speeds = segmentSpeeds
	g.lengths = segmentLengths
	g.segmentGeometry = segmentGeometry
	dg.g.Store(g)
	return nil
}

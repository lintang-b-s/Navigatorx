package datastructure

import (
	"bytes"
	"fmt"
	"sort"
	"sync/atomic"

	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/twpayne/go-polyline"
)

// ClientVertex  edge-based graph node. road segment. geometry is the geometry of the road segment
type ClientVertex struct {
	rnId     Index // id dari vertex (edge-based graph) di routing engine graph.go
	firstOut Index // firstOut index dari edge pertama dari vertex ini (edge yang tailnya vertex ini).

}

func NewClientVertex(rnId Index) ClientVertex {
	return ClientVertex{
		rnId:     rnId,
		firstOut: INVALID_SEGMENT_ID,
	}
}

// DynamicGraph read MapDataManager in this article: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
type DynamicGraph struct {
	g atomic.Pointer[AdjacencyArray]
}

func NewDynamicGraph() *DynamicGraph {
	dg := &DynamicGraph{}
	dg.g.Store(&AdjacencyArray{})
	return dg
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

func (dg *DynamicGraph) Reset() {
	dg.g.Store(&AdjacencyArray{})
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

func (dg *DynamicGraph) GetGraphSegmentId(uRnId Index) Index {
	g := dg.g.Load()
	n := len(g.vertices)

	u := sort.Search(n, func(i int) bool { // g.vertices already sorted by its rnId in ascending order (see map_attributes_engine.go GetMapAttributes)  O(log(n))
		return g.vertices[i].rnId >= uRnId
	})
	return Index(u)
}

func (dg *DynamicGraph) Rebuild(buf []byte) error {
	r := bytes.NewBuffer(buf)
	sr := s2.NewReader(r)
	br := util.NewBinaryReader(sr)

	n, err := br.Length()
	if err != nil {
		return fmt.Errorf("DynamicGraph.Rebuild: failed to read length of segments %v", err)
	}
	vertices := make([]ClientVertex, n)
	segmentGeometry := make([][]Coordinate, n)
	segmentSpeeds := make([]float64, n)
	segmentLengths := make([]float64, n)

	// see map_attributes_engine.go GetMapAttributes
	vm := make(map[Index]uint32, n)
	for u := uint32(0); u < n; u++ {
		rnId, err := br.Uint32()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment road network id %v", err)
		}
		speed, err := br.Float64()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment speed %v", err)
		}
		length, err := br.Float64()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment length %v", err)
		}
		gpoly, err := br.String()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read segment geometry polyline %v", err)
		}
		geometry, _, err := polyline.DecodeCoords([]byte(gpoly))
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to decode segment geometry polyline %v", err)
		}
		geometryCoords := NewCoordinates(geometry)
		vm[Index(rnId)] = u
		segmentGeometry[u] = geometryCoords
		segmentLengths[u] = length
		segmentSpeeds[u] = speed
		vertices[u] = NewClientVertex(Index(rnId))
	}

	// vertices already sorted by its rnId
	nt, err := br.Uint32()
	if err != nil {
		return fmt.Errorf("DynamicGraph.Rebuild: failed to read length of turns %v", err)
	}
	tails := make([]Index, nt)
	heads := make([]Index, nt)
	weights := make([]uint32, nt)
	outDegs := make([]Index, n)
	for i := uint32(0); i < nt; i++ {
		weight, err := br.Uint32()
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read edge weight %v", err)
		}
		u, err := br.Uint32() // road network (or routing engine) segment id for tail u
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read edge tail vertex %v", err)
		}
		v, err := br.Uint32() // road network (or routing engine) segment id for head v
		if err != nil {
			return fmt.Errorf("DynamicGraph.Rebuild: failed to read edge head vertex %v", err)
		}
		uId, ok := (vm[Index(u)])
		if !ok {
			return fmt.Errorf("DynamicGraph.Rebuild: road network segment id %v should have mapped to dynamic graph segment id", u)
		}
		vId, ok := (vm[Index(v)])
		if !ok {
			return fmt.Errorf("DynamicGraph.Rebuild: road network segment id %v should have mapped to dynamic graph segment id", v)
		}

		tails[i] = Index(uId)
		heads[i] = Index(vId)
		weights[i] = weight
		outDegs[uId]++
	}

	eo := Index(0)
	for u := Index(0); u < Index(n); u++ {
		vertices[u].firstOut = eo
		eo += outDegs[u]
	}

	dummy := NewClientVertex(INVALID_SEGMENT_ID)
	dummy.firstOut = Index(nt)
	segmentGeometry = append(segmentGeometry, make([]Coordinate, 0))
	segmentLengths = append(segmentLengths, 0)
	segmentSpeeds = append(segmentSpeeds, 0)

	vertices = append(vertices, dummy) // last dummy vertex

	// sort edges (u,v) by tail vertex u
	ePerm := make([]int, nt) // map from new edge id to old edge id
	for i := 0; i < int(nt); i++ {
		ePerm[i] = i
	}
	sort.Slice(ePerm, func(i, j int) bool {
		return tails[ePerm[i]] < tails[ePerm[j]]
	})
	heads = util.ApplyPermutation(heads, ePerm)
	weights = util.ApplyPermutation(weights, ePerm)

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

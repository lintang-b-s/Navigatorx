package tiler

import (
	"bytes"
	"testing"

	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

func TestWriteEdgeBinaryRoundTrip(t *testing.T) {
	vertices := []da.Vertex{
		da.NewVertex(-6.1, 106.8, 0),
		da.NewVertex(-6.2, 106.9, 1),
		da.NewVertex(0, 0, 2),
	}
	vertices[0].SetFirstOut(0)
	vertices[1].SetFirstOut(1)
	vertices[2].SetFirstOut(1)
	vertices[0].SetFirstIn(0)
	vertices[1].SetFirstIn(0)
	vertices[2].SetFirstIn(1)

	graph := da.NewGraph(
		vertices,
		[]da.OutEdge{da.NewOutEdge(0, 1, 0, 0)},
		[]da.InEdge{da.NewInEdge(0, 0, 0, 0)},
		nil,
		true,
		nil,
	)
	geometry := []da.Coordinate{
		da.NewFixedCoordinate(-61000000, 1068000000),
		da.NewFixedCoordinate(-62000000, 1069000000),
	}
	storage := da.NewGraphStorage(8)
	storage.AppendOsmNodePoints(geometry)
	storage.AppendEdgeMetadata(1, 0, da.Index(len(geometry)), 0, 0, 0, 1)
	graph.SetGraphStorage(storage)

	timeFunction := costfunction.NewTimeCostFunction(false, []float64{4}, []float64{12.5}, nil, nil)
	engine := NewTilingEngine(graph, zap.NewNop(), timeFunction)

	var tile bytes.Buffer
	compressed := s2.NewWriter(&tile)
	if err := engine.writeEdge(util.NewBinaryWriter(compressed), 0); err != nil {
		t.Fatal(err)
	}
	if err := compressed.Close(); err != nil {
		t.Fatal(err)
	}

	mapMatchGraph := da.InitializeMapMatchingGraph(graph.NumberOfVertices())
	if err := mapMatchGraph.RebuildMapMatchGraphFromReader(bytes.NewReader(tile.Bytes())); err != nil {
		t.Fatal(err)
	}
	if mapMatchGraph.NumofEdges() != 1 {
		t.Fatalf("NumofEdges() = %d, want 1", mapMatchGraph.NumofEdges())
	}
	edge := mapMatchGraph.GetOutEdge(0)
	if edge.GetRoadNetworkEdgeId() != 0 || edge.GetTail() != 0 || edge.GetHead() != 1 || edge.GetLength() != 12.5 {
		t.Fatalf("edge = id %d, tail %d, head %d, length %v", edge.GetRoadNetworkEdgeId(), edge.GetTail(), edge.GetHead(), edge.GetLength())
	}
	if len(edge.GetGeometry()) != len(geometry) {
		t.Fatalf("geometry length = %d, want %d", len(edge.GetGeometry()), len(geometry))
	}
	for i, coordinate := range edge.GetGeometry() {
		if coordinate != geometry[i] {
			t.Fatalf("geometry[%d] = %+v, want %+v", i, coordinate, geometry[i])
		}
	}
}

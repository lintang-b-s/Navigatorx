package datastructure

import (
	"bytes"
	"strings"
	"testing"

	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type mapMatchTileRecord struct {
	id       Index
	tail     Index
	head     Index
	length   float64
	geometry []Coordinate
}

func writeMapMatchTileRecord(t *testing.T, w *util.BinaryWriter, record mapMatchTileRecord) {
	t.Helper()
	if err := w.Uint32(uint32(record.id)); err != nil {
		t.Fatal(err)
	}
	if err := w.Uint32(uint32(record.tail)); err != nil {
		t.Fatal(err)
	}
	if err := w.Uint32(uint32(record.head)); err != nil {
		t.Fatal(err)
	}
	if err := w.Float64(record.length); err != nil {
		t.Fatal(err)
	}
	if err := w.Length(len(record.geometry)); err != nil {
		t.Fatal(err)
	}
	for _, coordinate := range record.geometry {
		if err := w.Int32(coordinate.GetFixedLat()); err != nil {
			t.Fatal(err)
		}
		if err := w.Int32(coordinate.GetFixedLon()); err != nil {
			t.Fatal(err)
		}
	}
}

func compressedMapMatchTile(t *testing.T, writePayload func(*util.BinaryWriter)) []byte {
	t.Helper()
	var output bytes.Buffer
	writer := s2.NewWriter(&output)
	writePayload(util.NewBinaryWriter(writer))
	if err := writer.Close(); err != nil {
		t.Fatal(err)
	}
	return output.Bytes()
}

func TestMapMatchingGraphRebuildBinaryTile(t *testing.T) {
	edgeWithHigherTail := mapMatchTileRecord{
		id:       9,
		tail:     2,
		head:     3,
		length:   12.5,
		geometry: []Coordinate{NewFixedCoordinate(-61234567, 1068123456)},
	}
	edgeWithLowerTail := mapMatchTileRecord{
		id:       7,
		tail:     0,
		head:     1,
		length:   3.75,
		geometry: []Coordinate{NewFixedCoordinate(-61000000, 1068000000), NewFixedCoordinate(-61100000, 1068100000)},
	}
	tile := compressedMapMatchTile(t, func(w *util.BinaryWriter) {
		writeMapMatchTileRecord(t, w, edgeWithHigherTail)
		writeMapMatchTileRecord(t, w, edgeWithLowerTail)
	})

	graph := InitializeMapMatchingGraph(4)
	if err := graph.RebuildMapMatchGraphFromReader(bytes.NewReader(tile)); err != nil {
		t.Fatal(err)
	}

	if graph.NumofEdges() != 2 {
		t.Fatalf("NumofEdges() = %d, want 2", graph.NumofEdges())
	}
	edges := graph.GetEdges()
	if edges[0].GetRoadNetworkEdgeId() != edgeWithLowerTail.id || edges[1].GetRoadNetworkEdgeId() != edgeWithHigherTail.id {
		t.Fatalf("edge order = [%d %d], want [%d %d]", edges[0].GetRoadNetworkEdgeId(), edges[1].GetRoadNetworkEdgeId(), edgeWithLowerTail.id, edgeWithHigherTail.id)
	}
	if edges[0].GetHead() != edgeWithLowerTail.head || edges[0].GetLength() != edgeWithLowerTail.length {
		t.Fatalf("first edge = head %d, length %v", edges[0].GetHead(), edges[0].GetLength())
	}
	if len(edges[0].GetGeometry()) != len(edgeWithLowerTail.geometry) {
		t.Fatalf("geometry length = %d, want %d", len(edges[0].GetGeometry()), len(edgeWithLowerTail.geometry))
	}
	for i, coordinate := range edges[0].GetGeometry() {
		if coordinate != edgeWithLowerTail.geometry[i] {
			t.Fatalf("geometry[%d] = %+v, want %+v", i, coordinate, edgeWithLowerTail.geometry[i])
		}
	}
	if localID, ok := graph.GetMapMatchEdgeId(edgeWithLowerTail.id); !ok || localID != 0 {
		t.Fatalf("GetMapMatchEdgeId(%d) = %d, %v, want 0, true", edgeWithLowerTail.id, localID, ok)
	}
	if localID, ok := graph.GetMapMatchEdgeId(edgeWithHigherTail.id); !ok || localID != 1 {
		t.Fatalf("GetMapMatchEdgeId(%d) = %d, %v, want 1, true", edgeWithHigherTail.id, localID, ok)
	}
}

func TestMapMatchingGraphRebuildConsumesDuplicateRecord(t *testing.T) {
	duplicateID := mapMatchTileRecord{
		id:       4,
		tail:     0,
		head:     1,
		length:   1,
		geometry: []Coordinate{NewFixedCoordinate(1, 2)},
	}
	tile := compressedMapMatchTile(t, func(w *util.BinaryWriter) {
		writeMapMatchTileRecord(t, w, duplicateID)
		duplicateID.geometry = []Coordinate{NewFixedCoordinate(3, 4), NewFixedCoordinate(5, 6)}
		writeMapMatchTileRecord(t, w, duplicateID)
		writeMapMatchTileRecord(t, w, mapMatchTileRecord{id: 5, tail: 1, head: 2, length: 2})
	})

	graph := InitializeMapMatchingGraph(3)
	if err := graph.RebuildMapMatchGraphFromReader(bytes.NewReader(tile)); err != nil {
		t.Fatal(err)
	}
	if graph.NumofEdges() != 2 {
		t.Fatalf("NumofEdges() = %d, want 2", graph.NumofEdges())
	}
	if _, ok := graph.GetMapMatchEdgeId(5); !ok {
		t.Fatal("edge after duplicate record was not read")
	}
}

func TestMapMatchingGraphRebuildEmptyBinaryTile(t *testing.T) {
	tile := compressedMapMatchTile(t, func(*util.BinaryWriter) {})
	graph := InitializeMapMatchingGraph(2)
	if err := graph.RebuildMapMatchGraphFromReader(bytes.NewReader(tile)); err != nil {
		t.Fatal(err)
	}
	if graph.NumofEdges() != 0 {
		t.Fatalf("NumofEdges() = %d, want 0", graph.NumofEdges())
	}
}

func TestMapMatchingGraphRebuildRejectsTruncatedBinaryRecord(t *testing.T) {
	tests := []struct {
		name         string
		writePayload func(*util.BinaryWriter)
		wantError    string
	}{
		{
			name: "edge ID",
			writePayload: func(w *util.BinaryWriter) {
				if err := w.Bytes([]byte{1}); err != nil {
					t.Fatal(err)
				}
			},
			wantError: "edge ID",
		},
		{
			name: "header",
			writePayload: func(w *util.BinaryWriter) {
				if err := w.Uint32(1); err != nil {
					t.Fatal(err)
				}
			},
			wantError: "tail",
		},
		{
			name: "geometry count",
			writePayload: func(w *util.BinaryWriter) {
				if err := w.Uint32(1); err != nil {
					t.Fatal(err)
				}
				if err := w.Uint32(0); err != nil {
					t.Fatal(err)
				}
				if err := w.Uint32(1); err != nil {
					t.Fatal(err)
				}
				if err := w.Float64(1); err != nil {
					t.Fatal(err)
				}
			},
			wantError: "geometry count",
		},
		{
			name: "coordinate payload",
			writePayload: func(w *util.BinaryWriter) {
				if err := w.Uint32(1); err != nil {
					t.Fatal(err)
				}
				if err := w.Uint32(0); err != nil {
					t.Fatal(err)
				}
				if err := w.Uint32(1); err != nil {
					t.Fatal(err)
				}
				if err := w.Float64(1); err != nil {
					t.Fatal(err)
				}
				if err := w.Uint32(1); err != nil {
					t.Fatal(err)
				}
				if err := w.Int32(2); err != nil {
					t.Fatal(err)
				}
			},
			wantError: "longitude",
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tile := compressedMapMatchTile(t, tt.writePayload)
			graph := InitializeMapMatchingGraph(2)
			err := graph.RebuildMapMatchGraphFromReader(bytes.NewReader(tile))
			if err == nil || !strings.Contains(err.Error(), tt.wantError) {
				t.Fatalf("RebuildMapMatchGraphFromReader() error = %v, want containing %q", err, tt.wantError)
			}
		})
	}
}

package datastructure

import (
	"bufio"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/bits-and-blooms/bitset"
	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg"
)

func TestGraphIO_MinResolutionRoundTrip(t *testing.T) {
	t.Parallel()

	vertices := []Vertex{
		NewVertex(0.0, 0.0, 0),
		NewVertex(0.0, 0.0, 1),
	}
	vertices[0].SetFirstOut(0)
	vertices[0].SetFirstIn(0)
	vertices[1].SetFirstOut(1)
	vertices[1].SetFirstIn(1)

	outEdges := []OutEdge{
		NewOutEdge(0, 0, 1, 1, 0, 0),
	}
	inEdges := []InEdge{
		NewInEdge(0, 0, 1, 1, 0, 0),
	}
	verticesOsmIDs := NewPackedSlice(BIT_SIZE_OSM_NODE_ID, 1)
	verticesOsmIDs.Append(1)
	verticesOsmIDs.Append(0)

	g := NewGraph(vertices, outEdges, inEdges, []pkg.TurnType{pkg.NONE}, true, verticesOsmIDs)
	g.SetCellNumbers([]Pv{0})
	g.SetOverlayMapping(map[SubVertex]Index{})
	g.maxEdgesInCell = 1
	g.outEdgeCellOffset = []Index{0}
	g.inEdgeCellOffset = []Index{0}
	g.SetSCCs([]Index{0})
	g.SetSCCCondensationAdj([][]Index{{}})
	g.SetBoundingBox(NewBoundingBox(0, 0, 0, 0))
	g.SetMinResolution(12.3456789)

	gs := BuildGraphStorage(
		[]Coordinate{},
		bitset.New(1),
		bitset.New(1),
		bitset.New(1),
		bitset.New(1),
		bitset.New(1),
	)
	gs.SetNewEdgeMetadata(
		NewPackedSlice(1, 1),
		[]Index{},
		[]Index{},
		[]uint32{},
		[]pkg.OsmHighwayType{},
		[]pkg.OsmHighwayType{},
		[]uint8{},
	)
	gs.BuildNameTable(map[uint32]string{})
	g.SetGraphStorage(gs)

	tmpFile := filepath.Join(t.TempDir(), "graph.s2")
	if err := g.WriteGraph(tmpFile); err != nil {
		t.Fatalf("WriteGraph failed: %v", err)
	}

	got, err := ReadGraph(tmpFile)
	if err != nil {
		t.Fatalf("ReadGraph failed: %v", err)
	}

	if got.GetMinResolution() != g.GetMinResolution() {
		t.Fatalf("minResolution mismatch: got=%v want=%v", got.GetMinResolution(), g.GetMinResolution())
	}
}

func TestReadGraph_RejectsOldHeaderFormat(t *testing.T) {
	t.Parallel()

	tmpFile := filepath.Join(t.TempDir(), "old_header.s2")
	f, err := os.Create(tmpFile)
	if err != nil {
		t.Fatalf("create temp file: %v", err)
	}

	w := s2.NewWriter(f)
	bw := bufio.NewWriter(w)
	if _, err := bw.WriteString("2 1 1 0 true\n"); err != nil {
		t.Fatalf("write old header: %v", err)
	}
	if err := bw.Flush(); err != nil {
		t.Fatalf("flush: %v", err)
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close s2 writer: %v", err)
	}
	if err := f.Close(); err != nil {
		t.Fatalf("close file: %v", err)
	}

	_, err = ReadGraph(tmpFile)
	if err == nil {
		t.Fatalf("expected error for old 5-field header, got nil")
	}
	if !strings.Contains(err.Error(), "expected: 6") {
		t.Fatalf("unexpected error: %v", err)
	}
}

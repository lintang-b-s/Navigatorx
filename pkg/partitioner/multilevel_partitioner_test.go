package partitioner

import (
	"path/filepath"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"go.uber.org/zap"
)

func TestMultilevelPartitionerSerialization(t *testing.T) {
	tmpDir := t.TempDir()
	filePath := filepath.Join(tmpDir, "test.mlp")

	vertices := []da.Vertex{
		da.NewVertex(1, 0, 0),
		da.NewVertex(2, 0, 0),
		da.NewVertex(3, 0, 0),
		da.NewVertex(4, 0, 0),
		da.NewVertex(5, 0, 0),
	}
	heads := []da.Index{1, 2, 3, 0}
	tails := []da.Index{0, 1, 2, 3}
	graph := da.NewGraph(vertices, heads, tails, true, []da.Index{0, 0, 0, 0}, []da.Index{0, 0, 0, 0})

	logger := zap.NewNop()
	mp := NewMultilevelPartitioner([]int{4, 2}, 2, 1, graph, logger, false)
	mp.cellVertices[0] = [][]da.Index{{0, 1}, {2, 3}}
	mp.cellVertices[1] = [][]da.Index{{0, 1, 2, 3}}

	if err := mp.writeMLPToFile(filePath); err != nil {
		t.Fatalf("writeMLPToFile failed: %v", err)
	}

	loaded, err := mp.ReadMLPFromFile(filePath)
	if err != nil {
		t.Fatalf("ReadMLPFromFile failed: %v", err)
	}

	if loaded.GetNumberOfLevels() != 2 {
		t.Fatalf("expected 2 levels, got %d", loaded.GetNumberOfLevels())
	}
	if loaded.GetNumberOfVertices() != 4 {
		t.Fatalf("expected 4 vertices, got %d", loaded.GetNumberOfVertices())
	}
}

package datastructure

import (
	"path/filepath"
	"testing"
)

func TestMultilevelPartitionSerialization(t *testing.T) {
	tmpDir := t.TempDir()
	filePath := filepath.Join(tmpDir, "partition.mlp")

	numCells := []uint32{4, 2}
	cellNumbers := []Pv{0, 1, 2, 3, 4, 5}
	pvOffset := []uint8{0, 2, 3}

	mlp := NewMultilevelPartition(cellNumbers, numCells, pvOffset)

	if err := mlp.WriteToFile(filePath); err != nil {
		t.Fatalf("WriteToFile failed: %v", err)
	}

	loaded, err := ReadMultilevelPartitionFromFile(filePath)
	if err != nil {
		t.Fatalf("ReadMultilevelPartitionFromFile failed: %v", err)
	}

	if loaded.GetNumberOfLevels() != mlp.GetNumberOfLevels() {
		t.Fatalf("level count mismatch: got %d, want %d", loaded.GetNumberOfLevels(), mlp.GetNumberOfLevels())
	}

	if loaded.GetNumberOfVertices() != mlp.GetNumberOfVertices() {
		t.Fatalf("vertices count mismatch: got %d, want %d", loaded.GetNumberOfVertices(), mlp.GetNumberOfVertices())
	}

	for l := 0; l < mlp.GetNumberOfLevels(); l++ {
		if loaded.GetNumberOfCellsInLevel(l) != mlp.GetNumberOfCellsInLevel(l) {
			t.Errorf("level %d cell count mismatch: got %d, want %d", l, loaded.GetNumberOfCellsInLevel(l), mlp.GetNumberOfCellsInLevel(l))
		}
	}

	for i := 0; i < mlp.GetNumberOfVertices(); i++ {
		if loaded.GetCellNumber(Index(i)) != mlp.GetCellNumber(Index(i)) {
			t.Errorf("vertex %d cell number mismatch: got %d, want %d", i, loaded.GetCellNumber(Index(i)), mlp.GetCellNumber(Index(i)))
		}
	}
}

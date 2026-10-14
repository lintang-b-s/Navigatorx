package datastructure

import (
	"path/filepath"
	"testing"
)

func TestSparseMatrixSerialization(t *testing.T) {
	tmpDir := t.TempDir()
	filePath := filepath.Join(tmpDir, "matrix.csr")

	matrix := NewSparseMatrix(5, 5, 0, func(a, b uint32) bool { return a == b })
	matrix.Set(10, 0, 1)
	matrix.Set(20, 1, 3)
	matrix.Set(30, 4, 4)

	if err := matrix.WriteToFile(filePath); err != nil {
		t.Fatalf("WriteToFile failed: %v", err)
	}

	loaded, err := ReadSparseMatrixFromFile(filePath, 0, func(a, b uint32) bool { return a == b })
	if err != nil {
		t.Fatalf("ReadSparseMatrixFromFile failed: %v", err)
	}

	if loaded.m != matrix.m || loaded.n != matrix.n {
		t.Fatalf("matrix dimensions mismatch: got (%d, %d), want (%d, %d)", loaded.m, loaded.n, matrix.m, matrix.n)
	}

	for r := 0; r < 5; r++ {
		for c := 0; c < 5; c++ {
			expected := matrix.Get(r, c)
			actual := loaded.Get(r, c)
			if expected != actual {
				t.Errorf("mismatch at (%d, %d): got %d, want %d", r, c, actual, expected)
			}
		}
	}
}

func TestSparseMatrixNonExistentFile(t *testing.T) {
	matrix, err := ReadSparseMatrixFromFile("non_existent_file.csr", 0, func(a, b uint32) bool { return a == b })
	if err != nil {
		t.Fatalf("expected nil error for non-existent file, got %v", err)
	}
	if matrix.m != 0 || matrix.n != 0 {
		t.Fatalf("expected empty matrix for non-existent file")
	}
}

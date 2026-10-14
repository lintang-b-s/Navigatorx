package datastructure

import (
	"fmt"
	"os"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
https://netlib.org/linalg/html_templates/node91.html#SECTION00931100000000000000

The Compressed Row Storage (CRS) format puts the subsequent nonzeros of the matrix rows in contiguous memory locations.
Assuming we have a nonsymmetric sparse matrix A with n rows and n columns,
 we create  vectors: one for floating-point numbers (val), and the other two for integers (col_ind, row_ptr).
The val vector stores the values of the nonzero elements of the matrix A, as they are traversed in a row-wise fashion.
The col_ind vector stores the column indexes of the elements in the val vector.
if val(k)=a_{i,j} then col_ind(k)=j
The row_ptr vector stores the locations in the val vector that start a row
if val(k)=a_{a,j} then row_ptr(i) \leq k < row_ptr(i+1)
by convention, we define row_ptr(n+1)=nnz+1, where nnz is the number of nonzeros in the matrix A.
Instead of storing O(n^2) elements, we need only O(2nnz+n+1) space
*/

type SparseMatrix struct {
	m, n int
	vals []uint32
	cols []uint32
	rows []uint32
	zero uint32
	eq   func(a, b uint32) bool
}

func NewSparseMatrix(m, n int, zero uint32, eq func(a, b uint32) bool) *SparseMatrix {
	rows := make([]uint32, m+1)
	for i := range rows {
		rows[i] = 1
	}

	if eq == nil {
		eq = func(a, b uint32) bool { return a == b }
	}

	return &SparseMatrix{
		m:    m,
		n:    n,
		rows: rows,
		zero: zero,
		eq:   eq,
	}
}

func (sm *SparseMatrix) Set(val uint32, row, col int) {
	row += 1 // 1-based
	col += 1

	pos := sm.rows[row-1] - 1
	currCol := uint32(0)

	for ; pos < sm.rows[row]-1; pos++ {
		currCol = sm.cols[pos]
		if currCol >= uint32(col) {
			break
		}
	}

	if currCol != uint32(col) {
		if !sm.eq(val, sm.zero) {
			sm.insert(int(pos), row, col, val)
		}
	} else if sm.eq(val, sm.zero) {
		sm.remove(int(pos), row)
	} else {
		sm.vals[pos] = val
	}
}

func (sm *SparseMatrix) Get(row, col int) uint32 {
	row += 1 // 1-based
	col += 1

	var currCol uint32

	for pos := sm.rows[row-1] - 1; pos < sm.rows[row]-1; pos++ {
		currCol = sm.cols[pos]
		if currCol == uint32(col) {
			return sm.vals[pos]
		} else if currCol > uint32(col) {
			break
		}
	}

	return sm.zero
}

func (sm *SparseMatrix) insert(index, row, col int, val uint32) {
	if sm.vals == nil {
		sm.vals = make([]uint32, 1)
		sm.vals[0] = val
		sm.cols = make([]uint32, 1)
		sm.cols[0] = uint32(col)
	} else {
		sm.vals = append(sm.vals[:index], append([]uint32{val}, sm.vals[index:]...)...)
		sm.cols = append(sm.cols[:index], append([]uint32{uint32(col)}, sm.cols[index:]...)...)
	}

	for i := row; i <= sm.m; i++ {
		sm.rows[i] += 1
	}
}

func (sm *SparseMatrix) remove(index, row int) {
	sm.vals = append(sm.vals[:index], sm.vals[index+1:]...)
	sm.cols = append(sm.cols[:index], sm.cols[index+1:]...)

	for i := row; i < sm.m; i++ {
		sm.rows[i] -= 1
	}
}

func (sm *SparseMatrix) WriteToFile(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {
		if err := w.Length(sm.m); err != nil {
			return err
		}
		if err := w.Length(sm.n); err != nil {
			return err
		}
		if err := w.WriteUint32s(sm.vals); err != nil {
			return err
		}
		if err := w.WriteUint32s(sm.cols); err != nil {
			return err
		}
		return w.WriteUint32s(sm.rows)
	})
}

func ReadSparseMatrixFromFile(filename string, zero uint32, eq func(a, b uint32) bool) (*SparseMatrix, error) {
	file, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		if os.IsNotExist(err) {
			return NewSparseMatrix(0, 0, zero, eq), nil
		}
		return nil, fmt.Errorf("ReadSparseMatrixFromFile: failed to open file %s: %w", filename, err)
	}
	defer file.Close()

	m, err := r.Length()
	if err != nil {
		return nil, fmt.Errorf("ReadSparseMatrixFromReader: failed to read m: %w", err)
	}
	n, err := r.Length()
	if err != nil {
		return nil, fmt.Errorf("ReadSparseMatrixFromReader: failed to read n: %w", err)
	}

	vals, err := r.ReadUint32s()
	if err != nil {
		return nil, fmt.Errorf("ReadSparseMatrixFromReader: failed to read vals: %w", err)
	}
	cols, err := r.ReadUint32s()
	if err != nil {
		return nil, fmt.Errorf("ReadSparseMatrixFromReader: failed to read cols: %w", err)
	}
	rows, err := r.ReadUint32s()
	if err != nil {
		return nil, fmt.Errorf("ReadSparseMatrixFromReader: failed to read rows: %w", err)
	}

	sm := NewSparseMatrix(int(m), int(n), zero, eq)
	sm.vals = vals
	sm.cols = cols
	sm.rows = rows

	return sm, nil
}

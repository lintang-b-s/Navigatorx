package datastructure

import (
	"bufio"
	"fmt"
	"os"
	"strconv"

	"github.com/dsnet/compress/bzip2"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"golang.org/x/exp/constraints"
)

/*
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

type SparseMatrix[T constraints.Integer | constraints.Float] struct {
	m, n int
	vals []T
	cols []int
	rows []int
	zero T
	eq   func(a, b T) bool
}

func NewSparseMatrix[T constraints.Integer | constraints.Float](m, n int, zero T, eq func(a, b T) bool) *SparseMatrix[T] {
	rows := make([]int, m+1)
	for i := range rows {
		rows[i] = 1
	}

	return &SparseMatrix[T]{
		m:    m,
		n:    n,
		rows: rows,
		zero: zero,
		eq:   eq,
	}
}

func (sm *SparseMatrix[T]) Set(val T, row, col int) {
	row += 1 // 1-based
	col += 1

	pos := sm.rows[row-1] - 1
	currCol := 0

	for ; pos < sm.rows[row]-1; pos++ {
		currCol = sm.cols[pos]
		if currCol >= col {
			break
		}
	}

	if currCol != col {
		if !sm.eq(val, sm.zero) {
			sm.insert(pos, row, col, val)
		}
	} else if sm.eq(val, sm.zero) {
		sm.remove(pos, row)
	} else {
		sm.vals[pos] = val
	}
}

func (sm *SparseMatrix[T]) Get(row, col int) T {
	row += 1 // 1-based
	col += 1

	var currCol int

	for pos := sm.rows[row-1] - 1; pos < sm.rows[row]-1; pos++ {
		currCol = sm.cols[pos]
		if currCol == col {
			return sm.vals[pos]
		} else if currCol > col {
			break
		}
	}

	return sm.zero
}

func (sm *SparseMatrix[T]) insert(index, row, col int, val T) {
	if sm.vals == nil {
		sm.vals = make([]T, 1)
		sm.vals[0] = val
		sm.cols = make([]int, 1)
		sm.cols[0] = col
	} else {
		sm.vals = append(sm.vals[:index], append([]T{val}, sm.vals[index:]...)...)
		sm.cols = append(sm.cols[:index], append([]int{col}, sm.cols[index:]...)...)
	}

	for i := row; i <= sm.m; i++ {
		sm.rows[i] += 1
	}
}

func (sm *SparseMatrix[T]) remove(index, row int) {
	sm.vals = append(sm.vals[:index], sm.vals[index+1:]...)
	sm.cols = append(sm.cols[:index], sm.cols[index+1:]...)

	for i := row; i < sm.m; i++ {
		sm.rows[i] -= 1
	}
}

func (sm *SparseMatrix[T]) WriteToFile(filename string) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	bz, err := bzip2.NewWriter(f, &bzip2.WriterConfig{})
	if err != nil {
		return err
	}
	defer bz.Close()

	w := bufio.NewWriter(bz)

	fmt.Fprintf(w, "%d %d %d %d %d\n", sm.m, sm.n, len(sm.vals), len(sm.cols), len(sm.rows))

	for i := 0; i < len(sm.vals); i++ {
		fmt.Fprintf(w, "%v", sm.vals[i])
		if i < len(sm.vals)-1 {
			fmt.Fprintf(w, " ")
		}
	}
	fmt.Fprintf(w, "\n")

	for i := 0; i < len(sm.cols); i++ {
		fmt.Fprintf(w, "%v", sm.cols[i])
		if i < len(sm.cols)-1 {
			fmt.Fprintf(w, " ")
		}
	}
	fmt.Fprintf(w, "\n")

	for i := 0; i < len(sm.rows); i++ {
		fmt.Fprintf(w, "%v", sm.rows[i])
		if i < len(sm.rows)-1 {
			fmt.Fprintf(w, " ")
		}
	}
	fmt.Fprintf(w, "\n")

	return w.Flush()
}

func ReadSparseMatrixFromFile[T constraints.Integer | constraints.Float](filename string,
	zero T, eq func(a, b T) bool) (*SparseMatrix[T], error) {

	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}

	defer f.Close()

	bz, err := bzip2.NewReader(f, nil)

	if err != nil {
		return nil, err
	}

	br := bufio.NewReader(bz)

	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens := fields(line)
	if len(tokens) != 5 {
		return nil, err
	}
	m := parseInt(tokens[0])
	n := parseInt(tokens[1])
	valsLen := parseInt(tokens[2])
	colsLen := parseInt(tokens[3])
	rowsLen := parseInt(tokens[4])

	sm := NewSparseMatrix[T](m, n, zero, eq)
	sm.vals = make([]T, valsLen)
	sm.cols = make([]int, colsLen)
	sm.rows = make([]int, rowsLen)

	// vals
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	if len(tokens) != valsLen {
		return nil, err
	}

	for i := 0; i < valsLen; i++ {
		token := tokens[i]
		if intVal, err := strconv.Atoi(token); err == nil {
			sm.vals[i] = any(intVal).(T)
			continue
		}

		if floatVal, err := strconv.ParseFloat(token, 64); err == nil {
			sm.vals[i] = any(floatVal).(T)
			continue
		}

	}

	// cols
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	if len(tokens) != colsLen {
		return nil, err
	}

	for i := 0; i < colsLen; i++ {
		token := tokens[i]

		val := parseInt(token)
		sm.cols[i] = val
	}

	// rows
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	if len(tokens) != rowsLen {
		return nil, err
	}

	for i := 0; i < rowsLen; i++ {
		token := tokens[i]

		val := parseInt(token)
		sm.rows[i] = val
	}

	return sm, nil
}

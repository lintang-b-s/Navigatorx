package datastructure

import (
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// MultilevelPartition adapted from: https://github.com/michaelwegner/CRP/blob/master/datastructures/MultiLevelPartition.h
// https://github.com/michaelwegner/CRP/blob/master/datastructures/MultiLevelPartition.cpp
// MultilevelPartition stores every cell information of each vertex on every level.
type MultilevelPartition struct {
	numCells    []uint32 // number of cells in the level-index overlay graph
	pvOffset    []uint8  // offset of each level in the bitpacked cell numbers
	cellNumbers []Pv
}

// NewPlainMLP creates and returns plain MultilevelPartition.
func NewPlainMLP() *MultilevelPartition {
	return &MultilevelPartition{}
}

// NewMultilevelPartition creates and returns multilevel partition.
func NewMultilevelPartition(cellNumbers []Pv, numCells []uint32, pvOffset []uint8) *MultilevelPartition {

	return &MultilevelPartition{
		numCells:    numCells,
		pvOffset:    pvOffset,
		cellNumbers: cellNumbers,
	}
}

// SetNumberOflevels sets number levels of multilevelpartition.
func (mp *MultilevelPartition) SetNumberOflevels(numLevels int) {
	mp.numCells = make([]uint32, numLevels)
}

// SetNumberOfVertices sets number of vertices.
func (mp *MultilevelPartition) SetNumberOfVertices(numVertices int) {
	mp.cellNumbers = make([]Pv, numVertices)
}

// SetNumberOfCellsInLevel sets number of cells in level.
func (mp *MultilevelPartition) SetNumberOfCellsInLevel(level int, numCells int) {
	mp.numCells[level] = uint32(numCells)
}

// ComputeBitmap computes bitmap.
func (mp *MultilevelPartition) ComputeBitmap() {
	mp.pvOffset = make([]uint8, len(mp.numCells)+1)
	for i := 0; i < len(mp.numCells); i++ {
		mp.pvOffset[i+1] = mp.pvOffset[i] + uint8(math.Ceil(math.Log2(float64(mp.numCells[i])))) // ceil(log2(numCells[i])) = number of bits needed to represent cell id in level-i
	}
}

// SetCell set cellNumber of vertex u in level=level to cellId
func (mp *MultilevelPartition) SetCell(level int, u int, cellId int) {
	mp.cellNumbers[u] |= Pv(cellId) << mp.pvOffset[level]
}

// GetCell returns cell Id at level=level of vertex u.
func (mp *MultilevelPartition) GetCell(level int, u int) Pv {
	// off the bits that in above level, then shift right to get the cell id in that level
	return (mp.cellNumbers[u] & ^(^Pv(0) << mp.pvOffset[level+1])) >> Pv(mp.pvOffset[level])
}

// GetNumberOfVertices returns number of vertices.
func (mp *MultilevelPartition) GetNumberOfVertices() int {
	return len(mp.cellNumbers)
}

// GetNumberOfLevels returns number of levels.
func (mp *MultilevelPartition) GetNumberOfLevels() int {
	return len(mp.numCells)
}

// GetNumberOfCellsInLevel returns number of cells in level.
func (mp *MultilevelPartition) GetNumberOfCellsInLevel(level int) int {
	return int(mp.numCells[level])
}

// GetPVOffsets returns offset of each level in the bitpacked cell numbers.
func (mp *MultilevelPartition) GetPVOffsets() []uint8 {
	return mp.pvOffset
}

// GetCellNumber returns cell number of vertex u.
func (mp *MultilevelPartition) GetCellNumber(u Index) Pv {
	return mp.cellNumbers[u]
}

// GetCellNumbers returns cell numbers.
func (mp *MultilevelPartition) GetCellNumbers() []Pv {
	return mp.cellNumbers
}

// SetCellNumber sets cell number.
func (mp *MultilevelPartition) SetCellNumber(i int, c Pv) {
	mp.cellNumbers[i] = c
}

// GetNumCells returns num cells.
func (mp *MultilevelPartition) GetNumCells() []uint32 {
	return mp.numCells
}

// WriteToFile writes to file.
func (mp *MultilevelPartition) WriteToFile(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {
		if err := w.WriteUint32s(mp.numCells); err != nil {
			return err
		}
		if err := w.Length(len(mp.cellNumbers)); err != nil {
			return err
		}
		for _, value := range mp.cellNumbers {
			if err := w.Uint64(uint64(value)); err != nil {
				return err
			}
		}
		return nil
	})
}

// ReadFromFile reads from file.
func (mp *MultilevelPartition) ReadFromFile(filename string) error {
	file, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	numCells, err := r.ReadUint32s()
	if err != nil {
		return err
	}

	cellValues, err := r.ReadUint64s()
	if err != nil {
		return err
	}

	cellNumbers := make([]Pv, len(cellValues))
	for i, value := range cellValues {
		cellNumbers[i] = Pv(value)
	}

	mp.numCells = numCells
	mp.ComputeBitmap()
	mp.cellNumbers = cellNumbers
	return nil
}

// ReadMlpFile reads mlp file.
func (mp *MultilevelPartition) ReadMlpFile() error {
	root := config.ProfilesRoot()
	filename := fmt.Sprintf("%s/%s/inertial_flow_%s.mlp", root, pkg.ProfileName, pkg.RegionName)
	return mp.ReadFromFile(filename)
}

// ReadMultilevelPartitionFromFile reads multilevel partition from file.
func ReadMultilevelPartitionFromFile(filename string) (*MultilevelPartition, error) {
	mp := NewPlainMLP()
	if err := mp.ReadFromFile(filename); err != nil {
		return nil, err
	}
	return mp, nil
}

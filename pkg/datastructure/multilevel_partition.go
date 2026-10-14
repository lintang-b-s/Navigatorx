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

func NewPlainMLP() *MultilevelPartition {
	return &MultilevelPartition{}
}

func NewMultilevelPartition(cellNumbers []Pv, numCells []uint32, pvOffset []uint8) *MultilevelPartition {

	return &MultilevelPartition{
		numCells:    numCells,
		pvOffset:    pvOffset,
		cellNumbers: cellNumbers,
	}
}

func (mp *MultilevelPartition) SetNumberOflevels(numLevels int) {
	mp.numCells = make([]uint32, numLevels)
}

func (mp *MultilevelPartition) SetNumberOfVertices(numVertices int) {
	mp.cellNumbers = make([]Pv, numVertices)
}

func (mp *MultilevelPartition) SetNumberOfCellsInLevel(level int, numCells int) {
	mp.numCells[level] = uint32(numCells)
}

func (mp *MultilevelPartition) ComputeBitmap() {
	mp.pvOffset = make([]uint8, len(mp.numCells)+1)
	for i := 0; i < len(mp.numCells); i++ {
		mp.pvOffset[i+1] = mp.pvOffset[i] + uint8(math.Ceil(math.Log2(float64(mp.numCells[i])))) // ceil(log2(numCells[i])) = number of bits needed to represent cell id in level-i
	}
}

// set cellNumber of vertexId in level=level to cellId
func (mp *MultilevelPartition) SetCell(level int, vertexId int, cellId int) {
	mp.cellNumbers[vertexId] |= Pv(cellId) << mp.pvOffset[level]
}

// get cellNumber of vertexId in level=level
func (mp *MultilevelPartition) GetCell(level int, vertexId int) Pv {
	// off the bits that in above level, then shift right to get the cell id in that level
	return (mp.cellNumbers[vertexId] & ^(^Pv(0) << mp.pvOffset[level+1])) >> Pv(mp.pvOffset[level])
}

func (mp *MultilevelPartition) GetNumberOfVertices() int {
	return len(mp.cellNumbers)
}

func (mp *MultilevelPartition) GetNumberOfLevels() int {
	return len(mp.numCells)
}

func (mp *MultilevelPartition) GetNumberOfCellsInLevel(level int) int {
	return int(mp.numCells[level])
}

func (mp *MultilevelPartition) GetPVOffsets() []uint8 {
	return mp.pvOffset
}

func (mp *MultilevelPartition) GetCellNumber(u Index) Pv {
	return mp.cellNumbers[u]
}

func (mp *MultilevelPartition) GetCellNumbers() []Pv {
	return mp.cellNumbers
}

func (mp *MultilevelPartition) SetCellNumber(i int, c Pv) {
	mp.cellNumbers[i] = c
}

func (mp *MultilevelPartition) GetNumCells() []uint32 {
	return mp.numCells
}

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

func (mp *MultilevelPartition) ReadMlpFile() error {
	root := config.ProfilesRoot()
	filename := fmt.Sprintf("%s/%s/inertial_flow_%s.mlp", root, pkg.ProfileName, pkg.RegionName)
	return mp.ReadFromFile(filename)
}

func ReadMultilevelPartitionFromFile(filename string) (*MultilevelPartition, error) {
	mp := NewPlainMLP()
	if err := mp.ReadFromFile(filename); err != nil {
		return nil, err
	}
	return mp, nil
}

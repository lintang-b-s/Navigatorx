package datastructure

import (
	"bufio"
	"fmt"
	"math"
	"os"
)

// MultilevelPartition. store every cell information of each vertex on every level.
type MultilevelPartition struct {
	numCells    []uint32 // number of cells in the level-index overlay graph
	pvOffset    []uint8 // offset of each level in the bitpacked cell numbers
	cellNumbers []Pv
}

func NewPlainMLP() *MultilevelPartition {
	return &MultilevelPartition{}
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

func (mp *MultilevelPartition) ReadMlpFile(filename string) error {

	f, err := os.Open(filename)

	if err != nil {
		return err
	}
	defer f.Close()

	scanner := bufio.NewScanner(f)

	if scanner.Scan() {
		line := scanner.Text()
		var numLevels int
		_, err := fmt.Sscanf(line, "%d", &numLevels)
		if err != nil {
			return err
		}
		mp.SetNumberOflevels(numLevels)
	}

	if err := scanner.Err(); err != nil {
		return err
	}

	mp.numCells = make([]uint32, mp.GetNumberOfLevels())
	for i := 0; i < len(mp.numCells); i++ {
		if scanner.Scan() {
			line := scanner.Text()
			_, err := fmt.Sscanf(line, "%d", &mp.numCells[i])
			if err != nil {
				return err
			}
		}
	}

	if err := scanner.Err(); err != nil {
		return err
	}

	mp.ComputeBitmap()
	var numVertices int
	if scanner.Scan() {
		line := scanner.Text()

		_, err := fmt.Sscanf(line, "%d", &numVertices)
		if err != nil {
			return err
		}
		mp.SetNumberOfVertices(numVertices)
	}

	if err := scanner.Err(); err != nil {
		return err
	}

	mp.cellNumbers = make([]Pv, numVertices)

	for i := 0; i < mp.GetNumberOfVertices(); i++ {
		if scanner.Scan() {
			line := scanner.Text()
			var cellNumber Pv
			_, err := fmt.Sscanf(line, "%d", &cellNumber)
			if err != nil {
				return err
			}
			mp.cellNumbers[i] = cellNumber
		}
	}

	if err := scanner.Err(); err != nil {
		return err
	}
	return nil
}

package datastructure

import (
	"bufio"
	"fmt"
	"math"
	"os"
)

// MultilevelPartition. store every cell information of each vertex on every level.
type MultilevelPartition struct {
	numCells    []uint32
	pvOffset    []uint8
	cellNumbers []pv
}

func NewPlainMLP() *MultilevelPartition {
	return &MultilevelPartition{}
}

func (mp *MultilevelPartition) SetNumberOflevels(numLevels int) {
	mp.numCells = make([]uint32, numLevels)
}

func (mp *MultilevelPartition) SetNumberOfVertices(numVertices int) {
	mp.cellNumbers = make([]pv, numVertices)
}

func (mp *MultilevelPartition) SetNumberOfCellsInLevel(level int, numCells int) {
	mp.numCells[level] = uint32(numCells)
}

func (mp *MultilevelPartition) ComputeBitmap() {
	mp.pvOffset = make([]uint8, len(mp.numCells)+1)
	for i := 0; i < len(mp.numCells); i++ {
		mp.pvOffset[i+1] = mp.pvOffset[i] + uint8(math.Ceil(math.Log2(float64(mp.numCells[i]))))
	}
}

func (mp *MultilevelPartition) setCell(level int, vertexId int, cellId int) {
	mp.cellNumbers[vertexId] |= pv(cellId) << mp.pvOffset[level]
}

func (mp *MultilevelPartition) getCell(level int, vertexId int) pv {
	return (mp.cellNumbers[vertexId] >> pv(mp.pvOffset[level])) & ^(^pv(0) << mp.pvOffset[level+1])
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

func (mp *MultilevelPartition) GetCellNumber(u Index) pv {
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

	mp.numCells = make([]uint32, len(mp.numCells))
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

	if scanner.Scan() {
		line := scanner.Text()
		var numVertices int
		_, err := fmt.Sscanf(line, "%d", &numVertices)
		if err != nil {
			return err
		}
	}

	if err := scanner.Err(); err != nil {
		return err
	}

	mp.cellNumbers = make([]pv, len(mp.cellNumbers))

	for i := 0; i < mp.GetNumberOfVertices(); i++ {
		if scanner.Scan() {
			line := scanner.Text()
			var cellNumber pv
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

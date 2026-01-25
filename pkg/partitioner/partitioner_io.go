package partitioner

import (
	"fmt"
	"math"
	"os"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

func (mp *MultilevelPartitioner) writeMLPToMLPFile(filename string) error {

	mlp := mp.BuildMLP()

	f, err := os.Create(filename)
	if err != nil {
		return err
	}

	defer f.Close()

	numCells := mlp.GetNumCells()
	cellNumbers := mlp.GetCellNumbers()

	_, err = f.WriteString(fmt.Sprintf("%d\n", len(numCells)))
	if err != nil {
		return err
	}

	for i := 0; i < len(numCells); i++ {
		_, err := f.WriteString(fmt.Sprintf("%d\n", numCells[i]))
		if err != nil {
			return err
		}
	}

	_, err = f.WriteString(fmt.Sprintf("%d\n", mp.graph.NumberOfVertices()))
	if err != nil {
		return err
	}

	for _, vertexID := range mp.graph.GetVerticeIds() {
		_, err := f.WriteString(fmt.Sprintf("%d\n", cellNumbers[vertexID]))
		if err != nil {
			return err
		}
	}

	return nil
}

func (mp *MultilevelPartitioner) BuildMLP() *datastructure.MultilevelPartition {
	numCells := make([]uint32, mp.l)
	for i := 0; i < mp.l; i++ {
		numCells[i] = uint32(len(mp.overlayNodes[i]))
	}

	pvOffset := make([]uint8, mp.l+1)
	for i := 0; i < mp.l; i++ {
		pvOffset[i+1] = pvOffset[i] + uint8(math.Ceil(math.Log2(float64(numCells[i])))) // ceil(log2(numCells[i])) = number of bits needed to represent cell id in level-i
	}

	cellNumbers := make([]datastructure.Pv, mp.graph.NumberOfVertices()) // 64 bit integer. rightmost contain level 0 cellId, leftmost contain level l-1 cellId

	for l := 0; l < mp.l; l++ {
		for cellId, vertexIds := range mp.overlayNodes[l] {
			for _, vertexId := range vertexIds {
				cellNumbers[vertexId] |= datastructure.Pv(cellId) << datastructure.Pv(pvOffset[l])
			}
		}
	}

	return datastructure.NewMultilevelPartition(cellNumbers, numCells, pvOffset)
}

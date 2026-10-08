package partitioner

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (mp *MultilevelPartitioner) SaveToFile() error {
	root := config.ProfilesRoot()
	filename := fmt.Sprintf("%s/%s/inertial_flow_%s.mlp", root, pkg.ProfileName, pkg.RegionName)
	return mp.writeMLPToFile(filename)
}

func (mp *MultilevelPartitioner) writeMLPToFile(filename string) error {
	mlp := mp.BuildMLP()
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {
		numCells := mlp.GetNumCells()
		cellNumbers := mlp.GetCellNumbers()
		if err := w.WriteUint32s(numCells); err != nil {
			return err
		}
		if err := w.Length(len(cellNumbers)); err != nil {
			return err
		}
		for _, value := range cellNumbers {
			if err := w.Uint64(uint64(value)); err != nil {
				return err
			}
		}
		return nil
	})
}

func (mp *MultilevelPartitioner) ReadMLPFromFile(filename string) (*da.MultilevelPartition, error) {
	file, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	numCells, err := r.ReadUint32s()
	if err != nil {
		return nil, err
	}

	cellValues, err := r.ReadUint64s()
	if err != nil {
		return nil, err
	}

	cellNumbers := make([]da.Pv, len(cellValues))
	for i, value := range cellValues {
		cellNumbers[i] = da.Pv(value)
	}

	mlp := da.NewPlainMLP()
	mlp.SetNumberOflevels(len(numCells))
	for i, c := range numCells {
		mlp.SetNumberOfCellsInLevel(i, int(c))
	}
	mlp.ComputeBitmap()
	mlp.SetNumberOfVertices(len(cellNumbers))
	for i, c := range cellNumbers {
		mlp.SetCellNumber(i, c)
	}
	return mlp, nil
}

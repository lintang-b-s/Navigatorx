package partitioner

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (mp *MultilevelPartitioner) writeMLPToMLPFile(filename string) error {

	mlp := mp.BuildMLP()

	if err := os.MkdirAll(filepath.Dir(filename), 0755); err != nil {
		return err
	}

	f, err := os.Create(filename)
	if err != nil {
		return err
	}

	defer f.Close()

	numCells := mlp.GetNumCells()
	cellNumbers := mlp.GetCellNumbers()

	_, err = fmt.Fprintf(f, "%d\n", len(numCells))
	if err != nil {
		return err
	}

	for i := 0; i < len(numCells); i++ {
		_, err := fmt.Fprintf(f, "%d\n", numCells[i])
		if err != nil {
			return err
		}
	}

	_, err = fmt.Fprintf(f, "%d\n", mp.graph.NumberOfVertices())
	if err != nil {
		return err
	}

	for _, vertexID := range mp.graph.GetVerticeIds() {
		_, err := fmt.Fprintf(f, "%d\n", cellNumbers[vertexID])
		if err != nil {
			return err
		}
	}

	return nil
}

func (mp *MultilevelPartitioner) BuildMLP() *da.MultilevelPartition {
	numCells := make([]uint32, mp.l)
	for i := 0; i < mp.l; i++ {
		numCells[i] = uint32(len(mp.cellVertices[i]))
	}

	pvOffset := make([]uint8, mp.l+1)
	for i := 0; i < mp.l; i++ {
		pvOffset[i+1] = pvOffset[i] + uint8(math.Ceil(math.Log2(float64(numCells[i])))) // ceil(log2(numCells[i])) = number of bits needed to represent cell id in level-i
	}

	cellNumbers := make([]da.Pv, mp.graph.NumberOfVertices()) // 64 bit integer. rightmost contain level 0 cellId, leftmost contain level l-1 cellId

	for l := 0; l < mp.l; l++ {
		for cellId, vertexIds := range mp.cellVertices[l] {
			for _, vertexId := range vertexIds {
				cellNumbers[vertexId] |= da.Pv(cellId) << da.Pv(pvOffset[l])
			}
		}
	}

	return da.NewMultilevelPartition(cellNumbers, numCells, pvOffset)
}

type ovCoord struct {
	ID  da.Index `json:"id"`
	Lat float64  `json:"lat"`
	Lon float64  `json:"lon"`
}

type cellVis struct {
	CellID   da.Pv     `json:"cell_id"`
	Vertices []ovCoord `json:"vertices"`
}

// WriteOverlayVerticesInLevel write overlay/boundary vertices ke file.
// buat nampilin kaya figure 1 di: https://www.microsoft.com/en-us/research/wp-content/uploads/2010/12/punchTR.pdf
// atau figure 1 di: https://aschild.github.io/papers/roadseparator.pdf
// buat visualizer nya ada di  eval/crp_alt/visualization/multilevel_partition_mlp_cells.html
func (mp *MultilevelPartitioner) WriteOverlayVerticesInLevel(filename string) error {

	mlp := mp.BuildMLP()
	levelInfo := da.NewLevelInfo(mlp.GetPVOffsets())
	overlayVerticesByLevel := make([]map[da.Pv][]da.Vertex, mp.l)
	for i := 0; i < mp.l; i++ {
		overlayVerticesByLevel[i] = make(map[da.Pv][]da.Vertex)
	}
	for u := da.Index(0); u < da.Index(mp.graph.NumberOfVertices()); u++ {
		uVertex := mp.graph.GetVertex(u)
		mp.graph.ForOutEdgesOfVertex(u, func(head, exitPoint da.Index) {
			v := head
			vVertex := mp.graph.GetVertex(v)

			uPv := mlp.GetCellNumber(u)
			vPv := mlp.GetCellNumber(v)
			overlayLevel := levelInfo.GetHighestDifferingLevel(uPv, vPv)

			if overlayLevel > 0 {
				overlayVerticesByLevel[overlayLevel-1][uPv] = append(overlayVerticesByLevel[overlayLevel-1][uPv], uVertex)
				overlayVerticesByLevel[overlayLevel-1][vPv] = append(overlayVerticesByLevel[overlayLevel-1][vPv], vVertex)
			}
		})
	}

	newOverlayVerticesByLevel := make([]map[da.Pv][]da.Vertex, mp.l)
	for l := 0; l < mp.l; l++ {
		newOverlayVerticesByLevel[l] = make(map[da.Pv][]da.Vertex)
		for cellId, verts := range overlayVerticesByLevel[l] {
			newOverlayVerticesByLevel[l][cellId] = verts
		}
		// karena overlay/boundary vertices di upper level juga merupakan overlay/boundary vertices di level bawahnya
		for uppperLevel := l + 1; uppperLevel < mp.l; uppperLevel++ {
			for cellId, verts := range overlayVerticesByLevel[uppperLevel] {
				newOverlayVerticesByLevel[l][cellId] = append(newOverlayVerticesByLevel[l][cellId], verts...)
			}
		}
	}

	dir := strings.Split(filename, ".mlp")[0]

	for l := 0; l < mp.l; l++ {
		cellVertices := make(map[da.Pv][]da.Vertex, 0)
		for cellId, verts := range newOverlayVerticesByLevel[l] {
			cellVertices[cellId] = append(cellVertices[cellId], util.RemoveDuplicates(verts)...)
		}

		cells := make([]cellVis, 0)
		for cellId, verts := range cellVertices {
			coords := make([]ovCoord, len(verts))
			for j := 0; j < len(verts); j++ {
				v := cellVertices[cellId][j]
				coords[j] = ovCoord{
					ID:  v.GetID(),
					Lat: v.GetLat(),
					Lon: v.GetLon(),
				}
			}

			cells = append(cells, cellVis{CellID: cellId, Vertices: coords})
		}

		levelFile := fmt.Sprintf("%s_boundary_level_%d.json", dir, l+1)
		if err := os.MkdirAll(filepath.Dir(levelFile), 0755); err != nil {
			return err
		}
		data, err := json.Marshal(cells)
		if err != nil {
			return err
		}
		if err := os.WriteFile(levelFile, data, 0644); err != nil {
			return err
		}
	}

	return nil
}

// WriteOverlayVerticesInLevel write vertices inside each cells ke file.
func (mp *MultilevelPartitioner) WriteMLPVisualizationInLevel(filename string) error {

	mlp := mp.BuildMLP()
	levelInfo := da.NewLevelInfo(mlp.GetPVOffsets())
	overlayVerticesByLevel := make([]map[da.Pv][]da.Vertex, mp.l)
	for i := 0; i < mp.l; i++ {
		overlayVerticesByLevel[i] = make(map[da.Pv][]da.Vertex)
	}
	for u := da.Index(0); u < da.Index(mp.graph.NumberOfVertices()); u++ {
		uVertex := mp.graph.GetVertex(u)

		for l := 0; l < mp.l; l++ {
			uPv := mlp.GetCellNumber(u)
			uPvl := levelInfo.GetCellNumberOnLevel(uint8(l+1), uPv)
			overlayVerticesByLevel[l][uPvl] = append(overlayVerticesByLevel[l][uPvl], uVertex)
		}

	}

	dir := strings.Split(filename, ".mlp")[0]

	for l := 0; l < mp.l; l++ {
		cells := make([]cellVis, 0)

		for cellId, verts := range overlayVerticesByLevel[l] {
			coords := make([]ovCoord, len(verts))

			for i, v := range verts {
				coords[i] = ovCoord{
					ID:  v.GetID(),
					Lat: v.GetLat(),
					Lon: v.GetLon(),
				}
			}

			cells = append(cells, cellVis{cellId, coords})
		}

		levelFile := fmt.Sprintf("%s_mlp_level_%d.json", dir, l+1)
		if err := os.MkdirAll(filepath.Dir(levelFile), 0755); err != nil {
			return err
		}
		data, err := json.Marshal(cells)
		if err != nil {
			return err
		}
		if err := os.WriteFile(levelFile, data, 0644); err != nil {
			return err
		}
	}

	return nil
}

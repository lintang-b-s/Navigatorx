package datastructure

import (
	"bufio"
	"fmt"
	"sort"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (og *OverlayGraph) WriteToFile(filename string) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {

		if err := w.Blob(og.levelInfo.offset); err != nil {
			return err
		}
		if err := writeIndices(w, og.vertexCountInLevel); err != nil {
			return err
		}
		if err := w.Length(len(og.overlayVertices)); err != nil {
			return err
		}
		for _, vertex := range og.overlayVertices {
			if err := w.Uint64(uint64(vertex.cellNumber)); err != nil {
				return err
			}
			for _, value := range []Index{vertex.neighborOverlayVertex, vertex.originalVertex, vertex.originalEdge} {
				if err := w.Uint32(uint32(value)); err != nil {
					return err
				}
			}
			if err := writeIndices(w, vertex.entryExitPoint); err != nil {
				return err
			}
		}
		if err := w.Uint32(og.weightVectorSize); err != nil {
			return err
		}
		if err := writeIndices(w, og.overlayIdMapping); err != nil {
			return err
		}
		if err := w.Length(len(og.cellMapping)); err != nil {
			return err
		}
		for _, cells := range og.cellMapping {
			keys := make([]Pv, 0, len(cells))
			for key := range cells {
				keys = append(keys, key)
			}
			sort.Slice(keys, func(i, j int) bool { return keys[i] < keys[j] })
			if err := w.Length(len(keys)); err != nil {
				return err
			}
			for _, key := range keys {
				cell := cells[key]
				if err := w.Uint64(uint64(key)); err != nil {
					return err
				}
				for _, value := range []uint32{
					uint32(cell.numEntryPoints),
					uint32(cell.numExitPoints),
					uint32(cell.cellOffset),
					uint32(cell.overlayIdOffset),
					cell.numOfOverlayVertices,
				} {
					if err := w.Uint32(value); err != nil {
						return err
					}
				}
			}
		}
		return nil
	})
}

func ReadOverlayGraph(filename string, _ *bufio.Reader) (*OverlayGraph, error) {
	file, r, err := util.OpenCompressedArtifact(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	offsets, err := r.Blob(maxGraphItems)
	if err != nil {
		return nil, err
	}
	if len(offsets) < 2 {
		return nil, fmt.Errorf("overlay graph requires at least one level")
	}
	vertexCountInLevel, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	if len(vertexCountInLevel) != len(offsets)-1 {
		return nil, fmt.Errorf("overlay level count does not match vertex-count levels")
	}
	vertexCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	vertices := make([]OverlayVertex, vertexCount)
	for i := range vertices {
		cellNumber, err := r.Uint64()
		if err != nil {
			return nil, err
		}
		vertices[i].cellNumber = Pv(cellNumber)
		fields := []*Index{&vertices[i].neighborOverlayVertex, &vertices[i].originalVertex, &vertices[i].originalEdge}
		for _, field := range fields {
			value, err := r.Uint32()
			if err != nil {
				return nil, err
			}
			*field = Index(value)
		}
		vertices[i].entryExitPoint, err = readIndices(r)
		if err != nil {
			return nil, err
		}
		if len(vertices[i].entryExitPoint) > len(vertexCountInLevel) {
			return nil, fmt.Errorf("overlay vertex %d has too many level entries", i)
		}
	}
	weightVectorSize, err := r.Uint32()
	if err != nil {
		return nil, err
	}
	overlayIDMapping, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	levelCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	if int(levelCount) != len(vertexCountInLevel) {
		return nil, fmt.Errorf("overlay cell-map levels do not match level count")
	}
	cellMapping := make([]map[Pv]Cell, levelCount)
	for level := range cellMapping {
		cellCount, err := r.Length(maxGraphItems)
		if err != nil {
			return nil, err
		}
		cellMapping[level] = make(map[Pv]Cell, cellCount)
		for range cellCount {
			cellNumber, err := r.Uint64()
			if err != nil {
				return nil, err
			}
			fields := [5]uint32{}
			for i := range fields {
				fields[i], err = r.Uint32()
				if err != nil {
					return nil, err
				}
			}
			cellMapping[level][Pv(cellNumber)] = Cell{
				numEntryPoints:       Index(fields[0]),
				numExitPoints:        Index(fields[1]),
				cellOffset:           Index(fields[2]),
				overlayIdOffset:      Index(fields[3]),
				numOfOverlayVertices: fields[4],
			}
		}
	}
	return NewOverlayGraphComplete(
		vertices,
		vertexCountInLevel,
		cellMapping,
		overlayIDMapping,
		NewLevelInfo(offsets),
		weightVectorSize,
	), nil
}

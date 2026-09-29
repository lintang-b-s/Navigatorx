package datastructure

import (
	"fmt"
	"sort"

	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func writePackedSlice(w *util.BinaryWriter, values *PackedSlice) error {
	if values == nil {
		return w.Bool(false)
	}
	if err := w.Bool(true); err != nil {
		return err
	}
	if err := w.Uint8(values.numberOfBits); err != nil {
		return err
	}
	if err := w.Uint64(values.numOfItems); err != nil {
		return err
	}
	if err := w.Length(len(values.data)); err != nil {
		return err
	}
	for _, value := range values.data {
		if err := w.Uint64(value); err != nil {
			return err
		}
	}
	if err := w.Blob(values.lowerOffset); err != nil {
		return err
	}
	return w.Blob(values.upperNumOfBits)
}

func readPackedSlice(r *util.BinaryReader) (*PackedSlice, error) {
	present, err := r.Bool()
	if err != nil || !present {
		return nil, err
	}
	bits, err := r.Uint8()
	if err != nil {
		return nil, err
	}
	if bits == 0 || bits > 64 {
		return nil, fmt.Errorf("invalid packed slice bit width %d", bits)
	}
	items, err := r.Uint64()
	if err != nil {
		return nil, err
	}

	dataLength, err := r.Length()
	if err != nil {
		return nil, err
	}
	data := make([]uint64, dataLength)
	for i := range data {
		data[i], err = r.Uint64()
		if err != nil {
			return nil, err
		}
	}
	lower, err := r.Blob()
	if err != nil {
		return nil, err
	}
	upper, err := r.Blob()
	if err != nil {
		return nil, err
	}
	if uint64(len(lower)) != items || uint64(len(upper)) != items {
		return nil, fmt.Errorf("packed slice metadata count does not match item count %d", items)
	}
	requiredWords := uint64(1)
	if items > 0 {
		requiredWords = (items*uint64(bits) + 63) / 64
	}
	if uint64(len(data)) != requiredWords {
		return nil, fmt.Errorf("packed slice has %d data words, expected %d", len(data), requiredWords)
	}
	return &PackedSlice{
		data:           data,
		numberOfBits:   bits,
		lowerOffset:    lower,
		upperNumOfBits: upper,
		numOfItems:     items,
	}, nil
}

func writeBitSet(w *util.BinaryWriter, value *bitset.BitSet) error {
	if value == nil {
		return w.Bool(false)
	}
	if err := w.Bool(true); err != nil {
		return err
	}
	if err := w.Uint64(uint64(value.Len())); err != nil {
		return err
	}
	for _, word := range value.Words() {
		if err := w.Uint64(word); err != nil {
			return err
		}
	}
	return nil
}

func readBitSet(r *util.BinaryReader) (*bitset.BitSet, error) {
	present, err := r.Bool()
	if err != nil || !present {
		return nil, err
	}
	length, err := r.Uint64()
	if err != nil {
		return nil, err
	}
	wordCount := (length + 63) / 64
	words := make([]uint64, wordCount)
	for i := range words {
		words[i], err = r.Uint64()
		if err != nil {
			return nil, err
		}
	}
	return bitset.FromWithLength(uint(length), words), nil
}

func writeIndices(w *util.BinaryWriter, values []Index) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	for _, value := range values {
		if err := w.Uint32(uint32(value)); err != nil {
			return err
		}
	}
	return nil
}

func readIndices(r *util.BinaryReader) ([]Index, error) {
	length, err := r.Length()
	if err != nil {
		return nil, err
	}
	values := make([]Index, length)
	for i := range values {
		value, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		values[i] = Index(value)
	}
	return values, nil
}

func (g *Graph) WriteGraph(filename string) error {
	return util.WriteCompressedFile(filename, func(w *util.BinaryWriter) error {

		if err := w.Bool(g.roadNetwork); err != nil {
			return err
		}
		if err := w.Float64(g.minResolution); err != nil {
			return err
		}
		if err := w.Length(len(g.vertices)); err != nil {
			return err
		}

		for _, v := range g.vertices {
			for _, value := range []Index{v.pvPtr, v.firstOut, v.firstIn, v.id} {
				if err := w.Uint32(uint32(value)); err != nil {
					return err
				}
			}
			if err := w.Int32(v.lat); err != nil {
				return err
			}
			if err := w.Int32(v.lon); err != nil {
				return err
			}
		}

		// heads & tails
		if err := w.Length(len(g.heads)); err != nil {
			return err
		}
		for _, head := range g.heads {
			if err := w.Uint32(uint32(head)); err != nil {
				return err
			}
		}
		if err := w.Length(len(g.tails)); err != nil {
			return err
		}
		for _, tail := range g.tails {
			if err := w.Uint32(uint32(tail)); err != nil {
				return err
			}
		}

		// entry points & exit points
		if err := w.Length(len(g.entryPoints)); err != nil {
			return err
		}
		for _, e := range g.entryPoints {
			if err := w.Uint32(uint32(e)); err != nil {
				return err
			}
		}
		if err := w.Length(len(g.exitPoints)); err != nil {
			return err
		}
		for _, e := range g.exitPoints {
			if err := w.Uint32(uint32(e)); err != nil {
				return err
			}
		}

		// overlay graph related

		if err := w.Length(len(g.cellNumbers)); err != nil {
			return err
		}
		for _, value := range g.cellNumbers {
			if err := w.Uint64(uint64(value)); err != nil {
				return err
			}
		}
		keys := make([]SubVertex, 0, len(g.overlayVertices))
		for key := range g.overlayVertices {
			keys = append(keys, key)
		}
		sort.Slice(keys, func(i, j int) bool {
			if keys[i].vId != keys[j].vId {
				return keys[i].vId < keys[j].vId
			}
			if keys[i].exitEntryOrder != keys[j].exitEntryOrder {
				return keys[i].exitEntryOrder < keys[j].exitEntryOrder
			}
			return !keys[i].exit && keys[j].exit
		})
		if err := w.Length(len(keys)); err != nil {
			return err
		}
		for _, key := range keys {
			if err := w.Uint32(uint32(key.vId)); err != nil {
				return err
			}
			if err := w.Uint32(uint32(key.exitEntryOrder)); err != nil {
				return err
			}
			if err := w.Bool(key.exit); err != nil {
				return err
			}
			if err := w.Uint32(uint32(g.overlayVertices[key])); err != nil {
				return err
			}
		}
		if err := w.Uint32(uint32(g.maxVerticesInCell)); err != nil {
			return err
		}

		if err := writeIndices(w, g.outEdgeCellOffset); err != nil {
			return err
		}
		if err := writeIndices(w, g.inEdgeCellOffset); err != nil {
			return err
		}

		// scc related
		if err := writeIndices(w, g.sccs); err != nil {
			return err
		}
		if err := w.Length(len(g.sccCondensationAdj)); err != nil {
			return err
		}
		for _, row := range g.sccCondensationAdj {
			if err := writeIndices(w, row); err != nil {
				return err
			}
		}

		for s := 0; s < len(g.sccReach); s++ {
			if err := w.Bitset(g.sccReach[s]); err != nil {
				return err
			}
		}

		// bounding box related
		for _, value := range []float64{g.boundingBox.minLat, g.boundingBox.minLon, g.boundingBox.maxLat, g.boundingBox.maxLon} {
			if err := w.Float64(value); err != nil {
				return err
			}
		}

		return nil
	})
}

func ReadGraph(filename string) (*Graph, error) {
	file, r, err := util.OpenCompressedFile(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	roadNetwork, err := r.Bool()
	if err != nil {
		return nil, err
	}
	minResolution, err := r.Float64()
	if err != nil {
		return nil, err
	}
	vertexCount, err := r.Length()
	if err != nil {
		return nil, err
	}

	vertices := make([]Vertex, vertexCount)
	for i := range vertices {
		fields := []*Index{&vertices[i].pvPtr, &vertices[i].firstOut, &vertices[i].firstIn, &vertices[i].id}
		for _, field := range fields {
			value, err := r.Uint32()
			if err != nil {
				return nil, err
			}
			*field = Index(value)
		}
		vertices[i].lat, err = r.Int32()
		if err != nil {
			return nil, err
		}
		vertices[i].lon, err = r.Int32()
		if err != nil {
			return nil, err
		}
	}
	heads, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	tails, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	entryPoints, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	exitPoints, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	if len(heads) != len(tails) || len(heads) != len(entryPoints) || len(heads) != len(exitPoints) {
		return nil, fmt.Errorf("edge array lengths do not match")
	}

	cellValues, err := r.ReadUint64s()
	if err != nil {
		return nil, err
	}
	cellNumbers := make([]Pv, len(cellValues))
	for i, value := range cellValues {
		cellNumbers[i] = Pv(value)
	}

	overlayCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	overlay := make(map[SubVertex]Index, overlayCount)
	for range overlayCount {
		original, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		order, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		exit, err := r.Bool()
		if err != nil {
			return nil, err
		}
		id, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		overlay[SubVertex{vId: Index(original), exitEntryOrder: Index(order), exit: exit}] = Index(id)
	}
	maxVerticesInCell, err := r.Uint32()
	if err != nil {
		return nil, err
	}

	outOffsets, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	inOffsets, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	if len(outOffsets) != len(cellNumbers) || len(inOffsets) != len(cellNumbers) {
		return nil, fmt.Errorf("cell offset counts do not match cell count")
	}
	sccs, err := readIndices(r)
	if err != nil {
		return nil, err
	}
	adjCount, err := r.Length()
	if err != nil {
		return nil, err
	}
	sccAdj := make([][]Index, adjCount)
	for i := range sccAdj {
		sccAdj[i], err = readIndices(r)
		if err != nil {
			return nil, err
		}
	}

	sccReach := make([]*bitset.BitSet, len(sccAdj))
	for s := range sccReach {
		sccReach[s], err = r.ReadBitset()
		if err != nil {
			return nil, err
		}
	}

	bounds := [4]float64{}
	for i := range bounds {
		bounds[i], err = r.Float64()
		if err != nil {
			return nil, err
		}
	}

	graph := NewGraph(vertices, heads, tails, roadNetwork, entryPoints, exitPoints)
	graph.cellNumbers = cellNumbers
	graph.overlayVertices = overlay
	graph.maxVerticesInCell = Index(maxVerticesInCell)
	graph.outEdgeCellOffset = outOffsets
	graph.inEdgeCellOffset = inOffsets
	graph.sccs = sccs
	graph.sccCondensationAdj = sccAdj
	graph.sccReach = sccReach
	graph.boundingBox = NewBoundingBox(bounds[0], bounds[1], bounds[2], bounds[3])
	graph.minResolution = minResolution
	return graph, nil
}

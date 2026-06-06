package datastructure

import (
	"bufio"
	"fmt"
	"math"
	"sort"

	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	maxGraphItems = uint32(math.MaxUint32)
	maxGraphText  = 64 << 20
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
	if items > uint64(maxGraphItems) {
		return nil, fmt.Errorf("packed slice item count %d is too large", items)
	}
	dataLength, err := r.Length(maxGraphItems)
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
	lower, err := r.Blob(maxGraphItems)
	if err != nil {
		return nil, err
	}
	upper, err := r.Blob(maxGraphItems)
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
	if length > uint64(maxGraphItems) {
		return nil, fmt.Errorf("bitset length %d is too large", length)
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
	length, err := r.Length(maxGraphItems)
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

func writeUint32s(w *util.BinaryWriter, values []uint32) error {
	if err := w.Length(len(values)); err != nil {
		return err
	}
	for _, value := range values {
		if err := w.Uint32(value); err != nil {
			return err
		}
	}
	return nil
}

func readUint32s(r *util.BinaryReader) ([]uint32, error) {
	length, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	values := make([]uint32, length)
	if err := r.ReadUint32s(values); err != nil {
		return nil, err
	}
	return values, nil
}

func readUint64s(r *util.BinaryReader) ([]uint64, error) {
	length, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	values := make([]uint64, length)
	if err := r.ReadUint64s(values); err != nil {
		return nil, err
	}
	return values, nil
}

func (g *Graph) WriteGraph(filename string) error {
	return util.WriteCompressedArtifact(filename, func(w *util.BinaryWriter) error {

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
			for _, value := range []Index{v.pvPtr, v.turnTablePtr, v.firstOut, v.firstIn, v.id} {
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
		if err := writePackedSlice(w, g.verticesOsmIds); err != nil {
			return err
		}
		if err := w.Length(len(g.outEdges)); err != nil {
			return err
		}
		for _, edge := range g.outEdges {
			if err := w.Uint32(uint32(edge.edgeId)); err != nil {
				return err
			}
			if err := w.Uint32(uint32(edge.head)); err != nil {
				return err
			}
			if err := w.Uint32(uint32(edge.entryPoint)); err != nil {
				return err
			}
			if err := w.Uint8(uint8(edge.hwType)); err != nil {
				return err
			}
			if err := w.Uint8(edge.flag); err != nil {
				return err
			}
		}
		if err := w.Length(len(g.inEdges)); err != nil {
			return err
		}
		for _, edge := range g.inEdges {
			if err := w.Uint32(uint32(edge.edgeId)); err != nil {
				return err
			}
			if err := w.Uint32(uint32(edge.tail)); err != nil {
				return err
			}
			if err := w.Uint32(uint32(edge.exitPoint)); err != nil {
				return err
			}
			if err := w.Uint8(uint8(edge.hwType)); err != nil {
				return err
			}
			if err := w.Uint8(edge.flag); err != nil {
				return err
			}
		}
		if err := w.Length(len(g.cellNumbers)); err != nil {
			return err
		}
		for _, value := range g.cellNumbers {
			if err := w.Uint64(uint64(value)); err != nil {
				return err
			}
		}
		if err := w.Length(len(g.turnTypeTable)); err != nil {
			return err
		}
		for _, value := range g.turnTypeTable {
			if err := w.Uint8(uint8(value)); err != nil {
				return err
			}
		}
		keys := make([]SubVertex, 0, len(g.overlayVertices))
		for key := range g.overlayVertices {
			keys = append(keys, key)
		}
		sort.Slice(keys, func(i, j int) bool {
			if keys[i].originalID != keys[j].originalID {
				return keys[i].originalID < keys[j].originalID
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
			if err := w.Uint32(uint32(key.originalID)); err != nil {
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
		if err := w.Uint32(uint32(g.maxEdgesInCell)); err != nil {
			return err
		}
		if err := writeIndices(w, g.outEdgeCellOffset); err != nil {
			return err
		}
		if err := writeIndices(w, g.inEdgeCellOffset); err != nil {
			return err
		}
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
		for _, value := range []float64{g.boundingBox.minLat, g.boundingBox.minLon, g.boundingBox.maxLat, g.boundingBox.maxLon} {
			if err := w.Float64(value); err != nil {
				return err
			}
		}
		return writeGraphStorage(w, g.graphStorage)
	})
}

func writeGraphStorage(w *util.BinaryWriter, gs *GraphStorage) error {
	if err := w.Length(len(gs.osmNodePoints)); err != nil {
		return err
	}
	for _, point := range gs.osmNodePoints {
		if err := w.Int32(point.GetFixedLat()); err != nil {
			return err
		}
		if err := w.Int32(point.GetFixedLon()); err != nil {
			return err
		}
	}
	if err := w.Uint8(gs.osmwayBitSize); err != nil {
		return err
	}
	if err := writePackedSlice(w, gs.edgeOsmWayId); err != nil {
		return err
	}
	metadataCount := len(gs.edgeStartPointsIndex)
	if len(gs.edgeEndPointsIndex) != metadataCount || len(gs.streetName) != metadataCount ||
		len(gs.roadClass) != metadataCount || len(gs.roadClassLink) != metadataCount || len(gs.lanes) != metadataCount {
		return fmt.Errorf("graph edge metadata lengths do not match")
	}
	if err := w.Length(metadataCount); err != nil {
		return err
	}
	for i := 0; i < metadataCount; i++ {
		if err := w.Uint32(uint32(gs.edgeStartPointsIndex[i])); err != nil {
			return err
		}
		if err := w.Uint32(uint32(gs.edgeEndPointsIndex[i])); err != nil {
			return err
		}
		if err := w.Uint32(gs.streetName[i]); err != nil {
			return err
		}
		if err := w.Uint8(uint8(gs.roadClass[i])); err != nil {
			return err
		}
		if err := w.Uint8(uint8(gs.roadClassLink[i])); err != nil {
			return err
		}
		if err := w.Uint8(gs.lanes[i]); err != nil {
			return err
		}
	}
	for _, value := range []*bitset.BitSet{gs.roundaboutFlag, gs.nodeTrafficLight, gs.streetDirectionForward, gs.streetDirectionBackward, gs.isCurvedFlag} {
		if err := writeBitSet(w, value); err != nil {
			return err
		}
	}
	if err := writeUint32s(w, gs.edgeGeohashes); err != nil {
		return err
	}
	if err := w.Length(len(gs.nameTable)); err != nil {
		return err
	}
	for _, value := range gs.nameTable {
		if err := w.String(value); err != nil {
			return err
		}
	}
	if err := w.Length(len(gs.conditionalBarrierNodes)); err != nil {
		return err
	}
	for _, value := range gs.conditionalBarrierNodes {
		if err := w.Int64(value.osmNodeId); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(gs.conditionalReversibleEdges)); err != nil {
		return err
	}
	for _, value := range gs.conditionalReversibleEdges {
		if err := w.Uint32(uint32(value.edgeId)); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(gs.conditionalSpeedLimits)); err != nil {
		return err
	}
	for _, value := range gs.conditionalSpeedLimits {
		if err := w.Uint32(uint32(value.edgeId)); err != nil {
			return err
		}
		if err := w.String(value.timeRangeSpeedVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(gs.conditionalTrafficModes)); err != nil {
		return err
	}
	for _, value := range gs.conditionalTrafficModes {
		if err := w.Uint32(uint32(value.edgeId)); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}
	if err := w.Length(len(gs.conditionalTurnRestrictions)); err != nil {
		return err
	}
	for _, value := range gs.conditionalTurnRestrictions {
		for _, id := range []Index{value.fromVId, value.viaVId, value.toVId} {
			if err := w.Uint32(uint32(id)); err != nil {
				return err
			}
		}
		if err := w.Bool(value.viaWay); err != nil {
			return err
		}
		if err := w.Uint8(uint8(value.turnType)); err != nil {
			return err
		}
		if err := writeIndices(w, value.viaEIds); err != nil {
			return err
		}
		if err := w.String(value.timeRangeVal); err != nil {
			return err
		}
	}
	return nil
}

func ReadGraph(filename string, _ *bufio.Reader) (*Graph, error) {
	file, r, err := util.OpenCompressedArtifact(filename)
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
	vertexCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	vertices := make([]Vertex, vertexCount)
	for i := range vertices {
		fields := []*Index{&vertices[i].pvPtr, &vertices[i].turnTablePtr, &vertices[i].firstOut, &vertices[i].firstIn, &vertices[i].id}
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
	vertexOsmIDs, err := readPackedSlice(r)
	if err != nil {
		return nil, err
	}
	outCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	outEdges := make([]OutEdge, outCount)
	for i := range outEdges {
		edgeID, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		head, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		outEdges[i].edgeId, outEdges[i].head = Index(edgeID), Index(head)
		entry, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		outEdges[i].entryPoint = Index(entry)
		hwType, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		outEdges[i].hwType = pkg.OsmHighwayType(hwType)
		outEdges[i].flag, err = r.Uint8()
		if err != nil {
			return nil, err
		}
	}
	inCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	if inCount != outCount {
		return nil, fmt.Errorf("in-edge count %d does not match out-edge count %d", inCount, outCount)
	}
	inEdges := make([]InEdge, inCount)
	for i := range inEdges {
		edgeID, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		tail, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		inEdges[i].edgeId, inEdges[i].tail = Index(edgeID), Index(tail)
		exit, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		inEdges[i].exitPoint = Index(exit)
		hwType, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		inEdges[i].hwType = pkg.OsmHighwayType(hwType)
		inEdges[i].flag, err = r.Uint8()
		if err != nil {
			return nil, err
		}
	}
	cellValues, err := readUint64s(r)
	if err != nil {
		return nil, err
	}
	cellNumbers := make([]Pv, len(cellValues))
	for i, value := range cellValues {
		cellNumbers[i] = Pv(value)
	}
	turnCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	turnTypes := make([]pkg.TurnType, turnCount)
	for i := range turnTypes {
		value, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		turnTypes[i] = pkg.TurnType(value)
	}
	overlayCount, err := r.Length(maxGraphItems)
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
		overlay[SubVertex{originalID: Index(original), exitEntryOrder: Index(order), exit: exit}] = Index(id)
	}
	maxEdges, err := r.Uint32()
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
	adjCount, err := r.Length(maxGraphItems)
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
	bounds := [4]float64{}
	for i := range bounds {
		bounds[i], err = r.Float64()
		if err != nil {
			return nil, err
		}
	}
	storage, err := readGraphStorage(r, int(outCount), int(vertexCount))
	if err != nil {
		return nil, err
	}
	graph := NewGraph(vertices, outEdges, inEdges, turnTypes, roadNetwork, vertexOsmIDs)
	graph.graphStorage = storage
	graph.cellNumbers = cellNumbers
	graph.overlayVertices = overlay
	graph.maxEdgesInCell = Index(maxEdges)
	graph.outEdgeCellOffset = outOffsets
	graph.inEdgeCellOffset = inOffsets
	graph.sccs = sccs
	graph.sccCondensationAdj = sccAdj
	graph.boundingBox = NewBoundingBox(bounds[0], bounds[1], bounds[2], bounds[3])
	graph.minResolution = minResolution
	return graph, nil
}

func readGraphStorage(r *util.BinaryReader, edgeCount, vertexCount int) (*GraphStorage, error) {
	pointCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	points := make([]Coordinate, pointCount)
	if err := r.ReadInt32Pairs(len(points), func(i int, lat, lon int32) {
		points[i] = NewFixedCoordinate(lat, lon)
	}); err != nil {
		return nil, err
	}
	bitSize, err := r.Uint8()
	if err != nil {
		return nil, err
	}
	osmWayIDs, err := readPackedSlice(r)
	if err != nil {
		return nil, err
	}
	metadataCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	if metadataCount != 0 && int(metadataCount) != edgeCount {
		return nil, fmt.Errorf("edge metadata count %d does not match edge count %d", metadataCount, edgeCount)
	}
	gs := &GraphStorage{
		osmNodePoints:        points,
		edgeOsmWayId:         osmWayIDs,
		osmwayBitSize:        bitSize,
		edgeStartPointsIndex: make([]Index, metadataCount),
		edgeEndPointsIndex:   make([]Index, metadataCount),
		streetName:           make([]uint32, metadataCount),
		roadClass:            make([]pkg.OsmHighwayType, metadataCount),
		roadClassLink:        make([]pkg.OsmHighwayType, metadataCount),
		lanes:                make([]uint8, metadataCount),
	}
	for i := 0; i < int(metadataCount); i++ {
		start, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		end, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		gs.edgeStartPointsIndex[i], gs.edgeEndPointsIndex[i] = Index(start), Index(end)
		gs.streetName[i], err = r.Uint32()
		if err != nil {
			return nil, err
		}
		roadClass, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		gs.roadClass[i] = pkg.OsmHighwayType(roadClass)
		roadClassLink, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		gs.roadClassLink[i] = pkg.OsmHighwayType(roadClassLink)
		gs.lanes[i], err = r.Uint8()
		if err != nil {
			return nil, err
		}
	}
	flags := []**bitset.BitSet{&gs.roundaboutFlag, &gs.nodeTrafficLight, &gs.streetDirectionForward, &gs.streetDirectionBackward, &gs.isCurvedFlag}
	for _, target := range flags {
		*target, err = readBitSet(r)
		if err != nil {
			return nil, err
		}
	}
	gs.edgeGeohashes, err = readUint32s(r)
	if err != nil {
		return nil, err
	}
	if len(gs.edgeGeohashes) != 0 && len(gs.edgeGeohashes) != edgeCount {
		return nil, fmt.Errorf("edge geohash count %d does not match edge count %d", len(gs.edgeGeohashes), edgeCount)
	}
	nameCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	gs.nameTable = make([]string, nameCount)
	for i := range gs.nameTable {
		gs.nameTable[i], err = r.String(maxGraphText)
		if err != nil {
			return nil, err
		}
	}
	barrierCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	gs.conditionalBarrierNodes = make([]ConditionalBarrierNode, barrierCount)
	for i := range gs.conditionalBarrierNodes {
		gs.conditionalBarrierNodes[i].osmNodeId, err = r.Int64()
		if err != nil {
			return nil, err
		}
		gs.conditionalBarrierNodes[i].timeRangeVal, err = r.String(maxGraphText)
		if err != nil {
			return nil, err
		}
	}
	reversibleCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	gs.conditionalReversibleEdges = make([]ConditionalReversibleEdge, reversibleCount)
	for i := range gs.conditionalReversibleEdges {
		id, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		gs.conditionalReversibleEdges[i].edgeId = Index(id)
		gs.conditionalReversibleEdges[i].timeRangeVal, err = r.String(maxGraphText)
		if err != nil {
			return nil, err
		}
	}
	speedCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	gs.conditionalSpeedLimits = make([]ConditionalSpeedLimit, speedCount)
	for i := range gs.conditionalSpeedLimits {
		id, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		gs.conditionalSpeedLimits[i].edgeId = Index(id)
		gs.conditionalSpeedLimits[i].timeRangeSpeedVal, err = r.String(maxGraphText)
		if err != nil {
			return nil, err
		}
	}
	modeCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	gs.conditionalTrafficModes = make([]ConditionalTrafficMode, modeCount)
	for i := range gs.conditionalTrafficModes {
		id, err := r.Uint32()
		if err != nil {
			return nil, err
		}
		gs.conditionalTrafficModes[i].edgeId = Index(id)
		gs.conditionalTrafficModes[i].timeRangeVal, err = r.String(maxGraphText)
		if err != nil {
			return nil, err
		}
	}
	turnCount, err := r.Length(maxGraphItems)
	if err != nil {
		return nil, err
	}
	gs.conditionalTurnRestrictions = make([]ConditionalTurnRestriction, turnCount)
	for i := range gs.conditionalTurnRestrictions {
		value := &gs.conditionalTurnRestrictions[i]
		ids := []*Index{&value.fromVId, &value.viaVId, &value.toVId}
		for _, target := range ids {
			id, err := r.Uint32()
			if err != nil {
				return nil, err
			}
			*target = Index(id)
		}
		value.viaWay, err = r.Bool()
		if err != nil {
			return nil, err
		}
		turnType, err := r.Uint8()
		if err != nil {
			return nil, err
		}
		value.turnType = pkg.TurnType(turnType)
		value.viaEIds, err = readIndices(r)
		if err != nil {
			return nil, err
		}
		value.timeRangeVal, err = r.String(maxGraphText)
		if err != nil {
			return nil, err
		}
	}
	if gs.nodeTrafficLight != nil && gs.nodeTrafficLight.Len() > uint(vertexCount) {
		return nil, fmt.Errorf("traffic-light bitset exceeds vertex count")
	}
	return gs, nil
}

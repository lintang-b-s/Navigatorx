package datastructure

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"os"
	"sort"
	"strings"

	"github.com/dsnet/compress/bzip2"
)

// OverlayVertex. overlay vertex information
// each overlay vertex corresponds to either an entry point or an exit point of a cell in some level
type OverlayVertex struct {
	originalVertex        Index   // original vertex id
	neighborOverlayVertex Index   // overlay vertex id that on another cell and incident to originalEdge
	cellNumber            Pv      // cell number of this overlay vertex. cellNumber = 64 bit uint with rightmost contain level 0 cellNumber, and to the left contain higher level cellNumber
	originalEdge          Index   // original index to outEdge(this overlay vertex is an exit point)/inEdge(this overlay vertex is an entry point) that incident to this overlay vertex
	entryExitPoint        []Index // stores for each level l on which this vertex is an overlay vertex the entry/exit point index in its cell on level l. index = level-l
	// entryExitPoint used in overlaygraph.overlayIdMapping. to get the overlay vertex id in this map, use key = cell.overlayIdOffset + entryExitPoint + (if exit point then + cell.numEntryPoints)
}

// Cell. cell/partition information
/*
Customizable Route Planning In Road Networks, Delling et al., Page 11:
First, for each cell C in the overlay graph, we keep three integers: pC (the number of entry points), qC
(the number of exit points), and fC (the position in W where the first entry of C’s matrix is represented).
During customization and queries, the cost of the shortcut between the i-th entry point and the j-th exit
point of C will be stored in W [fC + iqC + j].

*/
type Cell struct {
	numEntryPoints  Index // p_c // number of entry points
	numExitPoints   Index // q_c // number of exit points
	cellOffset      Index // f_c (the position in W where the first entry of C’s matrix is represented)
	overlayIdOffset Index // maps entry/exit point of cell to overlay vertex id
}

type OverlayGraph struct {
	/*
		(overlayVertices) To improve locality, we assign IDs to overlay vertices such that the boundary
		vertices of the highest level have the lowest IDs, followed by the boundary vertices of the second highest
		level (which are not on the highest), and so on. Within a level, we keep the same relative ordering as in the
		original graph.
	*/
	overlayVertices    []*OverlayVertex // all overlay vertices in the overlay graph. from the highest level to the lowest level, and sorted by their cell number in each level
	vertexCountInLevel []Index          // number of overlay vertices in each level (cumulative sum from highest level to lowest level)
	cellMapping        []map[Pv]*Cell   // cellNumber to Cell mapping for each level. index = level, cellNumber = Pv
	overlayIdMapping   []Index          // maps from key = cell.overlayIdOffset + entryExitPoint + (if exit point then + cell.numEntryPoints) to value = overlay entry/exit point (represented as overlay vertex id)
	levelInfo          *LevelInfo
	weightVectorSize   uint32 // size of one-dimensional weight array W.
}

func NewOverlayGraph(graph *Graph, mlp *MultilevelPartition) *OverlayGraph {
	og := &OverlayGraph{levelInfo: NewLevelInfo(mlp.GetPVOffsets())}
	og.build(graph, uint8(mlp.GetNumberOfLevels()))
	return og
}

func NewOverlayGraphComplete(vertices []*OverlayVertex, vertexCountInLevel []Index,
	cellMapping []map[Pv]*Cell, overlayIdMapping []Index, levelInfo *LevelInfo, weightVectorSize uint32) *OverlayGraph {
	return &OverlayGraph{overlayVertices: vertices, vertexCountInLevel: vertexCountInLevel,
		cellMapping: cellMapping, overlayIdMapping: overlayIdMapping, levelInfo: levelInfo,
		weightVectorSize: weightVectorSize}
}

func (og *OverlayGraph) build(graph *Graph, numberOfLevels uint8) {
	exitFlagArray := og.buildOverlayVertices(graph, numberOfLevels)
	og.buildCells(numberOfLevels, exitFlagArray)
}

func (og *OverlayGraph) GetLevelInfo() *LevelInfo {
	return og.levelInfo
}

func (og *OverlayGraph) numberOfVerticesInLevel(l int) Index {
	return og.vertexCountInLevel[l-1]
}

func (og *OverlayGraph) numberOfCellsInLevel(l int) int {
	return len(og.cellMapping[l-1])
}

func (og *OverlayGraph) GetAllCellsInLevel(l int) map[Pv]*Cell {
	return og.cellMapping[l-1]
}

func (og *OverlayGraph) GetWeightVectorSize() uint32 {
	return og.weightVectorSize
}

func (og *OverlayGraph) numberOfVertices() int {
	return len(og.overlayVertices)
}

func (og *OverlayGraph) getOverlayIdMapping() []Index {
	return og.overlayIdMapping
}

func (og *OverlayGraph) GetVertex(u Index) *OverlayVertex {
	return og.overlayVertices[u]
}

func (og *OverlayGraph) GetEntryPoint(cell *Cell, entryPointIndex Index) Index {
	return og.overlayIdMapping[cell.overlayIdOffset+entryPointIndex]
}

func (og *OverlayGraph) GetExitPoint(cell *Cell, exitPointIndex Index) Index {
	return og.overlayIdMapping[cell.overlayIdOffset+cell.numEntryPoints+exitPointIndex]
}

func (og *OverlayGraph) GetCell(cellNumber Pv, level int) *Cell {
	truncatedCellNumber := og.levelInfo.TruncateToLevel(cellNumber, uint8(level))
	cell, _ := og.cellMapping[level-1][truncatedCellNumber]
	return cell
}

func (og *OverlayGraph) buildOverlayVertices(g *Graph, numberOfLevels uint8) []bool {
	// iterate over all edges to determine overlay vertices
	// store these overlay vertices according to the highest level in which the edge
	//  that connect two overlay vertices is a boundary edge (i.e., the two endpoints are in different cells in that level)

	// overlayVerticesByLevel[l] contains all overlay vertices that are endpoints of boundary edges in level l+1
	overlayVerticesByLevel := make([][]*OverlayVertex, numberOfLevels)
	for start := 0; start < g.NumberOfVertices(); start++ {
		v := g.vertices[start]
		for e := v.firstOut; e < g.vertices[start+1].firstOut; e++ {
			edge := g.GetOutEdge(e)
			startPv := g.GetCellNumber(Index(start))
			targetPv := g.GetCellNumber(edge.head)
			overlayLevel := og.levelInfo.GetHighestDifferingLevel(startPv, targetPv) // check if edge is a boundary edge in any level
			if overlayLevel > 0 {
				// edge is a boundary edge in level=overlayLevel
				// so save the two overlay vertices that are the endpoints of the edge in overlayVerticesByLevel[overlayLevel-1]
				// neighborOverlayVertex of start vertex if the targetVertex in the overlay graph
				// index target vertex in overlayVerticesByLevel[overlayLevel-1] is len(overlayVerticesByLevel[overlayLevel-1])+1
				startVertex := &OverlayVertex{cellNumber: startPv, originalVertex: Index(start),
					originalEdge: e, neighborOverlayVertex: Index(len(overlayVerticesByLevel[overlayLevel-1]) + 1),
					entryExitPoint: make([]Index, overlayLevel)}
				overlayVerticesByLevel[overlayLevel-1] = append(overlayVerticesByLevel[overlayLevel-1], startVertex)

				// neighborOverlayVertex of target vertex is the startVertex in the overlay graph
				// index start vertex in overlayVerticesByLevel[overlayLevel-1] is len(overlayVerticesByLevel[overlayLevel-1])-1
				// because we just appended startVertex to overlayVerticesByLevel[overlayLevel-1]
				inEdge, _ := g.FindInEdge(Index(start), edge.head)
				targetVertex := &OverlayVertex{cellNumber: targetPv, originalVertex: edge.head,
					originalEdge: inEdge, neighborOverlayVertex: Index(len(overlayVerticesByLevel[overlayLevel-1]) - 1),
					entryExitPoint: make([]Index, overlayLevel)}
				overlayVerticesByLevel[overlayLevel-1] = append(overlayVerticesByLevel[overlayLevel-1], targetVertex)
			}
		}
	}

	// vertexCountInLevel[l] = number of overlay vertices in level l+1
	og.vertexCountInLevel = make([]Index, 0, len(overlayVerticesByLevel))

	for _, vertices := range overlayVerticesByLevel {
		og.vertexCountInLevel = append(og.vertexCountInLevel, Index(len(vertices)))
	}

	// make vertexCountInLevel cumulative from highest level to lowest level
	// overlayVertexCount = total number of overlay vertices
	overlayVertexCount := Index(0)
	for i := len(og.vertexCountInLevel) - 1; i >= 0; i-- {
		overlayVertexCount += og.vertexCountInLevel[i]
		og.vertexCountInLevel[i] = overlayVertexCount
	}

	// sort overlay vertices by their cell number
	// build map from subvertices to vertices of the overlay graph
	originalToOverlayVertex := make(map[SubVertex]Index, overlayVertexCount)
	exitFlagsArray := make([]bool, overlayVertexCount) // exitFlagsArray[i] = true if overlay vertex i is an exit point, false if it is an entry point

	// iterate over all levels
	for j := 0; j < len(overlayVerticesByLevel); j++ {
		verticesInLevelJ := overlayVerticesByLevel[j]

		// vertexOffset = starting index of overlay vertices in level j in overlayGraph.overlayVertices
		// highest level j=len(overlayVerticesByLevel)-1 has vertexOffset=0 (because vertexCountInLevel is cumulative suffix sum)
		// lower level j=0 has vertexOffset=suffixSum[0]
		vertexOffset := og.vertexCountInLevel[j] - Index(len(verticesInLevelJ))
		newToOldPosition := make([]Index, len(verticesInLevelJ)) // newToOldPosition contains the mapping from new position (sorted by cell number) to old position (unsorted)

		for k := 0; k < len(verticesInLevelJ); k++ {
			newToOldPosition[k] = Index(k)
		}

		// sort by cell number
		sort.Slice(newToOldPosition, func(a, b int) bool {
			return verticesInLevelJ[a].cellNumber < verticesInLevelJ[b].cellNumber
		})

		oldToNewPosition := make([]Index, len(newToOldPosition))
		for i := 0; i < len(newToOldPosition); i++ {
			oldToNewPosition[newToOldPosition[i]] = Index(i)
		}

		// sortedVertices contains the overlay vertices in level j sorted by cell number
		sortedVertices := make([]*OverlayVertex, 0, len(verticesInLevelJ))

		for i := 0; i < len(verticesInLevelJ); i++ {
			vertex := verticesInLevelJ[newToOldPosition[i]] // newToOldPosition already sorted by its cell number
			// new neighborOverlay vertex bakal start from vertexOffset=0 untuk cell level tertinggi. dan start from vertexOffset>0 untuk level dibawahnya.
			vertex.neighborOverlayVertex = oldToNewPosition[vertex.neighborOverlayVertex] + Index(vertexOffset)
			sortedVertices = append(sortedVertices, vertex)

			isExitPoint := false
			if newToOldPosition[i]%2 == 0 {
				isExitPoint = true // karena di push_back berurutan startVertex, targetVertex, startVertex, targetVertex, ...
			}
			exitFlagsArray[i+int(vertexOffset)] = isExitPoint // index exitflagsArray dari 0 itu overlay vertex di level teringgi sampai di index terakhir  itu last overlay vertex di level terendah.

			// order = offset/index of boundary outEdge/inEdge in outEdges/inEdges slice
			var order uint8
			if isExitPoint {
				order = uint8(g.GetExitOrder(vertex.originalVertex, vertex.originalEdge))
			} else {
				order = uint8(g.GetEntryOrder(vertex.originalVertex, vertex.originalEdge))
			}

			// subVertex = (originalVertexId, offset of boundary outedge/inEdge, is vertex a exit point}
			subVertex := SubVertex{originalID: vertex.originalVertex, turnOrder: order, exit: isExitPoint}
			originalToOverlayVertex[subVertex] = Index(i) + Index(vertexOffset)
		}

		overlayVerticesByLevel[j] = sortedVertices
	}

	og.overlayVertices = make([]*OverlayVertex, 0, overlayVertexCount)
	for i := len(overlayVerticesByLevel) - 1; i >= 0; i-- {
		og.overlayVertices = append(og.overlayVertices, overlayVerticesByLevel[i]...)
	}

	g.SetOverlayMapping(originalToOverlayVertex)
	return exitFlagsArray
}

func (og *OverlayGraph) buildCells(numberOfLevels uint8, exitFlagsArray []bool) {
	og.cellMapping = make([]map[Pv]*Cell, numberOfLevels)
	for l := 0; l < int(numberOfLevels); l++ {
		og.cellMapping[l] = make(map[Pv]*Cell)
	}
	cellOfset := 0
	overlayIdOffset := 0 // offset of first entry/exit point (overlay vertex) in og.overlayIdMapping for the each cell for each level

	// iterate from highest level to lowest level
	// for each level, iterate over all overlay vertices in that level
	// we iterate from the highest level because overlayVertices are sorted by level from highest to lowest level
	for l := int(numberOfLevels - 1); l >= 0; l-- {
		cellsInLevel := og.cellMapping[l]

		for v := Index(0); v < og.vertexCountInLevel[l]; v++ {
			vertex := og.overlayVertices[v]
			isExitPoint := exitFlagsArray[v] // is this overlay vertex an exit point or an entry point
			cellNumberInLevel := og.levelInfo.TruncateToLevel(vertex.cellNumber, uint8(l+1))

			cellPtr, ok := cellsInLevel[cellNumberInLevel]
			if !ok {
				// first time we encounter this cell in level l
				vertex.entryExitPoint[l] = 0

				var cell *Cell // create new cell
				if isExitPoint {
					cell = &Cell{numEntryPoints: 0, numExitPoints: 1}
				} else {
					cell = &Cell{numEntryPoints: 1, numExitPoints: 0}
				}
				cellsInLevel[cellNumberInLevel] = cell

			} else {
				// update existing cell exit/entry point count

				if isExitPoint {
					vertex.entryExitPoint[l] = cellPtr.numExitPoints
					cellPtr.numExitPoints++
				} else {
					vertex.entryExitPoint[l] = cellPtr.numEntryPoints
					cellPtr.numEntryPoints++
				}
			}
		}
		// update cell info
		for _, cell := range cellsInLevel {
			cell.overlayIdOffset = Index(overlayIdOffset)
			cell.cellOffset = Index(cellOfset)
			overlayIdOffset += int(cell.numEntryPoints + cell.numExitPoints)
			cellOfset += int(cell.numEntryPoints * cell.numExitPoints)
		}
		og.cellMapping[l] = cellsInLevel
	}

	// og.overlayIdMapping maps overlayIdOffset + offset of entry/exit point +  of cell to overlay vertex id
	og.overlayIdMapping = make([]Index, overlayIdOffset)
	for l := int(numberOfLevels) - 1; l >= 0; l-- {
		cellsInLevel := og.cellMapping[l]

		for v := Index(0); v < og.vertexCountInLevel[l]; v++ {
			vertex := og.overlayVertices[v]
			isExitVertex := exitFlagsArray[v]
			cellNumberInLevel := og.levelInfo.TruncateToLevel(vertex.cellNumber, uint8(l+1))
			cell := cellsInLevel[cellNumberInLevel]

			mappingIndex := cell.overlayIdOffset + vertex.entryExitPoint[l]
			if isExitVertex {
				mappingIndex += cell.numEntryPoints
			}

			og.overlayIdMapping[mappingIndex] = v
			og.cellMapping[l][cellNumberInLevel] = cell
		}
	}

	og.weightVectorSize = uint32(cellOfset) // size of  one-dimensional weight array W.
}

func (og *OverlayGraph) WriteToFile(filename string) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	bz, err := bzip2.NewWriter(f, &bzip2.WriterConfig{})
	if err != nil {
		return err
	}
	defer bz.Close()

	w := bufio.NewWriter(bz)
	defer w.Flush()

	offsets := og.levelInfo.GetOffsets()
	for i := 0; i < len(offsets); i++ {
		if i > 0 {
			fmt.Fprintf(w, " ")
		}
		fmt.Fprintf(w, "%d", offsets[i])
	}

	fmt.Fprintf(w, "\n")
	for l := 1; l <= og.levelInfo.GetLevelCount(); l++ {
		if l > 1 {
			fmt.Fprintf(w, " ")
		}
		fmt.Fprintf(w, "%d", og.numberOfVerticesInLevel(l))
	}

	fmt.Fprintf(w, "\n")

	for _, vertex := range og.overlayVertices {
		fmt.Fprintf(w, "%d %d %d %d", vertex.cellNumber, vertex.neighborOverlayVertex,
			vertex.originalVertex, vertex.originalEdge)
		for _, e := range vertex.entryExitPoint {
			fmt.Fprintf(w, " %d", e)
		}
		fmt.Fprintf(w, "\n")
	}

	fmt.Fprintf(w, "%d\n", og.weightVectorSize)

	overlayIdMapping := og.getOverlayIdMapping()
	for i, id := range overlayIdMapping {
		if i > 0 {
			fmt.Fprintf(w, " ")
		}
		fmt.Fprintf(w, "%d", id)
	}

	fmt.Fprintf(w, "\n")

	for l := 1; l <= og.levelInfo.GetLevelCount(); l++ {
		fmt.Fprintf(w, "%d\n", og.numberOfCellsInLevel(l))
		for cellNumber, cell := range og.cellMapping[l-1] {
			fmt.Fprintf(w, "%d %d %d %d %d\n", cellNumber, cell.numEntryPoints, cell.numExitPoints,
				cell.cellOffset, cell.overlayIdOffset)
		}
	}

	return nil
}

func ReadOverlayGraph(filename string) (*OverlayGraph, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}

	defer f.Close()

	bz, err := bzip2.NewReader(f, nil)

	if err != nil {
		return nil, err
	}

	br := bufio.NewReader(bz)

	readLine := func() (string, error) {
		line, err := br.ReadString('\n')
		if err != nil {
			if errors.Is(err, io.EOF) && len(line) > 0 {
			} else if err != nil {
				return "", err
			}
		}
		return strings.TrimRight(line, "\r\n"), nil
	}

	line, err := readLine()
	if err != nil {
		return nil, err
	}
	tokens := fields(line)
	offsets := []uint8{}

	for _, token := range tokens {
		offsets = append(offsets, uint8(parseInt(token)))
	}

	levelInfo := NewLevelInfo(offsets)

	line, err = readLine()
	tokens = fields(line)
	vertexCountInLevel := make([]Index, 0, len(tokens))
	for _, token := range tokens {
		vertexCountInLevel = append(vertexCountInLevel, Index(parseInt(token)))
	}

	vertexCount := vertexCountInLevel[0]
	vertices := make([]*OverlayVertex, 0, vertexCount)

	for i := Index(0); i < vertexCount; i++ {
		line, err = readLine()
		if err != nil {
			return nil, err
		}
		tokens = fields(line)
		var vertex *OverlayVertex = &OverlayVertex{}
		vertex.cellNumber = Pv(parseInt(tokens[0]))
		vertex.neighborOverlayVertex = Index(parseInt(tokens[1]))
		vertex.originalVertex = Index(parseInt(tokens[2]))
		vertex.originalEdge = Index(parseInt(tokens[3]))
		vertex.entryExitPoint = make([]Index, 0, len(tokens)-4)
		for j := 4; j < len(tokens); j++ {
			vertex.entryExitPoint = append(vertex.entryExitPoint, Index(parseInt(tokens[j])))
		}
		vertices = append(vertices, vertex)
	}

	line, err = readLine()
	if err != nil {
		return nil, err
	}
	weightVectorSize := uint32(parseInt(line))

	line, err = readLine()
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	overlayIdMapping := make([]Index, 0, len(tokens))
	for _, token := range tokens {
		overlayIdMapping = append(overlayIdMapping, Index(parseInt(token)))
	}

	cellMapping := make([]map[Pv]*Cell, levelInfo.GetLevelCount())
	for i := 0; i < levelInfo.GetLevelCount(); i++ {
		line, err = readLine()
		cellsInLevel := parseInt(line)
		for j := 0; j < cellsInLevel; j++ {
			line, err = readLine()
			if err != nil {
				return nil, err
			}
			tokens = fields(line)
			var cell *Cell = &Cell{}
			cellNumber := Pv(parseInt(tokens[0]))
			cell.numEntryPoints = Index(parseInt(tokens[1]))
			cell.numExitPoints = Index(parseInt(tokens[2]))
			cell.cellOffset = Index(parseInt(tokens[3]))
			cell.overlayIdOffset = Index(parseInt(tokens[4]))

			if cellMapping[i] == nil {
				cellMapping[i] = make(map[Pv]*Cell)
			}
			cellMapping[i][cellNumber] = cell
		}
	}

	return NewOverlayGraphComplete(vertices, vertexCountInLevel, cellMapping, overlayIdMapping, levelInfo, weightVectorSize), nil
}

func parseInt(s string) int {
	var n int
	fmt.Sscanf(s, "%d", &n)
	return n
}

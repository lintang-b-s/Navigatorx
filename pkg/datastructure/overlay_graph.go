package datastructure

import (
	"bufio"
	"fmt"
	"os"
	"sort"

	"github.com/dsnet/compress/bzip2"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// OverlayVertex. overlay vertex information
// each overlay vertex corresponds to either an entry point or an exit point of a cell in some level
type OverlayVertex struct {
	originalVertex        Index   // original vertex id
	neighborOverlayVertex Index   // overlay vertex id that on another cell and incident to originalEdge. can be a head if this overlay vertex is an exit point, or can be a tail if this overlay vertex is an entry point.
	cellNumber            Pv      // cell number of this overlay vertex. cellNumber = 64 bit uint with rightmost contain level 0 cellNumber, and to the left contain higher level cellNumber
	originalEdge          Index   // original index to outEdge(this overlay vertex is an exit point)/inEdge(this overlay vertex is an entry point) that incident to this overlay vertex
	entryExitPoint        []Index // stores for each level l on which this vertex is an overlay vertex (entry/exit point index in its cell on level l). index = level-l
	// entryExitPoint used in overlaygraph.overlayIdMapping. to get the overlay vertex id in overlayGraph.overlayIdMapping, use key = cell.overlayIdOffset + entryExitPoint + (if exit point then + cell.numEntryPoints)
}

func (ov *OverlayVertex) GetEntryExitPoint(level int) Index {
	return ov.entryExitPoint[level-1]
}

func (ov *OverlayVertex) GetEntryPointSize() uint8 {
	return uint8(len(ov.entryExitPoint))
}

func (ov *OverlayVertex) GetCellNumber() Pv {
	return ov.cellNumber
}

func (ov *OverlayVertex) GetOriginalVertex() Index {
	return ov.originalVertex
}

func (ov *OverlayVertex) GetOriginalEdge() Index {
	return ov.originalEdge
}

func (ov *OverlayVertex) GetNeighborOverlayVertex() Index {
	return ov.neighborOverlayVertex
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
	overlayIdOffset Index //  offset of first entry/exit point (overlay vertex) in og.overlayIdMapping for the each cell for each level
}

func (c *Cell) GetNumEntryPoints() Index {
	return c.numEntryPoints
}

func (c *Cell) GetNumExitPoints() Index {
	return c.numExitPoints
}

func (c *Cell) GetCellOffset() Index {
	return c.cellOffset
}

func (c *Cell) GetOverlayIdOffset() Index {
	return c.overlayIdOffset
}

type OverlayGraph struct {
	/*
			(overlayVertices) To improve spatial  locality, we assign IDs to overlay vertices such that the boundary
								vertices of the highest level have the lowest IDs, followed by the boundary vertices of the second highest
								level (which are not on the highest), and so on. Within a level, we keep the same relative ordering as in the
								original graph.


		og.overlayVertices only store unique overlay vertices,
		 with overlay vertex that is a border in highest level cells (border in highest level is also border in lower levels until level-1) stored at the beginning of the slice
		 it means that if we want to get overlay vertex for level 1 that is also a border vertex in level L (highest) we can use the same index (of the highest level border vertex) at the beginning of the array
	*/
	overlayVertices    []*OverlayVertex // all overlay vertices in the overlay graph. from the highest level to the lowest level, and sorted by their cell number in each level
	vertexCountInLevel []Index          // number of overlay vertices in each level (cumulative sum from highest level to lowest level)
	cellMapping        []map[Pv]*Cell   // cellNumber to Cell mapping for each level. index = level, cellNumber = Pv
	overlayIdMapping   []Index          // maps from key = cell.overlayIdOffset + entryExitPoint + (if exit point then + cell.numEntryPoints) to value = overlay entry/exit vertex of a cell (represented as overlay vertex id)
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

func (og *OverlayGraph) NumberOfVerticesInLevel(l int) Index {
	return og.vertexCountInLevel[l-1]
}

func (og *OverlayGraph) numberOfCellsInLevel(l int) int {
	return len(og.cellMapping[l-1])
}

func (og *OverlayGraph) GetAllCellsInLevel(l int) map[Pv]*Cell {
	return og.cellMapping[l-1]
}

func (og *OverlayGraph) ForVertices(handle func(id Index, v *OverlayVertex)) {
	for id, v := range og.overlayVertices {
		handle(Index(id), v)
	}
}

func (og *OverlayGraph) TruncateToLevel(cellNumber Pv, level uint8) Pv {
	return og.levelInfo.TruncateToLevel(cellNumber, level)
}

/*
GetQueryLevel. Customizable Route Planning in Road Networks, Delling et al., et al. Page 14

define its query level lst (v) as the highest level such that v is not
at the same cell as s or t. Equivalently, lst(v) is the maximum i such that ci (v) ∩ {s, t} = ∅.
To compute lst (v), we first determine the most significant differing bit of pv(s) and pv(v). (Recall that
pv(v) encodes the cell number of v on each level.) This bit indicates the topmost level ls(v) in which they
differ. We do the same for pv(t) and pv(v) to determine lt(v). The minimum of ls (v) and lt (v) is lst (v).
*/
func (og *OverlayGraph) GetQueryLevel(sCellNumber, tCellNumber, vCellNumber Pv) uint8 {
	return og.levelInfo.GetQueryLevel(sCellNumber, tCellNumber, vCellNumber)
}

func (og *OverlayGraph) GetWeightVectorSize() uint32 {
	return og.weightVectorSize
}

func (og *OverlayGraph) NumberOfOverlayVertices() int {
	return len(og.overlayVertices)
}

func (og *OverlayGraph) getOverlayIdMapping() []Index {
	return og.overlayIdMapping
}

func (og *OverlayGraph) GetVertex(u Index) *OverlayVertex {
	return og.overlayVertices[u]
}

func (og *OverlayGraph) GetEntryId(cell *Cell, entryPointIndex Index) Index {
	return og.overlayIdMapping[cell.overlayIdOffset+entryPointIndex]
}

func (og *OverlayGraph) GetExitId(cell *Cell, exitPointIndex Index) Index {
	return og.overlayIdMapping[cell.overlayIdOffset+cell.numEntryPoints+exitPointIndex]
}

func (og *OverlayGraph) GetCell(cellNumber Pv, level int) *Cell {
	truncatedCellNumber := og.levelInfo.TruncateToLevel(cellNumber, uint8(level))
	cell, _ := og.cellMapping[level-1][truncatedCellNumber]
	return cell
}

func (og *OverlayGraph) GetCellFromTruncatedCellNumber(truncatedCellNumber Pv, level int) *Cell {
	cell, _ := og.cellMapping[level-1][truncatedCellNumber]
	return cell
}

func (og *OverlayGraph) buildOverlayVertices(g *Graph, numberOfLevels uint8) []bool {
	// iterate over all edges to determine overlay vertices
	// store these overlay vertices according to the highest level in which the edge
	//  that connect two overlay vertices is a boundary edge (i.e., the two endpoints are in different cells in that level)

	// overlay/border vertices are vertices that have endpoint in other cell within same level
	// because overlay vertices at a high level also become overlay vertices at the lower level,
	// in this overlayGraph data structure we only store 1 overlay vertex for that case
	// each overlayVertex store neighborVertex, originalEdge (cut/boundary edge), cellNumber, originalVertexId, and its exitVertices index in its cell
	// each cell in each level contain all overlay vertices and shortest paths/shortcuts between each (entry,exit) its overlay vertices,
	//  each overlay vertices can be a exit Vertex (have edge that point to other entryVertex) or entry Vertex
	// og.overlayVertices only store unique overlay vertices,
	// with overlay vertex that is a border in highest level cells (border in highest level is also border in lower levels until level-1) stored at the beginning of the slice
	// it means that if we want to get overlay vertex for level 1 that is a border vertex in level L (highest) we can use the same index (of the highest level border vertex) at the beginning of the array

	// overlayVerticesByLevel[l] contains all overlay vertices that are endpoints of boundary edges in level l+1 (overlayVerticesByLevel is 0-based indexing)
	overlayVerticesByLevel := make([][]OverlayVertex, numberOfLevels)
	for start := 0; start < g.NumberOfVertices(); start++ {
		v := g.vertices[start]

		for e := v.firstOut; e < g.vertices[start+1].firstOut; e++ {
			edge := g.GetOutEdge(e)
			startPv := g.GetCellNumber(v.GetID())
			targetPv := g.GetCellNumber(edge.GetHead())
			overlayLevel := og.levelInfo.GetHighestDifferingLevel(startPv, targetPv) // check if edge is a boundary edge in any level, return the highest level in which the edge is a boundary edge
			if overlayLevel > 0 {

				// edge is a boundary edge in level=overlayLevel
				// so save the two overlay vertices that are the endpoints of the edge in overlayVerticesByLevel[overlayLevel-1]
				// neighborOverlayVertex of start vertex if the targetVertex in the overlay graph
				// index target vertex in overlayVerticesByLevel[overlayLevel-1] is len(overlayVerticesByLevel[overlayLevel-1])+1
				exitVertex := OverlayVertex{cellNumber: startPv, originalVertex: Index(start),
					originalEdge: e, neighborOverlayVertex: Index(len(overlayVerticesByLevel[overlayLevel-1]) + 1),
					entryExitPoint: make([]Index, overlayLevel)}
				overlayVerticesByLevel[overlayLevel-1] = append(overlayVerticesByLevel[overlayLevel-1], exitVertex)

				// neighborOverlayVertex of target vertex is the exitVertex in the overlay graph
				// index start vertex in overlayVerticesByLevel[overlayLevel-1] is len(overlayVerticesByLevel[overlayLevel-1])-1
				// because we just appended exitVertex to overlayVerticesByLevel[overlayLevel-1]
				inEdgeId, _ := g.FindInEdge(Index(start), edge.head)
				entryVertex := OverlayVertex{cellNumber: targetPv, originalVertex: edge.head,
					originalEdge: inEdgeId, neighborOverlayVertex: Index(len(overlayVerticesByLevel[overlayLevel-1]) - 1),
					entryExitPoint: make([]Index, overlayLevel)}
				overlayVerticesByLevel[overlayLevel-1] = append(overlayVerticesByLevel[overlayLevel-1], entryVertex)
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

	overlayVerticesInLevelFinal := make([][]*OverlayVertex, numberOfLevels)
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

		// sort overlay vertices in level-j by their cell number
		sort.Slice(newToOldPosition, func(i, j int) bool {
			return verticesInLevelJ[newToOldPosition[i]].cellNumber < verticesInLevelJ[newToOldPosition[j]].cellNumber
		})

		oldToNewPosition := make([]Index, len(newToOldPosition))
		for i := 0; i < len(newToOldPosition); i++ {
			oldToNewPosition[newToOldPosition[i]] = Index(i)
		}

		// sortedVertices contains the overlay vertices in level j sorted by cell number
		sortedVertices := make([]OverlayVertex, 0, len(verticesInLevelJ))

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
			var order int
			if isExitPoint {
				order = int(g.GetExitOrder(vertex.originalVertex, vertex.originalEdge))
			} else {
				order = int(g.GetEntryOrder(vertex.originalVertex, vertex.originalEdge))
			}

			// subVertex = (originalVertexId, offset of boundary outedge/inEdge, is vertex a exit point}
			subVertex := SubVertex{originalID: vertex.originalVertex, exitEntryOrder: order, exit: isExitPoint}
			originalToOverlayVertex[subVertex] = Index(i) + Index(vertexOffset)
		}

		overlayVerticesInLevelFinal[j] = make([]*OverlayVertex, len(sortedVertices))
		for k, v := range sortedVertices {
			overlayVerticesInLevelFinal[j][k] = &OverlayVertex{
				originalVertex:        v.originalVertex,
				originalEdge:          v.originalEdge,
				cellNumber:            v.cellNumber,
				neighborOverlayVertex: v.neighborOverlayVertex,
				entryExitPoint:        v.entryExitPoint,
			}
		}
	}

	og.overlayVertices = make([]*OverlayVertex, 0, overlayVertexCount)
	for i := len(overlayVerticesByLevel) - 1; i >= 0; i-- {
		og.overlayVertices = append(og.overlayVertices, overlayVerticesInLevelFinal[i]...)
	}

	g.SetOverlayMapping(originalToOverlayVertex)
	return exitFlagsArray
}

func (og *OverlayGraph) buildCells(numberOfLevels uint8, exitFlagsArray []bool) {
	og.cellMapping = make([]map[Pv]*Cell, numberOfLevels)
	for l := 0; l < int(numberOfLevels); l++ {
		og.cellMapping[l] = make(map[Pv]*Cell)
	}
	shorcutsWeightSize := 0
	overlayIdOffset := 0 // offset of first entry/exit point (overlay vertex) in og.overlayIdMapping for the each cell for each level

	// iterate from highest level to lowest level
	// for each level, iterate over all overlay vertices in that level
	// we iterate from the highest level because overlayVertices are sorted by level from highest to lowest level
	// for each vertex set the entryExitPoint in that level
	for l := int(numberOfLevels - 1); l >= 0; l-- {

		// og.overlayVertices already sorted by level descending and cell number ascending

		// overlayVertices in level numberOfLevels-1 (that are also a overlayVertices in lower level) are at the beginning of the array

		for v := Index(0); v < og.vertexCountInLevel[l]; v++ {
			vertex := og.overlayVertices[v]
			isExitPoint := exitFlagsArray[v] // is this overlay vertex an exit point or an entry point
			cellNumberInLevel := og.levelInfo.TruncateToLevel(vertex.cellNumber, uint8(l+1))

			cellPtr, ok := og.cellMapping[l][cellNumberInLevel]
			if !ok {
				// first time we encounter this cell in level l
				vertex.entryExitPoint[l] = 0

				var cell *Cell // create new cell
				if isExitPoint {
					cell = &Cell{numEntryPoints: 0, numExitPoints: 1}
					vertex.entryExitPoint[l] = 0
				} else {
					cell = &Cell{numEntryPoints: 1, numExitPoints: 0}
					vertex.entryExitPoint[l] = 0
				}
				og.cellMapping[l][cellNumberInLevel] = cell

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
		for key := range og.cellMapping[l] {
			og.cellMapping[l][key].overlayIdOffset = Index(overlayIdOffset)
			og.cellMapping[l][key].cellOffset = Index(shorcutsWeightSize)

			overlayVertexCountInCell := int(og.cellMapping[l][key].numEntryPoints + og.cellMapping[l][key].numExitPoints)
			overlayIdOffset += overlayVertexCountInCell
			shorcutsWeightSize += int(og.cellMapping[l][key].numEntryPoints * og.cellMapping[l][key].numExitPoints)
		}
	}

	// og.overlayIdMapping maps overlayIdOffset + offset of entry/exit point +  of cell to overlay vertex id
	og.overlayIdMapping = make([]Index, overlayIdOffset)
	for l := int(numberOfLevels) - 1; l >= 0; l-- {

		for v := Index(0); v < og.vertexCountInLevel[l]; v++ {
			vertex := og.overlayVertices[v]
			isExitVertex := exitFlagsArray[v]

			cellNumberInLevel := og.levelInfo.TruncateToLevel(vertex.cellNumber, uint8(l+1))
			cell := og.cellMapping[l][cellNumberInLevel]

			mappingIndex := cell.overlayIdOffset + vertex.entryExitPoint[l]

			if isExitVertex {
				mappingIndex += cell.numEntryPoints
			}

			og.overlayIdMapping[mappingIndex] = v
			og.cellMapping[l][cellNumberInLevel] = cell
		}
	}

	og.weightVectorSize = uint32(shorcutsWeightSize) // size of  one-dimensional weight array W.
}

// ForOutNeighborsOf. iterates over all outgoing-neighbors of u
func (og *OverlayGraph) ForOutNeighborsOf(u Index, level int, handle func(v Index, wOffset Index)) {
	uVertex := og.GetVertex(u)
	entryPoint := uVertex.GetEntryExitPoint(level)

	cell := og.GetCell(uVertex.GetCellNumber(), level)
	weightOffset := cell.GetCellOffset() + entryPoint*cell.GetNumExitPoints()
	overlayIdOffset := cell.GetOverlayIdOffset() + cell.GetNumEntryPoints()

	for i := Index(0); i < Index(cell.GetNumExitPoints()); i++ {
		handle(og.overlayIdMapping[overlayIdOffset+i], weightOffset+i)
	}
}

// ForInNeighborsOf. iterates over all incoming-neighbors of v
func (og *OverlayGraph) ForInNeighborsOf(v Index, level int, handle func(v Index, wOffset Index)) {
	vertex := og.GetVertex(v)
	exitPoint := vertex.GetEntryExitPoint(level)
	cell := og.GetCell(vertex.GetCellNumber(), level)
	weightOffset := cell.GetCellOffset() + exitPoint
	overlayIdOffset := cell.GetOverlayIdOffset()
	for i := Index(0); i < Index(cell.GetNumEntryPoints()); i++ {
		handle(og.overlayIdMapping[overlayIdOffset+i], weightOffset+i*cell.GetNumExitPoints())
	}
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
		fmt.Fprintf(w, "%d", og.NumberOfVerticesInLevel(l))
	}

	fmt.Fprintf(w, "\n")

	fmt.Fprintf(w, "%d \n", len(og.overlayVertices))

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

	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	tokens := fields(line)
	offsets := []uint8{}

	for _, token := range tokens {
		offsets = append(offsets, uint8(parseInt(token)))
	}

	levelInfo := NewLevelInfo(offsets)

	line, err = util.ReadLine(br)
	tokens = fields(line)
	vertexCountInLevel := make([]Index, 0, len(tokens))
	for _, token := range tokens {
		vertexCountInLevel = append(vertexCountInLevel, Index(parseInt(token)))
	}

	line, err = util.ReadLine(br)
	tokens = fields(line)
	vertexCount := parseInt(tokens[0])
	vertices := make([]*OverlayVertex, 0, vertexCount)

	for i := Index(0); i < Index(vertexCount); i++ {
		line, err = util.ReadLine(br)
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

	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	weightVectorSize := uint32(parseInt(line))

	line, err = util.ReadLine(br)
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
		line, err = util.ReadLine(br)
		cellsInLevel := parseInt(line)
		for j := 0; j < cellsInLevel; j++ {
			line, err = util.ReadLine(br)
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

	og := NewOverlayGraphComplete(vertices, vertexCountInLevel, cellMapping, overlayIdMapping, levelInfo, weightVectorSize)

	return og, nil
}

func parseInt(s string) int {
	var n int
	fmt.Sscanf(s, "%d", &n)
	return n
}

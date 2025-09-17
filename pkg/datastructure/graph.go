package datastructure

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"strconv"
	"strings"

	"github.com/dsnet/compress/bzip2"
)

type Index uint32

type Vertex struct {
	lat          float64
	lon          float64
	pvPtr        Index // pointer index to cellNumbers slice
	turnTablePtr Index
	firstOut     Index
	firstIn      Index
	id           Index
}

func NewVertex(lat, lon float64, id Index) Vertex {
	return Vertex{
		lat: lat,
		lon: lon,
		id:  id,
	}
}

func (v *Vertex) SetFirstOut(firstOut Index) {
	v.firstOut = firstOut
}

func (v *Vertex) SetFirstIn(firstIn Index) {
	v.firstIn = firstIn
}

func (v *Vertex) SetPvPtr(pvPtr Index) {
	v.pvPtr = pvPtr
}

func (v *Vertex) SetTurnTablePtr(turnTablePtr Index) {
	v.turnTablePtr = turnTablePtr
}

func (v *Vertex) GetID() Index {
	return v.id
}

func (v *Vertex) GetLat() float64 {
	return v.lat
}

func (v *Vertex) GetLon() float64 {
	return v.lon
}

func (v *Vertex) GetFirstOut() Index {
	return v.firstOut
}

func (v *Vertex) GetFirstIn() Index {
	return v.firstIn
}

func (v *Vertex) GetPvPtr() Index {
	return v.pvPtr
}

func (v *Vertex) GetTurnTablePtr() Index {
	return v.turnTablePtr
}

// outedge enters vertex head at entryPoint
type OutEdge struct {
	weight     float64 // minute
	dist       float64 // meter
	edgeId     Index
	head       Index
	entryPoint uint8
}

// inedge exits vertex tail at exitPoint
type InEdge struct {
	weight    float64 // minute
	dist      float64 // meter
	edgeId    Index
	tail      Index
	exitPoint uint8
}

func NewOutEdge(edgeId, head Index, weight, dist float64, entryPoint uint8) OutEdge {
	return OutEdge{
		edgeId:     edgeId,
		head:       head,
		weight:     weight,
		dist:       dist,
		entryPoint: entryPoint,
	}
}

func NewInEdge(edgeId, tail Index, weight, dist float64, exitPoint uint8) InEdge {
	return InEdge{
		edgeId:    edgeId,
		tail:      tail,
		weight:    weight,
		dist:      dist,
		exitPoint: exitPoint,
	}
}

func (e *OutEdge) GetEdgeSpeed() float64 {
	return e.dist / e.weight
}

func (e *OutEdge) GetHead() Index {
	return e.head
}

func (e *OutEdge) SetEntryPoint(p uint8) {
	e.entryPoint = p
}

func (e *InEdge) GetEdgeSpeed() float64 {
	return e.dist / e.weight
}

func (e *InEdge) GetTail() Index {
	return e.tail
}

func (e *InEdge) SetExitPoint(p uint8) {
	e.exitPoint = p
}

type SubVertex struct {
	originalID Index
	turnOrder  uint8
	exit       bool
}

type VertexIDPair struct {
	originalVertexID Index
	id               Index
}

// enum of turn_type
type TurnType uint8

const (
	LEFT_TURN TurnType = iota
	RIGHT_TURN
	STRAIGHT_ON
	U_TURN
	NO_ENTRY
	NONE
)

type Pv uint64

type Graph struct {
	vertices          []Vertex
	outEdges          []OutEdge
	inEdges           []InEdge
	turnTables        []TurnType
	cellNumbers       []Pv
	maxEdgesInCell    Index
	outEdgeCellOffset []Index
	inEdgeCellOffset  []Index
	overlayVertices   map[SubVertex]Index // graph vertices -> overlay vertices
}

func NewGraph(vertices []Vertex, forwardEdges []OutEdge, inEdges []InEdge, turnMatrices []TurnType) *Graph {
	return &Graph{vertices: vertices, outEdges: forwardEdges, inEdges: inEdges, turnTables: turnMatrices, maxEdgesInCell: 0}
}

func (g *Graph) NumberOfVertices() int {
	return len(g.vertices) - 1
}

func (g *Graph) NumberOfEdges() int {
	return len(g.outEdges)
}

func (g *Graph) GetOutDegree(u Index) Index {
	return g.vertices[u+1].firstOut - g.vertices[u].firstOut
}

func (g *Graph) GetInDegree(u Index) Index {
	return g.vertices[u+1].firstIn - g.vertices[u].firstIn
}

func (g *Graph) GetExitOffset(u Index) Index {
	return g.vertices[u].firstOut
}

func (g *Graph) GetEntryOffset(u Index) Index {
	return g.vertices[u].firstIn
}

func (g *Graph) GetOutEdge(e Index) OutEdge {
	return g.outEdges[e]
}

func (g *Graph) GetInEdge(e Index) InEdge {
	return g.inEdges[e]
}

func (g *Graph) FindInEdge(u, v Index) (Index, bool) {
	for e := g.vertices[v].firstIn; e < g.vertices[v+1].firstIn; e++ {
		if g.inEdges[e].tail == u {
			return e, true
		}
	}
	return 0, false
}

func (g *Graph) GetHeadOfInedge(e Index) Index {
	inEdge := g.GetInEdge(e)
	tail := g.vertices[inEdge.tail]
	outEdge := g.GetOutEdge(tail.firstOut + Index(inEdge.exitPoint))
	return outEdge.head
}

func (g *Graph) GetTailOfOutedge(e Index) Index {
	outEdge := g.GetOutEdge(e)
	head := g.vertices[outEdge.head]
	inEdge := g.GetInEdge(head.firstIn + Index(outEdge.entryPoint))
	return inEdge.tail
}

// GetExitOrder. return Index of exit point of a out edge (u,v) at vertex u.
func (g *Graph) GetExitOrder(u, outEdge Index) Index {
	return outEdge - g.vertices[u].firstOut
}

// GetEntryOrder. return Index of entry point of a in edge (u,v) at vertex v.
func (g *Graph) GetEntryOrder(v, InEdge Index) Index {
	return InEdge - g.vertices[v].firstIn
}

func (g *Graph) GetTurnType(u Index, entryPoint, exitPoint uint8) TurnType {
	turnTableOffset := g.vertices[u].turnTablePtr + Index(entryPoint)*Index(g.GetOutDegree(u)) + Index(exitPoint)
	return g.turnTables[turnTableOffset]
}

func (g *Graph) SetCellNumbers(cellNumbers []Pv) {
	g.cellNumbers = cellNumbers
}

func (g *Graph) SetOverlayMapping(overlayVertices map[SubVertex]Index) {
	g.overlayVertices = overlayVertices
}

func (g *Graph) GetOverlayVertex(u Index, turnOrder uint8, exit bool) (Index, bool) {
	subV := SubVertex{
		originalID: u,
		turnOrder:  turnOrder,
		exit:       exit,
	}
	id, exists := g.overlayVertices[subV]
	return id, exists
}

func (g *Graph) GetTUrntables() []TurnType {
	return g.turnTables
}

func (g *Graph) GetCellNumber(u Index) Pv {
	return g.cellNumbers[g.vertices[u].pvPtr]
}

func (g *Graph) GetNumberOfCellsNumbers() int {
	return len(g.cellNumbers)
}

func (g *Graph) GetNumberOfOverlayVertexMapping() int {
	return len(g.overlayVertices)
}

func (g *Graph) GetVertexCoordinates(u Index) (float64, float64) {
	v := g.vertices[u]
	return v.lat, v.lon
}

func (g *Graph) GetMaxEdgesInCell() Index {
	return g.maxEdgesInCell
}

func (g *Graph) GetOutEdgeCellOffset(v Index) Index {
	return g.outEdgeCellOffset[g.vertices[v].pvPtr]
}

func (g *Graph) GetInEdgeCellOffset(v Index) Index {
	return g.inEdgeCellOffset[g.vertices[v].pvPtr]
}

func (g *Graph) GetOutEdgeCellOffsets() []Index {
	return g.outEdgeCellOffset
}

func (g *Graph) GetInEdgeCellOffsets() []Index {
	return g.inEdgeCellOffset
}

func (g *Graph) GetVertices() []Vertex {
	vertices := make([]Vertex, 0, g.NumberOfVertices())
	for _, vertex := range g.vertices[:g.NumberOfVertices()] {
		vertices = append(vertices, vertex)
	}
	return vertices
}

func (g *Graph) SortVerticesByCellNumber() {
	cellVertices := make([][]struct {
		vertex        Vertex
		originalIndex Index
	}, g.GetNumberOfCellsNumbers()) // slice of slice of vertices in each cell

	numOutEdgesInCell := make([]Index, g.GetNumberOfCellsNumbers()) // number of outEdges in each cell
	numInEdgesInCell := make([]Index, g.GetNumberOfCellsNumbers())

	oEdges := make([][]OutEdge, g.NumberOfVertices()) // copy of original outEdges of each vertex
	iEdges := make([][]InEdge, g.NumberOfVertices())

	maxEdgesInCell := Index(0) // maximum number of edges in any cell
	for i := Index(0); i < Index(g.NumberOfVertices()); i++ {
		cell := g.vertices[i].pvPtr // cellNumber
		cellVertices[cell] = append(cellVertices[cell], struct {
			vertex        Vertex
			originalIndex Index
		}{vertex: g.vertices[i], originalIndex: i})

		oEdges[i] = make([]OutEdge, g.GetOutDegree(i))
		iEdges[i] = make([]InEdge, g.GetInDegree(i))

		k := Index(0)
		e := g.vertices[i].firstOut
		for e < g.vertices[i+1].firstOut {
			oEdges[i][k] = g.outEdges[e]
			e++
			k++
		}

		k = Index(0)
		e = g.vertices[i].firstIn
		for e < g.vertices[i+1].firstIn {
			iEdges[i][k] = g.inEdges[e]
			e++
			k++
		}

		numOutEdgesInCell[cell] += g.GetOutDegree(i)
		numInEdgesInCell[cell] += g.GetInDegree(i)

		if maxEdgesInCell < numOutEdgesInCell[cell] {
			maxEdgesInCell = numOutEdgesInCell[cell]
		}

		if maxEdgesInCell < numInEdgesInCell[cell] {
			maxEdgesInCell = numInEdgesInCell[cell]
		}
	}

	newId := make([]Index, g.NumberOfVertices()) // map from original vertex id to new vertex id
	newVId := 0                                  // new vertice id

	// create new vertex id
	for i := Index(0); i < Index(len(cellVertices)); i++ {
		for v := 0; v < len(cellVertices[i]); v++ {
			newId[cellVertices[i][v].originalIndex] = Index(newVId)
			newVId++
		}
	}

	vId := Index(0)
	outOffset := Index(0)                                   // new offset for outEdges for each vertex for each cell
	g.outEdgeCellOffset = make([]Index, len(g.cellNumbers)) // offset of first outEdge for each cell
	inOffset := Index(0)                                    // new offset for inEdges for each vertex for each cell
	g.inEdgeCellOffset = make([]Index, len(g.cellNumbers))  // offset of first outEdge for each cell

	// sort vertices by cell number
	for i := Index(0); i < Index(len(g.cellNumbers)); i++ {
		g.outEdgeCellOffset[i] = outOffset
		g.inEdgeCellOffset[i] = inOffset

		for v := Index(0); v < Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number
			g.vertices[vId] = cellVertices[i][v].vertex
			vOldId := cellVertices[i][v].originalIndex
			g.vertices[vId].id = Index(vId)
			g.vertices[vId].firstOut = outOffset
			g.vertices[vId].firstIn = inOffset

			// update outedges & inedges
			for k := Index(0); k < Index(len(oEdges[vOldId])); k++ {
				g.outEdges[outOffset] = oEdges[vOldId][k]
				g.outEdges[outOffset].head = newId[g.outEdges[outOffset].head]
				outOffset++
			}
			for k := Index(0); k < Index(len(iEdges[vOldId])); k++ {
				g.inEdges[inOffset] = iEdges[vOldId][k]
				g.inEdges[inOffset].tail = newId[g.inEdges[inOffset].tail]
				inOffset++
			}

			vId++
		}
	}

}

func (g *Graph) WriteGraph(filename string) error {
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

	fmt.Fprintf(w, "%d %d %d %d\n",
		g.NumberOfVertices(), g.NumberOfEdges(),
		g.GetNumberOfCellsNumbers(), g.GetNumberOfOverlayVertexMapping())

	for _, v := range g.GetVertices() {
		latF := strconv.FormatFloat(v.lat, 'f', -1, 64)
		lonF := strconv.FormatFloat(v.lon, 'f', -1, 64)

		fmt.Fprintf(w, "%d %d %d %d %s %s\n",
			v.pvPtr, v.turnTablePtr, v.firstOut, v.firstIn, latF, lonF)
	}

	for _, v := range g.outEdges {
		weightF := strconv.FormatFloat(v.weight, 'f', -1, 64)
		distF := strconv.FormatFloat(v.dist, 'f', -1, 64)

		fmt.Fprintf(w, "%d %d %s %s %d\n",
			v.edgeId, v.head, weightF, distF, v.entryPoint)
	}

	for _, v := range g.inEdges {
		weightF := strconv.FormatFloat(v.weight, 'f', -1, 64)
		distF := strconv.FormatFloat(v.dist, 'f', -1, 64)

		fmt.Fprintf(w, "%d %d %s %s %d\n",
			v.edgeId, v.tail, weightF, distF, v.exitPoint)
	}

	for _, cellNumber := range g.cellNumbers {
		fmt.Fprintf(w, "%d\n", cellNumber)
	}

	for i, turnType := range g.turnTables {
		fmt.Fprintf(w, "%d", turnType)
		if i < len(g.turnTables)-1 {
			fmt.Fprintf(w, " ")
		}
	}

	fmt.Fprintf(w, "\n")

	for origVertex, overlayVertex := range g.overlayVertices {
		fmt.Fprintf(w, "%d %d %t %d\n",
			origVertex.originalID, origVertex.turnOrder, origVertex.exit, overlayVertex)
	}

	fmt.Fprintf(w, "%d\n", g.maxEdgesInCell)
	for i := 0; i < len(g.outEdgeCellOffset); i++ {
		fmt.Fprintf(w, "%d", g.outEdgeCellOffset[i])
		if i < len(g.outEdgeCellOffset)-1 {
			fmt.Fprintf(w, " ")
		}
	}

	fmt.Fprintf(w, "\n")

	for i := 0; i < len(g.inEdgeCellOffset); i++ {
		fmt.Fprintf(w, "%d", g.inEdgeCellOffset[i])
		if i < len(g.inEdgeCellOffset)-1 {
			fmt.Fprintf(w, " ")
		}
	}

	fmt.Fprintf(w, "\n")

	return w.Flush()
}
func fields(s string) []string {

	return strings.Fields(s)
}

func parseIndex(s string) (Index, error) {
	u, err := strconv.ParseUint(s, 10, 64)
	if err != nil {
		return 0, err
	}
	if u > math.MaxUint32 {
		return 0, fmt.Errorf("value %s overflows uint32", s)
	}
	return Index(u), nil
}

func ReadGraph(filename string) (*Graph, error) {
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
	if len(tokens) != 4 {
		return nil, err
	}

	numVertices, err := parseIndex(tokens[0])
	if err != nil {
		return nil, err
	}

	numEdges, err := parseIndex(tokens[1])
	if err != nil {
		return nil, err
	}
	numCellNumbers, err := parseIndex(tokens[2])
	if err != nil {
		return nil, err
	}
	numOverlayMappings, err := parseIndex(tokens[3])
	if err != nil {
		return nil, err
	}

	vertices := make([]Vertex, numVertices+1)

	for i := 0; i < int(numVertices); i++ {
		vertexLine, err := readLine()
		if err != nil {
			return nil, err
		}
		vertices[i], err = parseVertex(vertexLine)
		if err != nil {
			return nil, err
		}
	}

	outEdges := make([]OutEdge, numEdges)
	for i := 0; i < int(numEdges); i++ {
		outEdgeLine, err := readLine()
		if err != nil {
			return nil, err
		}
		outEdges[i], err = parseOutEdge(outEdgeLine)
		if err != nil {
			return nil, err
		}
	}

	inEdges := make([]InEdge, numEdges)
	for i := 0; i < int(numEdges); i++ {
		inEdgeLine, err := readLine()
		if err != nil {
			return nil, err
		}
		inEdges[i], err = parseInEdge(inEdgeLine)
		if err != nil {
			return nil, err
		}
	}

	cellNumbers := make([]Pv, numCellNumbers)
	for i := 0; i < int(numCellNumbers); i++ {
		cnLine, err := readLine()
		if err != nil {
			return nil, err
		}
		cellNumber, err := strconv.ParseUint(cnLine, 10, 64)
		if err != nil {
			return nil, err
		}
		cellNumbers[i] = Pv(cellNumber)
	}

	turnTables := make([]TurnType, 0)
	line, err = readLine()
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	for _, token := range tokens {
		tt, err := strconv.ParseUint(token, 10, 8)
		if err != nil {
			return nil, err
		}
		turnTables = append(turnTables, TurnType(tt))
	}

	overlayVertices := make(map[SubVertex]Index)
	for i := 0; i < int(numOverlayMappings); i++ {
		overlayLine, err := readLine()
		if err != nil {
			return nil, err
		}
		tokens = fields(overlayLine)
		if len(tokens) != 4 {
			return nil, fmt.Errorf("expected 4 fields, got %d", len(tokens))
		}
		origID, err := parseIndex(tokens[0])
		if err != nil {
			return nil, err
		}
		turnOrder, err := strconv.ParseUint(tokens[1], 10, 8)
		if err != nil {
			return nil, err
		}
		exit, err := strconv.ParseBool(tokens[2])
		if err != nil {
			return nil, err
		}
		overlayId, err := parseIndex(tokens[3])
		if err != nil {
			return nil, err
		}
		subV := SubVertex{
			originalID: origID,
			turnOrder:  uint8(turnOrder),
			exit:       exit,
		}
		overlayVertices[subV] = overlayId
	}

	line, err = readLine()
	if err != nil {
		return nil, err
	}
	maxEdgesInCell, err := parseIndex(strings.TrimSpace(line))
	if err != nil {
		return nil, err
	}

	line, err = readLine()
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	if len(tokens) != int(numCellNumbers) {
		return nil, fmt.Errorf("expected %d out edge cell offsets, got %d", numCellNumbers, len(tokens))
	}
	outEdgeCellOffset := make([]Index, numCellNumbers)
	for i, token := range tokens {
		offset, err := parseIndex(token)
		if err != nil {
			return nil, err
		}
		outEdgeCellOffset[i] = offset
	}

	line, err = readLine()
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	if len(tokens) != int(numCellNumbers) {
		return nil, fmt.Errorf("expected %d in edge cell offsets, got %d", numCellNumbers, len(tokens))
	}
	inEdgeCellOffset := make([]Index, numCellNumbers)
	for i, token := range tokens {
		offset, err := parseIndex(token)
		if err != nil {
			return nil, err
		}
		inEdgeCellOffset[i] = offset
	}

	graph := NewGraph(vertices, outEdges, inEdges, turnTables)
	graph.SetCellNumbers(cellNumbers)
	graph.SetOverlayMapping(overlayVertices)
	graph.maxEdgesInCell = maxEdgesInCell
	graph.outEdgeCellOffset = outEdgeCellOffset
	graph.inEdgeCellOffset = inEdgeCellOffset
	return graph, nil
}

func parseVertex(line string) (Vertex, error) {
	emptyVertex := Vertex{}
	tokens := fields(line)
	if len(tokens) != 6 {
		return emptyVertex, fmt.Errorf("expected 6 fields, got %d", len(tokens))
	}
	pvPtr, err := parseIndex(tokens[0])
	if err != nil {
		return emptyVertex, err
	}
	ttPtr, err := parseIndex(tokens[1])
	if err != nil {
		return emptyVertex, err
	}
	firstOut, err := parseIndex(tokens[2])
	if err != nil {
		return emptyVertex, err
	}
	firstIn, err := parseIndex(tokens[3])
	if err != nil {
		return emptyVertex, err
	}
	lat, err := strconv.ParseFloat(tokens[4], 64)
	if err != nil {
		return emptyVertex, fmt.Errorf("lat: %w", err)
	}
	lon, err := strconv.ParseFloat(tokens[5], 64)
	if err != nil {
		return emptyVertex, fmt.Errorf("lon: %w", err)
	}
	return Vertex{
		pvPtr: pvPtr, turnTablePtr: ttPtr,
		firstOut: firstOut, firstIn: firstIn,
		lat: lat, lon: lon,
	}, nil
}

func parseOutEdge(line string) (OutEdge, error) {
	emptyEdge := OutEdge{}
	tokens := fields(line)
	if len(tokens) != 5 {
		return emptyEdge, fmt.Errorf("expected 5 fields, got %d", len(tokens))
	}
	edgeId, err := parseIndex(tokens[0])
	if err != nil {
		return emptyEdge, err
	}
	head, err := parseIndex(tokens[1])
	if err != nil {
		return emptyEdge, err
	}
	weight, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return emptyEdge, err
	}
	dist, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return emptyEdge, err
	}

	entryPoint, err := strconv.ParseUint(tokens[4], 10, 8)
	if err != nil {
		return emptyEdge, err
	}

	return NewOutEdge(edgeId, head, weight, dist, uint8(entryPoint)), nil
}

func parseInEdge(line string) (InEdge, error) {
	emptyEdge := InEdge{}
	tokens := fields(line)
	if len(tokens) != 5 {
		return emptyEdge, fmt.Errorf("expected 5 fields, got %d", len(tokens))
	}
	edgeId, err := parseIndex(tokens[0])
	if err != nil {
		return emptyEdge, err
	}
	tail, err := parseIndex(tokens[1])
	if err != nil {
		return emptyEdge, err
	}
	weight, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return emptyEdge, err
	}
	dist, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return emptyEdge, err
	}

	exitPoint, err := strconv.ParseUint(tokens[4], 10, 8)
	if err != nil {
		return emptyEdge, err
	}

	return NewInEdge(edgeId, tail, weight, dist, uint8(exitPoint)), nil
}

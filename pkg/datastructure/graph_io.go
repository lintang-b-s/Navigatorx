package datastructure

import (
	"bufio"
	"fmt"
	"io"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"

	"github.com/cockroachdb/errors"

	"github.com/bits-and-blooms/bitset"
	"github.com/klauspost/compress/s2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

func (g *Graph) WriteGraph(filename string) error {
	f, err := os.Create(filename)
	if err != nil {
		return errors.Wrapf(err, "WriteGraph: failed to create file: %s", filename)
	}
	defer f.Close()

	snp := s2.NewWriter(f)
	defer snp.Close()

	w := bufio.NewWriter(snp)

	// graph header
	if _, err = fmt.Fprintf(w, "%d %d %d %d\n",
		len(g.vertices), g.NumberOfEdges(),
		g.GetNumberOfCellsNumbers(), g.GetNumberOfOverlayVertexMapping()); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writing graph headers")
	}

	// graph vertices
	for i, v := range g.vertices {
		latF := strconv.FormatFloat(v.lat, 'f', -1, 64)
		lonF := strconv.FormatFloat(v.lon, 'f', -1, 64)

		if _, err = fmt.Fprintf(w, "%d %d %d %d %d %s %s %d\n",
			v.pvPtr, v.turnTablePtr, v.firstOut, v.firstIn, v.id, latF, lonF, v.osmId); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing vertex[%d]", i)
		}
	}

	// out edges
	for i, e := range g.outEdges {
		if _, err = fmt.Fprintf(w, "%d %d %s %s %d %d\n",
			e.edgeId, e.head,
			strconv.FormatFloat(e.weight, 'f', -1, 64),
			strconv.FormatFloat(e.dist, 'f', -1, 64),
			e.entryPoint, e.hwType,
		); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing outEdge[%d]", i)
		}
	}

	// in edges
	for i, e := range g.inEdges {
		if _, err = fmt.Fprintf(w, "%d %d %s %s %d %d\n",
			e.edgeId, e.tail,
			strconv.FormatFloat(e.weight, 'f', -1, 64),
			strconv.FormatFloat(e.dist, 'f', -1, 64),
			e.exitPoint, e.hwType); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing inEdge[%d]", i)
		}
	}

	// cell numbers
	for i, cn := range g.cellNumbers {
		if _, err = fmt.Fprintf(w, "%d\n", cn); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing cellNumber[%d]", i)
		}
	}

	// turn tables
	for i, tt := range g.turnTables {
		if _, err = fmt.Fprintf(w, "%d", tt); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing turnTable[%d]", i)
		}
		if i < len(g.turnTables)-1 {
			if _, err = fmt.Fprintf(w, " "); err != nil {
				return err
			}
		}
	}
	if _, err = fmt.Fprintf(w, "\n"); err != nil {
		return err
	}

	// overlay
	for k, v := range g.overlayVertices {
		if _, err = fmt.Fprintf(w, "%d %d %t %d\n",
			k.originalID, k.exitEntryOrder, k.exit, v); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing overlayVertices")
		}
	}

	// cell offsets
	if _, err = fmt.Fprintf(w, "%d\n", g.maxEdgesInCell); err != nil {
		return err
	}

	for i, val := range g.outEdgeCellOffset {
		if _, err = fmt.Fprintf(w, "%d", val); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing outEdgeCellOffset[%d]", i)
		}
		if i < len(g.outEdgeCellOffset)-1 {
			if _, err = fmt.Fprintf(w, " "); err != nil {
				return err
			}
		}
	}
	if _, err = fmt.Fprintf(w, "\n"); err != nil {
		return err
	}

	for i, val := range g.inEdgeCellOffset {
		if _, err = fmt.Fprintf(w, "%d", val); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing inEdgeCellOffset[%d]", i)
		}
		if i < len(g.inEdgeCellOffset)-1 {
			if _, err = fmt.Fprintf(w, " "); err != nil {
				return err
			}
		}
	}
	if _, err = fmt.Fprintf(w, "\n"); err != nil {
		return err
	}

	// sccs
	for i, val := range g.sccs {
		if _, err = fmt.Fprintf(w, "%d", val); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing scc[%d]", i)
		}
		if i < len(g.sccs)-1 {
			if _, err = fmt.Fprintf(w, " "); err != nil {
				return err
			}
		}
	}
	if _, err = fmt.Fprintf(w, "\n"); err != nil {
		return err
	}

	// bounding box
	if _, err = fmt.Fprintf(w, "%s %s %s %s\n",
		strconv.FormatFloat(g.boundingBox.GetMinLat(), 'f', -1, 64),
		strconv.FormatFloat(g.boundingBox.GetMinLon(), 'f', -1, 64),
		strconv.FormatFloat(g.boundingBox.GetMaxLat(), 'f', -1, 64),
		strconv.FormatFloat(g.boundingBox.GetMaxLon(), 'f', -1, 64)); err != nil {
		return err
	}

	// graph storage

	// osm node points
	if _, err = fmt.Fprintf(w, "%d\n", len(g.graphStorage.osmNodePoints)); err != nil { // ← move here
		return errors.Wrapf(err, "WriteGraph: failed writing osmNodePoints count")
	}

	for i, p := range g.graphStorage.osmNodePoints {
		if _, err = fmt.Fprintf(w, "%s %s\n",
			strconv.FormatFloat(p.Lat, 'f', -1, 64),
			strconv.FormatFloat(p.Lon, 'f', -1, 64)); err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing osmNodePoints[%d]", i)
		}
	}

	// map edge infos

	_, err = fmt.Fprintf(w, "%d\n", len(g.graphStorage.edgeInfos))
	if err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writing g.graphStorage.edgeInfos length %v", len(g.graphStorage.edgeInfos))

	}
	for i := 0; i < len(g.graphStorage.edgeInfos); i++ {
		edgeInfo := g.graphStorage.edgeInfos[i]
		_, err = fmt.Fprintf(w, "%d %d %d %d %d %d %d\n", edgeInfo.startPointsIndex, edgeInfo.endPointsIndex,
			edgeInfo.streetName, edgeInfo.roadClass, edgeInfo.roadClassLink, edgeInfo.lanes,
			edgeInfo.osmWayId)
		if err != nil {
			return errors.Wrapf(err, "WriteGraph: failed writing edgeInfos[%d]", i)
		}
	}

	// flags
	if _, err := g.graphStorage.roundaboutFlag.WriteTo(w); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writing roundaboutFlag")
	}

	if _, err := fmt.Fprintf(w, "\n"); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writid new line")
	}

	if _, err := g.graphStorage.nodeTrafficLight.WriteTo(w); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writing nodeTrafficLight")
	}
	if _, err := fmt.Fprintf(w, "\n"); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writid new line")
	}

	if _, err := g.graphStorage.streetDirectionForward.WriteTo(w); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writing streetDirectionForward")
	}
	if _, err := fmt.Fprintf(w, "\n"); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writid new line")
	}

	if _, err := g.graphStorage.streetDirectionBackward.WriteTo(w); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writing streetDirectionBackward")
	}

	if _, err := fmt.Fprintf(w, "\n"); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed writid new line")
	}

	// tag string

	sortedKeys := make([]uint32, 0)
	for key, _ := range g.graphStorage.tagStringIDMap.GetIdToStr() {

		sortedKeys = append(sortedKeys, key)
	}

	_, err = fmt.Fprintf(w, "%d\n", len(sortedKeys))
	if err != nil {
		return errors.Wrapf(err, "WriteGraph: failed to write tagString keys length: %v", len(sortedKeys))
	}

	sort.Slice(sortedKeys, func(i, j int) bool {
		return sortedKeys[i] < sortedKeys[j]
	})

	for _, key := range sortedKeys {
		val := g.graphStorage.tagStringIDMap.GetStr(key)

		_, err = fmt.Fprintf(w, "%d %s\n", key, strconv.Quote(val))
		if err != nil {
			return errors.Wrapf(err, "WriteGraph: failed to write key, strconv.Quote(val): %v, %v", key, strconv.Quote(val))
		}
	}

	// write scc condensation
	for i := 0; i < len(g.sccCondensationAdj); i++ {
		for j := 0; j < len(g.sccCondensationAdj[i]); j++ {
			_, err = fmt.Fprintf(w, "%d", g.sccCondensationAdj[i][j])
			if err != nil {
				return errors.Wrapf(err, "WriteGraph: failed to write  g.sccCondensationAdj[i][j]): %v", g.sccCondensationAdj[i][j])
			}

			if j < len(g.sccCondensationAdj[i])-1 {
				_, err = fmt.Fprintf(w, " ")
				if err != nil {
					return errors.Wrapf(err, "WriteGraph: failed to write new space")
				}
			}
		}

		if len(g.sccCondensationAdj[i]) == 0 {
			_, err = fmt.Fprintf(w, "empty")
			if err != nil {
				return errors.Wrapf(err, "WriteGraph: failed to write empty")
			}

		}

		_, err = fmt.Fprintf(w, "\n")
		if err != nil {
			return errors.Wrapf(err, "WriteGraph: failed to write new line")
		}
	}

	if err = w.Flush(); err != nil {
		return errors.Wrapf(err, "WriteGraph: failed to flush bufio writer")
	}

	return nil
}

func ParseIndex(s string) (Index, error) {
	u, err := strconv.ParseUint(s, 10, 32)
	if err != nil {
		return 0, errors.Wrapf(err, "ParseIndex strconv.ParseUint: error parsing string to integer %s", s)
	}
	if u > math.MaxUint32 {
		return 0, fmt.Errorf("ParseIndex: value %s overflows uint32", s)
	}
	return Index(u), nil
}

const (
	graphBufferSize = 4096 * 4
)

func ReadGraph(filename string) (*Graph, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed opening file: %s", filename)
	}

	defer f.Close()

	snp := s2.NewReader(f)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed creating s2.NewReader %s", filename)
	}

	br := bufio.NewReaderSize(snp, graphBufferSize)

	line, err := util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed reading line: %s", filename)
	}

	tokens := util.Fields(line)
	if len(tokens) != 4 {
		return nil, errors.Newf("ReadGraph: number of graph headers is not correct, expected: %v, got: %v", 4, len(tokens))
	}

	numVertices, err := ParseIndex(tokens[0])
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed parsing number of vertices: %v", tokens[0])
	}

	numEdges, err := ParseIndex(tokens[1])
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed parsing number of edges: %v", tokens[1])
	}
	numCellNumbers, err := ParseIndex(tokens[2])
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed parsing number of cells: %v", tokens[2])
	}
	numOverlayMappings, err := ParseIndex(tokens[3])
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed parsing number of overlay mappings: %v", tokens[3])
	}

	vertices := make([]Vertex, numVertices)

	for i := 0; i < int(numVertices); i++ {
		vertexLine, err := util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to read vertexLine")
		}
		vertices[i], err = parseVertex(vertexLine)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parse vertex: %v", vertexLine)
		}
	}

	outEdges := make([]OutEdge, numEdges)
	for i := 0; i < int(numEdges); i++ {
		outEdgeLine, err := util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to read outEdgeLine")
		}
		outEdges[i], err = parseOutEdge(outEdgeLine)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parse outEdge: %v", outEdgeLine)
		}
	}

	inEdges := make([]InEdge, numEdges)
	for i := 0; i < int(numEdges); i++ {
		inEdgeLine, err := util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to read inEdgeLine")
		}
		inEdges[i], err = parseInEdge(inEdgeLine)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parse inEdge: %v", inEdgeLine)
		}
	}

	cellNumbers := make([]Pv, numCellNumbers)
	for i := 0; i < int(numCellNumbers); i++ {
		cnLine, err := util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to read cnLine")
		}
		cellNumber, err := strconv.ParseUint(cnLine, 10, 64)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parse cellNumber: %v", cnLine)
		}
		cellNumbers[i] = Pv(cellNumber)
	}

	turnTables := make([]pkg.TurnType, 0)
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read turntables string")
	}
	tokens = util.Fields(line)
	for _, token := range tokens {
		tt, err := strconv.ParseUint(token, 10, 8)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parse uint turnTables: %v", token)
		}
		turnTables = append(turnTables, pkg.TurnType(tt))
	}

	overlayVertices := make(map[SubVertex]Index)
	for i := 0; i < int(numOverlayMappings); i++ {
		overlayLine, err := util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to read overlayLine")
		}
		tokens = util.Fields(overlayLine)
		if len(tokens) != 4 {
			return nil, fmt.Errorf("expected 4 overlayLine fields, got %d", len(tokens))
		}
		origID, err := ParseIndex(tokens[0])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseIndex origId: %v", tokens[0])
		}
		exitEntryOrder, err := strconv.ParseUint(tokens[1], 10, 32)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseUint exitEntryOrder: %v", tokens[1])
		}
		exit, err := strconv.ParseBool(tokens[2])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseBool exit: %v", tokens[2])
		}
		overlayId, err := ParseIndex(tokens[3])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseIndex overlayId: %v", tokens[3])
		}
		subV := SubVertex{
			originalID:     origID,
			exitEntryOrder: Index(exitEntryOrder),
			exit:           exit,
		}
		overlayVertices[subV] = overlayId
	}

	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine maxEdgesIncell")
	}
	maxEdgesInCell, err := ParseIndex(strings.TrimSpace(line))
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ParseIndex maxEdgesInCell: %v", line)
	}

	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine numCellNumbers")
	}
	tokens = util.Fields(line)
	if len(tokens) != int(numCellNumbers) {
		return nil, fmt.Errorf("expected %d numCellNumbers, got %d", numCellNumbers, len(tokens))
	}

	outEdgeCellOffset := make([]Index, numCellNumbers)
	for i, token := range tokens {
		offset, err := ParseIndex(token)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseIndex outEdgeCellOffset: %v", token)
		}
		outEdgeCellOffset[i] = offset
	}

	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine numCellNumbers")
	}
	tokens = util.Fields(line)
	if len(tokens) != int(numCellNumbers) {
		return nil, fmt.Errorf("expected %d numCellNumbers, got %d", numCellNumbers, len(tokens))
	}
	inEdgeCellOffset := make([]Index, numCellNumbers)
	for i, token := range tokens {
		offset, err := ParseIndex(token)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseIndex inEdgeCellOffset: %v", token)
		}
		inEdgeCellOffset[i] = offset
	}

	// read sccs
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine sccs")
	}
	tokens = util.Fields(line)
	if len(tokens) != int(numVertices-1) {
		return nil, fmt.Errorf("expected %d vertices, got %d", numVertices, len(tokens))
	}
	sccs := make([]Index, numVertices-1)
	for i, token := range tokens {
		scc, err := ParseIndex(token)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseIndex scc: %v", token)
		}
		sccs[i] = scc
	}

	// read bounding box
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine bounding box")
	}
	tokens = util.Fields(line)

	minLat, err := strconv.ParseFloat(tokens[0], 64)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ParseFloat min latitude %v", tokens[0])
	}
	minLon, err := strconv.ParseFloat(tokens[1], 64)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ParseFloat min longitude %v", tokens[1])
	}
	maxLat, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ParseFloat max latitude %v", tokens[2])
	}
	maxLon, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ParseFloat max longitude %v", tokens[3])
	}
	bb := NewBoundingBox(minLat, minLon, maxLat, maxLon)

	// read graph storage

	// osm way geometry points
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine osm way geometry points")
	}

	tokens = util.Fields(line)
	numOsmNodePoints, err := util.ParseInt(tokens[0])
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt numOsmNodePoints: %v", tokens[0])
	}

	osmNodePoints := make([]Coordinate, numOsmNodePoints)
	for i := 0; i < numOsmNodePoints; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine osm way geometry point")
		}
		tokens = util.Fields(line)

		lat, err := strconv.ParseFloat(tokens[0], 64)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseFloat osm way geometry point latitude: %v", tokens[0])
		}
		lon, err := strconv.ParseFloat(tokens[1], 64)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseFloat osm way geometry point longitude: %v", tokens[1])
		}
		osmNodePoints[i] = NewCoordinate(lat, lon)
	}

	// map edge info flag
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to readLine edgeInfos headers")
	}

	tokens = util.Fields(line)
	numEdgeInfos, err := util.ParseInt(tokens[0])
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt numEdgeInfos: %v", tokens[0])
	}

	edgeInfos := make([]EdgeExtraInfo, numEdgeInfos)
	for i := 0; i < numEdgeInfos; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to readLine mapEdgeInfo")
		}
		tokens = util.Fields(line)
		startsPointIndex, err := util.ParseInt(tokens[0])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt startsPointIndex: %v", tokens[0])
		}

		endPointIndex, err := util.ParseInt(tokens[1])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt endPointIndex: %v", tokens[1])
		}
		streetName, err := util.ParseUInt32(tokens[2])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseUInt32 streetName: %v", tokens[2])
		}
		roadClass, err := util.ParseUInt32(tokens[3])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseUInt32 roadClass: %v", tokens[3])
		}
		roadClassLink, err := util.ParseUInt32(tokens[4])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to ParseUInt32 roadClassLink: %v", tokens[4])
		}
		lanes, err := util.ParseInt(tokens[5])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt lanes: %v", tokens[5])
		}
		osmWayId, err := util.ParseInt(tokens[6])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt osmWayId: %v", tokens[6])
		}
		edgeInfos[i] = NewEdgeExtraInfo(streetName, roadClass,
			roadClassLink, uint8(lanes), Index(startsPointIndex), Index(endPointIndex),
			int64(osmWayId))
	}

	// roundabout flag
	roundaboutFlags := bitset.New(uint(numEdges))
	if _, err = roundaboutFlags.ReadFrom(br); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read bitset roundaboutFlags")
	}
	if _, err = br.ReadByte(); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read newline after roundaboutFlags")
	}

	// traffic light flag
	trafficLightFlags := bitset.New(uint(numEdges))
	if _, err = trafficLightFlags.ReadFrom(br); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read bitset trafficLightFlags")
	}
	if _, err = br.ReadByte(); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read newline after trafficLightFlags")
	}

	// street direction forward flag
	stretDirectionsForward := bitset.New(uint(numEdges))
	if _, err = stretDirectionsForward.ReadFrom(br); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read bitset streetDirectionsForward")
	}
	if _, err = br.ReadByte(); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read newline after streetDirectionsForward")
	}

	// street direction backward flag
	stretDirectionsBackward := bitset.New(uint(numEdges))
	if _, err = stretDirectionsBackward.ReadFrom(br); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read bitset streetDirectionsBackward")
	}

	if _, err = br.ReadByte(); err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to read newline after streetDirectionsBackward")
	}

	// tagstring idmap flag
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to readLine tagstring")
	}

	tokens = util.Fields(line)
	tagStringIdMap := util.NewIdMap()
	numIdMapItems, err := util.ParseInt(tokens[0])
	idToStr := make(map[uint32]string)

	if err != nil {
		return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt numIdMapItems: %v", tokens[0])
	}
	for i := 0; i < numIdMapItems; i++ {

		line, err = util.ReadLine(br)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to readLine idMapItem")
		}

		tokens = util.Fields(line)
		if len(tokens) < 2 {
			continue
		}

		val := tokens[1]

		if len(tokens) > 2 {
			for i := 2; i < len(tokens); i++ {
				val += " " + tokens[i]
			}
		}

		unquotedVal, err := strconv.Unquote(val)
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to Unquote: %v", val)
		}
		key, err := util.ParseUInt32(tokens[0])
		if err != nil {
			return nil, errors.Wrapf(err, "ReadGraph: failed to parseInt tagstring key: %v", tokens[0])
		}
		idToStr[key] = unquotedVal
	}

	sccCondensationAdj := make([][]Index, 0)
	for {
		line, err = util.ReadLine(br)
		if err != nil {
			if errors.Is(err, io.EOF) {
				break
			}
			return nil, errors.Wrapf(err, "ReadGraph: failed to ReadLine sccCondensationAdj")
		}
		tokens = util.Fields(line)
		if len(tokens) == 0 {
			continue
		}
		adj := make([]Index, 0)
		if tokens[0] != "empty" {
			for _, token := range tokens {
				scc, err := ParseIndex(token)
				if err != nil {
					return nil, errors.Wrapf(err, "ReadGraph: failed to ParseIndex scc: %v", token)
				}
				adj = append(adj, scc)
			}
		}
		sccCondensationAdj = append(sccCondensationAdj, adj)
	}

	graphStorage := BuildGraphStorage(osmNodePoints,
		roundaboutFlags, trafficLightFlags, edgeInfos,
		tagStringIdMap, stretDirectionsForward, stretDirectionsBackward)
	graphStorage.tagStringIDMap.ToStringArray(idToStr)

	graph := NewGraph(vertices, outEdges, inEdges, turnTables)
	graph.SetGraphStorage(graphStorage)
	graph.SetCellNumbers(cellNumbers)
	graph.SetOverlayMapping(overlayVertices)
	graph.maxEdgesInCell = maxEdgesInCell
	graph.outEdgeCellOffset = outEdgeCellOffset
	graph.inEdgeCellOffset = inEdgeCellOffset
	graph.SetSCCs(sccs)
	graph.SetSCCCondensationAdj(sccCondensationAdj)
	graph.SetBoundingBox(bb)

	return graph, nil
}

func parseVertex(line string) (Vertex, error) {
	tokens := util.Fields(line)
	if len(tokens) != 8 {
		return NewEmptyVertex(), fmt.Errorf("expected 8 fields, got %d", len(tokens))
	}
	pvPtr, err := ParseIndex(tokens[0])
	if err != nil {
		return NewEmptyVertex(), err
	}
	ttPtr, err := ParseIndex(tokens[1])
	if err != nil {
		return NewEmptyVertex(), err
	}
	firstOut, err := ParseIndex(tokens[2])
	if err != nil {
		return NewEmptyVertex(), err
	}
	firstIn, err := ParseIndex(tokens[3])
	if err != nil {
		return NewEmptyVertex(), err
	}

	id, err := ParseIndex(tokens[4])
	if err != nil {
		return NewEmptyVertex(), err
	}

	lat, err := strconv.ParseFloat(tokens[5], 64)
	if err != nil {
		return NewEmptyVertex(), fmt.Errorf("lat: %w", err)
	}
	lon, err := strconv.ParseFloat(tokens[6], 64)
	if err != nil {
		return NewEmptyVertex(), fmt.Errorf("lon: %w", err)
	}

	osmId, err := strconv.ParseUint(tokens[7], 10, 64)
	if err != nil {
		return NewEmptyVertex(), fmt.Errorf("osmId: %w", err)
	}

	return Vertex{
		pvPtr: pvPtr, turnTablePtr: ttPtr,
		firstOut: firstOut, firstIn: firstIn,
		lat: lat, lon: lon, id: id, osmId: osmId,
	}, nil
}

func parseOutEdge(line string) (OutEdge, error) {
	tokens := util.Fields(line)
	if len(tokens) != 6 {
		return NewEmptyOutEdge(), fmt.Errorf("expected 6 fields, got %d", len(tokens))
	}
	edgeId, err := ParseIndex(tokens[0])
	if err != nil {
		return NewEmptyOutEdge(), err
	}
	head, err := ParseIndex(tokens[1])
	if err != nil {
		return NewEmptyOutEdge(), err
	}
	weight, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return NewEmptyOutEdge(), err
	}
	dist, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return NewEmptyOutEdge(), err
	}

	entryPoint, err := strconv.ParseUint(tokens[4], 10, 32)
	if err != nil {
		return NewEmptyOutEdge(), err
	}

	hwType, err := strconv.ParseUint(tokens[5], 10, 8)
	if err != nil {
		return NewEmptyOutEdge(), err
	}

	e := NewOutEdge(edgeId, head, weight, dist, Index(entryPoint), pkg.OsmHighwayType(hwType))
	return e, nil
}

func parseInEdge(line string) (InEdge, error) {
	tokens := util.Fields(line)
	if len(tokens) != 6 {
		return NewEmptyInEdge(), fmt.Errorf("expected 6 fields, got %d", len(tokens))
	}
	edgeId, err := ParseIndex(tokens[0])
	if err != nil {
		return NewEmptyInEdge(), err
	}
	tail, err := ParseIndex(tokens[1])
	if err != nil {
		return NewEmptyInEdge(), err
	}
	weight, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return NewEmptyInEdge(), err
	}
	dist, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return NewEmptyInEdge(), err
	}

	exitPoint, err := strconv.ParseUint(tokens[4], 10, 32)
	if err != nil {
		return NewEmptyInEdge(), err
	}

	hwType, err := strconv.ParseUint(tokens[5], 10, 8)
	if err != nil {
		return NewEmptyInEdge(), err
	}

	e := NewInEdge(edgeId, tail, weight, dist, Index(exitPoint), pkg.OsmHighwayType(hwType))
	return e, nil
}

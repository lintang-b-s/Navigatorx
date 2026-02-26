package datastructure

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"

	"github.com/dsnet/compress/bzip2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

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

	fmt.Fprintf(w, "%d %d %d %d\n",
		len(g.vertices), g.NumberOfEdges(),
		g.GetNumberOfCellsNumbers(), g.GetNumberOfOverlayVertexMapping())

	for vId := 0; vId < len(g.vertices); vId++ {
		v := g.vertices[vId]
		latF := strconv.FormatFloat(v.lat, 'f', -1, 64)
		lonF := strconv.FormatFloat(v.lon, 'f', -1, 64)

		fmt.Fprintf(w, "%d %d %d %d %d %s %s %d\n",
			v.pvPtr, v.turnTablePtr, v.firstOut, v.firstIn, v.id, latF, lonF, v.osmId)
	}

	for _, v := range g.outEdges {
		weightF := strconv.FormatFloat(v.weight, 'f', -1, 64)
		distF := strconv.FormatFloat(v.dist, 'f', -1, 64)

		fmt.Fprintf(w, "%d %d %s %s %d %d %d\n",
			v.edgeId, v.head, weightF, distF, v.entryPoint, v.oriEdgeId, v.hwType)
	}

	for _, v := range g.inEdges {
		weightF := strconv.FormatFloat(v.weight, 'f', -1, 64)
		distF := strconv.FormatFloat(v.dist, 'f', -1, 64)

		fmt.Fprintf(w, "%d %d %s %s %d %d %d\n",
			v.edgeId, v.tail, weightF, distF, v.exitPoint, v.oriEdgeId, v.hwType)
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
			origVertex.originalID, origVertex.exitEntryOrder, origVertex.exit, overlayVertex)
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

	// write sccs
	for i := 0; i < len(g.sccs); i++ {
		fmt.Fprintf(w, "%d", g.sccs[i])
		if i < len(g.sccs)-1 {
			fmt.Fprintf(w, " ")
		}
	}

	fmt.Fprintf(w, "\n")

	minLat := strconv.FormatFloat(g.boundingBox.GetMinLat(), 'f', -1, 64)
	minLon := strconv.FormatFloat(g.boundingBox.GetMinLon(), 'f', -1, 64)
	maxLat := strconv.FormatFloat(g.boundingBox.GetMaxLat(), 'f', -1, 64)
	maxLon := strconv.FormatFloat(g.boundingBox.GetMaxLon(), 'f', -1, 64)
	fmt.Fprintf(w, "%s %s %s %s\n", minLat, minLon, maxLat, maxLon)

	// write graph storage

	fmt.Fprintf(w, "%d\n", len(g.graphStorage.globalPoints))
	for i := 0; i < len(g.graphStorage.globalPoints); i++ {
		point := g.graphStorage.globalPoints[i]
		pointLat := strconv.FormatFloat(point.Lat, 'f', -1, 64)
		pointLon := strconv.FormatFloat(point.Lon, 'f', -1, 64)
		fmt.Fprintf(w, "%s %s\n", pointLat, pointLon)
	}

	fmt.Fprintf(w, "%d\n", len(g.graphStorage.roundaboutFlag))
	for i := 0; i < len(g.graphStorage.roundaboutFlag); i++ {
		flag := g.graphStorage.roundaboutFlag[i]
		fmt.Fprintf(w, "%d \n", flag)
	}

	fmt.Fprintf(w, "%d\n", len(g.graphStorage.nodeTrafficLight))
	for i := 0; i < len(g.graphStorage.nodeTrafficLight); i++ {
		flag := g.graphStorage.nodeTrafficLight[i]
		fmt.Fprintf(w, "%d \n", flag)
	}

	fmt.Fprintf(w, "%d\n", len(g.graphStorage.mapEdgeInfo))
	for i := 0; i < len(g.graphStorage.mapEdgeInfo); i++ {
		edgeInfo := g.graphStorage.mapEdgeInfo[i]
		fmt.Fprintf(w, "%d %d %d %d %d %d %d\n", edgeInfo.startPointsIndex, edgeInfo.endPointsIndex,
			edgeInfo.streetName, edgeInfo.roadClass, edgeInfo.roadClassLink, edgeInfo.lanes,
			edgeInfo.osmWayId)
	}

	fmt.Fprintf(w, "%d\n", len(g.graphStorage.streetDirection))
	for wayId, streetDir := range g.graphStorage.streetDirection {
		fmt.Fprintf(w, "%d %t %t \n", wayId, streetDir[0], streetDir[1])
	}

	sortedKeys := make([]int, 0)
	for key, _ := range g.graphStorage.tagStringIDMap.IDToStr {

		sortedKeys = append(sortedKeys, key)
	}

	fmt.Fprintf(w, "%d\n", len(sortedKeys))

	sort.Slice(sortedKeys, func(i, j int) bool {
		return sortedKeys[i] < sortedKeys[j]
	})

	for _, key := range sortedKeys {
		val := g.graphStorage.tagStringIDMap.GetStr(key)

		fmt.Fprintf(w, "%d %s\n", key, strconv.Quote(val))
	}

	// write scc condensation
	for i := 0; i < len(g.sccCondensationAdj); i++ {
		for j := 0; j < len(g.sccCondensationAdj[i]); j++ {
			fmt.Fprintf(w, "%d", g.sccCondensationAdj[i][j])
			if j < len(g.sccCondensationAdj[i])-1 {
				fmt.Fprintf(w, " ")
			}
		}
		if len(g.sccCondensationAdj[i]) == 0 {
			fmt.Fprintf(w, "empty")
		}
		fmt.Fprintf(w, "\n")
	}

	return w.Flush()
}

func fields(s string) []string {

	return strings.Fields(s)
}

func ParseIndex(s string) (Index, error) {
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

	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens := fields(line)
	if len(tokens) != 4 {
		return nil, err
	}

	numVertices, err := ParseIndex(tokens[0])
	if err != nil {
		return nil, err
	}

	numEdges, err := ParseIndex(tokens[1])
	if err != nil {
		return nil, err
	}
	numCellNumbers, err := ParseIndex(tokens[2])
	if err != nil {
		return nil, err
	}
	numOverlayMappings, err := ParseIndex(tokens[3])
	if err != nil {
		return nil, err
	}

	vertices := make([]*Vertex, numVertices)

	for i := 0; i < int(numVertices); i++ {
		vertexLine, err := util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		vertices[i], err = parseVertex(vertexLine)
		if err != nil {
			return nil, err
		}
	}

	outEdges := make([]*OutEdge, numEdges)
	for i := 0; i < int(numEdges); i++ {
		outEdgeLine, err := util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		outEdges[i], err = parseOutEdge(outEdgeLine)
		if err != nil {
			return nil, err
		}
	}

	inEdges := make([]*InEdge, numEdges)
	for i := 0; i < int(numEdges); i++ {
		inEdgeLine, err := util.ReadLine(br)
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
		cnLine, err := util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		cellNumber, err := strconv.ParseUint(cnLine, 10, 64)
		if err != nil {
			return nil, err
		}
		cellNumbers[i] = Pv(cellNumber)
	}

	turnTables := make([]pkg.TurnType, 0)
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	for _, token := range tokens {
		tt, err := strconv.ParseUint(token, 10, 8)
		if err != nil {
			return nil, err
		}
		turnTables = append(turnTables, pkg.TurnType(tt))
	}

	overlayVertices := make(map[SubVertex]Index)
	for i := 0; i < int(numOverlayMappings); i++ {
		overlayLine, err := util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		tokens = fields(overlayLine)
		if len(tokens) != 4 {
			return nil, fmt.Errorf("expected 4 fields, got %d", len(tokens))
		}
		origID, err := ParseIndex(tokens[0])
		if err != nil {
			return nil, err
		}
		exitEntryOrder, err := strconv.ParseUint(tokens[1], 10, 8)
		if err != nil {
			return nil, err
		}
		exit, err := strconv.ParseBool(tokens[2])
		if err != nil {
			return nil, err
		}
		overlayId, err := ParseIndex(tokens[3])
		if err != nil {
			return nil, err
		}
		subV := SubVertex{
			originalID:     origID,
			exitEntryOrder: int(exitEntryOrder),
			exit:           exit,
		}
		overlayVertices[subV] = overlayId
	}

	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	maxEdgesInCell, err := ParseIndex(strings.TrimSpace(line))
	if err != nil {
		return nil, err
	}

	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	if len(tokens) != int(numCellNumbers) {
		return nil, fmt.Errorf("expected %d out edge cell offsets, got %d", numCellNumbers, len(tokens))
	}
	outEdgeCellOffset := make([]Index, numCellNumbers)
	for i, token := range tokens {
		offset, err := ParseIndex(token)
		if err != nil {
			return nil, err
		}
		outEdgeCellOffset[i] = offset
	}

	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	if len(tokens) != int(numCellNumbers) {
		return nil, fmt.Errorf("expected %d in edge cell offsets, got %d", numCellNumbers, len(tokens))
	}
	inEdgeCellOffset := make([]Index, numCellNumbers)
	for i, token := range tokens {
		offset, err := ParseIndex(token)
		if err != nil {
			return nil, err
		}
		inEdgeCellOffset[i] = offset
	}

	// read sccs
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	tokens = fields(line)
	if len(tokens) != int(numVertices-1) {
		return nil, fmt.Errorf("expected %d vertices, got %d", numVertices, len(tokens))
	}
	sccs := make([]Index, numVertices-1)
	for i, token := range tokens {
		scc, err := ParseIndex(token)
		if err != nil {
			return nil, err
		}
		sccs[i] = scc
	}

	// read bounding box
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	tokens = fields(line)

	minLat, err := strconv.ParseFloat(tokens[0], 64)
	if err != nil {
		return nil, fmt.Errorf("lat: %w", err)
	}
	minLon, err := strconv.ParseFloat(tokens[1], 64)
	if err != nil {
		return nil, fmt.Errorf("lon: %w", err)
	}
	maxLat, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return nil, fmt.Errorf("lat: %w", err)
	}
	maxLon, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return nil, fmt.Errorf("lon: %w", err)
	}
	bb := NewBoundingBox(minLat, minLon, maxLat, maxLon)

	// read graph storage
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	numGlobalPoints := parseInt(tokens[0])
	globalPoints := make([]Coordinate, numGlobalPoints)
	for i := 0; i < numGlobalPoints; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		tokens = fields(line)

		lat, err := strconv.ParseFloat(tokens[0], 64)
		if err != nil {
			return nil, fmt.Errorf("lat: %w", err)
		}
		lon, err := strconv.ParseFloat(tokens[1], 64)
		if err != nil {
			return nil, fmt.Errorf("lon: %w", err)
		}
		globalPoints[i] = NewCoordinate(lat, lon)
	}

	// roundabout flag
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	numRoundaboutFlag := parseInt(tokens[0])

	roundaboutFlag := make([]Index, numRoundaboutFlag)
	for i := 0; i < numRoundaboutFlag; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		tokens = fields(line)
		roundaboutFlag[i] = Index(parseInt(tokens[0]))
	}

	// trafic light flag
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	numTrafficFlag := parseInt(tokens[0])
	trafficLight := make([]Index, numTrafficFlag)
	for i := 0; i < numTrafficFlag; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		tokens = fields(line)
		trafficLight[i] = Index(parseInt(tokens[0]))
	}

	// map edge info flag
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	numMapEdgeInfos := parseInt(tokens[0])
	mapEdgeInfos := make([]EdgeExtraInfo, numMapEdgeInfos)
	for i := 0; i < numMapEdgeInfos; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		tokens = fields(line)
		startsPointIndex := parseInt(tokens[0])
		endPointIndex := parseInt(tokens[1])
		streetName := parseInt(tokens[2])
		roadClass := parseInt(tokens[3])
		roadClassLink := parseInt(tokens[4])
		lanes := parseInt(tokens[5])
		osmWayId := parseInt(tokens[6])

		mapEdgeInfos[i] = NewEdgeExtraInfo(streetName, uint8(roadClass),
			uint8(lanes), uint8(roadClassLink), Index(startsPointIndex), Index(endPointIndex),
			int64(osmWayId))
	}

	// street direction flag
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	numStreetDirections := parseInt(tokens[0])
	streetDirections := make(map[int64][2]bool, numStreetDirections)
	for i := 0; i < numStreetDirections; i++ {
		line, err = util.ReadLine(br)
		if err != nil {
			return nil, err
		}
		tokens = fields(line)
		wayId := parseInt(tokens[0])
		forward, err := strconv.ParseBool(tokens[1])
		if err != nil {
			return nil, err
		}
		backward, err := strconv.ParseBool(tokens[2])
		if err != nil {
			return nil, err
		}

		streetDirections[int64(wayId)] = [2]bool{forward, backward}
	}

	// tagstring idmap flag
	line, err = util.ReadLine(br)
	if err != nil {
		return nil, err
	}

	tokens = fields(line)
	tagStringIdMap := util.NewIdMap()
	numIdMapItems := parseInt(tokens[0])
	for i := 0; i < numIdMapItems; i++ {

		line, err = util.ReadLine(br)

		if err != nil {
			return nil, err
		}
		tokens = fields(line)
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
			return nil, err
		}
		key := parseInt(tokens[0])
		tagStringIdMap.SetID(key, unquotedVal)
	}

	sccCondensationAdj := make([][]Index, 0)
	for {
		line, err = util.ReadLine(br)
		if err != nil {
			if errors.Is(err, io.EOF) {
				break
			}
			return nil, err
		}
		tokens = fields(line)
		if len(tokens) == 0 {
			continue
		}
		adj := make([]Index, 0)
		if tokens[0] != "empty" {
			for _, token := range tokens {
				scc, err := ParseIndex(token)
				if err != nil {
					return nil, err
				}
				adj = append(adj, scc)
			}
		}
		sccCondensationAdj = append(sccCondensationAdj, adj)
	}

	graphStorage := BuildGraphStorage(globalPoints,
		roundaboutFlag, trafficLight, mapEdgeInfos,
		tagStringIdMap, streetDirections)

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

func parseVertex(line string) (*Vertex, error) {
	tokens := fields(line)
	if len(tokens) != 8 {
		return nil, fmt.Errorf("expected 8 fields, got %d", len(tokens))
	}
	pvPtr, err := ParseIndex(tokens[0])
	if err != nil {
		return nil, err
	}
	ttPtr, err := ParseIndex(tokens[1])
	if err != nil {
		return nil, err
	}
	firstOut, err := ParseIndex(tokens[2])
	if err != nil {
		return nil, err
	}
	firstIn, err := ParseIndex(tokens[3])
	if err != nil {
		return nil, err
	}

	id, err := ParseIndex(tokens[4])
	if err != nil {
		return nil, err
	}

	lat, err := strconv.ParseFloat(tokens[5], 64)
	if err != nil {
		return nil, fmt.Errorf("lat: %w", err)
	}
	lon, err := strconv.ParseFloat(tokens[6], 64)
	if err != nil {
		return nil, fmt.Errorf("lon: %w", err)
	}

	osmId, err := strconv.ParseUint(tokens[7], 10, 64)
	if err != nil {
		return nil, fmt.Errorf("osmId: %w", err)
	}

	return &Vertex{
		pvPtr: pvPtr, turnTablePtr: ttPtr,
		firstOut: firstOut, firstIn: firstIn,
		lat: lat, lon: lon, id: id, osmId: osmId,
	}, nil
}

func parseOutEdge(line string) (*OutEdge, error) {
	tokens := fields(line)
	if len(tokens) != 7 {
		return nil, fmt.Errorf("expected 7 fields, got %d", len(tokens))
	}
	edgeId, err := ParseIndex(tokens[0])
	if err != nil {
		return nil, err
	}
	head, err := ParseIndex(tokens[1])
	if err != nil {
		return nil, err
	}
	weight, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return nil, err
	}
	dist, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return nil, err
	}

	entryPoint, err := strconv.ParseUint(tokens[4], 10, 8)
	if err != nil {
		return nil, err
	}

	oriEdgeId, err := ParseIndex(tokens[5])
	if err != nil {
		return nil, err
	}

	hwType, err := strconv.ParseUint(tokens[6], 10, 8)
	if err != nil {
		return nil, err
	}

	e := NewOutEdge(edgeId, head, weight, dist, int(entryPoint), pkg.OsmHighwayType(hwType))
	e.SetOriginalEdgeId(oriEdgeId)
	return e, nil
}

func parseInEdge(line string) (*InEdge, error) {
	tokens := fields(line)
	if len(tokens) != 7 {
		return nil, fmt.Errorf("expected 7 fields, got %d", len(tokens))
	}
	edgeId, err := ParseIndex(tokens[0])
	if err != nil {
		return nil, err
	}
	tail, err := ParseIndex(tokens[1])
	if err != nil {
		return nil, err
	}
	weight, err := strconv.ParseFloat(tokens[2], 64)
	if err != nil {
		return nil, err
	}
	dist, err := strconv.ParseFloat(tokens[3], 64)
	if err != nil {
		return nil, err
	}

	exitPoint, err := strconv.ParseUint(tokens[4], 10, 8)
	if err != nil {
		return nil, err
	}
	oriEdgeId, err := ParseIndex(tokens[5])
	if err != nil {
		return nil, err
	}

	hwType, err := strconv.ParseUint(tokens[6], 10, 8)
	if err != nil {
		return nil, err
	}

	e := NewInEdge(edgeId, tail, weight, dist, int(exitPoint), pkg.OsmHighwayType(hwType))
	e.SetOriginalEdgeId(oriEdgeId)
	return e, nil
}

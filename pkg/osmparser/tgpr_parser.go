package osmparser

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strconv"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ut "github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type TGPRParser struct {
	nodes []NodeCoord
}

func NewTPGRParser() *TGPRParser {
	return &TGPRParser{
		nodes: make([]NodeCoord, 0),
	}
}

func (p *TGPRParser) ParseTGPRFile(coordFile, tgprFile string, logger *zap.Logger) (*datastructure.Graph, map[int64]*da.PWL, error) {
	f, err := os.Open(coordFile)

	if err != nil {
		return nil, nil, fmt.Errorf("cannot read file: %s, err: %v", coordFile, err)
	}
	defer f.Close()

	br := bufio.NewReader(f)
	line, err := ut.ReadLine(br)
	if err != nil {
		return nil, nil, err
	}

	ff := ut.Fields(line)
	n, err := strconv.Atoi(ff[0])
	if err != nil {
		return nil, nil, err
	}

	p.nodes = make([]NodeCoord, n)

	for i := 0; i < n; i++ {
		var (
			lat, lon float64
		)
		line, err = ut.ReadLine(br)
		if err != nil {
			return nil, nil, err
		}

		ff = ut.Fields(line)
		lat, err = strconv.ParseFloat(ff[1], 64)
		if err != nil {
			return nil, nil, err
		}
		lon, err = strconv.ParseFloat(ff[2], 64)
		if err != nil {
			return nil, nil, err
		}

		p.nodes[i] = NewNodeCoord(lat, lon)
	}

	// parse tpgr file

	f, err = os.Open(tgprFile)

	if err != nil {
		return nil, nil, fmt.Errorf("cannot read file: %s, err: %v", tgprFile, err)
	}
	defer f.Close()

	br = bufio.NewReader(f)
	line, err = ut.ReadLine(br)
	if err != nil {
		return nil, nil, err
	}

	ff = ut.Fields(line)
	n, err = strconv.Atoi(ff[0])
	if err != nil {
		return nil, nil, err
	}

	m, err := strconv.Atoi(ff[1])
	if err != nil {
		return nil, nil, err
	}

	ttProfile := make(map[int64]*da.PWL, m)

	edges := make([]Edge, m)

	graphStorage := datastructure.NewGraphStorage()

	for i := 0; i < m; i++ {
		var (
			u, v, q int
			t, y    float64
		)
		line, err = ut.ReadLine(br)
		if err != nil {
			return nil, nil, err
		}

		ff = ut.Fields(line)

		u, err = strconv.Atoi(ff[0])
		if err != nil {
			return nil, nil, err
		}
		v, err = strconv.Atoi(ff[1])
		if err != nil {
			return nil, nil, err
		}
		q, err = strconv.Atoi(ff[2])
		if err != nil {
			return nil, nil, err
		}

		points := []*da.Point{}

		mintt := pkg.INF_WEIGHT

		for j := 0; j < q*2; j += 2 {
			t, err = strconv.ParseFloat(ff[3+j], 64)
			if err != nil {
				return nil, nil, err
			}
			t /= 10 //  dari dataset nya:the unit of time is 0.1 seconds
			y, err = strconv.ParseFloat(ff[3+(j+1)], 64)
			if err != nil {
				return nil, nil, err
			}
			y /= 10

			points = append(points, da.NewPoint(t, y))
			mintt = math.Min(mintt, y)
		}

		ttProfile[int64(i)] = da.NewPWL(points)

		e := NewEdge(
			uint32(u),
			uint32(v),
			mintt,
			0,
			uint32(i),
		)
		edges[i] = e

		edgePoints := make([]datastructure.Coordinate, 0)
		edgePoints = append(edgePoints, da.NewCoordinate(p.nodes[u].lat, p.nodes[u].lon))
		edgePoints = append(edgePoints, da.NewCoordinate(p.nodes[v].lat, p.nodes[v].lon))

		startPointsIndex := graphStorage.GetGlobalPointsCount()

		graphStorage.AppendGlobalPoints(edgePoints)
		endPointsIndex := graphStorage.GetGlobalPointsCount()

		graphStorage.AppendMapEdgeInfo(datastructure.NewEdgeExtraInfo(
			0,
			0,
			0,
			0,
			datastructure.Index(startPointsIndex), datastructure.Index(endPointsIndex),
			int64(i),
		))
	}

	graph := p.BuildGraph(edges, graphStorage, uint32(len(p.nodes)), false)
	graph.SetGraphStorage(graphStorage)

	return graph, ttProfile, err
}

func (p *TGPRParser) BuildGraph(scannedEdges []Edge, graphStorage *datastructure.GraphStorage, numV uint32, skipUTurn bool) *datastructure.Graph {
	var (
		outEdges  [][]*datastructure.OutEdge = make([][]*datastructure.OutEdge, numV)
		inEdges   [][]*datastructure.InEdge  = make([][]*datastructure.InEdge, numV)
		inDegree  []int                      = make([]int, numV)
		outDegree []int                      = make([]int, numV)
		vertices  []*datastructure.Vertex    = make([]*datastructure.Vertex, numV+1)
	)

	for v := 0; v < int(numV)+1; v++ {
		vertices[v] = datastructure.NewVertex(0, 0, datastructure.Index(v))
	}

	lastEdgeId := uint32(0)
	edgeId := datastructure.Index(0)
	for _, e := range scannedEdges {
		u := datastructure.Index(e.from)
		v := datastructure.Index(e.to)

		outEdges[u] = append(outEdges[u], datastructure.NewOutEdge(edgeId,
			v, e.weight, e.distance, len(inEdges[v])))
		outDegree[u]++

		inEdges[v] = append(inEdges[v], datastructure.NewInEdge(edgeId,
			u, e.weight, e.distance, len(outEdges[u])-1))
		inDegree[v]++

		uData := p.nodes[u]
		vertices[u] = datastructure.NewVertex(uData.lat, uData.lon, u)

		vData := p.nodes[v]
		vertices[v] = datastructure.NewVertex(vData.lat, vData.lon, v)
		edgeId++
		if e.edgeID >= lastEdgeId {
			lastEdgeId = e.edgeID + 1
		}
	}

	for v := 0; v < len(vertices)-1; v++ {
		// we need to do this because crp query assume all vertex have at least one outEdge (at for target as source)

		dummyID := datastructure.Index(lastEdgeId)
		dummyOut := datastructure.NewOutEdge(dummyID, datastructure.Index(v),
			0, 0, len(inEdges[v]))
		outEdges[v] = append(outEdges[v], dummyOut)
		outDegree[v]++

		dummyIn := datastructure.NewInEdge(dummyID, datastructure.Index(v),
			0, 0, len(outEdges[v])-1)
		inEdges[v] = append(inEdges[v], dummyIn)
		inDegree[v]++
		lastEdgeId++
		graphStorage.AppendMapEdgeInfo(
			datastructure.NewEdgeExtraInfo(
				0,
				uint8(0),
				uint8(0),
				uint8(0),
				datastructure.Index(uint32(math.Pow(1, 30))), datastructure.Index(uint32(math.Pow(1, 30))),
				-1,
			),
		)
	}

	turnMatrices := make([][]pkg.TurnType, len(vertices)-1)
	// T_u[i*outDegree[u]+j] = turn type from inEdge i to outEdge j at vertex u

	// init turn matrices
	for i := 0; i < len(turnMatrices); i++ {
		turnMatrices[i] = make([]pkg.TurnType, outDegree[i]*inDegree[i])

		for j := 0; j < len(turnMatrices[i]); j++ {
			turnMatrices[i][j] = pkg.NONE
		}
	}

	matrices := make([]pkg.TurnType, 0)
	matrixOffset := 0

	for v := 0; v < len(vertices)-1; v++ {

		// set the turnTablePtr of vertex v to the current matrixOffset
		// matrix offset is index of the first element of turnMatrices[v] in the flattened matrices array
		vertices[v].SetTurnTablePtr(datastructure.Index(matrixOffset))
		// flatten the turnMatrices
		for i := 0; i < len(turnMatrices[v]); i++ {
			matrices = append(matrices, turnMatrices[v][i])
		}

		matrixOffset += len(turnMatrices[v])
	}

	outEdgeOffset := datastructure.Index(0)
	inEdgeOffset := datastructure.Index(0)

	for i := 0; i < len(vertices)-1; i++ {
		vertices[i].SetTurnTablePtr(vertices[i].GetTurnTablePtr())
		vertices[i].SetFirstOut(outEdgeOffset) // index of the first outEdge of vertex i in the flattened outEdges array
		vertices[i].SetFirstIn(inEdgeOffset)
		outEdgeOffset += datastructure.Index(len(outEdges[i]))
		inEdgeOffset += datastructure.Index(len(inEdges[i]))
	}

	vertices[len(vertices)-1] = datastructure.NewVertex(0, 0, datastructure.Index(len(vertices)-1))
	vertices[len(vertices)-1].SetFirstOut(outEdgeOffset)
	vertices[len(vertices)-1].SetFirstIn(inEdgeOffset)

	flattenOutEdges := flatten(outEdges)
	flattenInEdges := flatten(inEdges)
	graph := datastructure.NewGraph(vertices, flattenOutEdges, flattenInEdges, matrices)

	return graph
}

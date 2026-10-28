package extractor

import (
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func buildGraphHelper(t *testing.T, n int, adjList [][]da.PairEdge) (*da.Graph, *met.TimeFunction[float64], *da.RoadNetworkDataContainer) {
	t.Helper()

	nodeCoords := make([]da.Coordinate, n)
	for i := 0; i < n; i++ {
		nodeCoords[i] = da.NewCoordinate(float64(i), float64(i))
	}

	es := da.FlattenEdges(adjList)
	logger, err := logger.New()
	if err != nil {
		t.Fatal(err)
	}
	op := NewExtractor[float64](logger)
	acceptedNodeMap := make(map[int64]da.Coordinate, n)
	nodeToOsmID := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = nodeCoords[i]
		nodeToOsmID[da.Index(i)] = int64(i)
	}

	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmID(nodeToOsmID)

	rn := da.NewRoadNetworkDataContainerWithSize(len(es), n)
	for i, e := range es {
		startPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendOsmNodePoints([]da.Coordinate{
			nodeCoords[e.GetFrom()],
			nodeCoords[e.GetTo()],
		}, make([]uint64, 2))
		endPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendSegmentData(int64(i), da.Index(startPointsIndex), da.Index(endPointsIndex), 0, 0, 0, 1, da.NewEmptyTurnLanesData())
		rn.SetSegmentFlag(da.Index(i), 0)
	}

	g, wf, _, _ := op.BuildGraph(es, rn, uint32(n), false)
	rn.BuildNameTable(map[uint32]string{0: ""})

	return g, wf, rn
}

func createTurnTables(t *testing.T, g *da.Graph) ([]da.Index, []pkg.TurnType) {
	t.Helper()

	numV := g.NumberOfVertices()
	vTurnTableIds := make([]da.Index, numV)
	totalMatrixSize := 0
	for u := 0; u < numV; u++ {
		totalMatrixSize += int(g.GetInDegree(da.Index(u))) * int(g.GetOutDegree(da.Index(u)))
	}
	turnMatrix := make([]pkg.TurnType, totalMatrixSize)

	matrixOffset := 0
	for u := 0; u < numV; u++ {
		inDeg := int(g.GetInDegree(da.Index(u)))
		outDeg := int(g.GetOutDegree(da.Index(u)))
		vTurnTableIds[u] = da.Index(matrixOffset)
		matrixOffset += inDeg * outDeg
	}

	return vTurnTableIds, turnMatrix
}

func addTurnRestriction(t *testing.T, g *da.Graph, vTurnTableIds []da.Index, turnMatrix []pkg.TurnType, from, via, to da.Index, turnType pkg.TurnType) {
	t.Helper()

	var entryPoint uint8
	var foundIn bool
	g.ForInEdgesOf(via, func(eID, tail da.Index, _ uint8) {
		if tail == from {
			entryPoint = g.GetEntryOrder(via, eID)
			foundIn = true
		}
	})
	if !foundIn {
		t.Fatalf("incoming edge from %d to %d not found", from, via)
	}

	var exitPoint uint8
	var foundOut bool
	g.ForOutEdgesOf(via, func(eID, head da.Index, _ uint8) {
		if head == to {
			exitPoint = g.GetExitOrder(via, eID)
			foundOut = true
		}
	})
	if !foundOut {
		t.Fatalf("outgoing edge from %d to %d not found", via, to)
	}

	turnTableID := vTurnTableIds[via] + da.Index(entryPoint)*g.GetOutDegree(via) + da.Index(exitPoint)
	turnMatrix[turnTableID] = turnType
}

type coord struct {
	lat float64
	lon float64
}

type turnRestriction struct {
	from     da.Index
	via      da.Index
	to       da.Index
	turnType pkg.TurnType
}

// TestBuildEdgeBasedGraph tests BuildEdgeBasedGraph using table-driven tests
//
// read figure 3.1:	https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf#page=15.55
// there are 5 edge-based graph vertices (see figure 3.1 right)
// in figure 3.1 right, it should have 4 edges (excluding removed edges (2,1)->(1,5))
// the missing one is edge (1,2)->(2,1)
// in the test below, following the edge-based graph building algorithm
// edge-based graph (ebg) vertices are: (1,2), (1,3), (1,4), (1,5), (2,1).
// the ebg edges are: (1,2)->(2,1), (2,1)->(1,2), (2,1)->(1,3), (2,1)->(1,4).
// ebg edge (2,1)->(1,5) removed because of turn restrictions (2,1,5)
// another reference that explain edge-based graph are:
// 1. osrm edge-based graph: https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation
// 2. Geisberger, R. & Vetter, C. Routing With Turn Costs (2011): https://ae.iti.kit.edu/download/turn_ch.pdf
func TestBuildEdgeBasedGraph(t *testing.T) {
	const (
		v1 = 1
		v2 = 2
		v3 = 3
		v4 = 4
		v5 = 5
	)

	fig31AdjList := [][]da.PairEdge{
		v1: {
			da.NewPairEdge(v2, 1.0),
			da.NewPairEdge(v3, 1.0),
			da.NewPairEdge(v4, 1.0),
			da.NewPairEdge(v5, 1.0),
		},
		v2: {
			da.NewPairEdge(v1, 1.0),
		},
	}

	tests := []struct {
		name              string
		numNodes          int
		adjList           [][]da.PairEdge
		restrictions      []turnRestriction
		forbidAllTurns    bool
		wantNumVertices   int        // edge-based graph vertices
		wantNumEdges      int        // edge-based graph edges
		wantInDegrees     []da.Index // in-degrees of edge-based graph vertices
		wantOutDegrees    []da.Index // out-degrees of edge-based graph vertices
		wantMapping       []uint64
		wantCoords        []coord
		wantHeads         []da.Index              // heads vertex v of each edge-based graph edges (u,v).
		wantTails         []da.Index              // tail vertex u of each edge-based graph  reversed edges (v,u)
		wantWeights       []float64               // weights of each edge-based graph edges.
		wantOutEdges      map[da.Index][]da.Index // edge-based graph edges
		forbiddenOutEdges map[da.Index][]da.Index // turn restrictions or removed edge-based graph edges.
		wantOutTails      []da.Index              // tails vertex u of each edge-based graph edges (u,v).
		wantInHeads       []da.Index              // head vertex v of each edge-based graph reversed edges (v,u).
		wantFirstOut      []da.Index              // map from ebg vertex id=u to  its first edge (u,v) id
		wantFirstIn       []da.Index              // map from ebg vertex id=v to its first reversed edge (v,u) id
	}{
		{
			name:     "Figure3_1_Restricted", // figure 3.1 https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf#page=15.55
			numNodes: 6,
			adjList:  fig31AdjList,
			restrictions: []turnRestriction{
				{from: v2, via: v1, to: v5, turnType: pkg.NoEntry},
			},
			forbidAllTurns:  false,
			wantNumVertices: 5,                         // edge-based graph vertices: (1,2), (1,3), (1,4), (1,5), (2,1).
			wantNumEdges:    4,                         // edge-based graph edges. the ebg edges are: (1,2)->(2,1), (2,1)->(1,2), (2,1)->(1,3), (2,1)->(1,4).
			wantInDegrees:   []da.Index{1, 1, 1, 0, 1}, // in-degrees of edge-based graph vertices: (1,2), (1,3), (1,4), (1,5), (2,1).
			wantOutDegrees:  []da.Index{1, 0, 0, 0, 3}, // out-degrees of edge-based graph vertices: (1,2), (1,3), (1,4), (1,5), (2,1).
			wantMapping: []uint64{
				uint64(v1) | (uint64(v2) << 32),
				uint64(v1) | (uint64(v3) << 32),
				uint64(v1) | (uint64(v4) << 32),
				uint64(v1) | (uint64(v5) << 32),
				uint64(v2) | (uint64(v1) << 32),
			},
			wantCoords: []coord{
				{lat: float64(v1+v2) / 2.0, lon: float64(v1+v2) / 2.0},
				{lat: float64(v1+v3) / 2.0, lon: float64(v1+v3) / 2.0},
				{lat: float64(v1+v4) / 2.0, lon: float64(v1+v4) / 2.0},
				{lat: float64(v1+v5) / 2.0, lon: float64(v1+v5) / 2.0},
				{lat: float64(v2+v1) / 2.0, lon: float64(v2+v1) / 2.0},
			},
			wantHeads:   []da.Index{4, 0, 1, 2},        // heads vertex v of each edge-based graph edges (u,v). the ebg edges are: (1,2)->(2,1), (2,1)->(1,2), (2,1)->(1,3), (2,1)->(1,4).
			wantTails:   []da.Index{4, 4, 4, 0},        // tail vertex u of each edge-based graph  reversed edges (v,u). the ebg reversed edges are: (1,2)->(2,1), (1,3)->(2,1), (1,4)->(2,1), (2,1)->(1,2).
			wantWeights: []float64{1.0, 1.0, 1.0, 1.0}, // weights of each edge-based graph edges.
			wantOutEdges: map[da.Index][]da.Index{
				4: {0, 1, 2}, // ebg edges (2,1)->(1,2), (2,1)->(1,3), (2,1)->(1,4)
				0: {4},       // ebg edge  (1,2)->(2,1)
			},
			forbiddenOutEdges: map[da.Index][]da.Index{
				4: {3}, //  turn restriction (2,1)->(1,5)
			},
			wantOutTails: []da.Index{0, 4, 4, 4},       // tails vertex u of each edge-based graph edges (u,v). the ebg edges are: (1,2)->(2,1), (2,1)->(1,2), (2,1)->(1,3), (2,1)->(1,4).
			wantInHeads:  []da.Index{0, 1, 2, 4},       // head vertex v of each edge-based graph reversed edges (v,u). the ebg reversed edges are: (1,2)->(2,1), (1,3)->(2,1), (1,4)->(2,1), (2,1)->(1,2).
			wantFirstOut: []da.Index{0, 1, 1, 1, 1, 4}, // map from ebg vertex id=u to its first edge (u,v) id
			wantFirstIn:  []da.Index{0, 1, 2, 3, 3, 4}, // map from ebg vertex id=v to its first reversed edge (v,u) id

		},
		{
			name:            "Figure3_1_NoRestrictions", // remove turn restrictions (2,1,5) in figure 3.1 https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf#page=15.55
			numNodes:        6,
			adjList:         fig31AdjList,
			restrictions:    nil,
			forbidAllTurns:  false,
			wantNumVertices: 5,
			wantNumEdges:    5,
			wantInDegrees:   []da.Index{1, 1, 1, 1, 1},
			wantOutDegrees:  []da.Index{1, 0, 0, 0, 4},
			wantMapping: []uint64{
				uint64(v1) | (uint64(v2) << 32),
				uint64(v1) | (uint64(v3) << 32),
				uint64(v1) | (uint64(v4) << 32),
				uint64(v1) | (uint64(v5) << 32),
				uint64(v2) | (uint64(v1) << 32),
			},
			wantCoords: []coord{
				{lat: float64(v1+v2) / 2.0, lon: float64(v1+v2) / 2.0},
				{lat: float64(v1+v3) / 2.0, lon: float64(v1+v3) / 2.0},
				{lat: float64(v1+v4) / 2.0, lon: float64(v1+v4) / 2.0},
				{lat: float64(v1+v5) / 2.0, lon: float64(v1+v5) / 2.0},
				{lat: float64(v2+v1) / 2.0, lon: float64(v2+v1) / 2.0},
			},
			wantHeads:   []da.Index{4, 0, 1, 2, 3},
			wantTails:   []da.Index{4, 4, 4, 4, 0},
			wantWeights: []float64{1.0, 1.0, 1.0, 1.0, 1.0},
			wantOutEdges: map[da.Index][]da.Index{
				0: {4},
				4: {0, 1, 2, 3},
			},
			wantOutTails: []da.Index{0, 4, 4, 4, 4},
			wantInHeads:  []da.Index{0, 1, 2, 3, 4},
			wantFirstOut: []da.Index{0, 1, 1, 1, 1, 5},
			wantFirstIn:  []da.Index{0, 1, 2, 3, 4, 5},
		},
		{
			name:            "Figure3_1_AllTurnsForbidden", // forbid all possible turns or remove all edge-based graph edges in figure 3.1 https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf#page=15.55.
			numNodes:        6,
			adjList:         fig31AdjList,
			restrictions:    nil,
			forbidAllTurns:  true,
			wantNumVertices: 5,
			wantNumEdges:    0,
			wantInDegrees:   []da.Index{0, 0, 0, 0, 0},
			wantOutDegrees:  []da.Index{0, 0, 0, 0, 0},
			wantMapping: []uint64{
				uint64(v1) | (uint64(v2) << 32),
				uint64(v1) | (uint64(v3) << 32),
				uint64(v1) | (uint64(v4) << 32),
				uint64(v1) | (uint64(v5) << 32),
				uint64(v2) | (uint64(v1) << 32),
			},
			wantCoords: []coord{
				{lat: float64(v1+v2) / 2.0, lon: float64(v1+v2) / 2.0},
				{lat: float64(v1+v3) / 2.0, lon: float64(v1+v3) / 2.0},
				{lat: float64(v1+v4) / 2.0, lon: float64(v1+v4) / 2.0},
				{lat: float64(v1+v5) / 2.0, lon: float64(v1+v5) / 2.0},
				{lat: float64(v2+v1) / 2.0, lon: float64(v2+v1) / 2.0},
			},
			wantWeights:  nil,
			wantFirstOut: []da.Index{0, 0, 0, 0, 0, 0},
			wantFirstIn:  []da.Index{0, 0, 0, 0, 0, 0},
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			g, wf, rn := buildGraphHelper(t, tc.numNodes, tc.adjList)
			vTurnTableIds, turnMatrix := createTurnTables(t, g)

			if tc.forbidAllTurns {
				for i := range turnMatrix {
					turnMatrix[i] = pkg.NoEntry
				}
			} else {
				for _, r := range tc.restrictions {
					addTurnRestriction(t, g, vTurnTableIds, turnMatrix, r.from, r.via, r.to, r.turnType)
				}
			}

			ebg, ebgWf, ebgMapping, err := BuildEdgeBasedGraph(g, wf, vTurnTableIds, turnMatrix, rn)
			if err != nil {
				t.Fatal(err)
			}
			require.NotNil(t, ebg)
			require.NotNil(t, ebgWf)
			require.NotNil(t, ebgMapping)

			require.Equal(t, tc.wantNumVertices, ebg.NumberOfVertices())
			require.Equal(t, tc.wantNumEdges, ebg.NumberOfEdges())

			assert.True(t, ebg.IsRoadNetworkGraph())

			for i := 0; i < tc.wantNumVertices; i++ {
				v := da.Index(i)
				assert.Equal(t, tc.wantInDegrees[i], ebg.GetInDegree(v), "vertex %d in-degree", v)
				assert.Equal(t, tc.wantOutDegrees[i], ebg.GetOutDegree(v), "vertex %d out-degree", v)
			}

			assert.Equal(t, tc.wantMapping, ebgMapping)
			for i, mVal := range ebgMapping {
				tail := da.Index(mVal & 0xFFFFFFFF)
				head := da.Index(mVal >> 32)
				expectedTail := da.Index(tc.wantMapping[i] & 0xFFFFFFFF)
				expectedHead := da.Index(tc.wantMapping[i] >> 32)
				assert.Equal(t, expectedTail, tail, "ebgMapping[%d] tail", i)
				assert.Equal(t, expectedHead, head, "ebgMapping[%d] head", i)
			}

			for i := 0; i < tc.wantNumVertices; i++ {
				vertex := ebg.GetVertex(da.Index(i))
				c := vertex.GetCoordinate()
				assert.InDelta(t, tc.wantCoords[i].lat, c.GetLat(), util.EPS, "vertex %d lat", i)
				assert.InDelta(t, tc.wantCoords[i].lon, c.GetLon(), util.EPS, "vertex %d lon", i)
			}

			var gotHeads []da.Index
			var gotTails []da.Index
			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				gotHeads = append(gotHeads, ebg.GetHead(da.Index(eID)))
				gotTails = append(gotTails, ebg.GetTail(da.Index(eID)))
			}
			if len(tc.wantHeads) == 0 {
				assert.Empty(t, gotHeads)
			} else {
				assert.Equal(t, tc.wantHeads, gotHeads)
			}
			if len(tc.wantTails) == 0 {
				assert.Empty(t, gotTails)
			} else {
				assert.Equal(t, tc.wantTails, gotTails)
			}

			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				assert.Equal(t, tc.wantOutTails[eID], ebg.GetTailOfOutedge(da.Index(eID)), "edge %d outTail", eID)
			}

			for reID := 0; reID < ebg.NumberOfEdges(); reID++ {
				assert.Equal(t, tc.wantInHeads[reID], ebg.GetHeadOfInedge(da.Index(reID)), "inedge %d inHead", reID)
			}

			for u, wantOut := range tc.wantOutEdges {
				var gotOut []da.Index
				ebg.ForOutEdgesOf(u, func(_, head da.Index, _ uint8) {
					gotOut = append(gotOut, head)
				})
				if len(wantOut) == 0 {
					assert.Empty(t, gotOut)
				} else {
					assert.Equal(t, wantOut, gotOut, "ebg.ForOutEdgesOf(%d)", u)
				}
			}

			for u, forbidden := range tc.forbiddenOutEdges {
				var gotOut []da.Index
				ebg.ForOutEdgesOf(u, func(_, head da.Index, _ uint8) {
					gotOut = append(gotOut, head)
				})
				for _, fHead := range forbidden {
					assert.NotContains(t, gotOut, fHead, "ebg.ForOutEdgesOf(%d) forbidden edge to %d", u, fHead)
				}
			}

			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				revID := ebg.GetRevId(da.Index(eID))
				assert.Equal(t, da.Index(eID), ebg.GetOutId(revID), "edge %d rev outId", eID)
				assert.Equal(t, ebg.GetHead(da.Index(eID)), ebg.GetHeadOfInedge(revID), "edge %d head matching rev inedge head", eID)
				assert.Equal(t, ebg.GetTailOfOutedge(da.Index(eID)), ebg.GetTail(revID), "edge %d outTail matching rev tail", eID)
			}

			for i := 0; i <= tc.wantNumVertices; i++ {
				idx := da.Index(i)
				assert.Equal(t, tc.wantFirstOut[i], ebg.GetVertexFirstOut(idx), "vertex %d firstOut", i)
				assert.Equal(t, tc.wantFirstIn[i], ebg.GetVertexFirstIn(idx), "vertex %d firstIn", i)
			}

			dummyVertex := ebg.GetVertex(da.Index(tc.wantNumVertices))
			dummyCoord := dummyVertex.GetCoordinate()
			assert.InDelta(t, 0.0, dummyCoord.GetLat(), util.EPS)
			assert.InDelta(t, 0.0, dummyCoord.GetLon(), util.EPS)

			if len(tc.wantWeights) == 0 {
				assert.Empty(t, ebgWf.Getweights())
			} else {
				assert.InDeltaSlice(t, tc.wantWeights, ebgWf.Getweights(), util.EPS)
			}
			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				assert.InDelta(t, tc.wantWeights[eID], ebgWf.GetWeight(da.Index(eID)), util.EPS, "edge %d weight", eID)
			}

		})
	}
}

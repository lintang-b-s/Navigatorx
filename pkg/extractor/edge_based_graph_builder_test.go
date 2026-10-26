package extractor

import (
	"math"
	"testing"

	"github.com/google/go-cmp/cmp"
	"github.com/google/go-cmp/cmp/cmpopts"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type pairEdge struct {
	to     int
	weight float64
}

func newPairEdge(to int, weight float64) pairEdge {
	return pairEdge{to: to, weight: weight}
}

func flattenEdges(es [][]pairEdge) []Edge[float64] {
	totalEdges := 0
	for _, edges := range es {
		totalEdges += len(edges)
	}
	flatten := make([]Edge[float64], 0, totalEdges)
	for from, edges := range es {
		for _, e := range edges {
			length := uint32(math.Round(e.weight * 100))
			flatten = append(flatten, NewEdge(uint32(from), uint32(e.to), e.weight, length))
		}
	}
	return flatten
}

func buildGraphHelper(t *testing.T, n int, adjList [][]pairEdge) (*da.Graph, *met.TimeFunction[float64], *da.RoadNetworkDataContainer) {
	t.Helper()

	nodeCoords := make([]da.Coordinate, n)
	for i := 0; i < n; i++ {
		nodeCoords[i] = da.NewCoordinate(float64(i), float64(i))
	}

	es := flattenEdges(adjList)

	op := NewExtractor[float64]()
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
			nodeCoords[e.from],
			nodeCoords[e.to],
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
	vTurnTableIDs := make([]da.Index, numV)
	totalMatrixSize := 0
	for u := 0; u < numV; u++ {
		totalMatrixSize += int(g.GetInDegree(da.Index(u))) * int(g.GetOutDegree(da.Index(u)))
	}
	turnMatrix := make([]pkg.TurnType, totalMatrixSize)

	matrixOffset := 0
	for u := 0; u < numV; u++ {
		inDeg := int(g.GetInDegree(da.Index(u)))
		outDeg := int(g.GetOutDegree(da.Index(u)))
		vTurnTableIDs[u] = da.Index(matrixOffset)
		matrixOffset += inDeg * outDeg
	}

	return vTurnTableIDs, turnMatrix
}

func addTurnRestriction(t *testing.T, g *da.Graph, vTurnTableIDs []da.Index, turnMatrix []pkg.TurnType, from, via, to da.Index, turnType pkg.TurnType) {
	t.Helper()

	var entryPoint uint8
	var foundIn bool
	g.ForInEdgesOf(via, func(eID, tail da.Index, exitPoint uint8) {
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

	turnTableID := vTurnTableIDs[via] + da.Index(entryPoint)*g.GetOutDegree(via) + da.Index(exitPoint)
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

	fig31AdjList := [][]pairEdge{
		v1: {
			newPairEdge(v2, 1.0),
			newPairEdge(v3, 1.0),
			newPairEdge(v4, 1.0),
			newPairEdge(v5, 1.0),
		},
		v2: {
			newPairEdge(v1, 1.0),
		},
	}

	tests := []struct {
		name              string
		numNodes          int
		adjList           [][]pairEdge
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
				{from: v2, via: v1, to: v5, turnType: pkg.NO_ENTRY},
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
			vTurnTableIDs, turnMatrix := createTurnTables(t, g)

			if tc.forbidAllTurns {
				for i := range turnMatrix {
					turnMatrix[i] = pkg.NO_ENTRY
				}
			} else {
				for _, r := range tc.restrictions {
					addTurnRestriction(t, g, vTurnTableIDs, turnMatrix, r.from, r.via, r.to, r.turnType)
				}
			}

			ebg, ebgWf, ebgMapping := BuildEdgeBasedGraph(g, wf, vTurnTableIDs, turnMatrix, rn)
			if ebg == nil {
				t.Fatal("BuildEdgeBasedGraph() returned nil ebg")
			}
			if ebgWf == nil {
				t.Fatal("BuildEdgeBasedGraph() returned nil ebgWf")
			}
			if ebgMapping == nil {
				t.Fatal("BuildEdgeBasedGraph() returned nil ebgMapping")
			}

			if got, want := ebg.NumberOfVertices(), tc.wantNumVertices; got != want {
				t.Fatalf("ebg.NumberOfVertices() = %d, want %d", got, want)
			}
			if got, want := ebg.NumberOfEdges(), tc.wantNumEdges; got != want {
				t.Fatalf("ebg.NumberOfEdges() = %d, want %d", got, want)
			}

			if got, want := ebg.IsRoadNetworkGraph(), true; got != want {
				t.Errorf("ebg.IsRoadNetworkGraph() = %v, want %v", got, want)
			}

			for i := 0; i < tc.wantNumVertices; i++ {
				v := da.Index(i)
				if got, want := ebg.GetInDegree(v), tc.wantInDegrees[i]; got != want {
					t.Errorf("ebg.GetInDegree(%d) = %d, want %d", i, got, want)
				}

				if got, want := ebg.GetOutDegree(v), tc.wantOutDegrees[i]; got != want {
					t.Errorf("ebg.GetOutDegree(%d) = %d, want %d", i, got, want)
				}

			}

			if diff := cmp.Diff(tc.wantMapping, ebgMapping); diff != "" {
				t.Errorf("BuildEdgeBasedGraph() mapping mismatch (-want +got):\n%s", diff)
			}
			for i, mVal := range ebgMapping {
				tail := da.Index(mVal & 0xFFFFFFFF)
				head := da.Index(mVal >> 32)
				expectedTail := da.Index(tc.wantMapping[i] & 0xFFFFFFFF)
				expectedHead := da.Index(tc.wantMapping[i] >> 32)
				if tail != expectedTail || head != expectedHead {
					t.Errorf("ebgMapping[%d] decoded = (%d, %d), want (%d, %d)",
						i, tail, head, expectedTail, expectedHead)
				}
			}

			for i := 0; i < tc.wantNumVertices; i++ {
				vertex := ebg.GetVertex(da.Index(i))
				c := vertex.GetCoordinate()
				if !util.Eq(c.GetLat(), tc.wantCoords[i].lat) || !util.Eq(c.GetLon(), tc.wantCoords[i].lon) {
					t.Errorf("ebg.GetVertex(%d).GetCoordinate() = (%v, %v), want (%v, %v)",
						i, c.GetLat(), c.GetLon(), tc.wantCoords[i].lat, tc.wantCoords[i].lon)
				}
			}

			gotHeads := make([]da.Index, 0, ebg.NumberOfEdges())
			gotTails := make([]da.Index, 0, ebg.NumberOfEdges())
			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				gotHeads = append(gotHeads, ebg.GetHead(da.Index(eID)))
				gotTails = append(gotTails, ebg.GetTail(da.Index(eID)))
			}
			if diff := cmp.Diff(tc.wantHeads, gotHeads, cmpopts.EquateEmpty()); diff != "" {
				t.Errorf("ebg heads mismatch (-want +got):\n%s", diff)
			}
			if diff := cmp.Diff(tc.wantTails, gotTails, cmpopts.EquateEmpty()); diff != "" {
				t.Errorf("ebg tails mismatch (-want +got):\n%s", diff)
			}

			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				if got, want := ebg.GetTailOfOutedge(da.Index(eID)), tc.wantOutTails[eID]; got != want {
					t.Errorf("ebg.GetTailOfOutedge(%d) = %d, want %d", eID, got, want)
				}
			}

			for reID := 0; reID < ebg.NumberOfEdges(); reID++ {
				if got, want := ebg.GetHeadOfInedge(da.Index(reID)), tc.wantInHeads[reID]; got != want {
					t.Errorf("ebg.GetHeadOfInedge(%d) = %d, want %d", reID, got, want)
				}
			}

			for u, wantOut := range tc.wantOutEdges {
				var gotOut []da.Index
				ebg.ForOutEdgesOf(u, func(eID, head da.Index, entryPoint uint8) {
					gotOut = append(gotOut, head)
				})
				if diff := cmp.Diff(wantOut, gotOut, cmpopts.EquateEmpty()); diff != "" {
					t.Errorf("ebg.ForOutEdgesOf(%d) heads mismatch (-want +got):\n%s", u, diff)
				}
			}

			for u, forbidden := range tc.forbiddenOutEdges {
				var gotOut []da.Index
				ebg.ForOutEdgesOf(u, func(eID, head da.Index, entryPoint uint8) {
					gotOut = append(gotOut, head)
				})
				for _, fHead := range forbidden {
					for _, head := range gotOut {
						if head == fHead {
							t.Errorf("ebg.ForOutEdgesOf(%d) contains forbidden edge to %d, want omitted", u, fHead)
						}
					}
				}
			}

			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				revID := ebg.GetRevId(da.Index(eID))
				if got, want := ebg.GetOutId(revID), da.Index(eID); got != want {
					t.Errorf("ebg.GetOutId(ebg.GetRevId(%d)) = %d, want %d", eID, got, want)
				}

				if got, want := ebg.GetHead(da.Index(eID)), ebg.GetHeadOfInedge(revID); got != want {
					t.Errorf("ebg.GetHeadOfInedge(ebg.GetRevId(%d)) = %d, want %d (ebg.GetHead(%d))", eID, got, want, eID)
				}
				if got, want := ebg.GetTailOfOutedge(da.Index(eID)), ebg.GetTail(revID); got != want {
					t.Errorf("ebg.GetTail(ebg.GetRevId(%d)) = %d, want %d (ebg.GetTailOfOutedge(%d))", eID, got, want, eID)
				}
			}

			for i := 0; i <= tc.wantNumVertices; i++ {
				idx := da.Index(i)
				if got, want := ebg.GetVertexFirstOut(idx), tc.wantFirstOut[i]; got != want {
					t.Errorf("ebg.GetVertexFirstOut(%d) = %d, want %d", i, got, want)
				}
			}

			for i := 0; i <= tc.wantNumVertices; i++ {
				idx := da.Index(i)
				if got, want := ebg.GetVertexFirstIn(idx), tc.wantFirstIn[i]; got != want {
					t.Errorf("ebg.GetVertexFirstIn(%d) = %d, want %d", i, got, want)
				}
			}

			dummyVertex := ebg.GetVertex(da.Index(tc.wantNumVertices))
			dummyCoord := dummyVertex.GetCoordinate()
			if !util.Eq(dummyCoord.GetLat(), 0.0) || !util.Eq(dummyCoord.GetLon(), 0.0) {
				t.Errorf("ebg.GetVertex(%d).GetCoordinate() = (%v, %v), want (0, 0) [sentinel dummy]",
					tc.wantNumVertices, dummyCoord.GetLat(), dummyCoord.GetLon())
			}

			if diff := cmp.Diff(tc.wantWeights, ebgWf.Getweights(), cmpopts.EquateEmpty(), cmp.Comparer(util.Eq[float64])); diff != "" {
				t.Errorf("ebgWf.Getweights() mismatch (-want +got):\n%s", diff)
			}
			for eID := 0; eID < ebg.NumberOfEdges(); eID++ {
				if got, want := ebgWf.GetWeight(da.Index(eID)), tc.wantWeights[eID]; !util.Eq(got, want) {
					t.Errorf("ebgWf.GetWeight(%d) = %v, want %v", eID, got, want)
				}
			}

		})
	}
}

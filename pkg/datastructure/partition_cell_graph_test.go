package datastructure_test

import (
	"slices"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/stretchr/testify/assert"
)

func buildCellGraph(t *testing.T, n int, adjList [][]da.PairEdge) *da.CellGraph {
	t.Helper()

	nodeCoords := make([]da.Coordinate, n)
	for i := 0; i < n; i++ {
		nodeCoords[i] = da.NewCoordinateWithoutScaling(float64(i), float64(i))
	}

	es := da.FlattenEdges(adjList)

	op := extractor.NewExtractor[float64]()
	acceptedNodeMap := make(map[int64]da.Coordinate, n)
	nodeToOsmID := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = nodeCoords[i]
		nodeToOsmID[da.Index(i)] = int64(i)
	}

	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmID(nodeToOsmID)

	rn := da.NewRoadNetworkDataContainerWithSize(len(es), n)
	g, _, _, _ := op.BuildGraph(es, rn, uint32(n), false)

	cellVIDs := make([]da.Index, n)
	gv := make([]da.Index, n)
	for i := 0; i < n; i++ {
		cellVIDs[i] = da.Index(i)
		gv[i] = da.Index(i)
	}
	cg := da.NewCellGraph(g, cellVIDs, cellVIDs, gv, 0, da.Index(n))

	return cg
}

type tEdge struct {
	u, v da.Index
}

/*
TestPartitionCellGraph_No_AntiParallel_Edge test wether the cellGraph has anti-parallel edges.

test case 1 taken from CLRS 3rd ed. figure 26.2 (a): input directed graph
   -> v1 ---->v3
  /  /\	|	  /\ \
 /	 |	|	  |	  \
s	 |	|	  |	   > t
\    |  |     |   /
 \   | \/     |  /
  ->  v2 -----> v4


the network flow graph (cellGraph) should handle the anti-parallel edges (v1, v2) and (v2, v1)
by creating new vertex v', and replace edge (v1, v2) by the pair of edges (v1, v') and (v', v2)...
*/ // nolint: gofmt
func TestPartitionCellGraph_No_AntiParallel_Edge(t *testing.T) {
	const (
		ss     int = 0
		v1     int = 1
		v2     int = 2
		v3     int = 3
		v4     int = 4
		tt     int = 5
		vPrime int = 6
	)

	testCases := []struct {
		name                         string
		n                            int
		adjList                      [][]da.PairEdge
		sources                      []int
		sinks                        []int
		wantAntiParallelEdgesRemoved []tEdge
		wantCreatedNewEdges          []tEdge
	}{
		{
			name: "Figure_26_2_CLRS_3rd_ed", // want no anti-parallel edge (v1,v2) and (v2,v1)
			n:    6,
			adjList: [][]da.PairEdge{
				ss: {da.NewPairEdge(v1, 1), da.NewPairEdge(v2, 1)},
				v1: {da.NewPairEdge(v2, 1), da.NewPairEdge(v3, 1)},
				v2: {da.NewPairEdge(v1, 1), da.NewPairEdge(v4, 1)},
				v3: {da.NewPairEdge(v2, 1), da.NewPairEdge(tt, 1)},
				v4: {da.NewPairEdge(v3, 1), da.NewPairEdge(tt, 1)},
			},
			sources:                      []int{ss},
			sinks:                        []int{tt},
			wantAntiParallelEdgesRemoved: []tEdge{{da.Index(v1), da.Index(v2)}, {da.Index(v2), da.Index(v1)}},
			wantCreatedNewEdges:          []tEdge{{da.Index(v1), da.Index(vPrime)}, {da.Index(vPrime), da.Index(v2)}},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			cg := buildCellGraph(t, tc.n, tc.adjList)

			gotAntiParallelEdges := []tEdge{}
			gotCreatedNewEdges := []tEdge{}
			for u := da.Index(0); u < da.Index(cg.NumberOfNetworkFlowVertices()); u++ {
				uDeg := cg.OutDegree(da.Index(u))
				for j := da.Index(0); j < uDeg; j++ {
					v := cg.OutEdge(u, j)
					for _, e := range tc.wantAntiParallelEdgesRemoved {
						if e.u == u && e.v == v {
							gotAntiParallelEdges = append(gotAntiParallelEdges, e)
						}
					}
					for _, e := range tc.wantCreatedNewEdges {
						if e.u == u && e.v == v {
							gotCreatedNewEdges = append(gotCreatedNewEdges, e)
						}
					}
				}
			}

			if len(gotAntiParallelEdges) == len(tc.wantAntiParallelEdgesRemoved) {
				t.Errorf("gotAntiParallelEdgeExists = %v, want to remove one of the edges: %v", gotAntiParallelEdges, tc.wantAntiParallelEdgesRemoved)
			}

			assert.Equal(t, tc.wantCreatedNewEdges, gotCreatedNewEdges)
		})
	}
}

/*
TestPartitionCellGraph_Partition test wether the S,T partition correct.

test case 1 taken from CLRS 3rd ed. figure 26.2 (a): input directed graph
   -> v1 ---->v3
  /  /\	|	  /\ \
 /	 |	|	  |	  \
s	 |	|	  |	   > t
\    |  |     |   /
 \   | \/     |  /
  ->  v2 -----> v4

we partition the vertices into two subset: S and T
S=[s,v1,v2]
T=[v3,v4,T]
*/ // nolint: gofmt
func TestPartitionCellGraph_Partition(t *testing.T) {
	const (
		ss     int = 0
		v1     int = 1
		v2     int = 2
		v3     int = 3
		v4     int = 4
		tt     int = 5
		vPrime int = 6
	)

	testCases := []struct {
		name               string
		n                  int
		adjList            [][]da.PairEdge
		wantS, wantT       []da.Index
		wantExistsCutEdges bool
	}{
		{
			name: "Figure_26_2_CLRS_3rd_ed", // want no anti-parallel edge (v1,v2) and (v2,v1)
			n:    6,
			adjList: [][]da.PairEdge{
				ss: {da.NewPairEdge(v1, 1), da.NewPairEdge(v2, 1)},
				v1: {da.NewPairEdge(v2, 1), da.NewPairEdge(v3, 1)},
				v2: {da.NewPairEdge(v1, 1), da.NewPairEdge(v4, 1)},
				v3: {da.NewPairEdge(v2, 1), da.NewPairEdge(tt, 1)},
				v4: {da.NewPairEdge(v3, 1), da.NewPairEdge(tt, 1)},
			},
			wantS:              []da.Index{da.Index(ss), da.Index(v1), da.Index(v2)},
			wantT:              []da.Index{da.Index(v3), da.Index(v4), da.Index(tt)},
			wantExistsCutEdges: false,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			cg := buildCellGraph(t, tc.n, tc.adjList)

			gotSCG, gotTCG := cg.Partition(func(vId da.Index) bool {
				for _, u := range tc.wantS {
					if u == vId {
						return true
					}
				}
				return false
			})

			gotS := make([]da.Index, 0)
			gotT := make([]da.Index, 0)
			gotCutEdgesExists := false
			gotSCG.ForEachCellVertices(func(vId, gvId da.Index, coord da.Coordinate) {
				gotS = append(gotS, gvId)
				gotSCG.ForEdgesOf(vId, func(v da.Index, rev bool) {
					vv := gotSCG.GetGraphVertexId(v)
					if vv == da.INVALID_SEGMENT_ID {
						// vv is an auxiliary vertex (for handling anti-parallel edges), not a road network vertex
						return
					}
					if slices.Contains(tc.wantT, vv) {
						gotCutEdgesExists = true
					}
				})
			})

			gotTCG.ForEachCellVertices(func(vId, gvId da.Index, coord da.Coordinate) {
				gotT = append(gotT, gvId)
				gotTCG.ForEdgesOf(vId, func(v da.Index, rev bool) {
					vv := gotTCG.GetGraphVertexId(v)
					if vv == da.INVALID_SEGMENT_ID {
						// vv is an auxiliary vertex (for handling anti-parallel edges), not a road network vertex
						return
					}
					if slices.Contains(tc.wantS, vv) {
						gotCutEdgesExists = true
					}
				})
			})

			assert.Equal(t, tc.wantS, gotS)
			assert.Equal(t, tc.wantT, gotT)
			assert.Equal(t, tc.wantExistsCutEdges, gotCutEdgesExists)
		})
	}
}

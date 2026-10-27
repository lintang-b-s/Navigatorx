package partitioner

import (
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func buildCellGraph(t *testing.T, n int, adjList [][]da.PairEdge) (*da.Graph, *da.CellGraph) {
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

	return g, cg
}

// TestDinicUnitMaxFlow tests DinicUnitMaxFlow using table-driven tests.
func TestDinicUnitMaxFlow(t *testing.T) {
	// Node indices for Figure 26.3 test case:  https://www.cl.cam.ac.uk/teaching/1415/Algorithms/flowcut.pdf (all non-artificial edge capacities are set to unit capacity 1)
	// Sources: s1=0, s2=1, s3=2, s4=3, s5=4
	// Intermediate vertices: v1=5, v2=6, v3=7, v4=8
	// Sinks: t1=9, t2=10, t3=11
	const (
		s1 da.Index = 0
		s2 da.Index = 1
		s3 da.Index = 2
		s4 da.Index = 3
		s5 da.Index = 4

		v1 da.Index = 5
		v2 da.Index = 6
		v3 da.Index = 7
		v4 da.Index = 8

		t1 da.Index = 9
		t2 da.Index = 10
		t3 da.Index = 11

		nFigure26_3 = 12
	)

	tests := []struct {
		name                  string
		numVertices           int
		wantNumEdges          int
		adjList               [][]da.PairEdge
		sources               []da.Index
		sinks                 []da.Index
		wantMaxFlow           int
		wantCutEdgesCount     int
		wantNumNodesInPartTwo int
		wantPartitionOne      []da.Index
		wantPartitionTwo      []da.Index
		wantCutEdges          map[[2]da.Index]bool
	}{
		{
			name:         "Figure26_3_CLRS_3ed", //  https://www.cl.cam.ac.uk/teaching/1415/Algorithms/flowcut.pdf or CLRS 3ed (all non-artificial edge capacities are set to unit capacity 1)
			numVertices:  nFigure26_3,
			wantNumEdges: 14,
			adjList: [][]da.PairEdge{
				s1: {da.NewPairEdge(int(v1), 1.0)},
				s2: {da.NewPairEdge(int(v1), 1.0), da.NewPairEdge(int(v2), 1.0)},
				s3: {da.NewPairEdge(int(v2), 1.0), da.NewPairEdge(int(v3), 1.0)},
				s4: {da.NewPairEdge(int(v3), 1.0), da.NewPairEdge(int(v4), 1.0)},
				s5: {da.NewPairEdge(int(v4), 1.0)},
				v1: {da.NewPairEdge(int(t1), 1.0)},
				v2: {da.NewPairEdge(int(t1), 1.0), da.NewPairEdge(int(t2), 1.0)},
				v3: {da.NewPairEdge(int(t2), 1.0), da.NewPairEdge(int(t3), 1.0)},
				v4: {da.NewPairEdge(int(t3), 1.0)},
			},
			sources:               []da.Index{s1, s2, s3, s4, s5},
			sinks:                 []da.Index{t1, t2, t3},
			wantMaxFlow:           6,
			wantCutEdgesCount:     6,
			wantNumNodesInPartTwo: 5,
			wantPartitionOne:      []da.Index{s1, s2, s3, s4, s5, v1, v4},
			wantPartitionTwo:      []da.Index{v2, v3, t1, t2, t3},
			wantCutEdges: map[[2]da.Index]bool{
				{s2, v2}: true,
				{s3, v2}: true,
				{s3, v3}: true,
				{s4, v3}: true,
				{v1, t1}: true,
				{v4, t3}: true,
			},
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			g, cg := buildCellGraph(t, tc.numVertices, tc.adjList)

			require.Equal(t, tc.numVertices, g.NumberOfVertices())
			require.Equal(t, tc.wantNumEdges, g.NumberOfEdges())
			require.Equal(t, tc.numVertices, cg.NumberOfCellVertices())

			dn := NewDinicUnitMaxFlow(tc.numVertices, cg)
			dn.createArtificialSourceSink(tc.sources, tc.sinks)

			minCut := dn.ComputeMaxflowMinCut()

			assert.Equal(t, tc.wantMaxFlow, minCut.GetMaxFlow())
			assert.Equal(t, tc.wantCutEdgesCount, minCut.CutEdgesCount())

			assert.Equal(t, tc.wantNumNodesInPartTwo, minCut.GetNumNodesInPartitionTwo())

			for _, s := range tc.wantPartitionOne {
				assert.True(t, minCut.GetFlag(s), "node %d should be in partition one", s)
			}

			for _, sk := range tc.wantPartitionTwo {
				assert.False(t, minCut.GetFlag(sk), "node %d should be in partition two", sk)
			}

			gotCutEdges := make(map[[2]da.Index]bool)
			for u := da.Index(0); u < da.Index(tc.numVertices); u++ {

				cg.ForEdgesOf(u, func(v da.Index, rev bool) {
					uInS := minCut.GetFlag(u)
					vInT := !minCut.GetFlag(v)

					if !rev && (uInS && vInT) {
						gotCutEdges[[2]da.Index{u, v}] = true
					}
				})
			}
			if len(tc.wantCutEdges) == 0 {
				assert.Empty(t, gotCutEdges)
			} else {
				assert.Equal(t, tc.wantCutEdges, gotCutEdges)
			}

		})
	}
}

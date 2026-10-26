package partitioner

import (
	"math"
	"testing"

	"github.com/google/go-cmp/cmp"
	"github.com/google/go-cmp/cmp/cmpopts"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
)

type pairEdge struct {
	to     int
	weight float64
}

func newPairEdge(to int, weight float64) pairEdge {
	return pairEdge{to: to, weight: weight}
}

func flattenEdges(es [][]pairEdge) []extractor.Edge[float64] {
	totalEdges := 0
	for _, edges := range es {
		totalEdges += len(edges)
	}
	flatten := make([]extractor.Edge[float64], 0, totalEdges)
	for from, edges := range es {
		for _, e := range edges {
			length := uint32(math.Round(e.weight * 100))
			flatten = append(flatten, extractor.NewEdge(uint32(from), uint32(e.to), e.weight, length))
		}
	}
	return flatten
}

func buildCellGraph(t *testing.T, n int, adjList [][]pairEdge) (*da.Graph, *da.CellGraph) {
	t.Helper()

	nodeCoords := make([]da.Coordinate, n)
	for i := 0; i < n; i++ {
		nodeCoords[i] = da.NewCoordinateWithoutScaling(float64(i), float64(i))
	}

	es := flattenEdges(adjList)

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
	// Node indices for Figure 26.3 test case:  https://www.cl.cam.ac.uk/teaching/1415/Algorithms/flowcut.pdf
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
		adjList               [][]pairEdge
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
			name:         "Figure26_3_CLRS_3ed", //  https://www.cl.cam.ac.uk/teaching/1415/Algorithms/flowcut.pdf or CLRS 3ed
			numVertices:  nFigure26_3,
			wantNumEdges: 14,
			adjList: [][]pairEdge{
				s1: {newPairEdge(int(v1), 1.0)},
				s2: {newPairEdge(int(v1), 1.0), newPairEdge(int(v2), 1.0)},
				s3: {newPairEdge(int(v2), 1.0), newPairEdge(int(v3), 1.0)},
				s4: {newPairEdge(int(v3), 1.0), newPairEdge(int(v4), 1.0)},
				s5: {newPairEdge(int(v4), 1.0)},
				v1: {newPairEdge(int(t1), 1.0)},
				v2: {newPairEdge(int(t1), 1.0), newPairEdge(int(t2), 1.0)},
				v3: {newPairEdge(int(t2), 1.0), newPairEdge(int(t3), 1.0)},
				v4: {newPairEdge(int(t3), 1.0)},
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

			if got, want := g.NumberOfVertices(), tc.numVertices; got != want {
				t.Fatalf("g.NumberOfVertices() = %d, want %d", got, want)
			}
			if got, want := g.NumberOfEdges(), tc.wantNumEdges; got != want {
				t.Fatalf("g.NumberOfEdges() = %d, want %d", got, want)
			}
			if got, want := cg.NumberOfCellVertices(), tc.numVertices; got != want {
				t.Fatalf("cg.NumberOfCellVertices() = %d, want %d", got, want)
			}

			dn := NewDinicUnitMaxFlow(tc.numVertices, cg)
			dn.createArtificialSourceSink(tc.sources, tc.sinks)

			minCut := dn.ComputeMaxflowMinCut()

			if got, want := minCut.GetMaxFlow(), tc.wantMaxFlow; got != want {
				t.Errorf("minCut.GetMaxFlow() = %d, want %d", got, want)
			}
			if got, want := minCut.CutEdgesCount(), tc.wantCutEdgesCount; got != want {
				t.Errorf("minCut.CutEdgesCount() = %d, want %d", got, want)
			}

			if got, want := minCut.GetNumNodesInPartitionTwo(), tc.wantNumNodesInPartTwo; got != want {
				t.Errorf("minCut.GetNumNodesInPartitionTwo() = %d, want %d", got, want)
			}

			for _, s := range tc.wantPartitionOne {
				if got, want := minCut.GetFlag(s), true; got != want {
					t.Errorf("minCut.GetFlag(%d) = %t, want %t", s, got, want)
				}
			}

			for _, sk := range tc.wantPartitionTwo {
				if got, want := minCut.GetFlag(sk), false; got != want {
					t.Errorf("minCut.GetFlag(%d) = %t, want %t", sk, got, want)
				}
			}

			gotCutEdges := make(map[[2]da.Index]bool)
			for u := da.Index(0); u < da.Index(tc.numVertices); u++ {
				if !minCut.GetFlag(u) {
					continue
				}
				cg.ForEdgesOf(u, func(v da.Index, out bool) {
					if !minCut.GetFlag(v) {
						gotCutEdges[[2]da.Index{u, v}] = true
					}
				})
			}
			if diff := cmp.Diff(tc.wantCutEdges, gotCutEdges, cmpopts.EquateEmpty()); diff != "" {
				t.Errorf("cut edges mismatch (-want +got):\n%s", diff)
			}

		})
	}
}

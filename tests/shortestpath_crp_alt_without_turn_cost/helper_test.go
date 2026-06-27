package shortestpath_crp_alt_without_turn_cost

import (
	"math"
	"math/rand"
	"os"
	"runtime"
	"testing"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/lintang-b-s/Navigatorx/tests"
)

func TestMain(m *testing.M) {
	gopool.SetCap(int32(runtime.NumCPU()))
	os.Exit(m.Run())
}

const (
	graphFile        string = "./data/original_sp_test.ngraph"
	overlayGraphFile string = "./data/overlay_graph_sp_test.ngraph"
	landmarkFile     string = "./data/landmark_sp_test.nlm"
)

func buildCRP(t *testing.T, nodeCoords []osmparser.NodeCoord, adjList [][]tests.PairEdge, n int, Us []int, pgDirected bool) (*engine.Engine[float64], *da.Graph,
	[]da.Index, map[da.Index]da.Index, *landmark.Landmark[float64]) {

	da.CoordinatePrecision = 1e3

	es := tests.FlattenEdges(adjList)

	op := osmparser.NewOSMParserV2[float64]()
	acceptedNodeMap := make(map[int64]osmparser.NodeCoord, n)
	nodeToOsmId := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = nodeCoords[i]
		nodeToOsmId[da.Index(i)] = int64(i)
	}

	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmId)

	gs := da.NewGraphStorageWithSize(len(es), n)
	g, timeFunction, edgeInfoIds := op.BuildGraph(es, gs, uint32(n), false)

	t.Logf("number of vertices: %v, number of edges: %v", uint32(n), len(es))

	g.SetGraphStorage(gs)

	logger, err := logger.New()
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	ps := make([]int, len(Us))

	for i := 0; i < len(ps); i++ {
		pow := Us[i]
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps), 1,
		g, logger, true, true,
	)
	mp.RunMultilevelPartitioning()

	mlp := mp.BuildMLP()

	prep := preprocesser.NewPreprocessor(g, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(false)
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	util.USE_INT32 = false

	og := prep.GetOverlayGraph()
	cust := customizer.NewCustomizerDirect(
		g, og, prep.GetTimeFunction(), logger,
	)
	m, err := cust.CustomizeDirect()
	if err != nil {
		t.Fatalf("err: %v", err)
	}
	lm := landmark.NewLandmark[float64]()
	err = lm.PreprocessALT(1, m, g, logger)
	if err != nil {
		panic(err)
	}
	if err := lm.WriteLandmark(landmarkFile, g.NumberOfVertices()); err != nil {
		t.Fatalf("err: %v", err)
	}

	re, err := engine.NewEngineDirect(g, og, m, logger, landmarkFile)
	if err != nil {
		t.Fatalf("err: %v", err)
	}

	oldToNewVIdMap := prep.GetOldToNewVIdMap()
	newToOldVidMap := prep.GetNewToOldVIdMap()

	return re, g, oldToNewVIdMap, newToOldVidMap, lm
}

func RandomCoordinate(bb *da.BoundingBox, rd *rand.Rand) da.Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return da.NewCoordinate(lat, lon)
}

func assignNodeCoordinates(adjList [][]tests.PairEdge) []osmparser.NodeCoord {
	n := len(adjList)
	nodeCoords := make([]osmparser.NodeCoord, n)
	lmOne := da.Index(0)

	distLmOne := dijkstra(adjList, lmOne)

	lmTwo := da.Index(1)
	maxDistTwo := -1.0
	for v := da.Index(0); v < da.Index(n); v++ {
		if distLmOne[v] > maxDistTwo && v != lmOne {
			lmTwo = v
			maxDistTwo = distLmOne[v]
		}
	}

	distLmTwo := dijkstra(adjList, lmTwo)

	lmThree := da.Index(0)
	maxDistThree := -1.0
	for v := da.Index(0); v < da.Index(n); v++ {
		dd := min(distLmTwo[v], distLmOne[v])
		if dd > maxDistThree && v != lmTwo && v != lmOne {
			lmThree = v
			maxDistThree = dd
		}
	}

	distLmOneTwo := float64(maxDistTwo)

	distLmTwoThree := float64(distLmTwo[lmThree])
	distLmOneThree := float64(distLmOne[lmThree])

	nodeCoords[lmOne] = osmparser.NewNodeCoord(0, 0)
	nodeCoords[lmTwo] = osmparser.NewNodeCoord(distLmOneTwo, 0)

	nume := distLmOneThree*distLmOneThree - distLmTwoThree*distLmTwoThree + distLmOneTwo*distLmOneTwo
	denom := 2 * distLmOneTwo
	x3 := nume / denom
	y3 := math.Sqrt(distLmOneThree*distLmOneThree - x3*x3)
	nodeCoords[lmThree] = osmparser.NewNodeCoord(x3, y3)

	distLmThree := dijkstra(adjList, lmThree)

	// trilateration
	for v := da.Index(0); v < da.Index(n); v++ {
		if v == lmOne || v == lmTwo || v == lmThree {
			continue
		}

		nume = distLmOne[v]*distLmOne[v] - distLmTwo[v]*distLmTwo[v] + distLmOneTwo*distLmOneTwo
		denom = 2 * distLmOneTwo
		x := nume / denom

		nume = distLmOne[v]*distLmOne[v] - distLmThree[v]*distLmThree[v] + x3*x3 + y3*y3 - 2*x3*x
		denom = 2 * y3

		if util.Eq(y3, 0) {
			denom = 0.05
		}
		y := nume / denom

		nodeCoords[v] = osmparser.NewNodeCoord(y, x)
	}

	return nodeCoords
}

func dijkstra(adjList [][]tests.PairEdge, s da.Index) []float64 {
	n := len(adjList)

	dist := make([]float64, n)
	for v := 0; v < n; v++ {
		dist[v] = util.INF_WEIGHT_FLOAT
	}

	pq := da.NewQueryHeap[da.Index, float64](uint32(n), 100, da.ARRAY_STORAGE, true)
	emptyVertexInfo := da.NewVertexInfo(float64(0), da.NewVertexEdgePair(0, 0, false))

	dist[s] = 0
	pq.Insert(s, 0, emptyVertexInfo, s)

	for !pq.IsEmpty() {
		uNode := pq.ExtractMin()
		u := uNode.GetItem()

		for _, e := range adjList[u] {
			v := da.Index(e.To)
			newVCost := uNode.GetRank() + e.Weight
			vLabelled := util.Lt(dist[v], util.INF_WEIGHT_FLOAT)
			if !vLabelled || (vLabelled && newVCost <= dist[v]) {
				dist[v] = uNode.GetRank() + e.Weight
				if !vLabelled {
					pq.Insert(v, newVCost, emptyVertexInfo, v)
				} else {
					pq.DecreaseKey(v, newVCost, newVCost, emptyVertexInfo.GetParent())
				}
			}
		}
	}

	return dist
}

// func Kosaraju(adjList [][]tests.PairEdge) ([]da.Index, int) {
// 	// O(V+E)
// 	n := da.Index(len(adjList))

// 	revAdjList := make([][]tests.PairEdge, n)
// 	for u := da.Index(0); u < n; u++ {
// 		for _, e := range adjList[u] {
// 			v := da.Index(e.To)
// 			revAdjList[v] = append(revAdjList[v], tests.NewPairEdge(int(u), e.Weight))
// 		}
// 	}

// 	components := make([][]da.Index, 0, 10)

// 	order := make([]da.Index, 0, n)
// 	visited := make([]bool, n)
// 	for v := da.Index(0); v < n; v++ {
// 		// v is index of vertice id
// 		if !visited[v] {
// 			dfs(da.Index(v), &order, visited, false, adjList, revAdjList)
// 		}
// 	}

// 	util.ReverseG[da.Index](order)

// 	// reset visited
// 	visited = make([]bool, n)
// 	roots := make([]da.Index, n)

// 	for _, v := range order {
// 		if !visited[v] {
// 			component := make([]da.Index, 0, 10)
// 			dfs(v, &component, visited, true, adjList, revAdjList)
// 			components = append(components, component)
// 			root := v
// 			for _, node := range component {
// 				roots[node] = root
// 			}
// 		}
// 	}

// 	sccs := make([]da.Index, n)

// 	for i, component := range components {
// 		for _, v := range component {
// 			sccs[v] = da.Index(i)
// 		}
// 	}

// 	return sccs, len(components)
// }

// func dfs(v da.Index, output *[]da.Index, visited []bool,
// 	reversed bool, adjList [][]tests.PairEdge, revAdjList [][]tests.PairEdge) {
// 	// discovered v

// 	visited[v] = true

// 	if !reversed {
// 		for _, e := range adjList[v] {
// 			eHead := da.Index(e.To)

// 			if !visited[eHead] {
// 				dfs(eHead, output, visited, reversed, adjList, revAdjList)
// 			}
// 		}

// 	} else {

// 		for _, e := range revAdjList[v] {
// 			eTail := da.Index(e.To)

// 			if !visited[eTail] {
// 				dfs(eTail, output, visited, reversed, adjList, revAdjList)
// 			}
// 		}
// 	}

// 	// finished v
// 	*output = append(*output, v)
// }

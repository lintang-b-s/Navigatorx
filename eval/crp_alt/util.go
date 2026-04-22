package crpalt

import (
	"fmt"
	"math/rand"
	"os"
	"path/filepath"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
)

func BuildCRP(nodeCoords []osmparser.NodeCoord, adjList [][]PairEdge, n int, Us []int, name string) (*engine.Engine, *da.Graph,
	[]da.Index, map[da.Index]da.Index) {
	workingDir, err := util.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}

	outputDir := filepath.Join(workingDir, "data", "eval")
	var (
		graphFile        string = filepath.Join(outputDir, fmt.Sprintf("original_dimacs_%s.graph", name))
		overlayGraphFile string = filepath.Join(outputDir, fmt.Sprintf("overlay_graph_dimacs_%s.graph", name))
		metricsFile      string = filepath.Join(outputDir, fmt.Sprintf("metrics_dimacs_%s.txt", name))
		landmarkFile     string = filepath.Join(outputDir, fmt.Sprintf("landmark_dimacs_%s.lm", name))
		mlpFile                 = filepath.Join(outputDir, fmt.Sprintf("dimacs_%s.mlp", name))
		timeFunctionFile string = filepath.Join(outputDir, fmt.Sprintf("timefunction_dimacs_%s.txt", name))
		prep             *preprocesser.Preprocessor
	)

	if err := os.MkdirAll(outputDir, 0755); err != nil {
		panic(err)
	}

	doPreprocessCustomize := false
	if _, err := os.Stat(metricsFile); os.IsNotExist(err) {
		doPreprocessCustomize = true
	}

	es := flattenEdges(adjList)

	op := osmparser.NewOSMParserV2()
	acceptedNodeMap := make(map[int64]osmparser.NodeCoord, n)
	nodeToOsmId := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = nodeCoords[i]
		nodeToOsmId[da.Index(i)] = int64(i)
	}
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmId)

	gs := da.NewGraphStorageWithSize(len(es), n)
	g, edgeInfoIds := op.BuildGraph(es, gs, uint32(n), false) // roadnetwork false biar ada dummy edge (v,v)

	g.SetGraphStorage(gs)

	logger, err := logger.New()
	if err != nil {
		panic(err)
	}
	if doPreprocessCustomize {

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

		err = mp.SaveToFile(mlpFile)
		if err != nil {
			panic(err)
		}

		mlp := datastructure.NewPlainMLP()
		err = mlp.ReadMlpFile(mlpFile)
		if err != nil {
			panic(err)
		}

		prep = preprocesser.NewPreprocessor(g, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
		err = prep.PreProcessing(true)
		if err != nil {
			panic(err)
		}

		cust := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, logger)
		m, err := cust.Customize()
		if err != nil {
			panic(err)
		}

		lm := landmark.NewLandmark()
		numberOfLandmarks := viper.GetInt("landmarks")

		err = lm.PreprocessALT(numberOfLandmarks, m, cust.GetGraph(), logger)
		if err != nil {
			panic(err)
		}
		err = lm.WriteLandmark(landmarkFile, cust.GetGraph())
		if err != nil {
			panic(err)
		}
	} else {

		mlp := datastructure.NewPlainMLP()
		err = mlp.ReadMlpFile(mlpFile)
		if err != nil {
			panic(err)
		}
		prep = preprocesser.NewPreprocessor(g, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
		err = prep.PreProcessing(false)
		if err != nil {
			panic(err)
		}
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		panic(err)
	}

	oldToNewVIdMap := prep.GetOldToNewVIdMap()
	newToOldVidMap := prep.GetNewToOldVIdMap()

	return re, g, oldToNewVIdMap, newToOldVidMap
}

type PairEdge struct {
	to     int
	weight float64
}

func NewPairEdge(to int, weight float64) PairEdge {
	return PairEdge{to, weight}
}

func flattenEdges(es [][]PairEdge) []osmparser.Edge {
	flatten := make([]osmparser.Edge, 0, len(es))

	eid := 0

	for from, edges := range es {
		for _, e := range edges {
			flatten = append(flatten, osmparser.NewEdge(uint32(from), uint32(e.to), e.weight, e.weight, 0))
			eid++
		}
	}

	return flatten
}

type QueryParam struct {
	i, s, t da.Index
}

func (q *QueryParam) GetId() da.Index {
	return q.i
}

func (q *QueryParam) GetSource() da.Index {
	return q.s
}

func (q *QueryParam) GetTarget() da.Index {
	return q.t
}

func NewQueryParam(i, s, t da.Index) QueryParam {
	return QueryParam{i, s, t}
}

func RandomCoordinate(bb *da.BoundingBox, rd *rand.Rand) da.Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return da.NewCoordinate(lat, lon)
}

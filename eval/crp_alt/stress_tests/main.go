package main

import (
	"context"
	"flag"
	"fmt"
	"io"
	"math"
	"math/rand"
	"net/http"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"

	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	mlpFile                 = "stress_test_yogyakarta"
	url                     = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile                = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
	landmarkFile     string = "./data/landmark.lm"
)

/*
go run eval/crp_alt/stress_tests/main.go

stress test ini bertujuan untuk mencari apakah ada counterexample (output dari ALT CRP query bukan shortest path) dari implementasi  A*, landmarks, and triangle inequality (ALT) untuk query Customizable Route Planning (CRP) pada file
crp_bidir_astar_landmark.go

ref:
[1] Delling, D. et al. (2013) ‘Customizable Route Planning in Road Networks’. Available at: https://www.microsoft.com/en-us/research/publication/customizable-route-planning-in-road-networks/.
[2] goal-direction A*, landmarks, and triangle inequality (ALT) CRP query: Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.

untuk plain dijkstra find single source shortest paths (sssp), from s to all other vertices
untuk alt query crp find point-to-point(p2p) shortest paths, untuk semua p2p sp queries diatas
yogyakarta.osm.pbf di google drive ada 55136 vertices, dan stress tests ini dari shortest paths dari 50 sources ke 55136 vertices jadi ada sekitar 2.7568 x 10^6 p2p sp query
note that Customizable Route Planning (CRP) query/multilevel dijkstra (MLD) [https://github.com/Project-OSRM/osrm-backend]/ multilevel A*, landmarks, and triangle inequality (ALT) hanya mempercepat p2p sp query bukan mempercepat sssp query

stress tests ini selesai dalam 30 menit
dan akan kestop ketika ada counterexample
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphic #6 cpu cores #12 threads
ram: 16gb
*/
func main() {
	if err := os.MkdirAll("./data", 0755); err != nil {
		panic(err)
	}
	logger, err := log.New()
	if err != nil {
		panic(err)
	}
	if _, err := os.Stat(osmfFile); os.IsNotExist(err) {
		output, err := os.Create(osmfFile)
		if err != nil {
			panic(err)
		}
		defer output.Close()

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(url)
		if err != nil {
			panic(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			panic(err)
		}
		logger.Sugar().Infof("download complete")
	}

	osmParser := osmparser.NewOSMParserV2()

	graph, err := osmParser.Parse(fmt.Sprintf("%s", osmfFile), logger, false)
	if err != nil {
		panic(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
		if err != nil {
			panic(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		graph, logger,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		panic(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s", "crp_inertial_flow_"+mlpFile+".mlp"))
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger)
	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}

	logger.Sugar().Infof("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	m, err := custom.Customize()
	if err != nil {
		panic(err)
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(16, m, custom, logger)
	if err != nil {
		panic(err)
	}
	err = lm.WriteLandmark(landmarkFile, custom)
	if err != nil {
		panic(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		panic(err)
	}

	g := re.GetRoutingEngine().GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	n := int(math.Pow(10, 1)) * 5
	qset := make(map[da.Index]struct{})

	queries := make([]da.Index, 0, n)

	i := 0
	for i < n {
		s := da.Index(rd.Intn(V))

		if _, ok := qset[s]; ok {
			continue
		}

		qset[s] = struct{}{}
		queries = append(queries, da.Index(s))
		i++
	}
	numberOfVertices := g.NumberOfVertices()

	expectedSPTravelTimes := make([][]float64, n)
	// expectedShortestPaths := make([][][]da.Index, n) // kita gak assert vertex ids path nya karena makan memory gede banget awowakaowo
	// todo: mungkin bisa write ke file langsung buat expectedVertexIds nya?
	// for i := 0; i < n; i++ {
	// 	expectedShortestPaths[i] = make([][]da.Index, numberOfVertices)
	// }

	lock := sync.Mutex{}

	calcSpDijkstra := func(i int) any {
		s := queries[i]

		dijkstraQuery := routing.NewDijkstra(re.GetRoutingEngine())

		sps, _ := dijkstraQuery.ShortestPath(s)

		lock.Lock()
		// for t := 0; t < numberOfVertices; t++ {
		// 	spVertices := make([]da.Index, 0, len(spEdges[t]))
		// 	for k := 0; k < len(spEdges[t]); k++ {
		// 		spVertices = append(spVertices, spEdges[t][k].GetHead())
		// 	}
		// 	expectedShortestPaths[i][t] = append(expectedShortestPaths[i][t], spVertices...)
		// }
		expectedSPTravelTimes[i] = sps

		if (i+1)%5 == 0 {
			logger.Sugar().Infof("done query from source number: %v\n", i+1)
		}
		lock.Unlock()

		return nil
	}

	workersDijkstra := concurrent.NewWorkerPool[int, any](7, n)

	for i := 0; i < n; i++ {
		workersDijkstra.AddJob(i)
	}

	workersDijkstra.Close()
	workersDijkstra.Start(calcSpDijkstra)
	workersDijkstra.WaitDirect()

	type query struct {
		i, s, t da.Index
		id      da.Index
	}

	newQuery := func(i, s, t, id da.Index) query {
		return query{i, s, t, id}
	}

	type counterExampleData struct {
		expectedSp, crpALTSP           float64
		expectedSpEdges, crpALTSPEdges []da.Index
		counterexample                 bool
	}

	newCounterExampleData := func(expectedSp, crpALTSP float64, expectedSpEdges, crpALTSPEdges []da.Index, cx bool) counterExampleData {
		return counterExampleData{expectedSp: expectedSp, crpALTSP: crpALTSP, expectedSpEdges: expectedSpEdges, crpALTSPEdges: crpALTSPEdges,
			counterexample: cx}
	}

	calcSp := func(q query) counterExampleData {
		i := q.i
		s := q.s
		t := q.t
		id := q.id
		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1
		at := g.GetEntryOffset(t) + g.GetInDegree(t) - 1
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)
		sp, _, _, _, _ := crpQuery.ShortestPathSearch(as, at)

		expectedSp := expectedSPTravelTimes[i][t]
		// expectedSPEdges := expectedShortestPaths[i][t]

		counterexample := false
		if !da.EqEps(expectedSp, sp, 1e-5) { // shortcuts weights (hasil dari Customization phase of CRP yang diwrite ke file & read lagi ) mungkin gak terlalu presisi
			counterexample = true
		}

		// if len(spEdges) != len(expectedSPEdges) {
		// 	counterexample = true
		// }

		// for k := 0; k < len(spEdges); k++ {
		// 	if spEdges[k].GetHead() != expectedSPEdges[k] {
		// 		counterexample = true
		// 		break
		// 	}
		// }

		if (id+1)%5000 == 0 {
			logger.Sugar().Infof("done query id: %v\n", id+1)
		}
		if counterexample {
			// spVertices := make([]da.Index, 0, len(spEdges))
			// for k := 0; k < len(spEdges); k++ {
			// 	spVertices = append(spVertices, spEdges[k].GetHead())
			// }
			return newCounterExampleData(expectedSp, sp, []da.Index{}, []da.Index{}, true)
		}

		return newCounterExampleData(0, 0, nil, nil, false)
	}

	workers := concurrent.NewWorkerPool[query, counterExampleData](1, n*numberOfVertices)

	for i := 0; i < n; i++ {
		s := queries[i]

		for t := da.Index(0); t < da.Index(numberOfVertices); t++ {
			workers.AddJob(newQuery(da.Index(i), s, t, da.Index(i)*da.Index(numberOfVertices)+t))
		}
	}
	logger.Sugar().Infof("start crp query...")

	ctx, cancel := context.WithCancel(context.Background())
	workers.Close()
	workers.StartWithContext(ctx, calcSp)
	workers.Wait()

	for res := range workers.CollectResults() {
		if res.counterexample {
			logger.Sugar().Infof("found counterExample!!\n")

			logger.Sugar().Infof("expected shortest path travel time: %f, got: %f\n", res.expectedSp, res.crpALTSP)
			cancel()
			// logger.Sugar().Infof("\n expected shortest path vertex ids:\n")
			// for j := 0; j < len(res.expectedSpEdges); j++ {
			// 	logger.Sugar().Infof("%v, ", res.expectedSpEdges[j])
			// }

			// logger.Sugar().Infof("\n got shortest path vertex ids:\n")
			// for j := 0; j < len(res.crpALTSPEdges); j++ {
			// 	logger.Sugar().Infof("%v, ", res.crpALTSPEdges[j])
			// }

			logger.Sugar().Infof("\n")
		}
	}

	logger.Sugar().Infof("done yeaayy (~˘▾˘)~ (~˘▾˘)~ (~˘▾˘)~ (~˘▾˘)~ (~˘▾˘)~ (~˘▾˘)~ (~˘▾˘)~ (~˘▾˘)~!!!!!")
}

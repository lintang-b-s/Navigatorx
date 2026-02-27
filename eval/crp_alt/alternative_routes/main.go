package main

import (
	"flag"
	"fmt"
	"io"
	"math"
	"math/rand"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"

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
	err = util.ReadConfig()
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

	n := int(math.Pow(10, 4))
	qset := make(map[uint64]struct{})

	type query struct {
		s, t da.Index
	}

	newQuery := func(s, t da.Index) query {
		return query{
			s: s,
			t: t,
		}
	}

	queries := make([]query, 0, n)

	bitpack := func(i, j da.Index) uint64 {
		return uint64(i) | (uint64(j) << 30)
	}

	i := 0
	for i < n {
		s := da.Index(rd.Intn(V))
		t := da.Index(rd.Intn(V))
		if s == t {
			continue
		}
		if !g.VerticeUandVAreConnected(s, t) {
			continue
		}
		if _, ok := qset[bitpack(s, t)]; ok {
			continue
		}

		qset[bitpack(s, t)] = struct{}{}
		queries = append(queries, newQuery(s, t))
		i++
	}

	successRate := 0.0
	stretch := 0.0
	diversity := 0.0
	runtime := 0.0

	foundAltCount := 0

	for i := 0; i < len(queries); i++ {
		s := queries[i].s
		t := queries[i].t

		altSearch := routing.NewAlternativeRouteSearch(re.GetRoutingEngine(), 1.3, 0.8, 0.3, 0.35, lm)

		alts := altSearch.FindAlternativeRoutes(s, t, 4)

		if (i+1)%100 == 0 {
			fmt.Printf("processed %d queries\n", i+1)
		}
		runtime += float64(altSearch.GetRuntime())

		if len(alts) == 0 {
			continue
		}

		stretch += altSearch.GetStretch()
		diversity += altSearch.GetDiversity()
		successRate += 1.0
		foundAltCount++
	}

	successRate /= float64(len(queries))
	stretch /= float64(foundAltCount)
	diversity /= float64(foundAltCount)
	runtime /= float64(len(queries))

	fmt.Printf("success rate: %f\n", successRate)
	fmt.Printf("stretch: %f\n", stretch)
	fmt.Printf("diversity: %f\n", diversity)
	fmt.Printf("runtime: %f ms\n", runtime)
}

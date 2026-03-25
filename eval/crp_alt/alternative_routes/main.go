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
	graphFile        string = "./data/original_eval_alt.graph"
	overlayGraphFile string = "./data/overlay_graph_eval_alt.graph"
	metricsFile      string = "./data/metrics_eval_alt.txt"
	landmarkFile     string = "./data/landmark_eval_alt.lm"
)

const (
	alpha      = 0.25 // every subpath P' of alternative route with l(P') <= T = \alpha* l(Opt) is optimal (shortest path). l(Opt) is the cost/travel time of the shortest path
	gamma      = 0.8  // alternative routes at least 20% different than the shortest path
	epsilon    = 0.25 // alternative routes at most 25% longer than the shortest path
	upperBound = 1.15
)

/*
go run eval/crp_alt/alternative_routes/main.go

naik dari 41-45% -> 60-66% -> setelah benerin cara dapetin via vertices: 84%-88% success rate nya. lets gooo

osrm cuma 52-56%

todo6: bikin cara agar bisa eliminate banyak via vertices sebelum di unpack path nya ... (DONE)
todo: target p95 latency dengan 900vus endpoint alternative routes: 200ms dengan success rate alternative routes > 85%

sekarang (setelah filter candidates sebelum path unpacking) p95 latency 300vus endpoint alternative routes: 377ms  sucess rate > 87% -> setelah optimize lagi: p95 latency 300vus alternative routes: 25ms (eval/crp_alt/load_tests/k6_alternatives.js)
todo: cek heap allocations FindAlternativeRoutes pakai pprof, kurangin heap allocation dari FindAlternativeRoutes, benchmark: setiap FindAlternatveRoutes() 2mb/op, kayake bisa dikurangi lagi
-> setelah optimize, benchmark FindAlternativeRoutes sekitar 600kb/op

todo: benerin calculateApproxDistanceShare() (DONE)

todo: pindahin hasil eksperimen di repo baru + bandingin juga dg graphopper , valhalla
todo2: add evaluasi online map matching pakai dataset dari https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/


....
*/
func main() {
	if err := os.MkdirAll("./data", 0755); err != nil {
		panic(err)
	}
	logger, err := log.New()
	if err != nil {
		panic(err)
	}
	workingDir, err := os.Getwd()
	err = util.ReadConfig(workingDir)
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

	op := osmparser.NewOSMParserV2()

	graph, err := op.Parse(fmt.Sprintf("%s", osmfFile), logger, false)

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
		25,
		graph, logger, true, false, true,
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
	prep := preprocessor.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile)
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
		if !g.VerticeUToVConnected(s, t) {
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

		altSearch := routing.NewAlternativeRouteSearch(re.GetRoutingEngine(), upperBound, gamma, alpha, epsilon, lm)

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

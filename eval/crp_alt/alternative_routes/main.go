package main

import (
	"flag"
	"fmt"
	"io"
	"math"
	"math/rand"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"

	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	mlpFile                 = "./data/stress_test_yogyakarta.mlp"
	url                     = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile                = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original_eval_alt.ngraph"
	overlayGraphFile string = "./data/overlay_graph_eval_alt.ngraph"
	metricsFile      string = "./data/metrics_eval_alt.nmt"
	landmarkFile     string = "./data/landmark_eval_alt.nlm"
	timeFunctionFile string = "./data/timefunction_eval_alt.ntf"
)

func init() {
	flag.Parse()
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	err = config.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicleEnabled = pkg.GetIsVehicle()
	pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()
}

/*
go run eval/crp_alt/alternative_routes/main.go

naik dari 41-45% -> 60-66% -> setelah benerin cara dapetin via vertices: 79%-81% success rate nya. lets gooo

osrm cuma 52-56%

todo6: bikin cara agar bisa eliminate banyak via vertices sebelum di unpack path nya ... (DONE)
todo: target p95 latency dengan 900vus endpoint alternative routes: 200ms dengan success rate alternative routes >= 80% (DONE)
osrm  p95 latency 900vus: 230ms

todo: optimize directionBuilder.GetDrivingDirections() (DONE)

todo: pindahin hasil eksperimen di repo baru + bandingin juga dg graphopper , valhalla
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
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	err = config.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}

	if _, err := os.Stat(osmfFile); os.IsNotExist(err) {
		output, err := os.Create(osmfFile)
		if err != nil {
			panic(err)
		}

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(url)
		if err != nil {
			panic(err)
		}

		_, err = io.Copy(output, response.Body)
		if err != nil {
			panic(err)
		}

		if err = output.Close(); err != nil {
			panic(err)
		}

		if err = response.Body.Close(); err != nil {
			panic(err)
		}

		logger.Sugar().Infof("download complete")
	}

	op := osmparser.NewOSMParserV2[int32]()

	graph, timeFunction, edgeInfoIds, err := op.Parse(osmfFile, logger)

	if err != nil {
		panic(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := util.ParseTextInt(pss[i])
		if err != nil {
			panic(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		25,
		graph, logger, false, false,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		panic(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpFile)
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}

	logger.Sugar().Infof("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)

	_, err = custom.Customize()
	if err != nil {
		panic(err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
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
		if !g.PathExists(s, t) {
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
	altSearch := routing.NewAlternativeRouteSearch(re.GetRoutingEngine())

	emptyCoords := make([]da.Coordinate, 0)

	for i := 0; i < len(queries); i++ {
		s := queries[i].s
		t := queries[i].t

		sVertex := g.GetVertex(s)
		tVertex := g.GetVertex(t)

		sp := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, sVertex.GetFirstOut(), sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tp := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), tVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)

		alts, optTravelTime, dur := altSearch.FindAlternativeRoutes(sp, tp, 4, false, 0)

		if (i+1)%100 == 0 {
			fmt.Printf("processed %d queries\n", i+1)
		}
		runtime += float64(dur)

		if len(alts) == 0 {
			continue
		}

		stretch += altSearch.GetStretch(alts, optTravelTime)
		diversity += altSearch.GetDiversity(alts)

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

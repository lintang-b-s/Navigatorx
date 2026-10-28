package main

import (
	"flag"
	"fmt"
	"io"
	stdLog "log"
	"math"
	"math/rand"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"

	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	url      = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile = "./data/yogyakarta.osm.pbf"
)

func init() {
	flag.Parse()
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		stdLog.Fatal(err.Error())
	}
	err = config.ReadConfig(workingDir)
	if err != nil {
		stdLog.Fatal(err.Error())
	}
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicleEnabled = pkg.GetIsVehicle()
	pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()

	config.InitRegionName("yogyakarta", pkg.EVAL)
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

	logger, err := log.New()
	if err != nil {
		stdLog.Fatal(err.Error())
	}
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		logger.Fatal(err.Error())
	}
	outputDir := filepath.Join(workingDir, "data")
	if err := util.EnsureDirExists(outputDir); err != nil {
		logger.Fatal(err.Error())
	}

	err = config.ReadConfig(workingDir)
	if err != nil {
		logger.Fatal(err.Error())
	}

	if _, err := os.Stat(osmfFile); os.IsNotExist(err) {
		if err := util.EnsureDirExists(osmfFile); err != nil {
			logger.Fatal(err.Error())
		}
		output, err := os.Create(osmfFile)
		if err != nil {
			logger.Fatal(err.Error())
		}

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(url)
		if err != nil {
			logger.Fatal(err.Error())
		}

		_, err = io.Copy(output, response.Body)
		if err != nil {
			logger.Fatal(err.Error())
		}

		if err = output.Close(); err != nil {
			logger.Fatal(err.Error())
		}

		if err = response.Body.Close(); err != nil {
			logger.Fatal(err.Error())
		}

		logger.Sugar().Infof("download complete")
	}
	metricsFile := fmt.Sprintf("%s/%s/%s_metrics.nmt", config.ProfilesRoot(), pkg.ProfileName, pkg.RegionName)

	doPreprocessCustomize := false
	if _, err := os.Stat(metricsFile); os.IsNotExist(err) {
		doPreprocessCustomize = true
	}

	if doPreprocessCustomize {
		op := extractor.NewExtractor[int32](logger)
		_, ebg, rn, _, timeFunction, err := op.Extract(osmfFile)
		if err != nil {
			logger.Fatal(err.Error())
		}

		pss := strings.Split(*partitionSizes, ",")
		ps := make([]int, len(pss))
		for i := 0; i < len(ps); i++ {
			pow, err := util.ParseTextInt(pss[i])
			if err != nil {
				logger.Fatal(err.Error())
			}
			ps[i] = 1 << pow // 2^pow
		}

		mp := partitioner.NewMultilevelPartitioner(
			ps,
			len(ps),
			25,
			ebg, logger,
		)

		mp.RunMultilevelPartitioning()
		err = mp.SaveToFile()
		if err != nil {
			logger.Fatal(err.Error())
		}

		mlp := da.NewPlainMLP()
		err = mlp.ReadMlpFile()
		if err != nil {
			logger.Fatal(err.Error())
		}

		prep := preprocessor.NewPreprocessor(ebg, rn, timeFunction, mlp, logger)
		err = prep.PreProcessing(true)
		if err != nil {
			logger.Fatal(err.Error())
		}

		logger.Sugar().Infof("Preprocessing completed successfully.")

		custom := customizer.NewCustomizer[int32](logger)

		_, err = custom.Customize()
		if err != nil {
			logger.Fatal(err.Error())
		}
	}

	eng, err := engine.NewEngine[int32](logger)
	if err != nil {
		logger.Fatal(err.Error())
	}
	re := eng.GetRoutingEngine()
	g := re.GetGraph()
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
	locOpt := 0.0
	distSharing := 0.0
	diversity := 0.0
	runtime := 0.0

	foundAltCount := 0

	for i := 0; i < len(queries); i++ {
		s := queries[i].s
		t := queries[i].t

		crpQuery := routing.NewCRPQuery(re) // better via-vertices
		ars := routing.NewAlternativeRouteSearch(re)
		param := ars.ParameterByRequest(s, t)
		crpQuery.SetUpperBound(1.25)
		crpQuery.ForAlternatives(true)
		cost, optPath, found := crpQuery.ShortestPathSearch(s, t)
		if !found {
			continue
		}
		optCost := util.WeightToSeconds(cost)
		alts, dur := ars.ComputeAlternativeRoutes(s, t, 4, false, 0, optCost, optPath, crpQuery, param)
		crpQuery.ForAlternatives(false)
		crpQuery.Done()
		if (i+1)%100 == 0 {
			fmt.Printf("processed %d queries\n", i+1)
		}
		runtime += float64(dur)

		if len(alts) == 0 {
			continue
		}

		stretch += ars.GetStretch(alts, optCost)
		diversity += ars.GetDiversity(alts)
		locOpt += ars.GetLocalOptimality(alts, optCost)
		distSharing += ars.GetDistanceSharing(alts)
		successRate += 1.0
		foundAltCount++
	}

	successRate /= float64(len(queries))
	stretch /= float64(foundAltCount)
	distSharing /= float64(foundAltCount)
	locOpt /= float64(foundAltCount)
	diversity /= float64(foundAltCount)
	runtime /= float64(len(queries))

	fmt.Printf("success rate: %f %%\n", successRate*100)
	fmt.Printf("stretch: %f %% \n", stretch)
	fmt.Printf("diversity: %f\n", diversity)
	fmt.Printf("local optimality: %f %% \n", locOpt)
	fmt.Printf("distance sharing: %f %% \n", distSharing)

	fmt.Printf("avg runtime: %f ms\n", runtime)
}

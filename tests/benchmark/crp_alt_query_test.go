package benchmark

import (
	"flag"
	"fmt"
	"math/rand"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	stdLog "log"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	osmFile = "./data/yogyakarta.osm.pbf"
)

type query struct {
	s, t da.Index
}

func setup(b *testing.B, osmMap string) (*engine.Engine[int32], []query, *da.Graph, *zap.Logger) {
	b.Helper()
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
	mapName := strings.Split(strings.Split(osmMap, "/")[2], ".")[0]
	config.InitRegionName(fmt.Sprintf("benchmark_test_%s", mapName), pkg.TEST)

	root := config.ProfilesRoot()
	base := fmt.Sprintf("%s/%s/%s", root, pkg.ProfileName, pkg.RegionName)
	metricsFilePath := base + "_metrics.nmt"
	preprocessed := false
	_, err = os.Stat(metricsFilePath)
	if err == nil {
		preprocessed = true
	}

	logger.Sugar().Infof("preprocessed: %v", preprocessed)
	if !preprocessed {
		op := extractor.NewExtractor[int32](logger)
		_, ebg, rn, _, timeFunction, err := op.Extract(filepath.Join(workingDir, osmMap))
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

	re, err := engine.NewEngine[int32](logger)
	if err != nil {
		logger.Fatal(err.Error())
	}

	g := re.GetRoutingEngine().GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	n := 1000000
	qset := make(map[uint64]struct{})

	newQuery := func(s, t da.Index) query {
		return query{
			s: s,
			t: t,
		}
	}

	queries := make([]query, 0, n)

	bitpack := func(i, j da.Index) uint64 {
		return uint64(i) | (uint64(j) << 32)
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

	logger.Sugar().Infof("starting benchmark.....")

	return re, queries, g, logger
}

/*
cd tests/benchmark && go test -bench BenchmarkCRPALTQuery -benchmem -cpuprofile prof.cpu -memprofile prof.mem -benchtime=15s -o sp.test
go tool pprof -focus='ShortestPathSearch' sp.test prof.cpu

go test  ./tests/benchmark   -bench BenchmarkCRPALTQuery -benchmem -cpuprofile prof.cpu -memprofile prof.mem -benchtime=15s -o sp.test

perf stat -e cache-references,cache-misses  ./sp.test -test.run='^$' -test.bench=BenchmarkCRPALTQuery -test.benchtime=15s

// valgrind --tool=cachegrind --branch-sim=yes  ./sp.test  -test.run='^$' -test.bench=BenchmarkCRPALTQuery -test.benchtime=15s

[1] Delling, D. et al. (2013) ‘Customizable Route Planning in Road Networks’. Available at: https://www.microsoft.com/en-us/research/publication/customizable-route-planning-in-road-networks/.

cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkCRPALTQuery-12            99224            160123 ns/op                 0.1601 ms/op         6245 ops/sec          1914 B/op          8 allocs/op
PASS

optHeapify + Unidirectional ALT:
go test -bench BenchmarkCRPALTQuery -benchmem -cpuprofile prof.cpu -memprofile prof.mem -benchtime=15s -o sp.test

2026-11-03T16:34:51.933886599+07:00     info    preprocessed: true
2026-11-03T16:34:51.933946862+07:00     info    Starting query engine....
2026-11-03T16:34:51.933951891+07:00     info    Reading graph....
2026-11-03T16:34:51.959457584+07:00     info    Reading overlay graph....
2026-11-03T16:34:51.962037879+07:00     info    Reading metrics...
2026-11-03T16:34:52.138456639+07:00     info    starting benchmark.....
goos: linux
goarch: amd64
pkg: github.com/lintang-b-s/Navigatorx/tests/benchmark
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkCRPALTQuery-12           171201             92808 ns/op                 0.09281 ms/op       10775 ops/sec          1849 B/op          7 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       16.113s
*/
func BenchmarkCRPALTQuery(b *testing.B) {

	eng, queries, _, _ := setup(b, osmFile)
	re := eng.GetRoutingEngine()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	n := len(queries)

	start := time.Now()
	b.ResetTimer()

	for b.Loop() {
		i := rd.Intn(n)
		q := queries[i]
		s := q.s
		t := q.t
		crpQuery := routing.NewCRPUniALTQuery(re)
		crpQuery.ShortestPathSearch(s, t)
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")
}

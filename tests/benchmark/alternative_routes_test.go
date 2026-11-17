package benchmark

import (
	stdLog "log"
	"math/rand"
	"os"
	"runtime"
	"runtime/pprof"
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
go test ./tests/benchmark -bench BenchmarkAlternativeRoutes -benchmem  -benchtime=15s -o alt.test

[1] Delling, D. et al. (2013) ‘Customizable Route Planning in Road Networks’. Available at: https://www.microsoft.com/en-us/research/publication/customizable-route-planning-in-road-networks/.

2026-06-16T18:42:48.380061587+07:00     info    starting benchmark.....
BenchmarkAlternativeRoutes-12                      27051            663009 ns/op                 0.6630 ms/op         1508 ops/sec         47305 B/op        371 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       94.224s

CRP alternative routes query runtime match dengan hasil eksperimen ref [1] table 8, sekitar 3 ms

-17T15:28:34.829034405+07:00	info	starting benchmark.....
goos: linux
goarch: amd64
pkg: github.com/lintang-b-s/Navigatorx/tests/benchmark
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkAlternativeRoutes-12    	   16534	   1033144 ns/op	         1.033 ms/op	     967.9 ops/sec	 26384 B/op	    67 allocs/op
PASS
ok  	github.com/lintang-b-s/Navigatorx/tests/benchmark	39.632s

goarch: amd64
pkg: github.com/lintang-b-s/Navigatorx/tests/benchmark
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkAlternativeRoutes-12              30618            558370 ns/op                 0.5584 ms/op         1791 ops/sec         50778 B/op        178 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       26.070s
*/
func BenchmarkAlternativeRoutes(b *testing.B) {

	eng, queries, _, _ := setup(b, osmFile)
	re := eng.GetRoutingEngine()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	n := len(queries)

	// Create the profile output file before resetting timer
	f, err := os.Create("alt_cpu.prof")
	if err != nil {
		b.Fatal("could not create CPU profile: ", err)
	}
	defer f.Close() // nolint:gosec

	// Start CPU profiling right before the loop
	if err := pprof.StartCPUProfile(f); err != nil {
		b.Fatal("could not start CPU profile: ", err)
	}

	start := time.Now()
	for b.Loop() {
		i := rd.Intn(n)
		q := queries[i]
		s := q.s
		t := q.t

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
		alts := ars.ComputeAlternativeRoutes(s, t, 2, false, optCost, optPath, crpQuery, param)
		crpQuery.ForAlternatives(false)
		crpQuery.Done()
		for _, alt := range alts {
			altPath := alt.GetSegmentPath()
			da.PutIndexes(altPath)

		}
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	// Stop CPU profiling immediately after the loop finishes
	pprof.StopCPUProfile()
	fMem, err := os.Create("alt_mem.prof")
	if err != nil {
		stdLog.Fatal("could not create memory profile: ", err)
	}
	defer fMem.Close() // nolint:gosec

	runtime.GC() // get up-to-date statistics
	// Lookup("allocs") creates a profile similar to go test -memprofile.
	// Alternatively, use Lookup("heap") for a profile
	// that has inuse_space as the default index.
	if err := pprof.Lookup("allocs").WriteTo(fMem, 0); err != nil {
		stdLog.Fatal("could not write memory profile: ", err)
	}

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")
}

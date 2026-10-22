package benchmark

import (
	"math/rand"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

/*
cd tests/benchmark && go test -bench="^BenchmarkAlternativeRoutes$" -benchmem -cpuprofile prof_alt.cpu -memprofile prof_alt.mem -benchtime=15s -o=alt.test

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
*/
func BenchmarkAlternativeRoutes(b *testing.B) {
	// defer goleak.VerifyNone(b) // cuma cache ristretto yang leak

	eng, queries, _, _ := setup()
	start := time.Now()
	re := eng.GetRoutingEngine()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	n := len(queries)
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
		ars.ComputeAlternativeRoutes(s, t, 4, false, 0, optCost, optPath, crpQuery, param)
		crpQuery.ForAlternatives(false)
		crpQuery.Done()
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")
}

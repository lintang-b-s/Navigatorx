package benchmark

import (
	"math/rand"
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

/*
cd tests/benchmark && go test -bench BenchmarkAlternativeRoutes -benchmem -cpuprofile prof_alt.cpu -memprofile prof_alt.mem -benchtime=15s

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

	eng, queries, g, _ := setup()
	start := time.Now()
	re := eng.GetRoutingEngine()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	n := len(queries)
	for b.Loop() {
		i := rd.Intn(n)
		q := queries[i]

		s := q.s
		t := q.t

		as := g.GetExitOffset(s) + g.GetOutDegree(s) - 1
		at := g.GetEntryOffset(t) + g.GetInDegree(t) - 1

		sVertex := g.GetVertex(s)
		tVertex := g.GetVertex(t)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		crpQuery := routing.NewAlternativeRouteSearch(re)
		crpQuery.FindAlternativeRoutes(sPhantomNode, tPhantomNode, 3, false, 0)
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")

}

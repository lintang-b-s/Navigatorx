package benchmark

import (
	"math/rand"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
)

const (
	alpha      = 0.25 // every subpath P' of alternative route with l(P') <= T = \alpha* l(Opt) is optimal (shortest path). l(Opt) is the cost/travel time of the shortest path
	gamma      = 0.8  // alternative routes at least 20% different than the shortest path
	epsilon    = 0.25 // alternative routes at most 25% longer than the shortest path
	upperBound = 1.2
)

/*
cd tests/benchmark && go test -bench BenchmarkAlternativeRoutes -benchmem -cpuprofile prof_alt.cpu -memprofile prof_alt.mem -benchtime=15s

[1] Delling, D. et al. (2013) ‘Customizable Route Planning in Road Networks’. Available at: https://www.microsoft.com/en-us/research/publication/customizable-route-planning-in-road-networks/.

BenchmarkAlternativeRoutes-12               9158           1959775 ns/op                 1.960 ms/op           510.3 ops/sec      273223 B/op       1950 allocs/op

CRP alternative routes query runtime match dengan hasil eksperimen ref [1] table 8, sekitar 3 ms

todo: reduce memory space alloc / op lagi, now 270k B/op
ngaruh ke load test

todo2: optimize sampai p95 latency alternative routes ngalahin osrm
*/
func BenchmarkAlternativeRoutes(b *testing.B) {
	eng, queries, g, lm := setup()
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

		crpQuery := routing.NewAlternativeRouteSearch(re, lm)
		alts, _, _ := crpQuery.FindAlternativeRoutes(as, at, 3)
		for j := 0; j < len(alts); j++ {
			alt := alts[j]
			path := alt.GetCoords()
			edgePath := alt.GetPath()
			if len(edgePath) > 0 {
				re.DoneQuery(edgePath, path)
			}
		}
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")

}

package benchmark

import (
	"context"
	"math/rand"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

/*
go test -bench BenchmarkAlternativeRoutesService -benchmem -cpuprofile prof_alt_service.cpu -memprofile prof_alt_service.mem -benchtime=15s

pkg: github.com/lintang-b-s/Navigatorx/tests/benchmark
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkAlternativeRoutesService-12               19237            920091 ns/op                 0.9228 ms/op         1087 ops/sec         54794 B/op        487 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       32.122s

pkg: github.com/lintang-b-s/Navigatorx/tests/benchmark
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkAlternativeRoutesService-12               18016            915288 ns/op                 0.9179 ms/op         1093 ops/sec         79267 B/op        213 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       32.671s
*/
func BenchmarkAlternativeRoutesService(b *testing.B) {
	// defer goleak.VerifyNone(b) // cuma cache ristretto yang leak

	eng, queries, g, logger := setup()
	start := time.Now()
	re := eng.GetRoutingEngine()

	rtree := spatialindex.NewRtree()
	rtree.Build(re.GetGraph(), logger)
	altSearch := routing.NewAlternativeRouteSearch(re)

	rs, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true)
	if err != nil {
		b.Fatal(err)
	}

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	n := len(queries)
	for b.Loop() {
		i := rd.Intn(n)
		q := queries[i]

		s := q.s
		t := q.t

		sCoord := g.GetVertexCoordinate(s)
		tCoord := g.GetVertexCoordinate(t)

		_, _ = rs.AlternativeRouteSearch(context.Background(), sCoord.GetLat(), sCoord.GetLon(), tCoord.GetLat(), tCoord.GetLon(),
			3, false, 0, false)

	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")

}

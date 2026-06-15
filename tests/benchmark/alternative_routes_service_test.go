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
BenchmarkAlternativeRoutesService-12               13674           1275013 ns/op                 1.279 ms/op           784.3 ops/sec      121826 B/op        706 allocs/op
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

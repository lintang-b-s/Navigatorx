package benchmark

import (
	"context"
	"math/rand"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

/*
cd tests/benchmark && go test -bench BenchmarkShortestPathService -benchmem -cpuprofile prof_sp_service.cpu -memprofile prof_sp_service.mem -benchtime=15s -o sp_service.test

go test ./tests/benchmark  -bench BenchmarkShortestPathService -benchmem -cpuprofile prof_sp_service.cpu -memprofile prof_sp_service.mem -benchtime=15s -o sp_service.test

pkg: github.com/lintang-b-s/Navigatorx/tests/benchmark
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkShortestPathService-12            32131            530657 ns/op                 0.5328 ms/op         1884 ops/sec         16138 B/op        130 allocs/op
PASS

2026-06-17T16:07:11.600544051+07:00     info    R-tree spatial index built.
goos: linux
goarch: amd64
pkg: github.com/lintang-b-s/Navigatorx/tests/benchmark
cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkShortestPathService-12            29205            568201 ns/op                 0.5700 ms/op         1760 ops/sec          5955 B/op         33 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       32.103s

cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkShortestPathService-12            51124            337086 ns/op                 0.3371 ms/op         2967 ops/sec         41307 B/op         61 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       17.616s
*/
func BenchmarkShortestPathService(b *testing.B) {

	eng, queries, g, logger := setup(b, osmFile)
	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	rtreeb := spatialindex.NewRtree()
	if err := rtreeb.Build(re.GetGraph(), rn, logger); err != nil {
		b.Fatal(err)
	}

	rtree, err := spatialindex.ReadRtreeFromFile(logger)
	if err != nil {
		b.Fatal(err)
	}

	rs, err := usecases.NewRoutingService(logger, re, rn, rtree, 0.05, true)
	if err != nil {
		b.Fatal(err)
	}

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	n := len(queries)
	b.ResetTimer()
	start := time.Now()

	for b.Loop() {
		i := rd.Intn(n)
		q := queries[i]

		s := q.s
		t := q.t

		sCoord := g.GetVertexCoordinate(s)
		tCoord := g.GetVertexCoordinate(t)

		_, _, _, _, _, _ = rs.ShortestPath(context.Background(), sCoord.GetLat(), sCoord.GetLon(), tCoord.GetLat(), tCoord.GetLon(), false, 0, false, true)
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")
}

// // go test ./tests/benchmark  -bench BenchmarkShortestPathServiceParallel -benchmem -cpuprofile prof_sp_service_parallel.cpu -memprofile prof_sp_service_parallel.mem -benchtime=15s -o sp_service_parallel.test

// func BenchmarkShortestPathServiceParallel(b *testing.B) {

// 	eng, queries, g, logger := setup(b, osmFile)

// 	re := eng.GetRoutingEngine()
// 	rn := re.GetRoadNetworkContainer()
// 	rtreeb := spatialindex.NewRtree()
// 	if err := rtreeb.Build(re.GetGraph(), rn, logger); err != nil {
// 		b.Fatal(err)
// 	}

// 	rtree, err := spatialindex.ReadRtreeFromFile(logger)
// 	if err != nil {
// 		b.Fatal(err)
// 	}

// 	rs, err := usecases.NewRoutingService(logger, re, rn, rtree, 0.05, true)
// 	if err != nil {
// 		b.Fatal(err)
// 	}

// 	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
// 	n := len(queries)
// 	b.ResetTimer()

// 	b.RunParallel(func(p *testing.PB) {
// 		for p.Next() {
// 			i := rd.Intn(n)
// 			q := queries[i]

// 			s := q.s
// 			t := q.t

// 			sCoord := g.GetVertexCoordinate(s)
// 			tCoord := g.GetVertexCoordinate(t)

// 			_, _, _, _, _, _ = rs.ShortestPath(context.Background(), sCoord.GetLat(), sCoord.GetLon(), tCoord.GetLat(), tCoord.GetLon(), false, 0, false, false)
// 		}
// 	})
// }

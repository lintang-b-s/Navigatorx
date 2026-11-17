package benchmark

import (
	"context"
	"math/rand"
	"os"
	"runtime"
	"runtime/pprof"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

/*
go test ./tests/benchmark  -bench ^BenchmarkShortestPathService$ -benchmem -benchtime=15s

go tool pprof -alloc_space -focus="RoutingService.*ShortestPath" ./tests/benchmark/sp_service_mem.prof

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

	// Create the profile output file before resetting timer
	f, err := os.Create("sp_service_cpu.prof")
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
		sCoord := g.GetVertexCoordinate(s)
		tCoord := g.GetVertexCoordinate(t)
		_, _, _, drivingDirections, _, _ := rs.ShortestPath(context.Background(), sCoord.GetLat(), sCoord.GetLon(), tCoord.GetLat(), tCoord.GetLon(), false, 0, false, true)
		rs.Done(drivingDirections)
	}
	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	// Stop CPU profiling immediately after the loop finishes
	pprof.StopCPUProfile()
	fMem, err := os.Create("sp_service_mem.prof")
	if err != nil {
		b.Fatal("could not create memory profile: ", err) // Use t.Fatal if in a Test function
	}
	defer fMem.Close() // nolint:gosec

	runtime.GC() // get up-to-date statistics
	// Lookup("allocs") creates a profile similar to go test -memprofile.
	// Alternatively, use Lookup("heap") for a profile
	// that has inuse_space as the default index.
	if err := pprof.Lookup("allocs").WriteTo(fMem, 0); err != nil {
		b.Fatal("could not write memory profile: ", err)
	}

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")
}

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
go test ./tests/benchmark -bench BenchmarkAlternativeRoutesService -benchmem  -benchtime=15s -o alt_service.test

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

	eng, queries, g, logger := setup(b, osmFile)
	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	rtree := spatialindex.NewRtree()
	err := rtree.Build(re.GetGraph(), rn, logger)
	if err != nil {
		b.Fatal(err)
	}
	rs, err := usecases.NewRoutingService(logger, re, rn, rtree, 0.05, true)
	if err != nil {
		b.Fatal(err)
	}

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	n := len(queries)

	// Create the profile output file before resetting timer
	f, err := os.Create("alt_service_cpu.prof")
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

		_, _, _, altRoutesSteps, _ := rs.AlternativeRouteSearch(context.Background(), sCoord.GetLat(), sCoord.GetLon(), tCoord.GetLat(), tCoord.GetLon(),
			3, false, 0, false, true)
		for _, rsteps := range altRoutesSteps {
			rs.Done(rsteps)
		}

	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	// Stop CPU profiling immediately after the loop finishes
	pprof.StopCPUProfile()
	fMem, err := os.Create("alt_service_mem.prof")
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

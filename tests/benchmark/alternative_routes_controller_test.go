package benchmark

import (
	"fmt"
	"math/rand"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/julienschmidt/httprouter"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/tiler"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
)

/*
go test -bench BenchmarkAlternativeRoutesController -benchmem -cpuprofile prof_alt_controller.cpu -memprofile prof_alt_controller.mem -benchtime=15s

BenchmarkAlternativeRoutesController-12            34322            445420 ns/op                 0.4455 ms/op         2245 ops/sec         42467 B/op        285 allocs/op
PASS
ok      github.com/lintang-b-s/Navigatorx/tests/benchmark       29.818s

cpu: AMD Ryzen 5 7540U w/ Radeon(TM) 740M Graphics
BenchmarkAlternativeRoutesController-12    	   40689	    423370 ns/op	      0.4234 ms/op	    2362 ops/sec	 21972 B/op	    81 allocs/op
PASS
ok  	github.com/lintang-b-s/Navigatorx/tests/benchmark	34.139s
*/
func BenchmarkAlternativeRoutesController(b *testing.B) {
	eng, _, _, logger := setup()
	re := eng.GetRoutingEngine()

	g := re.GetGraph()
	bb := g.GetBoundingBox()
	rtree := spatialindex.NewRtree()
	rtree.Build(re.GetGraph(), logger)
	altSearch := routing.NewAlternativeRouteSearch(re)
	start := time.Now()

	rs, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true)
	if err != nil {
		b.Fatal(err)
	}

	cf := re.GetCostFunction()
	tilingEngine := tiler.NewTilingEngine(g, logger, cf)
	tilingService := usecases.NewTileService(logger, tilingEngine)

	api := controllers.New(rs, logger, tilingService)

	params := httprouter.Params{}
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))

	for b.Loop() {
		from := RandomCoordinate(bb, rd)
		to := RandomCoordinate(bb, rd)

		url := fmt.Sprintf("/routes/alternatives?"+
			"origin_lat=%v&origin_lon=%v"+
			"&destination_lat=%v&destination_lon=%v"+
			"&k=3", from.GetLat(), from.GetLon(), to.GetLat(), to.GetLon())
		req := httptest.NewRequest(http.MethodGet, url, nil)
		w := httptest.NewRecorder()

		api.AlternativeRoutes(w, req, params)
	}

	now := time.Since(start)
	msPerOp := float64(now.Milliseconds()) / float64(b.N)
	throughput := float64(b.N) / b.Elapsed().Seconds()

	b.ReportMetric(msPerOp, "ms/op")
	b.ReportMetric(throughput, "ops/sec")
}

func RandomCoordinate(bb *da.BoundingBox, rd *rand.Rand) da.Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return da.NewCoordinate(lat, lon)
}

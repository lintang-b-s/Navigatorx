package main

import (
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"math/rand/v2"

	"github.com/bytedance/gopkg/util/gopool"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	profileFilePath   = flag.String("profile", "./data/car.yaml", "profile file path")
	profileName       string
	regionName        = flag.String("region", "diy_solo_semarang", "region name")
	graphFile         string
	overlayGraphFile  string
	metricsFile       string
	landmarkFile      string
	timeFunctionFile  string
	transitionMHTFile string
)

func init() {
	flag.Parse()

	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	graphFile = fmt.Sprintf("./data/profiles/%s/%s_original.graph", profileName, *regionName)
	overlayGraphFile = fmt.Sprintf("./data/profiles/%s/%s_overlay_graph.graph", profileName, *regionName)
	landmarkFile = fmt.Sprintf("./data/profiles/%s/%s_landmark.lm", profileName, *regionName)
	metricsFile = fmt.Sprintf("./data/profiles/%s/%s_metrics.txt", profileName, *regionName)
	timeFunctionFile = fmt.Sprintf("./data/profiles/%s/%s_timefunction.txt", profileName, *regionName)
	transitionMHTFile = fmt.Sprintf("./data/profiles/%s/%s_transition_matrix.txt", profileName, *regionName)

	util.InitProfileConfig(profileName)
}

func main() {
	flag.Parse()
	logger, err := logger.New()
	if err != nil {
		panic(err)
	}

	routingEngine, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		panic(err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(routingEngine.GetRoutingEngine().GetGraph(), logger)
	graph, err := da.ReadGraph(graphFile)
	if err != nil {
		panic(err)
	}
	var N *da.SparseMatrix[int]
	_, err = os.Stat(transitionMHTFile)
	if err == nil {
		logger.Info("reading transition matrix from file...")

		N, err = da.ReadSparseMatrixFromFile[int](transitionMHTFile, int(0),
			func(a, b int) bool { return a == b })
		if err != nil {
			panic(err)
		}
	} else {
		N = da.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(),
			0, func(a, b int) bool { return a == b })
	}

	re := routingEngine.GetRoutingEngine()
	altSearch := routing.NewAlternativeRouteSearch(re)

	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.04, true)
	if err != nil {
		panic(err)
	}

	boundingBox := graph.GetBoundingBox()
	gopool.SetCap(40)
	var wg sync.WaitGroup
	var mut sync.Mutex
	var completedQueries atomic.Uint64

	for i := 0; i < 1e3; i++ {
		wg.Add(1)
		gopool.Go(func() {
			defer wg.Done()
			rd := rand.New(rand.NewPCG(uint64(time.Now().UnixNano()), uint64(time.Now().UnixNano())))

			src := RandomCoordinate(boundingBox, rd)
			dst := RandomCoordinate(boundingBox, rd)

			sp, tp := routingService.SnapOrigDestQueryToNearbyRoadSegments(src.GetLat(), src.GetLon(), dst.GetLat(), dst.GetLon())
			if sp.GetOutEdgeId() == da.INVALID_EDGE_ID && sp.GetInEdgeId() == da.INVALID_EDGE_ID {
				return
			}

			crpQuery := routing.NewCRPALTBidirectionalSearch(routingService.GetRoutingEngine(), 1.0)
			_, _, _, edgePath, found := crpQuery.ShortestPathSearch(sp, tp)
			if !found {
				return
			}

			mut.Lock()
			for j := 0; j < len(edgePath)-1; j++ {
				e := int(edgePath[j])
				eNext := int(edgePath[j+1])
				N.Set(N.Get(e, eNext)+1, e, eNext)
			}
			mut.Unlock()

			count := completedQueries.Add(1)
			if count%1e2 == 0 {
				fmt.Printf("completed query: %v\n", count)
				mut.Lock()
				err := N.WriteToFile(transitionMHTFile)
				mut.Unlock()
				if err != nil {
					panic(err)
				}
			}
		})
	}
	wg.Wait()

}

func RandomCoordinate(bb *da.BoundingBox, rd *rand.Rand) da.Coordinate {

	lat := bb.GetMinLat() + rd.Float64()*(bb.GetMaxLat()-bb.GetMinLat())
	lon := bb.GetMinLon() + rd.Float64()*(bb.GetMaxLon()-bb.GetMinLon())
	return da.NewCoordinate(lat, lon)
}

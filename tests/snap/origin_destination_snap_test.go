package snap

import (
	"flag"
	"fmt"
	"io"
	"math/rand"
	"net/http"
	"os"
	"strconv"
	"strings"
	"testing"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"go.uber.org/zap"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	mlpFile                 = "stress_test_yogyakarta"
	url                     = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile                = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original_query_test.graph"
	overlayGraphFile string = "./data/overlay_graph_query_test.graph"
	metricsFile      string = "./data/metrics_query_test.txt"
	landmarkFile     string = "./data/landmark_query_test.lm"
)

func setup(t *testing.T) (*engine.Engine, *landmark.Landmark, *zap.Logger) {
	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}
	workingDir, err := os.Getwd()
	err = util.ReadConfig(workingDir)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(osmfFile); os.IsNotExist(err) {
		output, err := os.Create(osmfFile)
		if err != nil {
			t.Fatal(err)
		}
		defer output.Close()

		t.Logf("downloading osm file......")
		response, err := http.Get(url)
		if err != nil {
			t.Fatal(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			t.Fatal(err)
		}
		t.Logf("download complete")
	}

	op := osmparser.NewOSMParserV2()

	graph, err := op.Parse(fmt.Sprintf("%s", osmfFile), logger, false)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := strconv.Atoi(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger, true, false, true,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s", "crp_inertial_flow_"+mlpFile+".mlp"))
	if err != nil {
		t.Fatal(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)

	m, err := custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(16, m, custom, logger)
	if err != nil {
		t.Fatal(err)
	}
	err = lm.WriteLandmark(landmarkFile, custom)
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, lm, logger
}

// todo: benerin logic snap origin destination bug, ada yang salah, ada nearby edges yang geometrynya terlalu jauh dari query
// kayake bug di graphStorage.edgeInfos (ada beberapa edges yang salah edge geometrynya) pas kita sortByCellNumber(), todo: debug & benerin ini
// cd tests/snap && go test -run TestOriginDestinationSnap  -v -timeout=0  -count=1
func TestOriginDestinationSnap(t *testing.T) {
	eng, lm, logger := setup(t)
	re := eng.GetRoutingEngine()
	g := re.GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	n := 10000
	qset := make(map[da.Index]struct{})

	type origDestPair struct {
		orig, dest da.Coordinate
	}

	newOrigDestPair := func(orig, dest da.Coordinate) origDestPair {
		return origDestPair{
			orig: orig,
			dest: dest,
		}
	}

	queries := make([]origDestPair, 0, n)

	i := 0
	for i < n {
		s := da.Index(rd.Intn(V))

		if _, ok := qset[s]; ok {
			continue
		}

		qset[s] = struct{}{}

		sCoord := g.GetVertex(s).GetCoordinate()

		rndDist := 0.002 + rd.Float64()*(0.02-0.002)
		rdBearing := rd.Float64() * 360.0

		sCoordNLat, sCoordNLon := geo.GetDestinationPoint(sCoord.GetLat(), sCoord.GetLon(), rdBearing, rndDist)

		target := da.Index(rd.Intn(V))

		if _, ok := qset[target]; ok {
			continue
		}

		qset[target] = struct{}{}

		tCoord := g.GetVertex(target).GetCoordinate()

		rndDist = 0.002 + rd.Float64()*(0.02-0.002)
		rdBearing = rd.Float64() * 360.0

		tCoordNLat, tCoordNLon := geo.GetDestinationPoint(tCoord.GetLat(), tCoord.GetLon(), rdBearing, rndDist)

		queries = append(queries, newOrigDestPair(da.NewCoordinate(sCoordNLat, sCoordNLon), da.NewCoordinate(tCoordNLat, tCoordNLon)))
		i++
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(re.GetGraph(), 0.06, logger)

	altSearch := routing.NewAlternativeRouteSearch(re, lm)

	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.08, true, true,
		lm)
	if err != nil {
		panic(err)
	}

	for _, q := range queries {
		_, _, snappedOrig, snappedDst := routingService.SnapOrigDestQueryToNearbyRoadSegments(q.orig.GetLat(), q.orig.GetLon(),
			q.dest.GetLat(), q.dest.GetLon())

		distToOrig := geo.CalculateHaversineDistance(q.orig.GetLat(), q.orig.GetLon(), snappedOrig.GetLat(), snappedOrig.GetLon())
		distToDest := geo.CalculateHaversineDistance(q.dest.GetLat(), q.dest.GetLon(), snappedDst.GetLat(), snappedDst.GetLon())

		if util.Gt(distToOrig, 0.5) || util.Gt(distToDest, 0.5) {
			t.Errorf("snapped origin or destination too far from origin and destination query: %v, %v", distToOrig, distToDest)
		}
	}
}

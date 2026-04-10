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
	workingDir, err := util.FindProjectWorkingDir()
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

	graph, edgeInfoIds, err := op.Parse(fmt.Sprintf("%s", osmfFile), logger)
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
		graph, logger, false, false,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s.mlp", mlpFile))
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
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
	err = lm.PreprocessALT(16, m, graph, logger)
	if err != nil {
		t.Fatal(err)
	}
	err = lm.WriteLandmark(landmarkFile, graph)
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, lm, logger
}

// cd tests/snap && go test -run TestOriginDestinationSnap  -v -timeout=0  -count=1
func TestOriginDestinationSnap(t *testing.T) {
	eng, _, logger := setup(t)
	re := eng.GetRoutingEngine()
	g := re.GetGraph()

	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	V := g.NumberOfVertices()

	n := 10000
	qset := make(map[uint64]struct{})

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
		target := da.Index(rd.Intn(V))

		if !g.PathExists(s, target) {
			continue
		}

		key := util.Bitpack(uint32(s), uint32(target))
		if _, ok := qset[key]; ok {
			continue
		}
		qset[key] = struct{}{}

		sCoord := g.GetVertexCoordinate(s)

		rndDist := 0.001 + rd.Float64()*(0.005-0.001)
		rdBearing := rd.Float64() * 360.0

		sCoordNLat, sCoordNLon := geo.GetDestinationPoint(sCoord.GetLat(), sCoord.GetLon(), rdBearing, rndDist)
		tCoord := g.GetVertexCoordinate(target)

		rndDist = 0.001 + rd.Float64()*(0.005-0.001)
		rdBearing = rd.Float64() * 360.0

		tCoordNLat, tCoordNLon := geo.GetDestinationPoint(tCoord.GetLat(), tCoord.GetLon(), rdBearing, rndDist)

		queries = append(queries, newOrigDestPair(da.NewCoordinate(sCoordNLat, sCoordNLon), da.NewCoordinate(tCoordNLat, tCoordNLon)))
		i++
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(re.GetGraph(), logger)

	altSearch := routing.NewAlternativeRouteSearch(re)

	routingService, err := usecases.NewRoutingService(logger, re, rtree, altSearch, 0.05, true, true)
	if err != nil {
		panic(err)
	}

	testCases := []struct {
		name                  string
		queryOriginCoord      da.Coordinate
		queryDestinationCoord da.Coordinate

		wantOrigin, wantDestination string
	}{
		{
			name:                  "Kebab Morgan Jl. Pandega Marta https://www.openstreetmap.org/way/132780420  -> Jalan Malioboro (openstreetmap.org/way/357658484)",
			queryOriginCoord:      da.NewCoordinate(-7.755813, 110.376565),
			queryDestinationCoord: da.NewCoordinate(-7.795240, 110.365404),
			wantOrigin:            "Jalan Pandega Marta",
			wantDestination:       "Jalan Malioboro",
		},
		{
			name:                  "Kebab Morgan Jl. Pandega Marta https://www.openstreetmap.org/way/132780420  -> jalan Sains FMIPA UGM https://www.openstreetmap.org/way/194146659",
			queryOriginCoord:      da.NewCoordinate(-7.755813, 110.376565),
			queryDestinationCoord: da.NewCoordinate(-7.767826, 110.376570),
			wantOrigin:            "Jalan Pandega Marta",
			wantDestination:       "Jalan Sains",
		},

		{
			name:                  "Kebab Morgan Jl. Pandega Marta https://www.openstreetmap.org/way/132780420  -> jalan Lempuyangan https://www.openstreetmap.org/way/301793294",
			queryOriginCoord:      da.NewCoordinate(-7.755813, 110.376565),
			queryDestinationCoord: da.NewCoordinate(-7.790425, 110.375894),
			wantOrigin:            "Jalan Pandega Marta",
			wantDestination:       "Jalan Lempuyangan",
		},

		{
			name:                  "Jalan Malioboro (openstreetmap.org/way/357658484)  -> Jalan Affandi https://www.openstreetmap.org/way/701751480",
			queryOriginCoord:      da.NewCoordinate(-7.795240, 110.365404),
			queryDestinationCoord: da.NewCoordinate(-7.759672, 110.395024),
			wantOrigin:            "Jalan Malioboro",
			wantDestination:       "Jalan Affandi",
		},
		{
			name:                  "Gang Suroyodo (https://www.openstreetmap.org/way/133584571)  -> Jalan Bandara Adisucipto https://www.openstreetmap.org/way/357836542",
			queryOriginCoord:      da.NewCoordinate(-7.764687, 110.381876),
			queryDestinationCoord: da.NewCoordinate(-7.784079, 110.437843),
			wantOrigin:            "Gang Suroyodo",
			wantDestination:       "Jalan Bandara Adisucipto",
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			sp, tp := routingService.SnapOrigDestQueryToNearbyRoadSegments(tc.queryOriginCoord.GetLat(), tc.queryOriginCoord.GetLon(),
				tc.queryDestinationCoord.GetLat(), tc.queryDestinationCoord.GetLon())

			sourceRoadSegmentName := g.GetStreetName(sp.GetOutEdgeId())

			destinationExitId := g.GetExitIdOfInEdge(tp.GetInEdgeId())
			destinationRoadSegmentName := g.GetStreetName(destinationExitId)
			if sourceRoadSegmentName != tc.wantOrigin {
				t.Errorf("want origin road segment: %v, got: %v", tc.wantOrigin, sourceRoadSegmentName)
			}

			if destinationRoadSegmentName != tc.wantDestination {
				t.Errorf("want destination road segment: %v, got: %v", tc.wantDestination, destinationRoadSegmentName)
			}
		})
	}

	t.Run("random input origin destination snap test", func(t *testing.T) {
		for _, q := range queries {
			sp, tp := routingService.SnapOrigDestQueryToNearbyRoadSegments(q.orig.GetLat(), q.orig.GetLon(),
				q.dest.GetLat(), q.dest.GetLon())

			snappedOrig := sp.GetSnappedCoord()
			snappedDst := tp.GetSnappedCoord()

			distToOrig := geo.CalculateGreatCircleDistance(q.orig.GetLat(), q.orig.GetLon(), snappedOrig.GetLat(), snappedOrig.GetLon())
			distToDest := geo.CalculateGreatCircleDistance(q.dest.GetLat(), q.dest.GetLon(), snappedDst.GetLat(), snappedDst.GetLon())

			if util.Gt(distToOrig, 0.06) || util.Gt(distToDest, 0.06) { // karena search radius 50 m,
				t.Errorf("snapped origin or destination too far from origin and destination query: %v, %v", distToOrig, distToDest)
			}
		}
	})
}

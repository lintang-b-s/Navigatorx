package snap

import (
	"flag"
	"math/rand"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/http/usecases"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"go.uber.org/zap"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	mlpFile                 = "./data/stress_test_yogyakarta.mlp"
	osmFile                 = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original_query_test.ngraph"
	overlayGraphFile string = "./data/overlay_graph_query_test.ngraph"
	metricsFile      string = "./data/metrics_query_test.nmt"
	timeFunctionFile string = "./data/timefunction_od_test.ntf"
	rnFile           string = "./data/od_rn_test.ndata"
)

var (
	landmarkFile string = config.ProfilesRoot() + "/landmark_query_test.nlm"
)

func setup(t *testing.T) (*engine.Engine[int32], *zap.Logger) {
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	outputDir := filepath.Join(workingDir, "data")
	if err := util.EnsureDirExists(outputDir); err != nil {
		panic(err)
	}
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	err = config.ReadConfig(workingDir)
	if err != nil {
		t.Fatal(err)
	}
	config.InitRegionName("snap_test", pkg.TEST)

	op := extractor.NewExtractor[int32]()
	nbg, ebg, rn, ebgMapping, timeFunction, err := op.Extract(filepath.Join(workingDir, osmFile), logger)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split(*partitionSizes, ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := util.ParseTextInt(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		nbg, logger,
	)

	mp.RunMultilevelPartitioning()
	mp.MapToEdgeBasedGraph(ebg, ebgMapping)
	err = mp.SaveToFile()
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile()
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(ebg, rn, timeFunction, mlp, logger)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer[int32](logger)

	_, err = custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine[int32](logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, logger
}

// go test ./tests/snap -run  TestOriginDestinationSnap  -v -timeout=0  -count=1
func TestOriginDestinationSnap(t *testing.T) {
	eng, logger := setup(t)
	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()

	rtree := spatialindex.NewRtree()
	rtree.Build(re.GetGraph(), rn, logger)

	altSearch := routing.NewAlternativeRouteSearch(re)

	routingService, err := usecases.NewRoutingService(logger, re, rn, rtree, altSearch, 0.04, true)
	if err != nil {
		panic(err)
	}
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
		sgeom := rn.GetSegmentGeometry(s)
		_, sp, _, _ := geo.ProjectPointOnSegmentGeometry(sgeom, sCoord.GetLat(), sCoord.GetLon())
		rndDist := 0.001 + rd.Float64()*(0.003-0.001)
		rdBearing := rd.Float64() * 360.0
		sCoordNLat, sCoordNLon := geo.GetDestinationPoint(sp.GetLat(), sp.GetLon(), rdBearing, rndDist)

		tCoord := g.GetVertexCoordinate(target)
		tgeom := rn.GetSegmentGeometry(target)
		_, tp, _, _ := geo.ProjectPointOnSegmentGeometry(tgeom, tCoord.GetLat(), tCoord.GetLon())
		rndDist = 0.001 + rd.Float64()*(0.003-0.001)
		rdBearing = rd.Float64() * 360.0
		tCoordNLat, tCoordNLon := geo.GetDestinationPoint(tp.GetLat(), tp.GetLon(), rdBearing, rndDist)

		queries = append(queries, newOrigDestPair(da.NewCoordinate(sCoordNLat, sCoordNLon), da.NewCoordinate(tCoordNLat, tCoordNLon)))
		i++
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
			queryDestinationCoord: da.NewCoordinate(-7.767855, 110.376506),
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
				tc.queryDestinationCoord.GetLat(), tc.queryDestinationCoord.GetLon(), false, da.INVALID_SEGMENT_ID)

			sourceRoadSegmentName := rn.GetStreetName(sp.GetVId())
			destinationRoadSegmentName := rn.GetStreetName(tp.GetVId())
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
				q.dest.GetLat(), q.dest.GetLon(), false, da.INVALID_SEGMENT_ID)

			snappedOrig := sp.GetSnappedCoord()
			snappedDst := tp.GetSnappedCoord()

			distToOrig := geo.CalculateGreatCircleDistance(q.orig.GetLat(), q.orig.GetLon(), snappedOrig.GetLat(), snappedOrig.GetLon())
			distToDest := geo.CalculateGreatCircleDistance(q.dest.GetLat(), q.dest.GetLon(), snappedDst.GetLat(), snappedDst.GetLon())

			if util.Gt(distToOrig, 0.05) || util.Gt(distToDest, 0.05) { // karena search radius 50 m,
				t.Errorf("snapped origin or destination too far from origin and destination query: %v, %v", distToOrig, distToDest)
			}
		}
	})

}

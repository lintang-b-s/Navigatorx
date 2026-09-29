package extractor

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
)

const (
	osmFile = "./data/yogyakarta.osm.pbf"
)

func init() {
	config.InitConfig()
	config.InitRegionName("extractor_test", pkg.TEST)
}

func setup(t *testing.T, osmFileTest string) (*da.Graph, *da.RoadNetworkDataContainer, *extractor.Extractor[int32]) {
	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	osmExtractor := extractor.NewExtractor[int32]()

	graph, rn, _, err := osmExtractor.Extract(filepath.Join(pkg.WorkingDir, osmFileTest), logger)
	if err != nil {
		t.Fatal(err)
	}

	return graph, rn, osmExtractor
}

// go test ./tests/extractor -run .
func TestOSMParser(t *testing.T) {

	testCases := []struct {
		name string

		osmFileTest    string
		roundAboutWay  map[uint64]struct{}
		streetNameWay  map[uint64]string
		highwayTypeWay map[uint64]pkg.OsmHighwayType
		roadLanes      map[uint64]uint8
	}{
		{
			name:        "file osm yogyakarta",
			osmFileTest: osmFile,
			roundAboutWay: map[uint64]struct{}{
				1460805468: {},
				1460805470: {},
				1427239361: {},
			},
			streetNameWay: map[uint64]string{
				24277036:  "Jalan Urip Sumoharjo",
				293600459: "Jl. Jenderal Sudirman",
			},
			highwayTypeWay: map[uint64]pkg.OsmHighwayType{
				24277036:  pkg.PRIMARY,
				293600459: pkg.PRIMARY,
			},
			roadLanes: map[uint64]uint8{
				24277036:  3,
				293600459: 4,
			},
		},
	}

	for _, tc := range testCases {
		graph, rn, _ := setup(t, tc.osmFileTest)
		n := graph.NumberOfVertices()

		graph.ForVertices(func(v da.Vertex, vId da.Index) {
			if vId == da.Index(n) {
				return
			}
			if vId >= da.Index(n) {
				t.Errorf("expected vertex id lesser than or equal to: %v, got: %v", n, vId)
			}

			// cek firstOut && firstIn
			graph.ForOutEdgesOf(vId, func(eId, head, entryPoint da.Index) {
				tail := graph.GetTailOfOutedge(eId)
				if tail != vId {
					t.Errorf("expected tail of outedge (%v, %v): %v, got: %v", vId, head, vId, tail)
				}
			})

			graph.ForInEdgesOf(vId, func(eId, tail da.Index, exitPoint da.Index) {
				head := graph.GetHeadOfInedge(eId)
				if head != vId {
					t.Errorf("expected head of inedge (%v, %v): %v, got: %v", tail, vId, vId, head)
				}
			})

			// cek roundabout
			if _, roundabout := tc.roundAboutWay[rn.GetOsmWayId(vId)]; roundabout && !rn.IsRoundabout(vId) {
				t.Errorf("expected edge with osm way id %v is a roundabout, got no", rn.GetOsmWayId(vId))
			}

			// cek edge geometry
			if len(rn.GetSegmentGeometry(vId)) < 2 {
				t.Errorf("expected number of edge geometry coordinates is greater than or equal to 2, got: %v", len(rn.GetSegmentGeometry(vId)))
			}

			// cek street name dari edge

			eOsmwayId := rn.GetOsmWayId(vId)

			gotStreetName := rn.GetStreetName(vId)
			if expectedStreetname, ok := tc.streetNameWay[eOsmwayId]; ok && expectedStreetname != gotStreetName {
				t.Errorf("expected edge with osm way id %v street name: %v, got: %v", eOsmwayId, expectedStreetname, gotStreetName)
			}

			gotRoadClass := rn.GetRoadClass(vId)
			if expectedHighwayType, ok := tc.highwayTypeWay[eOsmwayId]; ok && expectedHighwayType != gotRoadClass {
				t.Errorf("expected edge with osm way id %v highway type: %v, got: %v", eOsmwayId, expectedHighwayType, gotRoadClass)
			}

			gotRoadLanes := rn.GetRoadLanes(vId)
			if expectedRoadLane, ok := tc.roadLanes[eOsmwayId]; ok && expectedRoadLane != gotRoadLanes {
				t.Errorf("expected edge with osm way id %v road lanes: %v, got: %v", eOsmwayId, expectedRoadLane, gotRoadLanes)
			}

		})
	}
}

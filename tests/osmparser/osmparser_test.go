package osmparser

import (
	"flag"
	"fmt"
	"io"
	"net/http"
	"os"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

var (
	partitionSizes = flag.String("us", "8,10,11,12,14", "Multilevel Partition Sizes")
)

const (
	mlpFile                 = "stress_test_yogyakarta"
	url                     = "https://docs.google.com/uc?export=download&id=1gxrkLPTfuyDl_3KzlcV4MpGXxCKkgDlx"
	osmfFile                = "./data/yogyakarta.osm.pbf"
	graphFile        string = "./data/original.graph"
	overlayGraphFile string = "./data/overlay_graph.graph"
	metricsFile      string = "./data/metrics.txt"
	landmarkFile     string = "./data/landmark.lm"
)

type query struct {
	s, t da.Index
}

func setup(t *testing.T, osmfFileTest, urlTest string) (*da.Graph, *osmparser.OsmParser) {
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
	if _, err := os.Stat(osmfFileTest); os.IsNotExist(err) {
		output, err := os.Create(osmfFileTest)
		if err != nil {
			t.Fatal(err)
		}
		defer output.Close()

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(urlTest)
		if err != nil {
			t.Fatal(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			t.Fatal(err)
		}
		logger.Sugar().Infof("download complete")
	}

	osmParser := osmparser.NewOSMParserV2()

	graph, err := osmParser.Parse(fmt.Sprintf("%s", osmfFileTest), logger, false)
	if err != nil {
		t.Fatal(err)
	}
	return graph, osmParser
}

func TestOSMParser(t *testing.T) {

	testCases := []struct {
		name string

		urlTest, osmfFileTest string
		roundAboutWay         map[int64]struct{}
		streetNameWay         map[int64]string
		highwayTypeWay        map[int64]string
		roadLanes             map[int64]uint8
	}{
		{
			name:         "file osm yogyakarta",
			urlTest:      url,
			osmfFileTest: osmfFile,
			roundAboutWay: map[int64]struct{}{
				1460805468: struct{}{},
				1460805470: struct{}{},
				1427239361: struct{}{},
			},
			streetNameWay: map[int64]string{
				24277036:  "Jalan Urip Sumoharjo",
				293600459: "Jl. Jenderal Sudirman",
			},
			highwayTypeWay: map[int64]string{
				24277036:  "primary",
				293600459: "primary",
			},
			roadLanes: map[int64]uint8{
				24277036:  3,
				293600459: 4,
			},
		},
	}

	for _, tc := range testCases {
		graph, op := setup(t, tc.osmfFileTest, tc.urlTest)
		bb := graph.GetBoundingBox()
		n := graph.NumberOfVertices()
		osmNodeIdMap := op.GetNodeIdMap()
		matrixOffset := da.Index(0)
		graph.ForVertices(func(v *da.Vertex) {
			if v.GetID() == da.Index(n) {
				return
			}
			if v.GetID() >= da.Index(n) {
				t.Errorf("expected vertex id lesser than or equal to: %v, got: %v", n, v.GetID())
			}

			if util.Lt(v.GetLat(), bb.GetMinLat()) {
				t.Errorf("expected vertex latitude greater than or equal to: %v, got: %v", bb.GetMinLat(), v.GetLat())
			}

			if util.Lt(v.GetLon(), bb.GetMinLon()) {
				t.Errorf("expected vertex longitude greater than or equal to: %v, got: %v", bb.GetMinLon(), v.GetLon())
			}

			if util.Gt(v.GetLat(), bb.GetMaxLat()) {
				t.Errorf("expected vertex latitude lesser than or equal to: %v, got: %v", bb.GetMaxLat(), v.GetLat())
			}

			if util.Gt(v.GetLon(), bb.GetMaxLon()) {
				t.Errorf("expected vertex longitude lesser than or equal to: %v, got: %v", bb.GetMaxLon(), v.GetLon())
			}

			// cek v.osmId

			if osmNodeIdMap[int64(v.GetOsmID())] != v.GetID() {
				t.Errorf("expected osm id %v mapped to vertex id: %v, got: %v", int64(v.GetOsmID()), v.GetID(), osmNodeIdMap[int64(v.GetOsmID())])
			}

			// cek turn turnTablePtr

			//  matrixOffset <= turnTablePtr < matrixOffset + v.outDegree * v.inDegree
			deg := graph.GetOutDegree(v.GetID()) * graph.GetInDegree(v.GetID())
			if v.GetTurnTablePtr() >= matrixOffset+deg {
				t.Errorf("expected turnTablePtr less than: %v, got: %v", matrixOffset, v.GetTurnTablePtr())
			}

			matrixOffset += deg

			// cek firstOut && firstIn
			graph.ForOutEdgesOfWithId(v.GetID(), func(e *da.OutEdge, id da.Index) {
				if da.SkipDummyEdge(e) {
					return
				}
				
				_, inE := graph.GetTailOfOutedgeWithInEdge(id)
				if inE.GetTail() != v.GetID() {
					t.Errorf("expected tail of outedge (%v, %v): %v, got: %v", v.GetID(), e.GetHead(), v.GetID(), inE.GetTail())
				}

				if e.GetEdgeId() != id {
					t.Errorf("expected edge id: %v, got: %v", id, e.GetEdgeId())
				}

				// cek roundabout
				if _, roundabout := tc.roundAboutWay[graph.GetOsmWayId(e.GetEdgeInfoId())]; roundabout && !graph.IsRoundabout(e.GetEdgeInfoId()) {
					t.Errorf("expected edge with osm way id %v is a roundabout, got no", graph.GetOsmWayId(e.GetEdgeInfoId()))
				}

				// cek edge geometry
				if len(graph.GetEdgeGeometry(e.GetEdgeInfoId())) < 2 {
					t.Errorf("expected number of edge geometry coordinates is greater than or equal to 2, got: %v", len(graph.GetEdgeGeometry(e.GetEdgeInfoId())))
				}

				// cek street name dari edge

				eOsmwayId := graph.GetOsmWayId(e.GetEdgeInfoId())

				gotStreetName := graph.GetStreetName(e.GetEdgeInfoId())
				if expectedStreetname, ok := tc.streetNameWay[eOsmwayId]; ok && expectedStreetname != gotStreetName {
					t.Errorf("expected edge with osm way id %v street name: %v, got: %v", eOsmwayId, expectedStreetname, gotStreetName)
				}

				gotRoadClass := graph.GetRoadClass(e.GetEdgeInfoId())
				if expectedHighwayType, ok := tc.highwayTypeWay[eOsmwayId]; ok && expectedHighwayType != gotRoadClass {
					t.Errorf("expected edge with osm way id %v highway type: %v, got: %v", eOsmwayId, expectedHighwayType, gotRoadClass)
				}

				gotRoadLanes := graph.GetRoadLanes(e.GetEdgeInfoId())
				if expectedRoadLane, ok := tc.roadLanes[eOsmwayId]; ok && expectedRoadLane != gotRoadLanes {
					t.Errorf("expected edge with osm way id %v road lanes: %v, got: %v", eOsmwayId, expectedRoadLane, gotRoadLanes)
				}
			})

			graph.ForInEdgesOfWithId(v.GetID(), func(e *da.InEdge, id da.Index) {
				_, outE := graph.GetHeadOfInedgeWithOutEdge(id)
				if outE.GetHead() != v.GetID() {
					t.Errorf("expected head of inedge (%v, %v): %v, got: %v", e.GetTail(), v.GetID(), v.GetID(), outE.GetHead())
				}

				if e.GetEdgeId() != id {
					t.Errorf("expected edge id: %v, got: %v", id, e.GetEdgeId())
				}
			})

		})

		if graph.GetTurntablesLength() != int(matrixOffset) {
			t.Errorf("expected turn table length: %v, got: %v", graph.GetTurntablesLength(), matrixOffset)
		}
	}

}

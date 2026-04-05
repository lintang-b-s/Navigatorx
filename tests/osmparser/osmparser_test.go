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

func setup(t *testing.T, osmfFileTest, urlTest string) (*da.Graph, [][]da.Index, *osmparser.OsmParser) {
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

	graph, edgeInfoIds, err := osmParser.Parse(fmt.Sprintf("%s", osmfFileTest), logger, false)
	if err != nil {
		t.Fatal(err)
	}

	return graph, edgeInfoIds, osmParser
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
		graph, edgeInfoIds, op := setup(t, tc.osmfFileTest, tc.urlTest)
		bb := graph.GetBoundingBox()
		n := graph.NumberOfVertices()
		osmNodeIdMap := op.GetNodeIdMap()
		matrixOffset := da.Index(0)
		graph.ForVertices(func(v da.Vertex, id da.Index) {
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
			gotOsmId := int64(graph.GetVertexOsmId(v.GetID()))

			if osmNodeIdMap[gotOsmId] != v.GetID() {
				t.Errorf("expected osm id %v mapped to vertex id: %v, got: %v", gotOsmId, v.GetID(), osmNodeIdMap[gotOsmId])
			}

			// cek turn turnTablePtr

			//  matrixOffset <= turnTablePtr < matrixOffset + v.outDegree * v.inDegree
			deg := graph.GetOutDegree(v.GetID()) * graph.GetInDegree(v.GetID())
			if v.GetTurnTablePtr() > matrixOffset+deg {
				t.Errorf("expected turnTablePtr less than: %v, got: %v", matrixOffset, v.GetTurnTablePtr())
			}

			matrixOffset += deg

			// cek firstOut && firstIn
			graph.ForOutEdgeIdsOf(v.GetID(), func(eId da.Index) {

				tail, _ := graph.GetTailOfOutedgeWithInEdge(eId)
				eHead := graph.GetHeadOfOutEdge(id)
				if tail != v.GetID() {
					t.Errorf("expected tail of outedge (%v, %v): %v, got: %v", v.GetID(), eHead, v.GetID(), tail)
				}

				vExitPoint := graph.GetExitOrder(v.GetID(), eId)
				edgeInfoId := edgeInfoIds[v.GetID()][vExitPoint]

				// cek roundabout
				if _, roundabout := tc.roundAboutWay[graph.GetOsmWayId(edgeInfoId)]; roundabout && !graph.IsRoundabout(edgeInfoId) {
					t.Errorf("expected edge with osm way id %v is a roundabout, got no", graph.GetOsmWayId(edgeInfoId))
				}

				// cek edge geometry
				if len(graph.GetEdgeGeometry(edgeInfoId)) < 2 {
					t.Errorf("expected number of edge geometry coordinates is greater than or equal to 2, got: %v", len(graph.GetEdgeGeometry(edgeInfoId)))
				}

				// cek street name dari edge

				eOsmwayId := graph.GetOsmWayId(edgeInfoId)

				gotStreetName := graph.GetStreetName(edgeInfoId)
				if expectedStreetname, ok := tc.streetNameWay[eOsmwayId]; ok && expectedStreetname != gotStreetName {
					t.Errorf("expected edge with osm way id %v street name: %v, got: %v", eOsmwayId, expectedStreetname, gotStreetName)
				}

				gotRoadClass := graph.GetRoadClass(edgeInfoId)
				if expectedHighwayType, ok := tc.highwayTypeWay[eOsmwayId]; ok && expectedHighwayType != gotRoadClass {
					t.Errorf("expected edge with osm way id %v highway type: %v, got: %v", eOsmwayId, expectedHighwayType, gotRoadClass)
				}

				gotRoadLanes := graph.GetRoadLanes(edgeInfoId)
				if expectedRoadLane, ok := tc.roadLanes[eOsmwayId]; ok && expectedRoadLane != gotRoadLanes {
					t.Errorf("expected edge with osm way id %v road lanes: %v, got: %v", eOsmwayId, expectedRoadLane, gotRoadLanes)
				}
			})

			graph.ForInEdgeIdsOf(v.GetID(), func(id da.Index) {
				outEHead, _ := graph.GetHeadOfInedgeWithOutEdge(id)
				outETail := graph.GetTailOfInedge(id)
				if outEHead != v.GetID() {
					t.Errorf("expected head of inedge (%v, %v): %v, got: %v", outETail, v.GetID(), v.GetID(), outEHead)
				}

			})

		})

		if graph.GetTurntablesLength() != int(matrixOffset) {
			t.Errorf("expected turn table length: %v, got: %v", graph.GetTurntablesLength(), matrixOffset)
		}
	}

}

package osmparser

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/spf13/viper"
)

const (
	osmFile = "./data/yogyakarta.osm.pbf"
)

func init() {
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		panic(err)
	}
	err = config.ReadConfig(workingDir)
	if err != nil {
		panic(err)
	}
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicleEnabled = pkg.GetIsVehicle()
	pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()
}

func setup(t *testing.T, osmFileTest string) (*da.Graph, [][]da.Index, *osmparser.OsmParser) {
	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	osmParser := osmparser.NewOSMParserV2()

	graph, _, edgeInfoIds, err := osmParser.Parse(filepath.Join(pkg.WorkingDir, osmFileTest), logger)
	if err != nil {
		t.Fatal(err)
	}

	return graph, edgeInfoIds, osmParser
}

// go test ./tests/osmparser -run .
func TestOSMParser(t *testing.T) {

	testCases := []struct {
		name string

		osmFileTest    string
		roundAboutWay  map[int64]struct{}
		streetNameWay  map[int64]string
		highwayTypeWay map[int64]string
		roadLanes      map[int64]uint8
	}{
		{
			name:        "file osm yogyakarta",
			osmFileTest: osmFile,
			roundAboutWay: map[int64]struct{}{
				1460805468: {},
				1460805470: {},
				1427239361: {},
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
		graph, edgeInfoIds, op := setup(t, tc.osmFileTest)
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

		if graph.GetTurnTypeTableLength() != int(matrixOffset) {
			t.Errorf("expected turn table length: %v, got: %v", graph.GetTurnTypeTableLength(), matrixOffset)
		}
	}

}

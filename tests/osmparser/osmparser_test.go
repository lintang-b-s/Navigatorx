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

func setup(t *testing.T) (*da.Graph, *osmparser.OsmParser) {
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

		logger.Sugar().Infof("downloading osm file......")
		response, err := http.Get(url)
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

	graph, err := osmParser.Parse(fmt.Sprintf("%s", osmfFile), logger, false)
	if err != nil {
		t.Fatal(err)
	}
	return graph, osmParser
}

func TestOSMParser(t *testing.T) {

	graph, op := setup(t)
	bb := graph.GetBoundingBox()
	n := graph.NumberOfVertices()
	osmNodeIdMap := op.GetNodeIdMap()
	matrixOffset := da.Index(0)
	graph.ForVertices(func(v *da.Vertex) {
		if v.GetID() == da.Index(n) {
			return
		}
		if v.GetID() >= da.Index(n) {
			t.Errorf("expected vertex id less or equal than: %v, got: %v", n, v.GetID())
		}

		if da.Lt(v.GetLat(), bb.GetMinLat()) {
			t.Errorf("expected vertex latitude greater or equal than: %v, got: %v", bb.GetMinLat(), v.GetLat())
		}

		if da.Lt(v.GetLon(), bb.GetMinLon()) {
			t.Errorf("expected vertex longitude greater or equal than: %v, got: %v", bb.GetMinLon(), v.GetLon())
		}

		if da.Gt(v.GetLat(), bb.GetMaxLat()) {
			t.Errorf("expected vertex latitude less or equal than: %v, got: %v", bb.GetMaxLat(), v.GetLat())
		}

		if da.Gt(v.GetLon(), bb.GetMaxLon()) {
			t.Errorf("expected vertex longitude less or equal than: %v, got: %v", bb.GetMaxLon(), v.GetLon())
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
			_, inE := graph.GetTailOfOutedgeWithInEdge(id)
			if inE.GetTail() != v.GetID() {
				t.Errorf("expected tail of outedge (%v, %v): %v, got: %v", v.GetID(), e.GetHead(), v.GetID(), inE.GetTail())
			}

			if e.GetEdgeId() != id {
				t.Errorf("expected edge id: %v, got: %v", id, e.GetEdgeId())
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

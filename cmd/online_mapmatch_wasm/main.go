//go:build js && wasm

package main

import (
	"bytes"

	"fmt"

	"syscall/js"
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"go.uber.org/zap"
)

// terinspirasi dari: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e
// untuk implement fitur: https://github.com/lintang-b-s/Navigatorx/issues/10
// GOOS=js GOARCH=wasm go build -o ./bin/online_map_matcher.wasm ./cmd/online_mapmatch_wasm/main.go
// https://go.dev/wiki/WebAssembly

var (
	om        *online.OnlineMapMatchMHTClient
	graph     *da.MapMatchingGraph
	rt        *spatialindex.Rtree
	matrix    *da.SparseMatrix[int]
	logger    *zap.Logger
	keepAlive = make(chan struct{})
)

func main() {
	var err error

	config := zap.NewDevelopmentConfig()
	config.OutputPaths = []string{"stdout"}
	logger, err = config.Build()
	if err != nil {
		panic(err)
	}
	logger.Info("Navigatorx Online Map Matcher WASM Initializing...")

	rt = spatialindex.NewRtree()

	// expose to javascript
	js.Global().Set("OnlineMapMatch", js.FuncOf(onlineMapMatchJS))
	js.Global().Set("InitializeMapMatchingGraph", js.FuncOf(initializeMapMatchingGraphJS))
	js.Global().Set("RebuildMapMatchGraph", js.FuncOf(rebuildMapMatchGraphJS))

	logger.Info("Navigatorx Online Map Matcher WASM Ready.")

	<-keepAlive
}

func initializeMapMatchingGraphJS(this js.Value, args []js.Value) any {
	if len(args) < 2 {
		fmt.Println("InitializeMapMatchingGraphJS: missing arguments (expected numVertices, matrixBytes)")
		return nil
	}

	numVertices := args[0].Int()
	matrixBytesJS := args[1]
	matrixBytes := make([]byte, matrixBytesJS.Length())
	js.CopyBytesToGo(matrixBytes, matrixBytesJS)

	var err error
	matrix, err = da.ReadSparseMatrixFromReader[int](bytes.NewReader(matrixBytes), int(0),
		func(a, b int) bool { return a == b })
	if err != nil {
		fmt.Printf("Failed to read sparse matrix: %v\n", err)
		return nil
	}

	graph = da.InitializeMapMatchingGraph(numVertices)
	updateMapMatcher()

	return nil
}

func rebuildMapMatchGraphJS(this js.Value, args []js.Value) any {
	if len(args) < 1 {
		return nil
	}
	tileBytesJS := args[0]
	tileBytes := make([]byte, tileBytesJS.Length())
	js.CopyBytesToGo(tileBytes, tileBytesJS)

	err := graph.RebuildMapMatchGraphFromReader(bytes.NewReader(tileBytes))
	if err != nil {
		fmt.Printf("Failed to rebuild graph: %v\n", err)
		return nil
	}

	rt.Reset()
	rt.BuildMapMatch(graph, logger)
	updateMapMatcher()
	return nil
}

func updateMapMatcher() {
	om = online.NewOnlineMapMatchMHTClient(
		graph, rt,
		8.33333,   // initialSpeedMean
		8.3333,    // initialSpeedStd
		0.0001,    // posteriorThresold
		4.07,      // gpsStd
		0.0000001, // lp
		0.06,      // lc
		3,         // accelerationStd
		matrix,
	)
}

func onlineMapMatchJS(this js.Value, args []js.Value) any {
	if len(args) < 6 {
		return nil
	}

	// args dari nextjs: gps, k, candidates, speedMeanK, speedStdK, lastBearing
	gpsJS := args[0]
	k := args[1].Int()
	candidatesJS := args[2]
	speedMeanK := args[3].Float()
	speedStdK := args[4].Float()
	lastBearing := args[5].Float()

	// convert gpsJS to *da.GPSPoint
	gps := da.NewGPSPoint(
		gpsJS.Get("lat").Float(),
		gpsJS.Get("lon").Float(),
		time.Now(),
		gpsJS.Get("speed").Float(),
		gpsJS.Get("delta_time").Float(),
		gpsJS.Get("dead_reckoning").Bool(),
	)

	// convert candidatesJS to []*ma.Candidate
	var candidates []*ma.Candidate
	if !candidatesJS.IsNull() && !candidatesJS.IsUndefined() {
		candidates = make([]*ma.Candidate, 0, candidatesJS.Length())
		for i := 0; i < candidatesJS.Length(); i++ {
			candJS := candidatesJS.Index(i)
			roadNetworkEdgeId := da.Index(candJS.Get("roadnetwork_edge_id").Int())
			localId, ok := graph.GetMapMatchEdgeId(roadNetworkEdgeId)
			if !ok {

				// edgeId gak inlcuded di current graph tile
				continue
			}

			cand := ma.NewCandidate(
				localId,
				candJS.Get("weight").Float(),
				candJS.Get("length").Float(),
			)

			candidates = append(candidates, cand)
		}
	}

	matchedPoint, newCandidates, newSpeedMean, newSpeedStd := om.OnlineMapMatch(gps, k, candidates, speedMeanK, speedStdK, lastBearing)

	// convert ke javascript object
	res := make(map[string]any)
	if matchedPoint != nil {
		roadnetworkEdgeId := graph.GetRoadnetworkEdgeId(matchedPoint.GetEdgeId())

		res["matched_gps_point"] = map[string]any{
			"matched_coord": map[string]any{
				"lat": matchedPoint.GetMatchedCoord().GetLat(),
				"lon": matchedPoint.GetMatchedCoord().GetLon(),
			},
			"predicted_gps_coord": map[string]any{
				"lat": matchedPoint.GetPredictedGpsCoord().GetLat(),
				"lon": matchedPoint.GetPredictedGpsCoord().GetLon(),
			},
			"edge_initial_bearing": matchedPoint.GetBearing(),
			"roadnetwork_edge_id":  int(roadnetworkEdgeId),
			"gps_point": map[string]any{
				"lat":            matchedPoint.GetGpsPoint().Lat(),
				"lon":            matchedPoint.GetGpsPoint().Lon(),
				"time":           gps.Time().Format(time.RFC3339),
				"speed":          matchedPoint.GetGpsPoint().Speed(),
				"delta_time":     matchedPoint.GetGpsPoint().DeltaTime(),
				"dead_reckoning": matchedPoint.GetGpsPoint().GetDeadReckoning(),
			},
		}
	} else {
		res["matched_gps_point"] = nil
	}

	newCandsJS := make([]any, len(newCandidates))
	for i, c := range newCandidates {
		e := graph.GetOutEdge(c.EdgeId())
		roadNetworkEdgeId := e.GetRoadNetworkEdgeId()
		newCandsJS[i] = map[string]any{
			"roadnetwork_edge_id": int(roadNetworkEdgeId),
			"weight":              c.Weight(),
			"length":              c.Length(),
			"projected_lat":       c.GetProjectedCoord().GetLat(),
			"projected_lon":       c.GetProjectedCoord().GetLon(),
			"edge_bearing":        c.GetEdgeBearing(),
			"state_id":            c.GetStateId(),
		}
	}
	res["candidates"] = newCandsJS
	res["speed_mean_k"] = newSpeedMean
	res["speed_std_k"] = newSpeedStd
	res["edge_initial_bearing"] = 0.0
	if matchedPoint != nil {
		res["edge_initial_bearing"] = matchedPoint.GetBearing()
	}

	return res
}

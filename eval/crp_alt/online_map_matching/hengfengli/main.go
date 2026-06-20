package main

import (
	"bufio"
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"math/rand"
	"os"
	"strings"
	"time"

	onlinemapmatching "github.com/lintang-b-s/Navigatorx/eval/crp_alt/online_map_matching"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

/*

pastikan downloaded file tidak corrupt.

[1] Taguchi, S., Koide, S. and Yoshimura, T. (2019) “Online Map Matching With Route
Prediction,” IEEE Transactions on Intelligent Transportation Systems, 20(1), pp.
338–347. Available at: https://doi.org/10.1109/TITS.2018.2812147.

[2] Hu, H. et al. (2023) “AMM: An Adaptive Online Map Matching Algorithm,”
IEEE Transactions on Intelligent Transportation Systems, 24(5), pp. 5039–5051.
Available at: https://doi.org/10.1109/TITS.2023.3237519.


*/

// ini evaluasi implementasi online map matching pakai multiple hypothesis technique [1] (mapmatch_mht.go)
// https://web.archive.org/web/20170301001019/https://people.eng.unimelb.edu.au/henli/projects/map-matching/
const (
	datasetBundleFilePath     = "./data/eval/mapmatching/dataset_bundle.zip"
	datasetDirectoryPath      = "./data/eval/mapmatching"
	melbourneVerticesFilepath = "./data/eval/mapmatching/melbourne/complete-osm-map/vertex.txt"
	melbourneEdgesFilepath    = "./data/eval/mapmatching/melbourne/complete-osm-map/edges.txt"
	melbourneStreetsFilepath  = "./data/eval/mapmatching/melbourne/complete-osm-map/streets.txt"
	melbourneGPSTrackFilepath = "./data/eval/mapmatching/melbourne/gps_track.txt"
	melbourneGroundTruthPath  = "./data/eval/mapmatching/melbourne/groundtruth.txt"
	melbourneResultPolyline   = "./data/eval/mapmatching/melbourne/result_polyline.txt"
	melbourneGPSPolyline      = "./data/eval/mapmatching/melbourne/gps_track_polyline.txt"

	graphFile                         string = "./data/original_eval_mm_hl.ngraph"
	overlayGraphFile                  string = "./data/overlay_graph_eval_mm_hl.ngraph"
	metricsFile                       string = "./data/metrics_eval_mm_hl.nmt"
	transitionMatrixFilepath                 = "./data/eval/mapmatching/omm_transition_history_id_hl.ntm"
	mapmatchingDatasetBundleDriveLink        = "https://drive.google.com/uc?export=download&id=1WY0BPpu1M-e7grP33B_7BFrHlkj6kd_H"

	mlpFile                 = "./data/eval/mapmatching/online_map_match_mlp_hengfengli.mlp"
	landmarkFile            = "./data/eval/mapmatching/landmark_hl.nlm"
	timeFunctionFile string = "./data/timefunction_eval_mm_hl.ntf"
)

var (
	partitionSizes = []int{8, 11, 13, 14, 15}
)

func init() {
	flag.Parse()
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

type query struct {
	s, t da.Index
}

type melbourneVertex struct {
	id    int64
	osmID int64
	lon   float64
	lat   float64
}

type melbourneEdge struct {
	id       int64
	startID  int64
	endID    int64
	distance float64
}

type melbourneStreet struct {
	id       int64
	startID  int64
	startLon float64
	startLat float64
	endID    int64
	endLon   float64
	endLat   float64
	distance float64
}

type melbourneGPSPoint struct {
	timestamp int64
	lat       float64
	lon       float64
	t         time.Time
}

func newQuery(s, t da.Index) query {
	return query{
		s: s,
		t: t,
	}
}

func parseMelbourneVertexFile(filepath string) ([]melbourneVertex, error) {
	f, err := os.OpenFile(filepath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)
	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	expectedVertices, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, fmt.Errorf("parse vertex header count: %w", err)
	}

	vertices := make([]melbourneVertex, 0, expectedVertices)
	vertexByID := make(map[int64]melbourneVertex, expectedVertices)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 4 {
			return nil, fmt.Errorf("invalid vertex line: %q", line)
		}

		id, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, fmt.Errorf("parse vertex id: %w", err)
		}
		osmID, err := util.ParseTextInt64(ff[1])
		if err != nil {
			return nil, fmt.Errorf("parse vertex osm id: %w", err)
		}
		lon, err := util.ParseTextFloat64(ff[2])
		if err != nil {
			return nil, fmt.Errorf("parse vertex lon: %w", err)
		}
		lat, err := util.ParseTextFloat64(ff[3])
		if err != nil {
			return nil, fmt.Errorf("parse vertex lat: %w", err)
		}

		v := melbourneVertex{id: id, osmID: osmID, lon: lon, lat: lat}
		vertices = append(vertices, v)
		vertexByID[id] = v
	}

	if int64(len(vertices)) != expectedVertices {
		return nil, fmt.Errorf("vertex count mismatch: expected=%d actual=%d", expectedVertices, len(vertices))
	}
	return vertices, nil
}

func parseMelbourneEdgesFile(filepath string) ([]melbourneEdge, error) {
	f, err := os.OpenFile(filepath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)
	// Header line 1: number of vertices.
	if _, err := util.ReadLine(br); err != nil {
		return nil, err
	}
	// Header line 2: number of edges.
	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	expectedEdges, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, fmt.Errorf("parse edges header count: %w", err)
	}

	edges := make([]melbourneEdge, 0, expectedEdges)

	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 4 {
			return nil, fmt.Errorf("invalid edges line: %q", line)
		}
		id, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, fmt.Errorf("parse edge id: %w", err)
		}
		startID, err := util.ParseTextInt64(ff[1])
		if err != nil {
			return nil, fmt.Errorf("parse edge start id: %w", err)
		}
		endID, err := util.ParseTextInt64(ff[2])
		if err != nil {
			return nil, fmt.Errorf("parse edge end id: %w", err)
		}
		dist, err := util.ParseTextFloat64(ff[3])
		if err != nil {
			return nil, fmt.Errorf("parse edge distance: %w", err)
		}
		e := melbourneEdge{id: id, startID: startID, endID: endID, distance: dist}
		edges = append(edges, e)

	}

	if int64(len(edges)) != expectedEdges {
		return nil, fmt.Errorf("edge count mismatch: expected=%d actual=%d", expectedEdges, len(edges))
	}
	return edges, nil
}

func parseMelbourneStreetsFile(filepath string) ([]melbourneStreet, map[int64]melbourneStreet, error) {
	f, err := os.OpenFile(filepath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)
	line, err := util.ReadLine(br)
	if err != nil {
		return nil, nil, err
	}
	expectedEdges, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, nil, fmt.Errorf("parse streets header count: %w", err)
	}

	streets := make([]melbourneStreet, 0, expectedEdges)
	streetByID := make(map[int64]melbourneStreet, expectedEdges)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 10 {
			return nil, nil, fmt.Errorf("invalid streets line: %q", line)
		}
		id, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street edge id: %w", err)
		}
		startID, err := util.ParseTextInt64(ff[1])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street start id: %w", err)
		}
		startLon, err := util.ParseTextFloat64(ff[2])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street start lon: %w", err)
		}
		startLat, err := util.ParseTextFloat64(ff[3])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street start lat: %w", err)
		}
		endID, err := util.ParseTextInt64(ff[4])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street end id: %w", err)
		}
		endLon, err := util.ParseTextFloat64(ff[5])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street end lon: %w", err)
		}
		endLat, err := util.ParseTextFloat64(ff[6])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street end lat: %w", err)
		}
		dist, err := util.ParseTextFloat64(ff[7])
		if err != nil {
			return nil, nil, fmt.Errorf("parse street distance: %w", err)
		}
		s := melbourneStreet{
			id: id, startID: startID, startLon: startLon, startLat: startLat,
			endID: endID, endLon: endLon, endLat: endLat, distance: dist,
		}
		streets = append(streets, s)
		streetByID[id] = s
	}

	if int64(len(streets)) != expectedEdges {
		return nil, nil, fmt.Errorf("street count mismatch: expected=%d actual=%d", expectedEdges, len(streets))
	}
	return streets, streetByID, nil
}

func melbourneDatasetReady() bool {
	requiredFiles := []string{
		melbourneVerticesFilepath,
		melbourneEdgesFilepath,
		melbourneStreetsFilepath,
		melbourneGPSTrackFilepath,
		melbourneGroundTruthPath,
	}
	for _, filePath := range requiredFiles {
		if _, err := os.Stat(filePath); err != nil {
			return false
		}
	}
	return true
}

func buildCRPGraph() (*engine.Engine[int32], *da.Graph, *zap.Logger, *da.SparseMatrix[int], map[da.Index]int64, map[int64]float64, error) {

	flag.Parse()
	logger, err := log.New()
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: log.New() failed %w", err)
	}

	if !melbourneDatasetReady() {
		err = onlinemapmatching.Download(datasetBundleFilePath, mapmatchingDatasetBundleDriveLink, logger, "melbourne dataset bundle file")
		if err != nil {
			return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: download() failed %w", err)
		}

		err = onlinemapmatching.ExtractZip(datasetBundleFilePath, datasetDirectoryPath)

		if err != nil {
			return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: onlinemapmatching.ExtractZip() failed: %v", err)
		}
	}

	vertices, err := parseMelbourneVertexFile(melbourneVerticesFilepath)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: parse vertex file failed: %w", err)
	}
	edges, err := parseMelbourneEdgesFile(melbourneEdgesFilepath)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: parse edges file failed: %w", err)
	}
	_, streetByID, err := parseMelbourneStreetsFile(melbourneStreetsFilepath)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: parse streets file failed: %w", err)
	}

	graphStorage := da.NewGraphStorage(54)
	graphEdges := make([]osmparser.Edge[int32], 0, len(edges))
	for _, e := range edges {
		st, ok := streetByID[e.id]
		if !ok {
			return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: edge id %d not found in streets.txt", e.id)
		}
		if st.startID != e.startID || st.endID != e.endID {
			return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: edge id %d mismatch between edges.txt and streets.txt", e.id)
		}

		startPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendOsmNodePoints([]da.Coordinate{
			da.NewCoordinate(st.startLat, st.startLon),
			da.NewCoordinate(st.endLat, st.endLon),
		})

		endPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendEdgeMetadata(e.id, da.Index(startPointsIndex), da.Index(endPointsIndex), 0, 0, 0, 1)
		graphEdge := osmparser.NewFixedEdge(
			uint32(e.startID), uint32(e.endID), e.distance, e.distance, false, pkg.MOTORWAY,
		)

		graphEdges = append(graphEdges, graphEdge)
	}

	acceptedNodeMap := make(map[int64]osmparser.NodeCoord, len(vertices))
	nodeToOsmID := make(map[da.Index]int64, len(vertices))
	for _, v := range vertices {
		acceptedNodeMap[v.osmID] = osmparser.NewNodeCoord(v.lat, v.lon)
		nodeToOsmID[da.Index(v.id)] = v.osmID
	}

	op := osmparser.NewOSMParserV2[int32]()
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmID)
	graph, timeFunction, edgeInfoIds := op.BuildGraph(graphEdges, graphStorage, uint32(len(vertices)), true)
	graph.SetGraphStorage(graphStorage)

	ps := make([]int, len(partitionSizes))
	for i := 0; i < len(ps); i++ {
		pow := partitionSizes[i]
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger,
		false,
		false,
	)
	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: mp.SaveToFile() failed: %v", err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpFile)
	if err != nil {
		panic(err)
	}

	prep := prepo.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: prep.PreProcessing() failed: %v", err)
	}

	cust := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)
	_, err = cust.Customize()
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: cust.Customize() failed: %v", err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: engine.NewEngine() failed: %v", err)
	}

	logger.Sugar().Infof("customization phase of Customizable Route Planning (CRP) done....")

	n := graph.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))

	logger.Sugar().Infof("building transition matrix....")
	numQueries := 1000.0
	i := 0
	queries := make([]query, 0, n)

	for i < int(numQueries) {
		s := da.Index(rd.Intn(n))
		t := da.Index(rd.Intn(n))
		if s == t {
			continue
		}
		if !graph.PathExists(s, t) {
			continue
		}

		queries = append(queries, newQuery(s, t))
		i++
	}

	computeRoute := func(q query) []da.Index {
		s, t := q.s, q.t
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0)
		as := graph.GetDummyOutEdgeId(s)
		at := graph.GetDummyInEdgeId(t)

		sVertex := graph.GetVertex(s)
		tVertex := graph.GetVertex(t)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		_, _, _, edges, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		return edges
	}

	workers := concurrent.NewWorkerPool[query, []da.Index](100, 25_000)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	var N *da.SparseMatrix[int]
	_, err = os.Stat(transitionMatrixFilepath)
	if err == nil {
		logger.Info("reading transition matrix from file...")

		N, err = da.ReadSparseMatrixFromFile[int](transitionMatrixFilepath, int(0),
			func(a, b int) bool { return a == b })
		if err != nil {
			return nil, nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph:  da.ReadSparseMatrixFromFile() failed: %v", err)
		}
	} else {
		N = da.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(),
			0, func(a, b int) bool { return a == b })
	}

	go func() {
		counter := 0
		for spEdges := range workers.CollectResults() {
			if len(spEdges) == 0 {
				continue
			}

			for j := 0; j < len(spEdges)-1; j++ {
				e := int(spEdges[j])
				eNext := int(spEdges[j+1])
				N.Set(N.Get(e, eNext)+1, e, eNext)
			}
			counter++

			if counter%1e2 == 0 {
				fmt.Printf("completed query: %v\n", counter)
				if err := N.WriteToFile(transitionMatrixFilepath); err != nil {
					fmt.Printf("error writing transition matrix: %v\n", err)
				}
			}
		}
	}()

	for _, qq := range queries {
		workers.AddJob(qq)
	}

	workers.Close()
	workers.Wait()
	cancel()

	logger.Sugar().Infof(" transition matrix built....")

	graphEdgeIDToMelbourneEdgeID := make(map[da.Index]int64, graph.NumberOfEdges())
	graph.ForOutEdges(func(exitPoint, head da.Index, tail, entryId, entryPoint da.Index, percentage float64, eId da.Index) {
		graphEdgeIDToMelbourneEdgeID[eId] = graph.GetOsmWayId(eId)
	})

	edgeLengthByID := make(map[int64]float64, len(streetByID))
	for edgeID, street := range streetByID {
		edgeLengthByID[edgeID] = street.distance
	}

	return re, graph, logger, N, graphEdgeIDToMelbourneEdgeID, edgeLengthByID, nil
}

func readMelbourneGPSTrack(filePath string) ([]melbourneGPSPoint, error) {
	f, err := os.OpenFile(filePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)
	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	expectedPoints, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, fmt.Errorf("parse gps point count in %s: %w", filePath, err)
	}

	points := make([]melbourneGPSPoint, 0, expectedPoints)
	lineNo := 1
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		lineNo++
		if len(strings.TrimSpace(line)) == 0 {
			continue
		}

		ff := util.Fields(line)
		if len(ff) < 3 {
			return nil, fmt.Errorf("invalid gps line %d in %s: %q", lineNo, filePath, line)
		}

		timestamp, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, fmt.Errorf("parse gps timestamp line %d in %s: %w", lineNo, filePath, err)
		}
		lat, err := util.ParseTextFloat64(ff[1])
		if err != nil {
			return nil, fmt.Errorf("parse gps latitude line %d in %s: %w", lineNo, filePath, err)
		}
		lon, err := util.ParseTextFloat64(ff[2])
		if err != nil {
			return nil, fmt.Errorf("parse gps longitude line %d in %s: %w", lineNo, filePath, err)
		}

		points = append(points, melbourneGPSPoint{
			timestamp: timestamp,
			lat:       lat,
			lon:       lon,
			t:         time.Unix(timestamp, 0),
		})
	}

	if int64(len(points)) != expectedPoints {
		return nil, fmt.Errorf("gps point count mismatch in %s: expected=%d actual=%d", filePath, expectedPoints, len(points))
	}
	return points, nil
}

func readMelbourneGroundTruthEdges(filePath string) ([]int64, error) {
	f, err := os.OpenFile(filePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)
	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	expectedEdges, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, fmt.Errorf("parse groundtruth edge count in %s: %w", filePath, err)
	}

	edgeIDs := make([]int64, 0, expectedEdges)
	lineNo := 1
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		lineNo++
		if len(strings.TrimSpace(line)) == 0 {
			continue
		}

		ff := util.Fields(line)
		if len(ff) < 1 {
			return nil, fmt.Errorf("invalid groundtruth line %d in %s: %q", lineNo, filePath, line)
		}
		edgeID, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, fmt.Errorf("parse groundtruth edge id line %d in %s: %w", lineNo, filePath, err)
		}
		edgeIDs = append(edgeIDs, edgeID)
	}

	if int64(len(edgeIDs)) != expectedEdges {
		return nil, fmt.Errorf("groundtruth edge count mismatch in %s: expected=%d actual=%d", filePath, expectedEdges, len(edgeIDs))
	}
	return edgeIDs, nil
}

func evaluateMatchedRoute(mapMatchPointResult []*da.MatchedGPSPoint, groundTruthEdgeIDs []int64,
	graphEdgeIDToMelbourneEdgeID map[da.Index]int64, edgeLengthByID map[int64]float64) (float64, float64, error) {
	groundTruthEdgeSet := make(map[int64]float64, len(groundTruthEdgeIDs))
	for _, edgeID := range groundTruthEdgeIDs {
		edgeLength, ok := edgeLengthByID[edgeID]
		if !ok {
			return 0, 0, fmt.Errorf("ground truth edge id %d is missing from streets.txt", edgeID)
		}
		groundTruthEdgeSet[edgeID] = edgeLength
	}

	lengthOfCorrectRoute := 0.0
	for _, edgeLength := range groundTruthEdgeSet {
		lengthOfCorrectRoute += edgeLength
	}
	if util.Le(lengthOfCorrectRoute, 0) {
		return 0, 0, fmt.Errorf("ground truth route has zero length")
	}

	matchedEdgeSet := make(map[int64]float64)
	numCorrectMatchedRoads := 0.0
	numberOfRoadsOfMatchedTrips := 0.0
	for _, matchedPoint := range mapMatchPointResult {
		internalEdgeID := matchedPoint.GetEdgeId()
		if internalEdgeID == da.INVALID_EDGE_ID {
			continue
		}

		melbourneEdgeID, ok := graphEdgeIDToMelbourneEdgeID[internalEdgeID]
		if !ok {
			continue
		}
		edgeLength, ok := edgeLengthByID[melbourneEdgeID]
		if !ok {
			return 0, 0, fmt.Errorf("matched edge id %d is missing from streets.txt", melbourneEdgeID)
		}

		matchedEdgeSet[melbourneEdgeID] = edgeLength
		if _, inGroundTruth := groundTruthEdgeSet[melbourneEdgeID]; inGroundTruth {
			numCorrectMatchedRoads++
		}
		numberOfRoadsOfMatchedTrips++
	}
	if util.Le(numberOfRoadsOfMatchedTrips, 0) {
		return 0, 0, fmt.Errorf("trajectory produced no valid matched edges")
	}

	lengthOfErroneouslyAdded := 0.0
	for edgeID, edgeLength := range matchedEdgeSet {
		if _, inGroundTruth := groundTruthEdgeSet[edgeID]; !inGroundTruth {
			lengthOfErroneouslyAdded += edgeLength
		}
	}

	lengthOfErroneouslySubtracted := 0.0
	for edgeID, edgeLength := range groundTruthEdgeSet {
		if _, inMatchedRoute := matchedEdgeSet[edgeID]; !inMatchedRoute {
			lengthOfErroneouslySubtracted += edgeLength
		}
	}

	rmf := (lengthOfErroneouslyAdded + lengthOfErroneouslySubtracted) / lengthOfCorrectRoute
	crp := numCorrectMatchedRoads / numberOfRoadsOfMatchedTrips // section V Accuracy: https://mod.wict.pku.edu.cn/docs/20240422170836017278.pdf
	return rmf, crp, nil
}

func main() {
	re, graph, logger, N, graphEdgeIDToMelbourneEdgeID, edgeLengthByID, err := buildCRPGraph()
	if err != nil {
		panic(err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)
	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(graph, rtree, 8.33333, 8.3333, 0.001, 5.0, 0.000001,
		0.06, 3, N, func(eID da.Index) float64 {
			return re.GetRoutingEngine().GetSegmentLength(eID, true)
		}) // speed in meter/s

	gpsTraj, err := readMelbourneGPSTrack(melbourneGPSTrackFilepath)
	if err != nil {
		panic(err)
	}
	groundTruthEdgeIDs, err := readMelbourneGroundTruthEdges(melbourneGroundTruthPath)
	if err != nil {
		panic(err)
	}

	var (
		prevLat, prevLon float64
		prevTime         time.Time
		hasPrev          bool
		candidates       []*ma.Candidate
		speedMeanK       = 8.333
		speedStdK        = 8.333
		lastBearing      = 0.0
		k                = 1
		matchedPoint     *da.MatchedGPSPoint
	)

	mapMatchPointResult := make([]*da.MatchedGPSPoint, 0, len(gpsTraj))
	gpsCoords := make([]da.Coordinate, 0, len(gpsTraj))
	matchedCoords := make([]da.Coordinate, 0, len(gpsTraj))
	avgRuntimePerGpsPoint := 0.0
	nowDataset := time.Now()

	for _, gps := range gpsTraj {
		deltaTime := 1.0
		speed := 8.333
		if hasPrev {
			deltaTime = gps.t.Sub(prevTime).Seconds()
			if util.Gt(deltaTime, 0) {
				dist := geo.CalculateGreatCircleDistance(prevLat, prevLon, gps.lat, gps.lon)
				speed = util.KilometerToMeter(dist) / deltaTime
			}
		} else {
			hasPrev = true
		}

		prevLat, prevLon = gps.lat, gps.lon
		prevTime = gps.t

		now := time.Now()
		curGps := da.NewGPSPoint(gps.lat, gps.lon, gps.t, speed, deltaTime)
		matchedPoint, candidates, speedMeanK, speedStdK = onlineMapMatcherEngine.OnlineMapMatch(curGps, k, candidates, speedMeanK, speedStdK, lastBearing)
		k++
		lastBearing = matchedPoint.GetBearing()
		mapMatchPointResult = append(mapMatchPointResult, matchedPoint)
		gpsCoords = append(gpsCoords, da.NewCoordinate(gps.lat, gps.lon))
		avgRuntimePerGpsPoint += float64(time.Since(now).Microseconds())
		if matchedPoint.GetEdgeId() != da.INVALID_EDGE_ID {
			matchedCoords = append(matchedCoords, matchedPoint.GetMatchedCoord())
		}
	}

	totalRuntimeMillis := float64(time.Since(nowDataset).Microseconds()) / 1000.0
	totalPoints := float64(len(gpsTraj))
	avgRuntimePerGpsPoint /= float64(len(gpsTraj))

	rmf, crp, err := evaluateMatchedRoute(mapMatchPointResult, groundTruthEdgeIDs, graphEdgeIDToMelbourneEdgeID, edgeLengthByID)
	if err != nil {
		panic(err)
	}

	fmt.Printf("trajectory gps_track completed: points=%d RMF=%.6f CRP=%.6f avg_runtime=%.4f microseconds/gps point\n",
		len(gpsTraj), rmf, crp, avgRuntimePerGpsPoint)
	fmt.Printf("avg runtime per gps point: %v microseconds/gps point\n", avgRuntimePerGpsPoint)
	if util.Gt(totalRuntimeMillis, 0) {
		fmt.Printf("matching efficiency: %v points/ms\n", totalPoints/totalRuntimeMillis)
	}
	fmt.Printf("Route Mismatch Fraction (RMF): %v\n", rmf)
	fmt.Printf("Correct Road Percentage (CRP) or accuracy: %v\n", crp)

	gpsTrackPolyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(gpsCoords))
	if err := os.WriteFile(melbourneGPSPolyline, []byte(gpsTrackPolyline), 0644); err != nil {
		panic(err)
	}

	matchedPolyline := ""
	if len(matchedCoords) > 0 {
		matchedPolyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
	}
	if err := os.WriteFile(melbourneResultPolyline, []byte(matchedPolyline), 0644); err != nil {
		panic(err)
	}
	fmt.Printf("wrote matched polyline to %s\n", melbourneResultPolyline)
	fmt.Printf("wrote gps trajectory polyline to %s\n", melbourneGPSPolyline)
}

package onlinemapmatching

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"io"
	"math/rand"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/tiler"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/mmcloughlin/geohash"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

const (
	nkRoadNetworkDriveFile = "https://drive.google.com/uc?export=download&id=1ba1CcLbTRerbDVNN91wTNfrS85EJGhG6"
	nkGPSDataDriveFile     = "https://drive.google.com/uc?export=download&id=1QCrMnchOjCfOMQet9Oon-dmZ36MasTjA"
	nkGroundTruthDriveFile = "https://drive.google.com/uc?export=download&id=11LxzpV-VDCImDq3OWN3m3tukFKmwl9Fn"
	nkExpectedMinAccuracy  = 0.90
	nkExpectedMaxRMF       = 0.15
)

type nkEdge struct {
	eId         int64
	fromId      uint32
	toId        uint32
	speed       float64
	twoWay      bool
	vertexCount int
	geometry    []da.Coordinate
}

type nkQuery struct {
	s da.Index
	t da.Index
}

func nkNewQuery(s, t da.Index) nkQuery {
	return nkQuery{s: s, t: t}
}

func nkParseLineString(s string, vertexCount int) ([]da.Coordinate, error) {
	const prefix = "LINESTRING("
	content := s[len(prefix) : len(s)-1]
	parts := strings.Split(content, ",")
	coords := make([]da.Coordinate, 0, vertexCount)
	for _, p := range parts {
		p = strings.TrimSpace(p)
		xy := strings.Fields(p)
		if len(xy) < 2 {
			return nil, fmt.Errorf("invalid coordinates")
		}
		lon, err1 := strconv.ParseFloat(xy[0], 64)
		lat, err2 := strconv.ParseFloat(xy[1], 64)
		if err1 != nil || err2 != nil {
			return nil, fmt.Errorf("%v %v", err1, err2)
		}
		coords = append(coords, da.NewCoordinate(lat, lon))
	}
	return coords, nil
}

func nkDownload(filePath, url string, zlog *zap.Logger, t *testing.T, name string) error {
	if _, err := os.Stat(filePath); os.IsNotExist(err) {
		t.Logf("downloading evaluation %s dataset.....", name)
		zlog.Sugar().Infof("downloading evaluation %s dataset.....", name)

		dir := filepath.Dir(filePath)
		if err := os.MkdirAll(dir, os.ModePerm); err != nil {
			return fmt.Errorf("download: MkdirAll failed %v", err)
		}

		output, err := os.Create(filePath)
		if err != nil {
			return fmt.Errorf("download: Create failed %v", err)
		}
		defer output.Close()

		t.Logf("downloading file......")
		zlog.Sugar().Infof("downloading file......")
		response, err := http.Get(url)
		if err != nil {
			return fmt.Errorf("download: http.Get failed %v", err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			return fmt.Errorf("download: io.Copy failed %v", err)
		}

		t.Logf("download complete")
		zlog.Sugar().Infof("download complete")
	}
	return nil
}

func nkBuildRoadNetworkCRPGraph(t *testing.T, workingDir string) (*engine.Engine, *da.Graph, *zap.Logger, *da.SparseMatrix[int], map[int64]float64, error) {
	zlog, err := logger.New()
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	config.InitProfileConfig("car", "newsonkrumm")

	roadnetworkFilepath := filepath.Join(workingDir, "data/eval/mapmatching/road_network.txt")
	graphFile := filepath.Join(workingDir, "data/original_eval_mm.graph")
	mlpFile := filepath.Join(workingDir, "data/eval/mapmatching/online_map_match_mlp_newsonkrumm.mlp")
	overlayGraphFile := filepath.Join(workingDir, "data/overlay_graph_eval_mm.graph")
	metricsFile := filepath.Join(workingDir, "data/metrics_eval_mm.txt")
	landmarkFile := filepath.Join(workingDir, "data/eval/mapmatching/landmark_nk.lm")
	timeFunctionFile := filepath.Join(workingDir, "data/timefunction_eval_mm.txt")
	transitionMatrixFilepath := filepath.Join(workingDir, "data/eval/mapmatching/transition_matrix_newsonkrumm.txt")

	if err := nkDownload(roadnetworkFilepath, nkRoadNetworkDriveFile, zlog, t, "road network"); err != nil {
		return nil, nil, nil, nil, nil, err
	}

	t.Logf("building road network graph & running preprocessing, customization phase of Customizable Route Planning CRP....")
	zlog.Sugar().Infof("building road network graph & running preprocessing, customization phase of Customizable Route Planning CRP....")

	f, err := os.OpenFile(roadnetworkFilepath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)
	edges := make([]nkEdge, 0)
	nodeIdMap := make(map[int64]uint32)
	nodeCoords := make([]da.Coordinate, 0)
	lineId := 0
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		if lineId == 0 {
			lineId++
			continue
		}
		lineId++
		ff := util.Fields(line)
		edgeID, err := strconv.ParseInt(ff[0], 10, 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		fromID, err := strconv.ParseInt(ff[1], 10, 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		toID, err := strconv.ParseInt(ff[2], 10, 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		twoWayInt, err := strconv.ParseInt(ff[3], 10, 32)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		twoWay := twoWayInt == 1
		speed, err := strconv.ParseFloat(ff[4], 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		vertexCount, err := strconv.ParseInt(ff[5], 10, 32)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}

		idx := strings.Index(line, "LINESTRING(")
		if idx == -1 {
			return nil, nil, nil, nil, nil, fmt.Errorf("LINESTRING not found")
		}
		edgeGeometry, err := nkParseLineString(line[idx:], int(vertexCount))
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}

		fromNID, ok := nodeIdMap[fromID]
		if !ok {
			fromNID = uint32(len(nodeIdMap))
			nodeIdMap[fromID] = fromNID
			nodeCoords = append(nodeCoords, edgeGeometry[0])
		}

		toNID, ok := nodeIdMap[toID]
		if !ok {
			toNID = uint32(len(nodeIdMap))
			nodeIdMap[toID] = toNID
			nodeCoords = append(nodeCoords, edgeGeometry[len(edgeGeometry)-1])
		}

		edges = append(edges, nkEdge{eId: edgeID, fromId: fromNID, toId: toNID, speed: speed, twoWay: twoWay, vertexCount: int(vertexCount), geometry: edgeGeometry})
	}

	graphStorage := da.NewGraphStorage(54)
	graphEdges := make([]osmparser.Edge, 0, len(edges))
	edgeLength := make(map[int64]float64)

	for _, e := range edges {
		if e.fromId == e.toId {
			continue
		}
		distance := 0.0
		for i := 0; i < len(e.geometry); i++ {
			if i > 0 {
				distance += geo.CalculateGreatCircleDistance(e.geometry[i-1].GetLat(), e.geometry[i-1].GetLon(), e.geometry[i].GetLat(), e.geometry[i].GetLon())
			}
		}
		distanceInMeter := util.KilometerToMeter(distance)
		travelTimeWeight := distanceInMeter / e.speed
		edgeLength[e.eId] = distanceInMeter

		startPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendOsmNodePoints(e.geometry)
		endPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendEdgeMetadata(int64(e.eId), da.Index(startPointsIndex), da.Index(endPointsIndex), 0, 0, 0, 1)
		graphEdges = append(graphEdges, osmparser.NewEdge(e.fromId, e.toId, travelTimeWeight, distanceInMeter, false, pkg.MOTORWAY))

		if e.twoWay {
			graphStorage.AppendEdgeMetadata(int64(e.eId)*2, da.Index(endPointsIndex), da.Index(startPointsIndex), 0, 0, 0, 1)
			graphEdges = append(graphEdges, osmparser.NewEdge(e.toId, e.fromId, travelTimeWeight, distanceInMeter, false, pkg.MOTORWAY))
		}
	}

	op := osmparser.NewOSMParserV2()
	n := len(nodeIdMap)
	acceptedNodeMap := make(map[int64]osmparser.NodeCoord, n)
	nodeToOsmID := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = osmparser.NewNodeCoord(nodeCoords[i].GetLat(), nodeCoords[i].GetLon())
		nodeToOsmID[da.Index(i)] = int64(i)
	}
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmID)
	g, edgeInfoIds := op.BuildGraph(graphEdges, graphStorage, uint32(len(nodeIdMap)), true)
	g.SetGraphStorage(graphStorage)

	us := []int{8, 11, 14, 16}
	ps := make([]int, len(us))
	for i := range ps {
		ps[i] = 1 << us[i]
	}
	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 1, g, zlog, false, false)
	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(mlpFile); err != nil {
		return nil, nil, nil, nil, nil, err
	}
	mlp := da.NewPlainMLP()
	if err := mlp.ReadMlpFile(mlpFile); err != nil {
		return nil, nil, nil, nil, nil, err
	}
	prep := preprocesser.NewPreprocessor(g, mlp, zlog, graphFile, overlayGraphFile, edgeInfoIds)
	if err := prep.PreProcessing(true); err != nil {
		return nil, nil, nil, nil, nil, err
	}
	cust := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, zlog)
	if _, err := cust.Customize(); err != nil {
		return nil, nil, nil, nil, nil, err
	}
	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, zlog)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}
	g = re.GetRoutingEngine().GetGraph()

	t.Logf("customization phase of Customizable Route Planning CRP done....")
	zlog.Sugar().Infof("customization phase of Customizable Route Planning CRP done....")

	n = g.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	t.Logf("building transition matrix....")
	zlog.Sugar().Infof("building transition matrix....")

	numQueries := 5000
	queries := make([]nkQuery, 0, n)
	for len(queries) < numQueries {
		s := da.Index(rd.Intn(n))
		tt := da.Index(rd.Intn(n))
		if s == tt || !g.PathExists(s, tt) {
			continue
		}
		queries = append(queries, nkNewQuery(s, tt))
	}

	computeRoute := func(q nkQuery) []da.Index {
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0)
		as := g.GetDummyOutEdgeId(q.s)
		at := g.GetDummyInEdgeId(q.t)
		sVertex := g.GetVertex(q.s)
		tVertex := g.GetVertex(q.t)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)
		_, _, _, routeEdges, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		return routeEdges
	}

	workers := concurrent.NewWorkerPool[nkQuery, []da.Index](100, 5)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	var N *da.SparseMatrix[int]
	if _, err := os.Stat(transitionMatrixFilepath); err == nil {
		t.Logf("reading transition matrix from file...")
		zlog.Info("reading transition matrix from file...")
		N, err = da.ReadSparseMatrixFromFile[int](transitionMatrixFilepath, int(0), func(a, b int) bool { return a == b })
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
	} else {
		N = da.NewSparseMatrix[int](g.NumberOfEdges(), g.NumberOfEdges(), 0, func(a, b int) bool { return a == b })
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
			if counter%100 == 0 {
				t.Logf("completed query: %v", counter)
				if err := N.WriteToFile(transitionMatrixFilepath); err != nil {
					t.Logf("error writing transition matrix: %v", err)
				}
			}
		}
	}()

	for _, q := range queries {
		workers.AddJob(q)
	}
	workers.Close()
	workers.Wait()
	cancel()

	t.Logf(" transition matrix built....")
	zlog.Sugar().Infof(" transition matrix built....")

	return re, g, zlog, N, edgeLength, nil
}

func nkReadGPSTrajectory(t *testing.T, gpsDataFilepath string) []*da.GPSPoint {
	t.Helper()

	f, err := os.OpenFile(gpsDataFilepath, os.O_RDONLY, 0644)
	if err != nil {
		t.Fatalf("OpenFile(gps) failed: %v", err)
	}
	defer f.Close()

	br := bufio.NewReader(f)
	gpsTraj := make([]*da.GPSPoint, 0)
	var (
		prevLat, prevLon float64
		prevTime         time.Time
		hasPrev          bool
	)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			t.Fatalf("read gps line failed: %v", err)
		}
		ff := util.Fields(line)
		if len(ff) == 0 || ff[0] == "Date" {
			continue
		}

		dateTime := ff[0] + " " + ff[1]
		lat, err := strconv.ParseFloat(ff[2], 64)
		if err != nil {
			t.Fatalf("parse lat failed: %v", err)
		}
		lon, err := strconv.ParseFloat(ff[3], 64)
		if err != nil {
			t.Fatalf("parse lon failed: %v", err)
		}
		curGPSTime, err := time.Parse("02-Jan-2006 15:04:05", dateTime)
		if err != nil {
			t.Fatalf("parse timestamp failed: %v", err)
		}

		deltaTime := 1.0
		speed := 8.333
		if hasPrev {
			deltaTime = curGPSTime.Sub(prevTime).Seconds()
			dist := geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon)
			speed = util.KilometerToMeter(dist) / deltaTime
		} else {
			hasPrev = true
		}
		prevLat, prevLon, prevTime = lat, lon, curGPSTime

		gpsTraj = append(gpsTraj, da.NewGPSPoint(lat, lon, curGPSTime, speed, deltaTime))
	}
	return gpsTraj
}

func nkEvaluateMatchedRoute(t *testing.T, g *da.Graph, groundTruthDataFilepath string, edgeLength map[int64]float64,
	mapMatchPointResult []*da.MatchedGPSPoint) (float64, float64) {
	t.Helper()

	groundTruthFile, err := os.Open(groundTruthDataFilepath)
	if err != nil {
		t.Fatalf("open ground truth failed: %v", err)
	}
	defer groundTruthFile.Close()
	brGroundTruth := bufio.NewReader(groundTruthFile)

	traversedEdgesLength := make(map[int64]float64, 576)
	traversedEdges := make(map[int64]bool, 576)
	for {
		line, err := util.ReadLine(brGroundTruth)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			t.Fatalf("read ground truth line failed: %v", err)
		}
		ff := util.Fields(line)
		if len(ff) == 0 || ff[0] == "Edge" {
			continue
		}
		edgeID, err := strconv.ParseInt(ff[0], 10, 64)
		if err != nil {
			t.Fatalf("parse edge id failed: %v", err)
		}
		traversedEdgesLength[edgeID] = edgeLength[edgeID]
		traversedEdges[edgeID] = ff[1] == "1"
	}

	lengthOfCorrectRoute := 0.0
	for _, eLength := range traversedEdgesLength {
		lengthOfCorrectRoute += eLength
	}
	lengthOfErrAdded := 0.0
	lengthOfErrSubtracted := 0.0
	numOfCorrectMatchedRoads := 0.0
	numberOfRoadsOfMatchedTrips := 0.0
	matchedEdgeSet := make(map[int64]float64)

	for _, point := range mapMatchPointResult {
		curMatchedEID := point.GetEdgeId()
		if curMatchedEID == da.INVALID_EDGE_ID {
			continue
		}
		matchedDataEID := g.GetOsmWayId(curMatchedEID)
		curMatchedEdge := g.GetOutEdge(curMatchedEID)
		matchedEdgeSet[matchedDataEID] = curMatchedEdge.GetLength()
		_, inGroundTruthReversed := traversedEdges[matchedDataEID]
		_, inGroundTruthForward := traversedEdges[matchedDataEID/2]
		if inGroundTruthForward || inGroundTruthReversed {
			numOfCorrectMatchedRoads++
		}
		numberOfRoadsOfMatchedTrips++
	}

	for matchedDataEID, eLength := range matchedEdgeSet {
		_, inGroundTruthReversed := traversedEdges[matchedDataEID]
		_, inGroundTruthForward := traversedEdges[matchedDataEID/2]
		if !inGroundTruthForward && !inGroundTruthReversed {
			lengthOfErrAdded += eLength
		}
	}

	for eID := range traversedEdges {
		eLengthForward, inMapMatchResultForward := matchedEdgeSet[eID]
		eLengthReversed, inMapMatchResultReversed := matchedEdgeSet[eID*2]
		eLength := util.MaxFloat(eLengthForward, eLengthReversed)
		if !inMapMatchResultForward && !inMapMatchResultReversed {
			lengthOfErrSubtracted += eLength
		}
	}

	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	crp := 0.0

	crp = numOfCorrectMatchedRoads / numberOfRoadsOfMatchedTrips // section V Accuracy: https://mod.wict.pku.edu.cn/docs/20240422170836017278.pdf
	return crp, rmf
}

// go test ./tests/map_matching  -run TestNewsonKrummOnlineMapMatching  -v -timeout=0  -count=1
func TestNewsonKrummOnlineMapMatching(t *testing.T) {
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		t.Fatalf("FindProjectWorkingDir() failed: %v", err)
	}
	if err := config.ReadConfig(workingDir); err != nil {
		t.Fatalf("ReadConfig() failed: %v", err)
	}
	vehicleType := viper.GetString("vehicle_type")
	pkg.VehicleType = pkg.GetVehicleType(vehicleType)
	pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
	pkg.IsVehicleEnabled = pkg.GetIsVehicle()
	pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()

	_, g, zlog, N, edgeLength, err := nkBuildRoadNetworkCRPGraph(t, workingDir)
	if err != nil {
		t.Fatalf("nkBuildRoadNetworkCRPGraph() failed: %v", err)
	}

	gpsDataFilepath := filepath.Join(workingDir, "data/eval/mapmatching/gps_data.txt")
	groundTruthDataFilepath := filepath.Join(workingDir, "data/eval/mapmatching/ground_truth.txt")
	if err := nkDownload(gpsDataFilepath, nkGPSDataDriveFile, zlog, t, "gps trajectory"); err != nil {
		t.Fatalf("download gps data failed: %v", err)
	}
	if err := nkDownload(groundTruthDataFilepath, nkGroundTruthDriveFile, zlog, t, "ground truth"); err != nil {
		t.Fatalf("download ground truth failed: %v", err)
	}

	rtree := spatialindex.NewRtree()

	mg := da.InitializeMapMatchingGraph(g.NumberOfVertices())
	onlineMM := online.NewOnlineMapMatchMHTClient(mg, rtree, 8.33333, 8.3333, 0.0001, 5.0, 0.0000001, 0.04, 3, N)

	f, err := os.OpenFile(gpsDataFilepath, os.O_RDONLY, 0644)
	if err != nil {
		t.Fatalf("OpenFile(gps) failed: %v", err)
	}
	defer f.Close()
	br := bufio.NewReader(f)

	var (
		prevLat, prevLon float64
		prevTime         time.Time
		hasPrev          bool
		candidates       []*ma.Candidate
		speedMeanK       = 8.333
		speedStdK        = 8.333
		lastBearing      = 0.0
		k                = 1
	)
	mapMatchPointResult := make([]*da.MatchedGPSPoint, 0)
	avgRuntimePerGPSPoint := 0.0
	totalRuntime := 0.0
	nowDataset := time.Now()

	centerGeohash := uint64(0)

	tilingEngine := tiler.NewTilingEngine(g, zlog)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			t.Fatalf("read gps line failed: %v", err)
		}
		ff := util.Fields(line)
		if ff[0] == "Date" {
			continue
		}
		dateTime := ff[0] + " " + ff[1]
		lat, err := strconv.ParseFloat(ff[2], 64)
		if err != nil {
			t.Fatalf("parse lat failed: %v", err)
		}
		lon, err := strconv.ParseFloat(ff[3], 64)
		if err != nil {
			t.Fatalf("parse lon failed: %v", err)
		}
		curGPSTime, err := time.Parse("02-Jan-2006 15:04:05", dateTime)
		if err != nil {
			t.Fatalf("parse timestamp failed: %v", err)
		}

		deltaTime := 1.0
		speed := 8.333
		if hasPrev {
			deltaTime = curGPSTime.Sub(prevTime).Seconds()
			if util.Gt(deltaTime, 0) {
				dist := geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon)
				speed = util.KilometerToMeter(dist) / deltaTime
			}
		} else {
			hasPrev = true
		}
		prevLat, prevLon, prevTime = lat, lon, curGPSTime

		now := time.Now()
		curGPS := da.NewGPSPoint(lat, lon, curGPSTime, speed, deltaTime)

		currGeohash := geohash.EncodeIntWithPrecision(curGPS.Lat(), curGPS.Lon(), tiler.GeohashBits)
		if centerGeohash != currGeohash {

			rnCands := make([]*ma.Candidate, 0, len(candidates))
			for _, cand := range candidates {
				if cand == nil {
					continue
				}
				eId := cand.EdgeId()

				rnCands = append(rnCands, ma.NewCandidate(mg.GetRoadnetworkEdgeId(eId), cand.Weight(), cand.Length()))
			}

			tileFilepath := tilingEngine.GetTileFilePath(geohash.ConvertIntToString(currGeohash, tiler.GeohashPrecision))
			err = mg.RebuildMapMatchGraph(tileFilepath)
			if err != nil {
				if errors.Is(err, os.ErrNotExist) {
					centerGeohash = currGeohash
					candidates = candidates[:0]
					mg.Reset()
					rtree.Reset()
					continue
				}
				t.Fatal(err)
			}

			// Rebuild the R-tree with the new tile data
			rtree.Reset()
			rtree.BuildMapMatch(mg, zlog)

			onlineMM = online.NewOnlineMapMatchMHTClient(
				mg, rtree,
				8.33333,   // initialSpeedMean (m/s )
				8.3333,    // initialSpeedStd
				0.0001,    // posteriorThreshold
				5.0,       // gpsStd (meters)
				0.0000001, // lp
				0.04,      // lc (km ~40m search radius)
				3.0,       // accelerationStd
				N,
			)

			updatedCands := make([]*ma.Candidate, 0, len(rnCands))
			for _, snapshot := range rnCands {
				newMapMatchEdgeID, ok := mg.GetMapMatchEdgeId(snapshot.EdgeId())
				if !ok {
					continue
				}
				updatedCands = append(updatedCands, ma.NewCandidate(newMapMatchEdgeID, snapshot.Weight(), snapshot.Length()))
			}
			candidates = updatedCands

			centerGeohash = currGeohash
		}

		matchedPoint, nextCandidates, nextSpeedMeanK, nextSpeedStdK := onlineMM.OnlineMapMatch(curGPS, k, candidates, speedMeanK, speedStdK, lastBearing)

		candidates, speedMeanK, speedStdK = nextCandidates, nextSpeedMeanK, nextSpeedStdK
		k++
		lastBearing = matchedPoint.GetBearing()

		if matchedPoint.GetEdgeId() != da.INVALID_EDGE_ID {
			rnEdgeId := mg.GetRoadnetworkEdgeId(matchedPoint.GetEdgeId())
			matchedPoint.SetEdgeId(rnEdgeId)
		}
		mapMatchPointResult = append(mapMatchPointResult, matchedPoint)
		avgRuntimePerGPSPoint += float64(time.Since(now).Microseconds())

	}

	totalPoints := float64(k - 1)
	runtimeDataset := time.Since(nowDataset).Milliseconds()
	totalRuntime += float64(runtimeDataset)

	groundTruthFile, err := os.Open(groundTruthDataFilepath)
	if err != nil {
		t.Fatalf("open ground truth failed: %v", err)
	}
	defer groundTruthFile.Close()
	brGroundTruth := bufio.NewReader(groundTruthFile)

	traversedEdgesLength := make(map[int64]float64, 576)
	traversedEdges := make(map[int64]bool, 576)
	for {
		line, err := util.ReadLine(brGroundTruth)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			t.Fatalf("read ground truth line failed: %v", err)
		}
		ff := util.Fields(line)
		if ff[0] == "Edge" {
			continue
		}
		edgeID, err := strconv.ParseInt(ff[0], 10, 64)
		if err != nil {
			t.Fatalf("parse edge id failed: %v", err)
		}
		traversedEdgesLength[edgeID] = edgeLength[edgeID]
		traversedEdges[edgeID] = ff[1] == "1"
	}

	lengthOfCorrectRoute := 0.0
	for _, eLength := range traversedEdgesLength {
		lengthOfCorrectRoute += eLength
	}
	lengthOfErrAdded := 0.0
	lengthOfErrSubtracted := 0.0
	numOfCorrectMatchedRoads := 0.0
	numberOfRoadsOfMatchedTrips := 0.0
	matchedEdgeSet := make(map[int64]float64)
	matchedCoords := make([]da.Coordinate, 0, len(mapMatchPointResult))

	for _, point := range mapMatchPointResult {
		curMatchedEID := point.GetEdgeId()
		if curMatchedEID == da.INVALID_EDGE_ID {
			continue
		}
		matchedDataEID := g.GetOsmWayId(curMatchedEID)
		curMatchedEdge := g.GetOutEdge(curMatchedEID)
		matchedEdgeSet[matchedDataEID] = curMatchedEdge.GetLength()
		_, inGroundTruthReversed := traversedEdges[matchedDataEID]
		_, inGroundTruthForward := traversedEdges[matchedDataEID/2]
		if inGroundTruthForward || inGroundTruthReversed {
			numOfCorrectMatchedRoads++
		}
		matchedCoords = append(matchedCoords, point.GetMatchedCoord())
		numberOfRoadsOfMatchedTrips++
	}

	for matchedDataEID, eLength := range matchedEdgeSet {
		_, inGroundTruthReversed := traversedEdges[matchedDataEID]
		_, inGroundTruthForward := traversedEdges[matchedDataEID/2]
		if !inGroundTruthForward && !inGroundTruthReversed {
			lengthOfErrAdded += eLength
		}
	}

	for eID := range traversedEdges {
		eLengthForward, inMapMatchResultForward := matchedEdgeSet[eID]
		eLengthReversed, inMapMatchResultReversed := matchedEdgeSet[eID*2]
		eLength := util.MaxFloat(eLengthForward, eLengthReversed)
		if !inMapMatchResultForward && !inMapMatchResultReversed {
			lengthOfErrSubtracted += eLength
		}
	}

	avgRuntimePerGPSPoint /= float64(k - 1)
	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	crp := numOfCorrectMatchedRoads / numberOfRoadsOfMatchedTrips
	t.Logf("Route Mismatch Fraction (RMF): %v", rmf)
	t.Logf("Correct Road Percentage (CRP) or accuracy: %v", crp)
	t.Logf("avg runtime per gpt point: %v microseconds/gps point", avgRuntimePerGPSPoint)
	t.Logf("matching efficiency: %v points/ms", totalPoints/totalRuntime)

	polyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
	polyFile, err := os.Create(filepath.Join(workingDir, "polyline.txt"))
	if err != nil {
		t.Fatalf("create polyline file failed: %v", err)
	}
	defer polyFile.Close()
	if _, err := polyFile.Write([]byte(polyline)); err != nil {
		t.Fatalf("write polyline file failed: %v", err)
	}

	if crp < nkExpectedMinAccuracy {
		t.Fatalf("accuracy below threshold: got %v, want >= %v", crp, nkExpectedMinAccuracy)
	}
	if rmf > nkExpectedMaxRMF {
		t.Fatalf("RMF above threshold: got %v, want <= %v", rmf, nkExpectedMaxRMF)
	}
}

package onlinemapmatching

import (
	"bufio"
	"bytes"
	"encoding/csv"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"testing"
	"time"

	evalutil "github.com/lintang-b-s/Navigatorx/eval/crp_alt/online_map_matching"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/offline"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

const (
	ohmmGisCupRoadNetworkDriveFile = "https://drive.google.com/uc?export=download&id=1RS_3rt48WR1l-mJUqjKV9k7SusLfPgyc"
	ohmmMelbourneDatasetDriveFile  = "https://drive.google.com/uc?export=download&id=1WY0BPpu1M-e7grP33B_7BFrHlkj6kd_H"

	ohmmExpectedMinGisCupAccuracy = 0.90
	ohmmExpectedMaxGisCupRMF      = 0.15
	ohmmExpectedMinMelbAccuracy   = 0.85
	ohmmExpectedMaxMelbRMF        = 0.15
	ohmmExpectedMinHHCDFAt014     = 0.92
	ohmmExpectedMinHHCDFAt040     = 0.99
)

var (
	ohmmInitOnce sync.Once
	ohmmInitErr  error
)

type ohmmEdgeGeometry struct {
	length   float64
	roadType pkg.OsmHighwayType
	coords   []da.Coordinate
}

type ohmmGisCupRoadNetworkPaths struct {
	nodesFilePath        string
	edgesFilePath        string
	edgeGeometryFilePath string
}

type ohmmGisCupTrackPoint struct {
	timeSec float64
	lat     float64
	lon     float64
}

type ohmmGisCupCase struct {
	id             string
	inputFilePath  string
	outputFilePath string
}

type ohmmMelbourneVertex struct {
	id    int64
	osmID int64
	lon   float64
	lat   float64
}

type ohmmMelbourneEdge struct {
	id       int64
	startID  int64
	endID    int64
	distance float64 // meters
}

type ohmmMelbourneStreet struct {
	id       int64
	startID  int64
	startLon float64
	startLat float64
	endID    int64
	endLon   float64
	endLat   float64
	distance float64
}

type ohmmMelbourneGPSPoint struct {
	timestamp int64
	lat       float64
	lon       float64
	t         time.Time
}

func ohmmEnsureConfig(t *testing.T) string {
	t.Helper()

	var workingDir string
	ohmmInitOnce.Do(func() {
		var err error
		workingDir, err = config.FindProjectWorkingDir()
		if err != nil {
			ohmmInitErr = err
			return
		}
		err = config.ReadConfig(workingDir)
		if err != nil {
			ohmmInitErr = err
			return
		}
		vehicleType := viper.GetString("vehicle_type")
		pkg.VehicleType = pkg.GetVehicleType(vehicleType)
		pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
		pkg.IsVehicleEnabled = pkg.GetIsVehicle()
		pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()
	})
	if ohmmInitErr != nil {
		t.Fatalf("failed init config: %v", ohmmInitErr)
	}
	if workingDir == "" {
		var err error
		workingDir, err = config.FindProjectWorkingDir()
		if err != nil {
			t.Fatalf("FindProjectWorkingDir() failed: %v", err)
		}
	}
	return workingDir
}

func ohmmEngineFilesExist(paths ...string) bool {
	for _, path := range paths {
		if _, err := os.Stat(path); err != nil {
			return false
		}
	}
	return true
}

func ohmmProjectPath(workingDir, path string) string {
	return filepath.Join(workingDir, strings.TrimPrefix(path, "./"))
}

func ohmmPrepareCRPFiles(t *testing.T, graph *da.Graph, timeFunction *costfunction.TimeFunction[int32], edgeInfoIDs [][]da.Index, logger *zap.Logger, partitionSizes []int,
	mlpFile, graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile string) *engine.Engine[int32] {
	t.Helper()

	for _, path := range []string{mlpFile, graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile} {
		if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
			t.Fatalf("create CRP output directory failed for %s: %v", path, err)
		}
	}

	ps := make([]int, len(partitionSizes))
	for i, pow := range partitionSizes {
		ps[i] = 1 << pow
	}

	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 1, graph, logger, false, false)
	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(mlpFile); err != nil {
		t.Fatalf("save mlp failed: %v", err)
	}

	mlp := da.NewPlainMLP()
	if err := mlp.ReadMlpFile(mlpFile); err != nil {
		t.Fatalf("read mlp failed: %v", err)
	}
	prep := prepo.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIDs)
	if err := prep.PreProcessing(true); err != nil {
		t.Fatalf("preprocessing failed: %v", err)
	}
	cust := customizer.NewCustomizer[int32](graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)
	if _, err := cust.Customize(); err != nil {
		t.Fatalf("customize failed: %v", err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		t.Fatalf("new engine failed: %v", err)
	}
	return re
}

func ohmmLocateGisCupRoadNetwork(root string) (ohmmGisCupRoadNetworkPaths, error) {
	var paths ohmmGisCupRoadNetworkPaths
	err := filepath.WalkDir(root, func(path string, entry os.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if entry.IsDir() {
			return nil
		}

		switch strings.ToLower(entry.Name()) {
		case "wa_nodes.txt":
			paths.nodesFilePath = path
		case "wa_edges.txt":
			paths.edgesFilePath = path
		case "wa_edgegeometry.txt":
			paths.edgeGeometryFilePath = path
		}
		return nil
	})
	if err != nil {
		return ohmmGisCupRoadNetworkPaths{}, err
	}
	if paths.nodesFilePath == "" || paths.edgesFilePath == "" || paths.edgeGeometryFilePath == "" {
		return ohmmGisCupRoadNetworkPaths{}, fmt.Errorf("missing WA road network files under %s", root)
	}
	return paths, nil
}

func ohmmEnsureGisCupRoadNetwork(t *testing.T, workingDir string, logger *zap.Logger) ohmmGisCupRoadNetworkPaths {
	t.Helper()

	extractDir := filepath.Join(workingDir, "data/eval/mapmatching/giscup/road_network")
	if paths, err := ohmmLocateGisCupRoadNetwork(extractDir); err == nil {
		return paths
	}

	zipPath := filepath.Join(workingDir, "data/eval/mapmatching/giscup/road_network.zip")
	if err := evalutil.Download(zipPath, ohmmGisCupRoadNetworkDriveFile, logger, "GIS Cup road network"); err != nil {
		t.Fatalf("download GIS Cup road network failed: %v", err)
	}
	if err := evalutil.ExtractZip(zipPath, extractDir); err != nil {
		t.Fatalf("extract GIS Cup road network failed: %v", err)
	}

	paths, err := ohmmLocateGisCupRoadNetwork(extractDir)
	if err != nil {
		t.Fatalf("locate GIS Cup road network failed: %v", err)
	}
	return paths
}

// https://web.archive.org/web/20130127211936/http://depts.washington.edu/giscup/home
func ohmmReadGisCupNodes(nodesFilePath string) ([]da.Coordinate, map[int64]uint32, map[int64]osmparser.NodeCoord, map[da.Index]int64, error) {
	f, err := os.OpenFile(nodesFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	defer f.Close()

	nodeCoords := make([]da.Coordinate, 0)
	nodeIDToIndex := make(map[int64]uint32)
	acceptedNodeMap := make(map[int64]osmparser.NodeCoord)
	nodeToOsmID := make(map[da.Index]int64)

	br := bufio.NewReader(f)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, nil, nil, nil, err
		}
		fields := util.Fields(line)
		if len(fields) == 0 {
			continue
		}
		if len(fields) < 3 {
			return nil, nil, nil, nil, fmt.Errorf("invalid node line %q", line)
		}

		nodeID, err := util.ParseTextInt64(fields[0])
		if err != nil {
			return nil, nil, nil, nil, err
		}
		lat, err := util.ParseTextFloat64(fields[1])
		if err != nil {
			return nil, nil, nil, nil, err
		}
		lon, err := util.ParseTextFloat64(fields[2])
		if err != nil {
			return nil, nil, nil, nil, err
		}

		internalIndex := uint32(len(nodeCoords))
		nodeCoords = append(nodeCoords, da.NewCoordinate(lat, lon))
		nodeIDToIndex[nodeID] = internalIndex
		acceptedNodeMap[nodeID] = osmparser.NewNodeCoord(lat, lon)
		nodeToOsmID[da.Index(internalIndex)] = nodeID
	}
	return nodeCoords, nodeIDToIndex, acceptedNodeMap, nodeToOsmID, nil
}

// https://web.archive.org/web/20120528201458/http://depts.washington.edu/giscup/roadnetwork
func ohmmReadGisCupEdgeGeometry(edgeGeometryFilePath string) (map[int64]ohmmEdgeGeometry, error) {
	f, err := os.OpenFile(edgeGeometryFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	geometries := make(map[int64]ohmmEdgeGeometry)
	br := bufio.NewReader(f)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		lineBytes := bytes.TrimSpace([]byte(line))
		if len(line) == 0 {
			continue
		}

		fields := bytes.Split(lineBytes, []byte("^"))
		if len(fields) < 8 {
			return nil, fmt.Errorf("invalid edge geometry line %q", line)
		}
		edgeID, err := util.ParseTextInt64(string(bytes.TrimSpace(fields[0])))
		if err != nil {
			return nil, err
		}
		length, err := util.ParseTextFloat64(string(bytes.TrimSpace(fields[3])))
		if err != nil {
			return nil, err
		}
		coordFields := fields[4:]
		if len(coordFields)%2 != 0 {
			return nil, fmt.Errorf("uneven edge geometry coordinates for %d", edgeID)
		}

		coords := make([]da.Coordinate, 0, len(coordFields)/2)
		for i := 0; i < len(coordFields); i += 2 {
			lat, err := util.ParseTextFloat64(string(bytes.TrimSpace(coordFields[i])))
			if err != nil {
				return nil, err
			}
			lon, err := util.ParseTextFloat64(string(bytes.TrimSpace(coordFields[i+1])))
			if err != nil {
				return nil, err
			}
			coords = append(coords, da.NewCoordinate(lat, lon))
		}
		roadType := pkg.GetHighwayType(string(bytes.TrimSpace(fields[2])))
		if roadType == pkg.UNKNOWN {
			roadType = pkg.ROAD
		}
		geometries[edgeID] = ohmmEdgeGeometry{length: length, roadType: roadType, coords: coords}
	}
	return geometries, nil
}

func ohmmFallbackGeometry(edgeID int64, from, to uint32, nodeCoords []da.Coordinate) ohmmEdgeGeometry {
	fromCoord := nodeCoords[from]
	toCoord := nodeCoords[to]
	length := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
		fromCoord.GetLat(), fromCoord.GetLon(),
		toCoord.GetLat(), toCoord.GetLon(),
	))
	return ohmmEdgeGeometry{length: length, roadType: pkg.ROAD, coords: []da.Coordinate{fromCoord, toCoord}}
}

// https://web.archive.org/web/20120528201458/http://depts.washington.edu/giscup/roadnetwork
func ohmmBuildGraphFromGisCupFiles(paths ohmmGisCupRoadNetworkPaths) (*da.Graph, *costfunction.TimeFunction[int32], [][]da.Index, map[int64]float64, error) {
	nodeCoords, nodeIDToIndex, acceptedNodeMap, nodeToOsmID, err := ohmmReadGisCupNodes(paths.nodesFilePath)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	edgeGeometries, err := ohmmReadGisCupEdgeGeometry(paths.edgeGeometryFilePath)
	if err != nil {
		return nil, nil, nil, nil, err
	}

	f, err := os.OpenFile(paths.edgesFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	defer f.Close()

	graphStorage := da.NewGraphStorage(54)
	graphEdges := make([]osmparser.Edge[int32], 0)
	edgeLengths := make(map[int64]float64)

	br := bufio.NewReader(f)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, nil, nil, nil, err
		}
		fields := util.Fields(line)
		if len(fields) == 0 {
			continue
		}
		if len(fields) < 4 {
			return nil, nil, nil, nil, fmt.Errorf("invalid edge line %q", line)
		}

		edgeID, err := util.ParseTextInt64(fields[0])
		if err != nil {
			return nil, nil, nil, nil, err
		}
		fromNodeID, err := util.ParseTextInt64(fields[1])
		if err != nil {
			return nil, nil, nil, nil, err
		}
		toNodeID, err := util.ParseTextInt64(fields[2])
		if err != nil {
			return nil, nil, nil, nil, err
		}
		cost, err := util.ParseTextFloat64(fields[3])
		if err != nil {
			return nil, nil, nil, nil, err
		}

		fromIndex, ok := nodeIDToIndex[fromNodeID]
		if !ok {
			return nil, nil, nil, nil, fmt.Errorf("missing from node %d", fromNodeID)
		}
		toIndex, ok := nodeIDToIndex[toNodeID]
		if !ok {
			return nil, nil, nil, nil, fmt.Errorf("missing to node %d", toNodeID)
		}
		if fromIndex == toIndex {
			continue
		}

		geometry, ok := edgeGeometries[edgeID]
		if !ok || len(geometry.coords) < 2 || util.Le(geometry.length, 0) {
			geometry = ohmmFallbackGeometry(edgeID, fromIndex, toIndex, nodeCoords)
		}

		startPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendOsmNodePoints(geometry.coords)
		endPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendEdgeMetadata(edgeID, da.Index(startPointsIndex), da.Index(endPointsIndex), 0, geometry.roadType, 0, 1)

		graphEdge := osmparser.NewFixedEdge(
			fromIndex, toIndex, cost, geometry.length, false, geometry.roadType,
		)

		graphEdge.SetFromOSMId(uint64(fromNodeID))
		graphEdge.SetToOSMId(uint64(toNodeID))
		graphEdges = append(graphEdges, graphEdge)
		edgeLengths[edgeID] = geometry.length
	}

	op := osmparser.NewOSMParserV2[int32]()
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmID)
	graph, timeFunction, edgeInfoIDs := op.BuildGraph(graphEdges, graphStorage, uint32(len(nodeCoords)), true)
	graph.SetGraphStorage(graphStorage)
	return graph, timeFunction, edgeInfoIDs, edgeLengths, nil
}

// https://web.archive.org/web/20120528201458/http://depts.washington.edu/giscup/roadnetwork
func ohmmBuildGisCupCRPGraph(t *testing.T, workingDir string) (*engine.Engine[int32], *da.Graph, *zap.Logger, map[int64]float64) {
	t.Helper()

	pkg.RegionName = "giscup"

	logger, err := log.New()
	if err != nil {
		t.Fatalf("log.New failed: %v", err)
	}
	paths := ohmmEnsureGisCupRoadNetwork(t, workingDir, logger)
	graph, timeFunction, edgeInfoIDs, edgeLengths, err := ohmmBuildGraphFromGisCupFiles(paths)
	if err != nil {
		t.Fatalf("build GIS Cup graph failed: %v", err)
	}

	graphFile := filepath.Join(workingDir, "data/eval/mapmatching/giscup/offline_hmm_original_giscup.ngraph")
	overlayGraphFile := filepath.Join(workingDir, "data/eval/mapmatching/giscup/offline_hmm_overlay_graph_giscup.ngraph")
	metricsFile := filepath.Join(workingDir, "data/eval/mapmatching/giscup/offline_hmm_metrics_giscup.nmt")
	mlpFile := filepath.Join(workingDir, "data/eval/mapmatching/giscup/offline_hmm_mlp_giscup.mlp")
	landmarkFile := filepath.Join(workingDir, "data/eval/mapmatching/giscup/offline_hmm_landmark_giscup.nlm")
	timeFunctionFile := filepath.Join(workingDir, "data/eval/mapmatching/giscup/offline_hmm_timefunction_giscup.ntf")

	var re *engine.Engine[int32]
	if ohmmEngineFilesExist(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile) {
		re, err = engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
		if err != nil {
			t.Fatalf("load GIS Cup engine failed: %v", err)
		}
	} else {
		re = ohmmPrepareCRPFiles(t, graph, timeFunction, edgeInfoIDs, logger, []int{8, 11, 14, 16},
			mlpFile, graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile)
	}
	return re, re.GetRoutingEngine().GetGraph(), logger, edgeLengths
}

func ohmmReadGisCupTrack(trackPath string) ([]ohmmGisCupTrackPoint, error) {
	f, err := os.OpenFile(trackPath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	reader := csv.NewReader(f)
	reader.FieldsPerRecord = -1
	reader.TrimLeadingSpace = true
	points := make([]ohmmGisCupTrackPoint, 0)
	for {
		record, err := reader.Read()
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		if len(record) < 3 || strings.TrimSpace(record[0]) == "" {
			continue
		}

		timeSec, err := util.ParseTextFloat64(strings.TrimSpace(record[0]))
		if err != nil {
			return nil, err
		}
		lat, err := util.ParseTextFloat64(strings.TrimSpace(record[1]))
		if err != nil {
			return nil, err
		}
		lon, err := util.ParseTextFloat64(strings.TrimSpace(record[2]))
		if err != nil {
			return nil, err
		}
		points = append(points, ohmmGisCupTrackPoint{timeSec: timeSec, lat: lat, lon: lon})
	}
	return points, nil
}

func ohmmReadGisCupGroundTruth(outputPath string) ([]int64, error) {
	f, err := os.OpenFile(outputPath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	reader := csv.NewReader(f)
	reader.FieldsPerRecord = -1
	reader.TrimLeadingSpace = true
	edgeIDs := make([]int64, 0)
	for {
		record, err := reader.Read()
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		if len(record) < 2 || strings.TrimSpace(record[0]) == "" {
			continue
		}
		edgeID, err := util.ParseTextInt64(strings.TrimSpace(record[1]))
		if err != nil {
			return nil, err
		}
		edgeIDs = append(edgeIDs, edgeID)
	}
	return edgeIDs, nil
}

func ohmmListGisCupCases(workingDir string) ([]ohmmGisCupCase, error) {
	inputDir := filepath.Join(workingDir, "data/eval/mapmatching/GisContestTrainingData/input")
	outputDir := filepath.Join(workingDir, "data/eval/mapmatching/GisContestTrainingData/output")
	matches, err := filepath.Glob(filepath.Join(inputDir, "input_*.txt"))
	if err != nil {
		return nil, err
	}
	sort.Strings(matches)

	cases := make([]ohmmGisCupCase, 0, len(matches))
	for _, inputPath := range matches {
		name := filepath.Base(inputPath)
		id := strings.TrimSuffix(strings.TrimPrefix(name, "input_"), ".txt")
		outputPath := filepath.Join(outputDir, "output_"+id+".txt")
		if _, err := os.Stat(outputPath); err != nil {
			return nil, err
		}
		cases = append(cases, ohmmGisCupCase{id: id, inputFilePath: inputPath, outputFilePath: outputPath})
	}
	return cases, nil
}

func ohmmGisCupGPSTrajectory(points []ohmmGisCupTrackPoint) []*da.GPSPoint {
	gpsTraj := make([]*da.GPSPoint, 0, len(points))
	var prev ohmmGisCupTrackPoint
	for i, point := range points {
		deltaTime := 1.0
		speed := 8.333
		if i > 0 {
			deltaTime = point.timeSec - prev.timeSec
			if util.Gt(deltaTime, 0) {
				dist := geo.CalculateGreatCircleDistance(prev.lat, prev.lon, point.lat, point.lon)
				speed = util.KilometerToMeter(dist) / deltaTime
			}
		}
		sec, frac := math.Modf(point.timeSec)
		gpsTraj = append(gpsTraj, da.NewGPSPoint(point.lat, point.lon, time.Unix(int64(sec), int64(frac*1e9)), speed, deltaTime))
		prev = point
	}
	return gpsTraj
}

func ohmmComputeEdgeSetMetrics(graph *da.Graph, groundTruthEdgeIDs []int64, matchedPoints []*da.MatchedGPSPoint,
	edgeLengths map[int64]float64) (float64, float64) {
	groundTruthEdgeIDs = ohmmGisCupGroundTruthForMatchedObservations(groundTruthEdgeIDs, matchedPoints)

	groundTruthSet := make(map[int64]bool)
	lengthOfCorrectRoute := 0.0
	for _, edgeID := range groundTruthEdgeIDs {
		if groundTruthSet[edgeID] {
			continue
		}
		groundTruthSet[edgeID] = true
		lengthOfCorrectRoute += edgeLengths[edgeID]
	}

	matchedEdgeSet := make(map[int64]float64)
	for _, point := range matchedPoints {
		if point.GetEdgeId() == da.INVALID_EDGE_ID {
			continue
		}
		dataEdgeID := graph.GetOsmWayId(point.GetEdgeId())
		length, ok := edgeLengths[dataEdgeID]
		if !ok {
			continue
		}
		matchedEdgeSet[dataEdgeID] = length
	}

	correctMatched := 0.0
	for edgeID := range matchedEdgeSet {
		if groundTruthSet[edgeID] {
			correctMatched++
		}
	}

	crp := 0.0
	if len(matchedEdgeSet) > 0 {
		crp = correctMatched / float64(len(matchedEdgeSet))
	}
	if util.Eq(lengthOfCorrectRoute, 0) {
		return crp, math.Inf(1)
	}

	lengthOfErrAdded := 0.0
	for edgeID, length := range matchedEdgeSet {
		if !groundTruthSet[edgeID] {
			lengthOfErrAdded += length
		}
	}

	lengthOfErrSubtracted := 0.0
	for edgeID := range groundTruthSet {
		if _, ok := matchedEdgeSet[edgeID]; !ok {
			lengthOfErrSubtracted += edgeLengths[edgeID]
		}
	}
	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	return crp, rmf
}

func ohmmGisCupGroundTruthForMatchedObservations(groundTruthEdgeIDs []int64, matchedPoints []*da.MatchedGPSPoint) []int64 {
	if len(groundTruthEdgeIDs) == 0 || len(matchedPoints) == 0 {
		return groundTruthEdgeIDs
	}

	filtered := make([]int64, 0, len(matchedPoints))
	seenObservationIds := make(map[uint32]bool, len(matchedPoints))
	for _, matchedPoint := range matchedPoints {
		obsID := matchedPoint.GetObservationId()
		if seenObservationIds[obsID] || int(obsID) >= len(groundTruthEdgeIDs) {
			continue
		}
		seenObservationIds[obsID] = true
		filtered = append(filtered, groundTruthEdgeIDs[obsID])
	}

	if len(filtered) == 0 {
		return groundTruthEdgeIDs
	}
	return filtered
}

func ohmmParseMelbourneVertexFile(filePath string) ([]ohmmMelbourneVertex, error) {
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
	expectedVertices, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, err
	}

	vertices := make([]ohmmMelbourneVertex, 0, expectedVertices)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 4 {
			return nil, fmt.Errorf("invalid vertex line %q", line)
		}
		id, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, err
		}
		osmID, err := util.ParseTextInt64(ff[1])
		if err != nil {
			return nil, err
		}
		lon, err := util.ParseTextFloat64(ff[2])
		if err != nil {
			return nil, err
		}
		lat, err := util.ParseTextFloat64(ff[3])
		if err != nil {
			return nil, err
		}
		vertices = append(vertices, ohmmMelbourneVertex{id: id, osmID: osmID, lon: lon, lat: lat})
	}
	return vertices, nil
}

func ohmmParseMelbourneEdgesFile(filePath string) ([]ohmmMelbourneEdge, error) {
	f, err := os.OpenFile(filePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)
	if _, err := util.ReadLine(br); err != nil {
		return nil, err
	}
	line, err := util.ReadLine(br)
	if err != nil {
		return nil, err
	}
	expectedEdges, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, err
	}

	edges := make([]ohmmMelbourneEdge, 0, expectedEdges)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 4 {
			return nil, fmt.Errorf("invalid edge line %q", line)
		}
		id, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, err
		}
		startID, err := util.ParseTextInt64(ff[1])
		if err != nil {
			return nil, err
		}
		endID, err := util.ParseTextInt64(ff[2])
		if err != nil {
			return nil, err
		}
		dist, err := util.ParseTextFloat64(ff[3])
		if err != nil {
			return nil, err
		}
		edges = append(edges, ohmmMelbourneEdge{id: id, startID: startID, endID: endID, distance: dist})
	}
	return edges, nil
}

func ohmmParseMelbourneStreetsFile(filePath string) (map[int64]ohmmMelbourneStreet, error) {
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
		return nil, err
	}

	streetByID := make(map[int64]ohmmMelbourneStreet, expectedEdges)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 8 {
			return nil, fmt.Errorf("invalid street line %q", line)
		}
		id, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, err
		}
		startID, err := util.ParseTextInt64(ff[1])
		if err != nil {
			return nil, err
		}
		startLon, err := util.ParseTextFloat64(ff[2])
		if err != nil {
			return nil, err
		}
		startLat, err := util.ParseTextFloat64(ff[3])
		if err != nil {
			return nil, err
		}
		endID, err := util.ParseTextInt64(ff[4])
		if err != nil {
			return nil, err
		}
		endLon, err := util.ParseTextFloat64(ff[5])
		if err != nil {
			return nil, err
		}
		endLat, err := util.ParseTextFloat64(ff[6])
		if err != nil {
			return nil, err
		}
		dist, err := util.ParseTextFloat64(ff[7])
		if err != nil {
			return nil, err
		}
		streetByID[id] = ohmmMelbourneStreet{
			id: id, startID: startID, startLon: startLon, startLat: startLat,
			endID: endID, endLon: endLon, endLat: endLat, distance: dist,
		}
	}
	return streetByID, nil
}

// https://web.archive.org/web/20170301001019/https://people.eng.unimelb.edu.au/henli/projects/map-matching/
func ohmmBuildMelbourneCRPGraph(t *testing.T, workingDir string) (*engine.Engine[int32], *da.Graph, *zap.Logger, map[da.Index]int64, map[int64]float64) {
	t.Helper()

	pkg.RegionName = "hengfengli"

	logger, err := log.New()
	if err != nil {
		t.Fatalf("log.New failed: %v", err)
	}

	vertexPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/complete-osm-map/vertex.txt")
	edgesPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/complete-osm-map/edges.txt")
	streetsPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/complete-osm-map/streets.txt")

	vertices, err := ohmmParseMelbourneVertexFile(vertexPath)
	if err != nil {
		t.Fatalf("parse Melbourne vertices failed: %v", err)
	}
	edges, err := ohmmParseMelbourneEdgesFile(edgesPath)
	if err != nil {
		t.Fatalf("parse Melbourne edges failed: %v", err)
	}
	streetByID, err := ohmmParseMelbourneStreetsFile(streetsPath)
	if err != nil {
		t.Fatalf("parse Melbourne streets failed: %v", err)
	}

	graphStorage := da.NewGraphStorage(54)
	graphEdges := make([]osmparser.Edge[int32], 0, len(edges))
	for _, e := range edges {
		st, ok := streetByID[e.id]
		if !ok {
			t.Fatalf("Melbourne edge id %d missing from streets.txt", e.id)
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
	graph, timeFunction, edgeInfoIDs := op.BuildGraph(graphEdges, graphStorage, uint32(len(vertices)), true)
	graph.SetGraphStorage(graphStorage)

	graphFile := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_original_hl.ngraph")
	overlayGraphFile := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_overlay_graph_hl.ngraph")
	metricsFile := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_metrics_hl.nmt")
	mlpFile := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_mlp_hl.mlp")
	landmarkFile := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_landmark_hl.nlm")
	timeFunctionFile := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_timefunction_hl.ntf")

	var re *engine.Engine[int32]
	if ohmmEngineFilesExist(graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile) {
		re, err = engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
		if err != nil {
			t.Fatalf("load Melbourne engine failed: %v", err)
		}
	} else {
		re = ohmmPrepareCRPFiles(t, graph, timeFunction, edgeInfoIDs, logger, []int{8, 11, 13, 14, 15},
			mlpFile, graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile)
	}

	loadedGraph := re.GetRoutingEngine().GetGraph()
	graphEdgeIDToMelbourneEdgeID := make(map[da.Index]int64, loadedGraph.NumberOfEdges())
	loadedGraph.ForOutEdges(func(exitPoint, head da.Index, tail, entryId, entryPoint da.Index, percentage float64, eId da.Index) {
		graphEdgeIDToMelbourneEdgeID[eId] = loadedGraph.GetOsmWayId(eId)
	})

	edgeLengthByID := make(map[int64]float64, len(streetByID))
	for edgeID, street := range streetByID {
		edgeLengthByID[edgeID] = street.distance
	}
	return re, loadedGraph, logger, graphEdgeIDToMelbourneEdgeID, edgeLengthByID
}

func ohmmReadMelbourneGPSTrack(filePath string) ([]ohmmMelbourneGPSPoint, error) {
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
		return nil, err
	}

	points := make([]ohmmMelbourneGPSPoint, 0, expectedPoints)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 3 {
			continue
		}
		timestamp, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, err
		}
		lat, err := util.ParseTextFloat64(ff[1])
		if err != nil {
			return nil, err
		}
		lon, err := util.ParseTextFloat64(ff[2])
		if err != nil {
			return nil, err
		}
		points = append(points, ohmmMelbourneGPSPoint{timestamp: timestamp, lat: lat, lon: lon, t: time.Unix(timestamp, 0)})
	}
	return points, nil
}

func ohmmReadMelbourneGroundTruthEdges(filePath string) ([]int64, error) {
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
		return nil, err
	}

	edgeIDs := make([]int64, 0, expectedEdges)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) == 0 {
			continue
		}
		edgeID, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, err
		}
		edgeIDs = append(edgeIDs, edgeID)
	}
	return edgeIDs, nil
}

func ohmmMelbourneGPSTrajectory(points []ohmmMelbourneGPSPoint) []*da.GPSPoint {
	gpsTraj := make([]*da.GPSPoint, 0, len(points))
	var prev ohmmMelbourneGPSPoint
	for i, point := range points {
		deltaTime := 1.0
		speed := 8.333
		if i > 0 {
			deltaTime = point.t.Sub(prev.t).Seconds()
			if util.Gt(deltaTime, 0) {
				dist := geo.CalculateGreatCircleDistance(prev.lat, prev.lon, point.lat, point.lon)
				speed = util.KilometerToMeter(dist) / deltaTime
			}
		}
		gpsTraj = append(gpsTraj, da.NewGPSPoint(point.lat, point.lon, point.t, speed, deltaTime))
		prev = point
	}
	return gpsTraj
}

func ohmmComputeMelbourneMetrics(matchedPoints []*da.MatchedGPSPoint, groundTruthEdgeIDs []int64,
	graphEdgeIDToMelbourneEdgeID map[da.Index]int64, edgeLengthByID map[int64]float64) (float64, float64, error) {
	groundTruthEdgeSet := make(map[int64]float64, len(groundTruthEdgeIDs))
	for _, edgeID := range groundTruthEdgeIDs {
		edgeLength, ok := edgeLengthByID[edgeID]
		if !ok {
			return 0, 0, fmt.Errorf("ground truth edge id %d missing from streets.txt", edgeID)
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
	for _, matchedPoint := range matchedPoints {
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
			return 0, 0, fmt.Errorf("matched edge id %d missing from streets.txt", melbourneEdgeID)
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

	lengthOfErrAdded := 0.0
	for edgeID, edgeLength := range matchedEdgeSet {
		if _, inGroundTruth := groundTruthEdgeSet[edgeID]; !inGroundTruth {
			lengthOfErrAdded += edgeLength
		}
	}

	lengthOfErrSubtracted := 0.0
	for edgeID, edgeLength := range groundTruthEdgeSet {
		if _, inMatchedRoute := matchedEdgeSet[edgeID]; !inMatchedRoute {
			lengthOfErrSubtracted += edgeLength
		}
	}

	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	crp := numCorrectMatchedRoads / numberOfRoadsOfMatchedTrips
	return crp, rmf, nil
}

func ohmmWritePolyline(t *testing.T, filePath string, matchedPoints []*da.MatchedGPSPoint) {
	t.Helper()

	matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))
	for _, point := range matchedPoints {
		if point.GetEdgeId() == da.INVALID_EDGE_ID {
			continue
		}
		matchedCoords = append(matchedCoords, point.GetMatchedCoord())
	}
	if err := os.MkdirAll(filepath.Dir(filePath), 0755); err != nil {
		t.Fatalf("create polyline output dir failed: %v", err)
	}
	polyline := ""
	if len(matchedCoords) > 0 {
		polyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
	}
	if err := os.WriteFile(filePath, []byte(polyline), 0644); err != nil {
		t.Fatalf("write polyline failed: %v", err)
	}
}

func ohmmBuildHanwenHuCRPGraph(t *testing.T) (*engine.Engine[int32], *da.Graph, *zap.Logger) {
	t.Helper()

	logger, err := log.New()
	if err != nil {
		t.Fatalf("log.New failed: %v", err)
	}

	if ohmmEngineFilesExist(hhGraphFile, hhOverlayGraphFile, hhMetricsFile, hhLandmarkFile, hhTimeFunctionFile) {
		re, err := engine.NewEngine[int32](hhGraphFile, hhOverlayGraphFile, hhMetricsFile, hhLandmarkFile, hhTimeFunctionFile, logger)
		if err != nil {
			t.Fatalf("load Hanwen-Hu engine failed: %v", err)
		}
		return re, re.GetRoutingEngine().GetGraph(), logger
	}

	re, _, logger, _ := hhBuildCRPGraph(t)
	return re, re.GetRoutingEngine().GetGraph(), logger
}

func ohmmHanwenHuGPSTrajectory(t *testing.T, gpsTraj []map[string]string) []*da.GPSPoint {
	t.Helper()

	locatetime, err := util.ParseTextInt64(gpsTraj[0]["locatetime"])
	if err != nil {
		t.Fatalf("parse locatetime failed: %v", err)
	}
	startTime, err := hhUnixTimestampToTime(locatetime)
	if err != nil {
		t.Fatalf("convert unix time failed: %v", err)
	}

	result := make([]*da.GPSPoint, 0, len(gpsTraj))
	var (
		prevLat, prevLon float64
		hasPrev          bool
	)
	for i, gps := range gpsTraj {
		lat, err := util.ParseTextFloat64(gps["lat"])
		if err != nil {
			t.Fatalf("parse lat failed: %v", err)
		}
		lon, err := util.ParseTextFloat64(gps["lon"])
		if err != nil {
			t.Fatalf("parse lon failed: %v", err)
		}

		deltaTime := 2.0
		speed := 8.333
		if hasPrev {
			dist := geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon)
			speed = util.KilometerToMeter(dist) / deltaTime
		} else {
			hasPrev = true
		}
		prevLat, prevLon = lat, lon

		curGpsTime := startTime.Add(time.Duration(i) * 2 * time.Second)
		result = append(result, da.NewGPSPoint(lat, lon, curGpsTime, speed, deltaTime))
	}
	return result
}

func ohmmGroundTruthLength(t *testing.T, groundTruth []map[string]string) float64 {
	t.Helper()

	length := 0.0
	for j := 1; j < len(groundTruth); j++ {
		prevGt := groundTruth[j-1]
		prevLat, err := util.ParseTextFloat64(prevGt["lat"])
		if err != nil {
			t.Fatalf("parse gt prev lat failed: %v", err)
		}
		prevLon, err := util.ParseTextFloat64(prevGt["lon"])
		if err != nil {
			t.Fatalf("parse gt prev lon failed: %v", err)
		}
		gt := groundTruth[j]
		lat, err := util.ParseTextFloat64(gt["lat"])
		if err != nil {
			t.Fatalf("parse gt lat failed: %v", err)
		}
		lon, err := util.ParseTextFloat64(gt["lon"])
		if err != nil {
			t.Fatalf("parse gt lon failed: %v", err)
		}
		length += util.KilometerToMeter(geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon))
	}
	return length
}

func ohmmMatchedLength(matchedPoints []*da.MatchedGPSPoint) float64 {
	start := 0
	for start < len(matchedPoints) && matchedPoints[start].GetEdgeId() == da.INVALID_EDGE_ID {
		start++
	}
	if start >= len(matchedPoints) {
		return 0
	}

	prevMp := matchedPoints[start]
	length := 0.0
	for j := start + 1; j < len(matchedPoints); j++ {
		mp := matchedPoints[j]
		if mp.GetEdgeId() == da.INVALID_EDGE_ID {
			continue
		}
		length += util.KilometerToMeter(geo.CalculateGreatCircleDistance(
			prevMp.GetMatchedCoord().GetLat(), prevMp.GetMatchedCoord().GetLon(),
			mp.GetMatchedCoord().GetLat(), mp.GetMatchedCoord().GetLon(),
		))
		prevMp = mp
	}
	return length
}

func ohmmEmpiricalCDF(sortedValues []float64, x float64) float64 {
	if len(sortedValues) == 0 {
		return 0
	}
	count := sort.Search(len(sortedValues), func(i int) bool {
		return sortedValues[i] > x
	})
	return float64(count) / float64(len(sortedValues))
}

// go test ./tests/map_matching -run TestGisCupOfflineHMMMapMatching -v -timeout=0 -count=1
func TestGisCupOfflineHMMMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	re, graph, logger, edgeLengths := ohmmBuildGisCupCRPGraph(t, workingDir)

	cases, err := ohmmListGisCupCases(workingDir)
	if err != nil {
		t.Skipf("GIS Cup training data unavailable: %v", err)
	}
	if len(cases) == 0 {
		t.Skip("GIS Cup training data unavailable")
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)
	hmm := offline.NewHiddenMarkovModelMapMatching(graph, re.GetRoutingEngine(), rtree)

	totalCRP := 0.0
	totalRMF := 0.0
	totalPoints := 0
	for _, tc := range cases {
		points, err := ohmmReadGisCupTrack(tc.inputFilePath)
		if err != nil {
			t.Fatalf("read GIS Cup track %s failed: %v", tc.id, err)
		}
		groundTruthEdgeIDs, err := ohmmReadGisCupGroundTruth(tc.outputFilePath)
		if err != nil {
			t.Fatalf("read GIS Cup ground truth %s failed: %v", tc.id, err)
		}

		gpsTraj := ohmmGisCupGPSTrajectory(points)
		now := time.Now()
		matchedPoints := hmm.MapMatch(gpsTraj)
		avgRuntime := float64(time.Since(now).Milliseconds()) / float64(len(gpsTraj))
		if len(matchedPoints) == 0 {
			t.Fatalf("GIS Cup case %s produced no matched points", tc.id)
		}

		crp, rmf := ohmmComputeEdgeSetMetrics(graph, groundTruthEdgeIDs, matchedPoints, edgeLengths)
		t.Logf("GIS Cup case %s: CRP=%v RMF=%v matched=%d/%d avg_runtime=%v Milliseconds/gps point",
			tc.id, crp, rmf, len(matchedPoints), len(gpsTraj), avgRuntime)

		ohmmWritePolyline(t, filepath.Join(workingDir, "data/eval/mapmatching/GisContestTrainingData/polylines/offline_hmm_result_polyline_"+tc.id+".txt"), matchedPoints)
		totalCRP += crp
		totalRMF += rmf
		totalPoints += len(gpsTraj)
	}

	avgCRP := totalCRP / float64(len(cases))
	avgRMF := totalRMF / float64(len(cases))
	t.Logf("GIS Cup offline HMM aggregate: cases=%d points=%d avg_CRP=%v avg_RMF=%v", len(cases), totalPoints, avgCRP, avgRMF)
	if avgCRP < ohmmExpectedMinGisCupAccuracy {
		t.Fatalf("GIS Cup accuracy below threshold: got %v, want >= %v", avgCRP, ohmmExpectedMinGisCupAccuracy)
	}
	if avgRMF > ohmmExpectedMaxGisCupRMF {
		t.Fatalf("GIS Cup RMF above threshold: got %v, want <= %v", avgRMF, ohmmExpectedMaxGisCupRMF)
	}
}

// go test ./tests/map_matching -run TestHengfengLiOfflineHMMMapMatching -v -timeout=0 -count=1
func TestHengfengLiOfflineHMMMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	re, graph, logger, graphEdgeIDToMelbourneEdgeID, edgeLengthByID := ohmmBuildMelbourneCRPGraph(t, workingDir)

	gpsPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/gps_track.txt")
	groundTruthPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/groundtruth.txt")
	points, err := ohmmReadMelbourneGPSTrack(gpsPath)
	if err != nil {
		t.Fatalf("read Melbourne GPS track failed: %v", err)
	}
	groundTruthEdgeIDs, err := ohmmReadMelbourneGroundTruthEdges(groundTruthPath)
	if err != nil {
		t.Fatalf("read Melbourne ground truth failed: %v", err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)
	hmm := offline.NewHiddenMarkovModelMapMatching(graph, re.GetRoutingEngine(), rtree)
	gpsTraj := ohmmMelbourneGPSTrajectory(points)

	now := time.Now()
	matchedPoints := hmm.MapMatch(gpsTraj)
	avgRuntime := float64(time.Since(now).Milliseconds()) / float64(len(gpsTraj))
	if len(matchedPoints) == 0 {
		t.Fatalf("Hengfeng Li Melbourne trajectory produced no matched points")
	}

	crp, rmf, err := ohmmComputeMelbourneMetrics(matchedPoints, groundTruthEdgeIDs, graphEdgeIDToMelbourneEdgeID, edgeLengthByID)
	if err != nil {
		t.Fatalf("compute Melbourne metrics failed: %v", err)
	}
	t.Logf("Hengfeng Li offline HMM: CRP=%v RMF=%v matched=%d/%d avg_runtime=%v Milliseconds/gps point",
		crp, rmf, len(matchedPoints), len(gpsTraj), avgRuntime)

	ohmmWritePolyline(t, filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_result_polyline.txt"), matchedPoints)
	if crp < ohmmExpectedMinMelbAccuracy {
		t.Fatalf("Hengfeng Li accuracy below threshold: got %v, want >= %v", crp, ohmmExpectedMinMelbAccuracy)
	}
	if rmf > ohmmExpectedMaxMelbRMF {
		t.Fatalf("Hengfeng Li RMF above threshold: got %v, want <= %v", rmf, ohmmExpectedMaxMelbRMF)
	}
}

// go test ./tests/map_matching -run TestHanwenHuOfflineHMMMapMatching -v -timeout=0 -count=1
func TestHanwenHuOfflineHMMMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	re, graph, logger := ohmmBuildHanwenHuCRPGraph(t)

	shanghaiDataFilePath := ohmmProjectPath(workingDir, hhShanghaiDataFilePath)
	shanghaiTestDataPath := ohmmProjectPath(workingDir, hhShanghaiTestDataPath)
	shanghaiGroundTruthPath := ohmmProjectPath(workingDir, hhShanghaiGroundTruthPath)
	shanghaiPolylinesPath := ohmmProjectPath(workingDir, hhShanghaiPolylinesPath)

	if err := hhDownload(shanghaiDataFilePath, hhShanghaiDatasetDriveFile, logger, "shanghai dataset"); err != nil {
		t.Fatalf("download Shanghai dataset failed: %v", err)
	}
	gzFile, err := os.Open(shanghaiDataFilePath)
	if err != nil {
		t.Fatalf("open Shanghai tar.gz failed: %v", err)
	}
	defer gzFile.Close()
	if _, err := os.Stat(shanghaiTestDataPath); err != nil {
		if err := hhExtractTarGz(gzFile, filepath.Join(workingDir, "data/eval/mapmatching")); err != nil {
			t.Fatalf("extract Shanghai tar.gz failed: %v", err)
		}
	}
	if _, err := os.Stat(shanghaiTestDataPath); err != nil {
		t.Fatalf("extract Shanghai tar.gz failed: %v", err)
	}

	gpsTrajectories, err := hhReadAllCSVInDir(shanghaiTestDataPath)
	if err != nil {
		t.Fatalf("read Hanwen-Hu trajectories failed: %v", err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)
	hmm := offline.NewHiddenMarkovModelMapMatching(graph, re.GetRoutingEngine(), rtree)

	trajectoryNames := make([]string, 0, len(gpsTrajectories))
	for trajName := range gpsTrajectories {
		trajectoryNames = append(trajectoryNames, trajName)
	}
	sort.Strings(trajectoryNames)

	matchingErrors := make([]float64, 0, len(trajectoryNames))
	totalPoints := 0
	totalRuntime := 0.0
	avgRuntimePerGpsPointAll := 0.0
	for _, trajName := range trajectoryNames {
		gpsTraj := gpsTrajectories[trajName]
		nowDataset := time.Now()
		gpsPoints := ohmmHanwenHuGPSTrajectory(t, gpsTraj)

		now := time.Now()
		matchedPoints := hmm.MapMatch(gpsPoints)
		avgRuntimePerGpsPoint := float64(time.Since(now).Milliseconds()) / float64(len(gpsPoints))
		if len(matchedPoints) == 0 {
			t.Fatalf("Hanwen-Hu trajectory %s produced no matched points", trajName)
		}

		trackID := hhTrackIDFromName(trajName)
		resultPolylinePath := filepath.Join(shanghaiPolylinesPath, fmt.Sprintf("offline_hmm_result_polyline_%s.txt", trackID))
		ohmmWritePolyline(t, resultPolylinePath, matchedPoints)

		groundTruth, err := hhReadCSV(filepath.Join(shanghaiGroundTruthPath, trajName))
		if err != nil {
			t.Fatalf("read Hanwen-Hu ground truth failed for %s: %v", trajName, err)
		}
		groundTruthLength := ohmmGroundTruthLength(t, groundTruth)
		matchLength := ohmmMatchedLength(matchedPoints)
		if util.Le(groundTruthLength, 0) {
			t.Fatalf("Hanwen-Hu trajectory %s has zero ground truth length", trajName)
		}

		matchingError := math.Abs(matchLength-groundTruthLength) / groundTruthLength
		matchingErrors = append(matchingErrors, matchingError)
		avgRuntimePerGpsPointAll += avgRuntimePerGpsPoint
		totalRuntime += float64(time.Since(nowDataset).Milliseconds())
		totalPoints += len(gpsPoints)
		t.Logf("Hanwen-Hu trajectory %s: matching_error=%v matched=%d/%d avg_runtime=%v Milliseconds/gps point",
			trajName, matchingError, len(matchedPoints), len(gpsPoints), avgRuntimePerGpsPoint)
	}

	sort.Float64s(matchingErrors)
	avgRuntimePerGpsPointAll /= float64(len(trajectoryNames))
	cdfAt014 := ohmmEmpiricalCDF(matchingErrors, 0.14)
	cdfAt040 := ohmmEmpiricalCDF(matchingErrors, 0.40)
	t.Logf("Hanwen-Hu offline HMM aggregate: trajectories=%d points=%d avg_runtime=%v Milliseconds/gps point CDF(<=0.14)=%v CDF(<=0.40)=%v",
		len(trajectoryNames), totalPoints, avgRuntimePerGpsPointAll, cdfAt014, cdfAt040)
	if util.Gt(totalRuntime, 0) {
		t.Logf("Hanwen-Hu offline HMM matching efficiency: %v points/ms", float64(totalPoints)/totalRuntime)
	}

	if cdfAt014 < ohmmExpectedMinHHCDFAt014 {
		t.Fatalf("Hanwen-Hu CDF(<=0.14) below threshold: got %v, want >= %v", cdfAt014, ohmmExpectedMinHHCDFAt014)
	}
	if cdfAt040 < ohmmExpectedMinHHCDFAt040 {
		t.Fatalf("Hanwen-Hu CDF(<=0.40) below threshold: got %v, want >= %v", cdfAt040, ohmmExpectedMinHHCDFAt040)
	}
}

// go test ./tests/map_matching -run TestNewsonKrummOfflineHMMMapMatching -v -timeout=0 -count=1
func TestNewsonKrummOfflineHMMMapMatching(t *testing.T) {
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

	re, g, zlog, _, edgeLength, err := nkBuildRoadNetworkCRPGraph(t, workingDir)
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
	rtree.Build(g, zlog)

	gpsTraj := nkReadGPSTrajectory(t, gpsDataFilepath)
	if len(gpsTraj) == 0 {
		t.Fatalf("empty GPS trajectory")
	}

	hmm := offline.NewHiddenMarkovModelMapMatching(g, re.GetRoutingEngine(), rtree)
	now := time.Now()
	mapMatchPointResult := hmm.MapMatch(gpsTraj)
	avgRuntimePerGPSPoint := float64(time.Since(now).Milliseconds()) / float64(len(gpsTraj))
	if len(mapMatchPointResult) == 0 {
		t.Fatalf("offline HMM produced no matched points")
	}

	matchedCoords := make([]da.Coordinate, 0, len(mapMatchPointResult))
	for _, point := range mapMatchPointResult {
		if point.GetEdgeId() == da.INVALID_EDGE_ID {
			continue
		}
		matchedCoords = append(matchedCoords, point.GetMatchedCoord())
	}
	if len(matchedCoords) > 0 {
		offlinePolylinePath := filepath.Join(workingDir, "data/eval/mapmatching/offline_newson_polyline.txt")
		if err := os.MkdirAll(filepath.Dir(offlinePolylinePath), 0755); err != nil {
			t.Fatalf("create polyline output directory failed: %v", err)
		}
		polyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
		if err := os.WriteFile(offlinePolylinePath, []byte(polyline), 0644); err != nil {
			t.Fatalf("write offline polyline file failed: %v", err)
		}
	}

	crp, rmf := nkEvaluateMatchedRoute(t, g, groundTruthDataFilepath, edgeLength, mapMatchPointResult)
	t.Logf("Route Mismatch Fraction (RMF): %v", rmf)
	t.Logf("Correct Road Percentage (CRP) or accuracy: %v", crp)
	t.Logf("matched points: %d/%d", len(mapMatchPointResult), len(gpsTraj))
	t.Logf("avg runtime per gps point: %v milliseconds/gps point", avgRuntimePerGPSPoint)

	if crp < nkExpectedMinAccuracy {
		t.Fatalf("accuracy below threshold: got %v, want >= %v", crp, nkExpectedMinAccuracy)
	}
	if rmf > nkExpectedMaxRMF {
		t.Fatalf("RMF above threshold: got %v, want <= %v", rmf, nkExpectedMaxRMF)
	}
}

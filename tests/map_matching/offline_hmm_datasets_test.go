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

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/offline"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	evalutil "github.com/lintang-b-s/Navigatorx/tests"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

const (
	ohmmGisCupRoadNetworkDriveFile = "https://drive.google.com/uc?export=download&id=1RS_3rt48WR1l-mJUqjKV9k7SusLfPgyc"
	ohmmMelbourneDatasetDriveFile  = "https://drive.google.com/uc?export=download&id=1WY0BPpu1M-e7grP33B_7BFrHlkj6kd_H"

	ohmmExpectedMaxGisCupRMF  = 0.15
	ohmmExpectedMaxMelbRMF    = 0.15
	ohmmExpectedMinHHCDFAt014 = 0.92
	ohmmExpectedMinHHCDFAt040 = 0.99
)

var (
	ohmmInitOnce sync.Once
	ohmmInitErr  error
)

type ohmmSegmentGeometry struct {
	length   float64
	roadType pkg.OsmHighwayType
	coords   []da.Coordinate
}

type ohmmGisCupRoadNetworkPaths struct {
	nodesFilePath        string
	EdgesFilePath        string
	EdgeGeometryFilePath string
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

type ohmmMelbourneSegment struct {
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

func ohmmEnsureGisCupRoadNetwork(t *testing.T, workingDir string, logger *zap.Logger) ohmmGisCupRoadNetworkPaths {
	t.Helper()

	base := filepath.Join(workingDir, "data/eval/mapmatching/giscup/road_network")

	zipPath := filepath.Join(workingDir, "data/eval/mapmatching/giscup/road_network.zip")
	if err := evalutil.Download(zipPath, ohmmGisCupRoadNetworkDriveFile, logger, "GIS Cup road network"); err != nil {
		t.Fatalf("download GIS Cup road network failed: %v", err)
	}
	if err := evalutil.ExtractZip(zipPath, base); err != nil {
		t.Fatalf("extract GIS Cup road network failed: %v", err)
	}
	paths := ohmmGisCupRoadNetworkPaths{nodesFilePath: fmt.Sprintf("%s/WA_Nodes.txt", base),
		EdgesFilePath:        fmt.Sprintf("%s/WA_Edges.txt", base),
		EdgeGeometryFilePath: fmt.Sprintf("%s/WA_EdgeGeometry.txt", base)}

	return paths
}

// https://web.archive.org/web/20130127211936/http://depts.washington.edu/giscup/home
func ohmmReadGisCupNodes(nodesFilePath string) ([]da.Coordinate, map[int64]uint32, map[int64]extractor.NodeCoord, map[da.Index]int64, error) {
	f, err := os.OpenFile(nodesFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	defer f.Close()

	nodeCoords := make([]da.Coordinate, 0)
	nodeIDToIndex := make(map[int64]uint32)
	acceptedNodeMap := make(map[int64]extractor.NodeCoord)
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
		acceptedNodeMap[nodeID] = extractor.NewNodeCoord(lat, lon)
		nodeToOsmID[da.Index(internalIndex)] = nodeID
	}
	return nodeCoords, nodeIDToIndex, acceptedNodeMap, nodeToOsmID, nil
}

// https://web.archive.org/web/20120528201458/http://depts.washington.edu/giscup/roadnetwork
func ohmmReadGisCupSegmentGeometry(EdgeGeometryFilePath string) (map[uint64]ohmmSegmentGeometry, error) {
	f, err := os.OpenFile(EdgeGeometryFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	geometries := make(map[uint64]ohmmSegmentGeometry)
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
			return nil, fmt.Errorf("invalid Segment geometry line %q", line)
		}
		SegmentID, err := util.ParseTextUInt64(string(bytes.TrimSpace(fields[0])))
		if err != nil {
			return nil, err
		}
		length, err := util.ParseTextFloat64(string(bytes.TrimSpace(fields[3])))
		if err != nil {
			return nil, err
		}
		coordFields := fields[4:]
		if len(coordFields)%2 != 0 {
			return nil, fmt.Errorf("uneven Segment geometry coordinates for %d", SegmentID)
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
		geometries[SegmentID] = ohmmSegmentGeometry{length: length, roadType: roadType, coords: coords}
	}
	return geometries, nil
}

// https://web.archive.org/web/20120528201458/http://depts.washington.edu/giscup/roadnetwork
func ohmmBuildGraphFromGisCupFiles(paths ohmmGisCupRoadNetworkPaths) (*da.Graph, *metrics.TimeFunction[int32], *da.RoadNetworkDataContainer, map[uint64]float64, error) {
	nodeCoords, nodeIDToIndex, acceptedNodeMap, nodeToOsmID, err := ohmmReadGisCupNodes(paths.nodesFilePath)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	segmentGeometries, err := ohmmReadGisCupSegmentGeometry(paths.EdgeGeometryFilePath)
	if err != nil {
		return nil, nil, nil, nil, err
	}

	f, err := os.OpenFile(paths.EdgesFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	defer f.Close()

	rn := da.NewRoadNetworkDataContainer(54)
	graphSegments := make([]extractor.Edge[int32], 0)
	SegmentLengths := make(map[uint64]float64)

	br := bufio.NewReader(f)
	segId := 0
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
			return nil, nil, nil, nil, fmt.Errorf("invalid Segment line %q", line)
		}

		SegmentID, err := util.ParseTextUInt64(fields[0])
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

		geometry, ok := segmentGeometries[SegmentID]

		startPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendOsmNodePoints(geometry.coords, make([]uint64, len(geometry.coords)))
		endPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendSegmentData(int64(SegmentID), da.Index(startPointsIndex), da.Index(endPointsIndex), 0, geometry.roadType, 0, 1, da.NewEmptyTurnLanesData())
		rn.SetSegmentFlag(da.Index(segId), 0)

		graphSegment := extractor.NewEdge[int32](
			fromIndex, toIndex, int32(cost), util.RoundCentimeters(geometry.length),
		)

		graphSegment.SetFromOSMId(uint64(fromNodeID))
		graphSegment.SetToOSMId(uint64(toNodeID))
		graphSegments = append(graphSegments, graphSegment)
		SegmentLengths[SegmentID] = geometry.length
		segId++
	}

	op := extractor.NewExtractor[int32]()
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmID)
	g, timeFunction, vertexTurnTablePtr, flattenTurnMatrices := op.BuildGraph(graphSegments, rn, uint32(len(nodeCoords)), true)
	rn.BuildNameTable(map[uint32]string{0: ""})
	g, timeFunction = extractor.BuildEdgeBasedGraph(g, timeFunction, vertexTurnTablePtr, flattenTurnMatrices, rn)

	return g, timeFunction, rn, SegmentLengths, nil
}

// https://web.archive.org/web/20120528201458/http://depts.washington.edu/giscup/roadnetwork
func ohmmBuildGisCupCRPGraph(t *testing.T, workingDir string) (*engine.Engine[int32], *da.Graph, *zap.Logger, map[uint64]float64) {
	t.Helper()

	config.InitRegionName("giscup", pkg.TEST)

	logger, err := log.New()
	if err != nil {
		t.Fatalf("log.New failed: %v", err)
	}
	paths := ohmmEnsureGisCupRoadNetwork(t, workingDir, logger)
	graph, timeFunction, rn, SegmentLengths, err := ohmmBuildGraphFromGisCupFiles(paths)
	if err != nil {
		t.Fatalf("build GIS Cup graph failed: %v", err)
	}

	partitionSizes := []int{8, 11, 14, 16}
	ps := make([]int, len(partitionSizes))
	for i, pow := range partitionSizes {
		ps[i] = 1 << pow
	}

	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 1, graph, logger, false, false)
	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(); err != nil {
		t.Fatalf("save mlp failed: %v", err)
	}

	mlp := da.NewPlainMLP()
	if err := mlp.ReadMlpFile(); err != nil {
		t.Fatalf("read mlp failed: %v", err)
	}
	prep := prepo.NewPreprocessor(graph, rn, timeFunction, mlp, logger)
	if err := prep.PreProcessing(true); err != nil {
		t.Fatalf("preprocessing failed: %v", err)
	}
	cust := customizer.NewCustomizer[int32](logger)
	if _, err := cust.Customize(); err != nil {
		t.Fatalf("customize failed: %v", err)
	}
	eng, err := engine.NewEngine[int32](logger)
	if err != nil {
		t.Fatalf("new engine failed: %v", err)
	}

	re := eng.GetRoutingEngine()
	g := re.GetGraph()
	return eng, g, logger, SegmentLengths
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

func ohmmReadGisCupGroundTruth(outputPath string) ([]uint64, error) {
	f, err := os.OpenFile(outputPath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	reader := csv.NewReader(f)
	reader.FieldsPerRecord = -1
	reader.TrimLeadingSpace = true
	SegmentIDs := make([]uint64, 0)
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
		SegmentID, err := util.ParseTextUInt64(strings.TrimSpace(record[1]))
		if err != nil {
			return nil, err
		}
		SegmentIDs = append(SegmentIDs, SegmentID)
	}
	return SegmentIDs, nil
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

func ohmmComputeGiscupMetrics(graph *da.Graph, rn *da.RoadNetworkDataContainer, groundTruthSegmentIds []uint64, matchedPoints []*da.MatchedGPSPoint,
	SegmentLengths map[uint64]float64) float64 {

	groundTruthSet := make(map[uint64]bool)
	lengthOfCorrectRoute := 0.0
	for _, SegmentID := range groundTruthSegmentIds {
		lengthOfCorrectRoute += SegmentLengths[SegmentID]
		groundTruthSet[SegmentID] = true
	}

	matchedSegmentSet := make(map[uint64]float64)

	for _, point := range matchedPoints {
		dataSegmentID := rn.GetOsmWayId(point.GetSegmentId())
		length := SegmentLengths[dataSegmentID]
		matchedSegmentSet[dataSegmentID] = length

	}

	lengthOfErrAdded := 0.0
	for SegmentID, length := range matchedSegmentSet {
		if !groundTruthSet[SegmentID] {
			lengthOfErrAdded += length
		}
	}

	lengthOfErrSubtracted := 0.0
	for SegmentID := range groundTruthSet {
		if _, ok := matchedSegmentSet[SegmentID]; !ok {
			lengthOfErrSubtracted += SegmentLengths[SegmentID]
		}
	}

	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute

	return rmf
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

func ohmmParseMelbourneEdgesFile(filePath string) ([]ohmmMelbourneSegment, error) {
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
	expectedSegments, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, err
	}

	edges := make([]ohmmMelbourneSegment, 0, expectedSegments)
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, err
		}
		ff := util.Fields(line)
		if len(ff) < 4 {
			return nil, fmt.Errorf("invalid Segment line %q", line)
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
		edges = append(edges, ohmmMelbourneSegment{id: id, startID: startID, endID: endID, distance: dist})
	}
	return edges, nil
}

func ohmmParseMelbourneStreetsFile(filePath string) (map[uint64]ohmmMelbourneStreet, error) {
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
	expectedSegments, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, err
	}

	streetByID := make(map[uint64]ohmmMelbourneStreet, expectedSegments)
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
		id, err := util.ParseTextUInt64(ff[0])
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
			id: int64(id), startID: startID, startLon: startLon, startLat: startLat,
			endID: endID, endLon: endLon, endLat: endLat, distance: dist,
		}
	}
	return streetByID, nil
}

func ohmmPrepareCRPFiles(t *testing.T, graph *da.Graph, timeFunction *metrics.TimeFunction[int32], rn *da.RoadNetworkDataContainer, logger *zap.Logger, partitionSizes []int,
) *engine.Engine[int32] {
	t.Helper()

	ps := make([]int, len(partitionSizes))
	for i, pow := range partitionSizes {
		ps[i] = 1 << pow
	}

	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 1, graph, logger, false, false)
	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(); err != nil {
		t.Fatalf("save mlp failed: %v", err)
	}

	mlp := da.NewPlainMLP()
	if err := mlp.ReadMlpFile(); err != nil {
		t.Fatalf("read mlp failed: %v", err)
	}
	prep := prepo.NewPreprocessor(graph, rn, timeFunction, mlp, logger)
	if err := prep.PreProcessing(true); err != nil {
		t.Fatalf("preprocessing failed: %v", err)
	}
	cust := customizer.NewCustomizer[int32](logger)
	if _, err := cust.Customize(); err != nil {
		t.Fatalf("customize failed: %v", err)
	}

	re, err := engine.NewEngine[int32](logger)
	if err != nil {
		t.Fatalf("new engine failed: %v", err)
	}
	return re
}

// https://web.archive.org/web/20170301001019/https://people.eng.unimelb.edu.au/henli/projects/map-matching/
func ohmmBuildMelbourneCRPGraph(t *testing.T, workingDir string) (*engine.Engine[int32], *da.Graph, *zap.Logger, map[da.Index]uint64, map[uint64]float64) {
	t.Helper()

	config.InitRegionName("hengfengli", pkg.TEST)

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
		t.Fatalf("parse Melbourne Segments failed: %v", err)
	}
	streetByID, err := ohmmParseMelbourneStreetsFile(streetsPath)
	if err != nil {
		t.Fatalf("parse Melbourne streets failed: %v", err)
	}

	rn := da.NewRoadNetworkDataContainer(54)
	graphSegments := make([]extractor.Edge[int32], 0, len(edges))
	for i, e := range edges {
		st := streetByID[uint64(e.id)]
		startPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendOsmNodePoints([]da.Coordinate{
			da.NewCoordinate(st.startLat, st.startLon),
			da.NewCoordinate(st.endLat, st.endLon),
		}, make([]uint64, 2))
		endPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendSegmentData(e.id, da.Index(startPointsIndex), da.Index(endPointsIndex), 0, 0, 0, 1, da.NewEmptyTurnLanesData())
		graphSegment := extractor.NewEdge[int32](
			uint32(e.startID), uint32(e.endID),
			int32(e.distance), util.RoundCentimeters(e.distance))
		rn.SetSegmentFlag(da.Index(i), 0)

		graphSegments = append(graphSegments, graphSegment)
	}

	acceptedNodeMap := make(map[int64]extractor.NodeCoord, len(vertices))
	nodeToOsmID := make(map[da.Index]int64, len(vertices))
	for _, v := range vertices {
		acceptedNodeMap[v.osmID] = extractor.NewNodeCoord(v.lat, v.lon)
		nodeToOsmID[da.Index(v.id)] = v.osmID
	}

	op := extractor.NewExtractor[int32]()
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmID)
	graph, wf, vertexTurnTablePtr, flattenTurnMatrices := op.BuildGraph(graphSegments, rn, uint32(len(vertices)), true)
	rn.BuildNameTable(map[uint32]string{0: ""})
	graph, wf = extractor.BuildEdgeBasedGraph(graph, wf, vertexTurnTablePtr, flattenTurnMatrices, rn)

	eng := ohmmPrepareCRPFiles(t, graph, wf, rn, logger, []int{8, 11, 13, 14, 15})

	re := eng.GetRoutingEngine()
	g := re.GetGraph()
	graphEdgeIdToDataEdgeId := make(map[da.Index]uint64, g.NumberOfEdges())

	g.ForVertices(func(v da.Vertex, segId da.Index) {
		graphEdgeIdToDataEdgeId[segId] = rn.GetOsmWayId(segId)
	})

	segmentLengthByID := make(map[uint64]float64, len(streetByID))
	for segmentID, street := range streetByID {
		segmentLengthByID[segmentID] = street.distance
	}
	return eng, g, logger, graphEdgeIdToDataEdgeId, segmentLengthByID
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

func ohmmReadMelbourneGroundTruthSegments(filePath string) ([]uint64, error) {
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
	expectedSegments, err := util.ParseTextInt64(strings.TrimSpace(line))
	if err != nil {
		return nil, err
	}

	SegmentIDs := make([]uint64, 0, expectedSegments)
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
		SegmentID, err := util.ParseTextUInt64(ff[0])
		if err != nil {
			return nil, err
		}
		SegmentIDs = append(SegmentIDs, SegmentID)
	}
	return SegmentIDs, nil
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

func ohmmComputeMelbourneMetrics(matchedPoints []*da.MatchedGPSPoint, groundTruthSegmentIds []uint64,
	graphEdgeIdToDataEdgeId map[da.Index]uint64, segmentLengthByID map[uint64]float64) float64 {
	groundTruthSegmentSet := make(map[uint64]float64, len(groundTruthSegmentIds))
	for _, SegmentID := range groundTruthSegmentIds {
		segmentLength := segmentLengthByID[SegmentID]
		groundTruthSegmentSet[SegmentID] = segmentLength
	}

	lengthOfCorrectRoute := 0.0
	for _, segmentLength := range groundTruthSegmentSet {
		lengthOfCorrectRoute += segmentLength
	}

	matchedSegmentSet := make(map[uint64]float64)

	numMatchedPoints := 0.0
	for _, matchedPoint := range matchedPoints {
		geId := matchedPoint.GetSegmentId()
		dataSegmentID := graphEdgeIdToDataEdgeId[geId]
		segmentLength := segmentLengthByID[dataSegmentID]
		matchedSegmentSet[dataSegmentID] = segmentLength
		numMatchedPoints++
	}

	lengthOfErrAdded := 0.0
	for SegmentID, SegmentLength := range matchedSegmentSet {
		if _, inGroundTruth := groundTruthSegmentSet[SegmentID]; !inGroundTruth {
			lengthOfErrAdded += SegmentLength
		}
	}

	lengthOfErrSubtracted := 0.0
	for SegmentID, SegmentLength := range groundTruthSegmentSet {
		if _, inMatchedRoute := matchedSegmentSet[SegmentID]; !inMatchedRoute {
			lengthOfErrSubtracted += SegmentLength
		}
	}

	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	return rmf
}

func ohmmWritePolyline(t *testing.T, filePath string, matchedPoints []*da.MatchedGPSPoint) {
	t.Helper()

	matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))
	for _, point := range matchedPoints {
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

	config.InitRegionName("hanwenhu", pkg.TEST)

	eng, _, logger, _ := hhBuildCRPGraph(t)
	re := eng.GetRoutingEngine()
	g := re.GetGraph()
	return eng, g, logger
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
	for start < len(matchedPoints) && matchedPoints[start].GetSegmentId() == da.INVALID_SEGMENT_ID {
		start++
	}
	if start >= len(matchedPoints) {
		return 0
	}

	prevMp := matchedPoints[start]
	length := 0.0
	for j := start + 1; j < len(matchedPoints); j++ {
		mp := matchedPoints[j]
		if mp.GetSegmentId() == da.INVALID_SEGMENT_ID {
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

// todo: update kode ini, adjust setelah pakai edge-based graph

// go test ./tests/map_matching -run TestGisCupOfflineHMMMapMatching -v -timeout=0 -count=1
func TestGisCupOfflineHMMMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	eng, graph, logger, SegmentLengths := ohmmBuildGisCupCRPGraph(t, workingDir)

	cases, err := ohmmListGisCupCases(workingDir)
	if err != nil {
		t.Skipf("GIS Cup training data unavailable: %v", err)
	}
	if len(cases) == 0 {
		t.Skip("GIS Cup training data unavailable")
	}

	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	rtree := spatialindex.NewRtree()
	rtree.Build(graph, rn, logger)
	hmm := offline.NewHiddenMarkovModelMapMatching(graph, re, rtree)

	totalRMF := 0.0
	totalPoints := 0
	for _, tc := range cases {
		points, err := ohmmReadGisCupTrack(tc.inputFilePath)
		if err != nil {
			t.Fatalf("read GIS Cup track %s failed: %v", tc.id, err)
		}
		groundTruthSegmentIds, err := ohmmReadGisCupGroundTruth(tc.outputFilePath)
		if err != nil {
			t.Fatalf("read GIS Cup ground truth %s failed: %v", tc.id, err)
		}

		gpsTraj := ohmmGisCupGPSTrajectory(points)
		now := time.Now()
		matchedPoints := hmm.MapMatch(gpsTraj)
		runtime := float64(time.Since(now).Seconds())
		if len(matchedPoints) == 0 {
			t.Fatalf("GIS Cup case %s produced no matched points", tc.id)
		}

		rmf := ohmmComputeGiscupMetrics(graph, rn, groundTruthSegmentIds, matchedPoints, SegmentLengths)
		t.Logf("GIS Cup case %s: RMF=%v matched=%d/%d runtime=%v seconds",
			tc.id, rmf, len(matchedPoints), len(gpsTraj), runtime)

		ohmmWritePolyline(t, filepath.Join(workingDir, "data/eval/mapmatching/GisContestTrainingData/polylines/offline_hmm_result_polyline_"+tc.id+".txt"), matchedPoints)
		totalRMF += rmf
		totalPoints += len(gpsTraj)
	}

	avgRMF := totalRMF / float64(len(cases))
	t.Logf("GIS Cup offline HMM aggregate: cases=%d points=%d avg_RMF=%v", len(cases), totalPoints, avgRMF)
	if avgRMF > ohmmExpectedMaxGisCupRMF {
		t.Fatalf("GIS Cup RMF above threshold: got %v, want <= %v", avgRMF, ohmmExpectedMaxGisCupRMF)
	}
}

// go test ./tests/map_matching -run TestHengfengLiOfflineHMMMapMatching -v -timeout=0 -count=1
func TestHengfengLiOfflineHMMMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	eng, graph, logger, graphEdgeIdToDataEdgeId, segmentLengthByID := ohmmBuildMelbourneCRPGraph(t, workingDir)

	gpsPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/gps_track.txt")
	groundTruthPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/groundtruth.txt")
	points, err := ohmmReadMelbourneGPSTrack(gpsPath)
	if err != nil {
		t.Fatalf("read Melbourne GPS track failed: %v", err)
	}
	groundTruthSegmentIds, err := ohmmReadMelbourneGroundTruthSegments(groundTruthPath)
	if err != nil {
		t.Fatalf("read Melbourne ground truth failed: %v", err)
	}

	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, rn, logger)
	hmm := offline.NewHiddenMarkovModelMapMatching(graph, re, rtree)
	gpsTraj := ohmmMelbourneGPSTrajectory(points)

	now := time.Now()
	matchedPoints := hmm.MapMatch(gpsTraj)
	runtime := float64(time.Since(now).Seconds())

	rmf := ohmmComputeMelbourneMetrics(matchedPoints, groundTruthSegmentIds, graphEdgeIdToDataEdgeId, segmentLengthByID)

	t.Logf("Hengfeng Li offline HMM: RMF=%v matched=%d/%d runtime=%v seconds",
		rmf, len(matchedPoints), len(gpsTraj), runtime)

	ohmmWritePolyline(t, filepath.Join(workingDir, "data/eval/mapmatching/melbourne/offline_hmm_result_polyline.txt"), matchedPoints)

	if rmf > ohmmExpectedMaxMelbRMF {
		t.Fatalf("Hengfeng Li RMF above threshold: got %v, want <= %v", rmf, ohmmExpectedMaxMelbRMF)
	}
}

// go test ./tests/map_matching -run TestHanwenHuOfflineHMMMapMatching -v -timeout=0 -count=1
func TestHanwenHuOfflineHMMMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	eng, graph, logger := ohmmBuildHanwenHuCRPGraph(t)

	projectPath := func(path string) string {
		return filepath.Join(workingDir, strings.TrimPrefix(path, "./"))
	}

	shanghaiDataFilePath := projectPath(hhShanghaiDataFilePath)
	shanghaiTestDataPath := projectPath(hhShanghaiTestDataPath)
	shanghaiGroundTruthPath := projectPath(hhShanghaiGroundTruthPath)
	shanghaiPolylinesPath := projectPath(hhShanghaiPolylinesPath)

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
	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	rtree.Build(graph, rn, logger)
	hmm := offline.NewHiddenMarkovModelMapMatching(graph, re, rtree)

	trajectoryNames := make([]string, 0, len(gpsTrajectories))
	for trajName := range gpsTrajectories {
		trajectoryNames = append(trajectoryNames, trajName)
	}
	sort.Strings(trajectoryNames)

	matchingErrors := make([]float64, 0, len(trajectoryNames))
	totalPoints := 0
	avgRuntime := 0.0
	for _, trajName := range trajectoryNames {
		gpsTraj := gpsTrajectories[trajName]

		gpsPoints := ohmmHanwenHuGPSTrajectory(t, gpsTraj)

		now := time.Now()
		matchedPoints := hmm.MapMatch(gpsPoints)
		runtime := float64(time.Since(now).Seconds())

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

		matchingError := math.Abs(matchLength-groundTruthLength) / groundTruthLength // https://github.com/Hanwen-Hu/AMM/blob/main/Algorithm/src/main/java/Matching.java see Matching Error at main function
		matchingErrors = append(matchingErrors, matchingError)
		avgRuntime += runtime

		totalPoints += len(gpsPoints)
		t.Logf("Hanwen-Hu trajectory %s: matching_error=%v matched=%d/%d runtime=%v seconds",
			trajName, matchingError, len(matchedPoints), len(gpsPoints), runtime)
	}

	sort.Float64s(matchingErrors)
	avgRuntime /= float64(len(trajectoryNames))
	cdfAt014 := ohmmEmpiricalCDF(matchingErrors, 0.14)
	cdfAt040 := ohmmEmpiricalCDF(matchingErrors, 0.40)
	t.Logf("Hanwen-Hu offline HMM aggregate: trajectories=%d, points=%d, avg_runtime=%v Seconds, CDF(<=0.14)=%v CDF(<=0.40)=%v",
		len(trajectoryNames), totalPoints, avgRuntime, cdfAt014, cdfAt040)

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

	eng, g, zlog, _, SegmentLength, err := nkBuildRoadNetworkCRPGraph(t, workingDir)
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

	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	rtree := spatialindex.NewRtree()
	rtree.Build(g, rn, zlog)

	gpsTraj := nkReadGPSTrajectory(t, gpsDataFilepath)
	if len(gpsTraj) == 0 {
		t.Fatalf("empty GPS trajectory")
	}

	hmm := offline.NewHiddenMarkovModelMapMatching(g, re, rtree)
	now := time.Now()
	mapMatchPointResult := hmm.MapMatch(gpsTraj)
	runtime := float64(time.Since(now).Seconds())

	matchedCoords := make([]da.Coordinate, 0, len(mapMatchPointResult))
	for _, point := range mapMatchPointResult {
		if point.GetSegmentId() == da.INVALID_SEGMENT_ID {
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

	rmf := nkEvaluateMatchedRoute(t, g, rn, groundTruthDataFilepath, SegmentLength, mapMatchPointResult)
	t.Logf("Route Mismatch Fraction (RMF): %v", rmf)
	t.Logf("matched points: %d/%d", len(mapMatchPointResult), len(gpsTraj))
	t.Logf("runtime: %v seconds", runtime)

	if rmf > nkExpectedMaxRMF {
		t.Fatalf("RMF above threshold: got %v, want <= %v", rmf, nkExpectedMaxRMF)
	}
}

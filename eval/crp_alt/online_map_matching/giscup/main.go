package main

import (
	"bufio"
	"bytes"
	"context"
	"encoding/csv"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"time"

	onlinemapmatching "github.com/lintang-b-s/Navigatorx/eval/crp_alt/online_map_matching"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

/*
go run eval/crp_alt/online_map_matching/giscup/main.go
https://web.archive.org/web/20130127211936/http://depts.washington.edu/giscup/home

GIS Cup 2012 training data:
- Road nodes: <NodeId> <lat> <long>
- Road edges: <EdgeId> <from> <to> <cost>
- Edge geometry: <EdgeId>^<Name>^<Type>^<Length>^<Lat_1>^<Lon_1>^...^<Lat_n>^<Lon_n>
- GPS input: <Time>,<Latitude>,<Longitude>
- Ground truth output: <Time>,<EdgeId>,<Confidence>
*/

const (
	giscupRoadNetworkDriveFile = "https://drive.google.com/uc?export=download&id=1RS_3rt48WR1l-mJUqjKV9k7SusLfPgyc"
	giscupRoadZipFilePath      = "./data/eval/mapmatching/giscup/road_network.zip"
	giscupRoadExtractDir       = "./data/eval/mapmatching/giscup/road_network"

	giscupTrainingRoot        = "./data/eval/mapmatching/GisContestTrainingData"
	giscupTrainingInputDir    = "./data/eval/mapmatching/GisContestTrainingData/input"
	giscupTrainingOutputDir   = "./data/eval/mapmatching/GisContestTrainingData/output"
	giscupPredictionOutputDir = "./data/eval/mapmatching/GisContestTrainingData/predicted_output"
	giscupPolylineOutputDir   = "./data/eval/mapmatching/GisContestTrainingData/polylines"

	graphFile                = "./data/eval/mapmatching/giscup/original_giscup.ngraph"
	overlayGraphFile         = "./data/eval/mapmatching/giscup/overlay_graph_giscup.ngraph"
	metricsFile              = "./data/eval/mapmatching/giscup/metrics_giscup.nmt"
	mlpFile                  = "./data/eval/mapmatching/giscup/online_map_match_mlp_giscup.mlp"
	landmarkFile             = "./data/eval/mapmatching/giscup/landmark_giscup.nlm"
	timeFunctionFile         = "./data/eval/mapmatching/giscup/timefunction_giscup.ntf"
	transitionMatrixFilepath = "./data/eval/mapmatching/giscup/omm_transition_history_giscup.ntm"

	defaultSpeedMPS = 8.33333
)

var partitionSizes = []int{8, 11, 14, 16}

type roadNetworkPaths struct {
	nodesFilePath        string
	edgesFilePath        string
	edgeGeometryFilePath string
}

type edgeGeometry struct {
	length   float64
	roadType pkg.OsmHighwayType
	coords   []da.Coordinate
}

type trackPoint struct {
	timeSec float64
	lat     float64
	lon     float64
}

type trajectoryCase struct {
	id             string
	inputFilePath  string
	outputFilePath string
}

type evalMetrics struct {
	caseID          string
	crp             float64
	rmf             float64
	points          int
	runtimeMS       float64
	pointRuntimeMic float64
}

type query struct {
	s da.Index
	t da.Index
}

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

func newQuery(s, t da.Index) query {
	return query{s: s, t: t}
}

func ensureGisCupRoadNetwork(logger *zap.Logger) (roadNetworkPaths, error) {
	if paths, err := locateRoadNetworkFiles(giscupRoadExtractDir); err == nil {
		return paths, nil
	}

	if err := onlinemapmatching.Download(giscupRoadZipFilePath, giscupRoadNetworkDriveFile, logger, "GIS Cup road network"); err != nil {
		return roadNetworkPaths{}, fmt.Errorf("ensureGisCupRoadNetwork: download failed: %w", err)
	}

	if err := onlinemapmatching.ExtractZip(giscupRoadZipFilePath, giscupRoadExtractDir); err != nil {
		return roadNetworkPaths{}, fmt.Errorf("ensureGisCupRoadNetwork: extract zip failed: %w", err)
	}

	paths, err := locateRoadNetworkFiles(giscupRoadExtractDir)
	if err != nil {
		return roadNetworkPaths{}, err
	}
	return paths, nil
}

func locateRoadNetworkFiles(root string) (roadNetworkPaths, error) {
	var paths roadNetworkPaths
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
		return roadNetworkPaths{}, fmt.Errorf("locateRoadNetworkFiles: walk failed: %w", err)
	}
	if paths.nodesFilePath == "" || paths.edgesFilePath == "" || paths.edgeGeometryFilePath == "" {
		return roadNetworkPaths{}, fmt.Errorf("locateRoadNetworkFiles: missing WA_Nodes.txt, WA_Edges.txt, or WA_EdgeGeometry.txt under %s", root)
	}
	return paths, nil
}

func readGisCupNodes(nodesFilePath string) ([]da.Coordinate, map[int64]uint32, map[int64]osmparser.NodeCoord, map[da.Index]int64, error) {
	f, err := os.OpenFile(nodesFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("readGisCupNodes: open failed: %w", err)
	}
	defer f.Close()

	nodeCoords := make([]da.Coordinate, 0)
	nodeIDToIndex := make(map[int64]uint32)
	acceptedNodeMap := make(map[int64]osmparser.NodeCoord)
	nodeToOsmID := make(map[da.Index]int64)

	br := bufio.NewReader(f)
	lineNo := 0
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("readGisCupNodes: read line failed: %w", err)
		}
		lineNo++
		fields := util.Fields(line)
		if len(fields) == 0 {
			continue
		}
		if len(fields) < 3 {
			return nil, nil, nil, nil, fmt.Errorf("readGisCupNodes: invalid line %d", lineNo)
		}

		nodeID, err := util.ParseTextInt64(fields[0])
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("readGisCupNodes: parse node id at line %d failed: %w", lineNo, err)
		}
		lat, err := util.ParseTextFloat64(fields[1])
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("readGisCupNodes: parse lat at line %d failed: %w", lineNo, err)
		}
		lon, err := util.ParseTextFloat64(fields[2])
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("readGisCupNodes: parse lon at line %d failed: %w", lineNo, err)
		}

		internalIndex := uint32(len(nodeCoords))
		nodeCoords = append(nodeCoords, da.NewCoordinate(lat, lon))
		nodeIDToIndex[nodeID] = internalIndex
		acceptedNodeMap[nodeID] = osmparser.NewNodeCoord(lat, lon)
		nodeToOsmID[da.Index(internalIndex)] = nodeID
	}
	if len(nodeCoords) == 0 {
		return nil, nil, nil, nil, fmt.Errorf("readGisCupNodes: no nodes loaded from %s", nodesFilePath)
	}
	return nodeCoords, nodeIDToIndex, acceptedNodeMap, nodeToOsmID, nil
}

func readGisCupEdgeGeometry(edgeGeometryFilePath string) (map[int64]edgeGeometry, error) {
	f, err := os.OpenFile(edgeGeometryFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, fmt.Errorf("readGisCupEdgeGeometry: open failed: %w", err)
	}
	defer f.Close()

	geometries := make(map[int64]edgeGeometry)
	br := bufio.NewReader(f)
	lineNo := 0
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, fmt.Errorf("readGisCupEdgeGeometry: read line failed: %w", err)
		}
		lineNo++
		lineBytes := bytes.TrimSpace([]byte(line))
		if len(line) == 0 {
			continue
		}

		fields := bytes.Split(lineBytes, []byte("^"))
		if len(fields) < 8 {
			return nil, fmt.Errorf("readGisCupEdgeGeometry: invalid line %d", lineNo)
		}

		edgeID, err := util.ParseTextInt64(string(bytes.TrimSpace(fields[0])))
		if err != nil {
			return nil, fmt.Errorf("readGisCupEdgeGeometry: parse edge id at line %d failed: %w", lineNo, err)
		}
		length, err := util.ParseTextFloat64(string(bytes.TrimSpace(fields[3])))
		if err != nil {
			return nil, fmt.Errorf("readGisCupEdgeGeometry: parse length at line %d failed: %w", lineNo, err)
		}

		coordFields := fields[4:]
		if len(coordFields)%2 != 0 {
			return nil, fmt.Errorf("readGisCupEdgeGeometry: uneven coordinate fields at line %d", lineNo)
		}
		coords := make([]da.Coordinate, 0, len(coordFields)/2)
		for i := 0; i < len(coordFields); i += 2 {
			lat, err := util.ParseTextFloat64(string(bytes.TrimSpace(coordFields[i])))
			if err != nil {
				return nil, fmt.Errorf("readGisCupEdgeGeometry: parse lat at line %d failed: %w", lineNo, err)
			}
			lon, err := util.ParseTextFloat64(string(bytes.TrimSpace(coordFields[i+1])))
			if err != nil {
				return nil, fmt.Errorf("readGisCupEdgeGeometry: parse lon at line %d failed: %w", lineNo, err)
			}
			coords = append(coords, da.NewCoordinate(lat, lon))
		}
		if len(coords) < 2 {
			return nil, fmt.Errorf("readGisCupEdgeGeometry: too few coordinates at line %d", lineNo)
		}

		roadType := pkg.GetHighwayType(string(bytes.TrimSpace(fields[2])))
		if roadType == pkg.UNKNOWN {
			roadType = pkg.ROAD
		}
		geometries[edgeID] = edgeGeometry{
			length:   length,
			roadType: roadType,
			coords:   coords,
		}
	}
	if len(geometries) == 0 {
		return nil, fmt.Errorf("readGisCupEdgeGeometry: no geometries loaded from %s", edgeGeometryFilePath)
	}
	return geometries, nil
}

func fallbackEdgeGeometry(edgeID int64, from, to uint32, nodeCoords []da.Coordinate) edgeGeometry {
	fromCoord := nodeCoords[from]
	toCoord := nodeCoords[to]
	length := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
		fromCoord.GetLat(), fromCoord.GetLon(),
		toCoord.GetLat(), toCoord.GetLon(),
	))
	return edgeGeometry{
		length:   length,
		roadType: pkg.ROAD,
		coords:   []da.Coordinate{fromCoord, toCoord},
	}
}

func buildGraphFromGisCupFiles(paths roadNetworkPaths) (*da.Graph, *costfunction.TimeFunction[int32], [][]da.Index, map[int64]float64, error) {
	nodeCoords, nodeIDToIndex, acceptedNodeMap, nodeToOsmID, err := readGisCupNodes(paths.nodesFilePath)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	edgeGeometries, err := readGisCupEdgeGeometry(paths.edgeGeometryFilePath)
	if err != nil {
		return nil, nil, nil, nil, err
	}

	f, err := os.OpenFile(paths.edgesFilePath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: open edges failed: %w", err)
	}
	defer f.Close()

	graphStorage := da.NewGraphStorage(54)
	graphEdges := make([]osmparser.Edge[int32], 0)
	edgeLengths := make(map[int64]float64)

	br := bufio.NewReader(f)
	lineNo := 0
	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: read edge line failed: %w", err)
		}
		lineNo++
		fields := util.Fields(line)
		if len(fields) == 0 {
			continue
		}
		if len(fields) < 4 {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: invalid edge line %d", lineNo)
		}

		edgeID, err := util.ParseTextInt64(fields[0])
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: parse edge id at line %d failed: %w", lineNo, err)
		}
		fromNodeID, err := util.ParseTextInt64(fields[1])
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: parse from node id at line %d failed: %w", lineNo, err)
		}
		toNodeID, err := util.ParseTextInt64(fields[2])
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: parse to node id at line %d failed: %w", lineNo, err)
		}
		cost, err := util.ParseTextFloat64(fields[3])
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: parse cost at line %d failed: %w", lineNo, err)
		}

		fromIndex, ok := nodeIDToIndex[fromNodeID]
		if !ok {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: missing from node %d at line %d", fromNodeID, lineNo)
		}
		toIndex, ok := nodeIDToIndex[toNodeID]
		if !ok {
			return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: missing to node %d at line %d", toNodeID, lineNo)
		}
		if fromIndex == toIndex {
			continue
		}

		geometry, ok := edgeGeometries[edgeID]
		if !ok {
			geometry = fallbackEdgeGeometry(edgeID, fromIndex, toIndex, nodeCoords)
		}
		if util.Le(geometry.length, 0) {
			geometry.length = fallbackEdgeGeometry(edgeID, fromIndex, toIndex, nodeCoords).length
		}

		startPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendOsmNodePoints(geometry.coords)
		endPointsIndex := graphStorage.GetOsmNodePointsCount()
		graphStorage.AppendEdgeMetadata(
			edgeID,
			da.Index(startPointsIndex), da.Index(endPointsIndex),
			0, geometry.roadType, 0,
			1,
		)

		graphEdge := osmparser.NewFixedEdge(
			fromIndex, toIndex, cost, geometry.length, false, geometry.roadType,
		)

		graphEdge.SetFromOSMId(uint64(fromNodeID))
		graphEdge.SetToOSMId(uint64(toNodeID))
		graphEdges = append(graphEdges, graphEdge)
		edgeLengths[edgeID] = geometry.length
	}
	if len(graphEdges) == 0 {
		return nil, nil, nil, nil, fmt.Errorf("buildGraphFromGisCupFiles: no edges loaded from %s", paths.edgesFilePath)
	}

	op := osmparser.NewOSMParserV2[int32]()
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmID)
	graph, timeFunction, edgeInfoIDs := op.BuildGraph(graphEdges, graphStorage, uint32(len(nodeCoords)), true)
	graph.SetGraphStorage(graphStorage)
	return graph, timeFunction, edgeInfoIDs, edgeLengths, nil
}

func buildCRPGraph() (*engine.Engine[int32], *da.Graph, *zap.Logger, *da.SparseMatrix[int], map[int64]float64, error) {
	logger, err := log.New()
	if err != nil {
		return nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: log.New failed: %w", err)
	}

	paths, err := ensureGisCupRoadNetwork(logger)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	graph, timeFunction, edgeInfoIDs, edgeLengths, err := buildGraphFromGisCupFiles(paths)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	logger.Sugar().Infof("loaded GIS Cup graph: vertices=%d edges=%d", graph.NumberOfVertices(), graph.NumberOfEdges())

	ps := make([]int, len(partitionSizes))
	for i, pow := range partitionSizes {
		ps[i] = 1 << pow
	}

	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 1, graph, logger, false, false)
	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(mlpFile); err != nil {
		return nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: SaveToFile failed: %w", err)
	}

	mlp := da.NewPlainMLP()
	if err := mlp.ReadMlpFile(mlpFile); err != nil {
		return nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: ReadMlpFile failed: %w", err)
	}

	prep := preprocesser.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIDs)
	if err := prep.PreProcessing(true); err != nil {
		return nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: PreProcessing failed: %w", err)
	}

	cust := customizer.NewCustomizer[int32](graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)
	if _, err := cust.Customize(); err != nil {
		return nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: Customize failed: %w", err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		return nil, nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: NewEngine failed: %w", err)
	}

	graph = re.GetRoutingEngine().GetGraph()
	transitionMatrix, err := buildOrReadTransitionMatrix(re, graph, logger)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	logger.Sugar().Infof("customization phase of Customizable Route Planning (CRP) done")
	return re, graph, logger, transitionMatrix, edgeLengths, nil
}

func buildOrReadTransitionMatrix(re *engine.Engine[int32], graph *da.Graph, logger *zap.Logger) (*da.SparseMatrix[int], error) {
	if _, err := os.Stat(transitionMatrixFilepath); err == nil {
		logger.Info("reading transition matrix from file...")
		matrix, err := da.ReadSparseMatrixFromFile[int](
			transitionMatrixFilepath,
			0,
			func(a, b int) bool { return a == b },
		)
		if err != nil {
			return nil, fmt.Errorf("buildOrReadTransitionMatrix: ReadSparseMatrixFromFile failed: %w", err)
		}
		return matrix, nil
	}

	logger.Sugar().Infof("building transition matrix...")
	n := graph.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	numQueries := 5000
	queries := make([]query, 0, numQueries)
	for len(queries) < numQueries {
		s := da.Index(rd.Intn(n))
		t := da.Index(rd.Intn(n))
		if s == t || !graph.PathExists(s, t) {
			continue
		}
		queries = append(queries, newQuery(s, t))
	}

	computeRoute := func(q query) []da.Index {
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0)
		as := graph.GetDummyOutEdgeId(q.s)
		at := graph.GetDummyInEdgeId(q.t)
		sVertex := graph.GetVertex(q.s)
		tVertex := graph.GetVertex(q.t)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)
		_, _, _, edges, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		return edges
	}

	matrix := da.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(), 0, func(a, b int) bool { return a == b })
	workers := concurrent.NewWorkerPool[query, []da.Index](100, 25_000)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	done := make(chan struct{})
	go func() {
		defer close(done)
		counter := 0
		for spEdges := range workers.CollectResults() {
			if len(spEdges) == 0 {
				continue
			}
			for j := 0; j < len(spEdges)-1; j++ {
				e := int(spEdges[j])
				eNext := int(spEdges[j+1])
				matrix.Set(matrix.Get(e, eNext)+1, e, eNext)
			}
			counter++
			if counter%100 == 0 {
				fmt.Printf("completed query: %v\n", counter)
				if err := matrix.WriteToFile(transitionMatrixFilepath); err != nil {
					fmt.Printf("error writing transition matrix: %v\n", err)
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
	<-done

	if err := matrix.WriteToFile(transitionMatrixFilepath); err != nil {
		return nil, fmt.Errorf("buildOrReadTransitionMatrix: WriteToFile failed: %w", err)
	}
	logger.Sugar().Infof("transition matrix built")
	return matrix, nil
}

func readTrackFile(trackPath string) ([]trackPoint, error) {
	f, err := os.OpenFile(trackPath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, fmt.Errorf("readTrackFile: open failed: %w", err)
	}
	defer f.Close()

	reader := csv.NewReader(f)
	reader.FieldsPerRecord = -1
	reader.TrimLeadingSpace = true

	points := make([]trackPoint, 0)
	lineNo := 0
	for {
		record, err := reader.Read()
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, fmt.Errorf("readTrackFile: read CSV failed at line %d: %w", lineNo+1, err)
		}
		lineNo++
		if len(record) == 0 || strings.TrimSpace(record[0]) == "" {
			continue
		}
		if len(record) < 3 {
			return nil, fmt.Errorf("readTrackFile: invalid line %d", lineNo)
		}

		timeSec, err := util.ParseTextFloat64(strings.TrimSpace(record[0]))
		if err != nil {
			return nil, fmt.Errorf("readTrackFile: parse time at line %d failed: %w", lineNo, err)
		}
		lat, err := util.ParseTextFloat64(strings.TrimSpace(record[1]))
		if err != nil {
			return nil, fmt.Errorf("readTrackFile: parse latitude at line %d failed: %w", lineNo, err)
		}
		lon, err := util.ParseTextFloat64(strings.TrimSpace(record[2]))
		if err != nil {
			return nil, fmt.Errorf("readTrackFile: parse longitude at line %d failed: %w", lineNo, err)
		}
		points = append(points, trackPoint{timeSec: timeSec, lat: lat, lon: lon})
	}
	if len(points) == 0 {
		return nil, fmt.Errorf("readTrackFile: no points loaded from %s", trackPath)
	}
	return points, nil
}

func readGroundTruthFile(outputPath string) ([]int64, error) {
	f, err := os.OpenFile(outputPath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, fmt.Errorf("readGroundTruthFile: open failed: %w", err)
	}
	defer f.Close()

	reader := csv.NewReader(f)
	reader.FieldsPerRecord = -1
	reader.TrimLeadingSpace = true

	edgeIDs := make([]int64, 0)
	lineNo := 0
	for {
		record, err := reader.Read()
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, fmt.Errorf("readGroundTruthFile: read CSV failed at line %d: %w", lineNo+1, err)
		}
		lineNo++
		if len(record) == 0 || strings.TrimSpace(record[0]) == "" {
			continue
		}
		if len(record) < 2 {
			return nil, fmt.Errorf("readGroundTruthFile: invalid line %d", lineNo)
		}
		edgeID, err := util.ParseTextInt64(strings.TrimSpace(record[1]))
		if err != nil {
			return nil, fmt.Errorf("readGroundTruthFile: parse edge id at line %d failed: %w", lineNo, err)
		}
		edgeIDs = append(edgeIDs, edgeID)
	}
	if len(edgeIDs) == 0 {
		return nil, fmt.Errorf("readGroundTruthFile: no edge ids loaded from %s", outputPath)
	}
	return edgeIDs, nil
}

func listTrajectoryCases() ([]trajectoryCase, error) {
	matches, err := filepath.Glob(filepath.Join(giscupTrainingInputDir, "input_*.txt"))
	if err != nil {
		return nil, fmt.Errorf("listTrajectoryCases: glob failed: %w", err)
	}
	sort.Strings(matches)
	cases := make([]trajectoryCase, 0, len(matches))
	for _, inputPath := range matches {
		name := filepath.Base(inputPath)
		id := strings.TrimSuffix(strings.TrimPrefix(name, "input_"), ".txt")
		outputPath := filepath.Join(giscupTrainingOutputDir, "output_"+id+".txt")
		if _, err := os.Stat(outputPath); err != nil {
			return nil, fmt.Errorf("listTrajectoryCases: missing ground truth for %s at %s", inputPath, outputPath)
		}
		cases = append(cases, trajectoryCase{id: id, inputFilePath: inputPath, outputFilePath: outputPath})
	}
	if len(cases) == 0 {
		return nil, fmt.Errorf("listTrajectoryCases: no input_*.txt files found under %s", giscupTrainingInputDir)
	}
	return cases, nil
}

func writePredictionFile(caseID string, points []trackPoint, matchedPoints []*da.MatchedGPSPoint, graph *da.Graph) error {
	if err := os.MkdirAll(giscupPredictionOutputDir, 0755); err != nil {
		return fmt.Errorf("writePredictionFile: MkdirAll failed: %w", err)
	}
	outputPath := filepath.Join(giscupPredictionOutputDir, "output_"+caseID+".txt")
	f, err := os.OpenFile(outputPath, os.O_CREATE|os.O_TRUNC|os.O_WRONLY, 0644)
	if err != nil {
		return fmt.Errorf("writePredictionFile: open failed: %w", err)
	}
	defer f.Close()

	writer := csv.NewWriter(f)
	for i, point := range points {
		edgeID := int64(-1)
		if i < len(matchedPoints) && matchedPoints[i].GetEdgeId() != da.INVALID_EDGE_ID {
			edgeID = graph.GetOsmWayId(matchedPoints[i].GetEdgeId())
		}
		if err := writer.Write([]string{
			strconv.FormatFloat(point.timeSec, 'f', -1, 64),
			strconv.FormatInt(edgeID, 10),
		}); err != nil {
			return fmt.Errorf("writePredictionFile: write failed: %w", err)
		}
	}
	writer.Flush()
	if err := writer.Error(); err != nil {
		return fmt.Errorf("writePredictionFile: flush failed: %w", err)
	}
	return nil
}

func writePolylineFiles(caseID string, points []trackPoint, matchedPoints []*da.MatchedGPSPoint) error {
	if err := os.MkdirAll(giscupPolylineOutputDir, 0755); err != nil {
		return fmt.Errorf("writePolylineFiles: MkdirAll failed: %w", err)
	}

	matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))
	for _, mp := range matchedPoints {
		if mp.GetEdgeId() == da.INVALID_EDGE_ID {
			continue
		}
		matchedCoords = append(matchedCoords, mp.GetMatchedCoord())
	}
	resultPolyline := ""
	if len(matchedCoords) > 0 {
		resultPolyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
	}

	gpsCoords := make([]da.Coordinate, 0, len(points))
	for _, p := range points {
		gpsCoords = append(gpsCoords, da.NewCoordinate(p.lat, p.lon))
	}
	gpsTrackPolyline := ""
	if len(gpsCoords) > 0 {
		gpsTrackPolyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(gpsCoords))
	}

	resultPath := filepath.Join(giscupPolylineOutputDir, "result_polyline_"+caseID+".txt")
	if err := os.WriteFile(resultPath, []byte(resultPolyline), 0644); err != nil {
		return fmt.Errorf("writePolylineFiles: write result polyline failed: %w", err)
	}

	gpsTrackPath := filepath.Join(giscupPolylineOutputDir, "gps_track_"+caseID+".txt")
	if err := os.WriteFile(gpsTrackPath, []byte(gpsTrackPolyline), 0644); err != nil {
		return fmt.Errorf("writePolylineFiles: write gps track polyline failed: %w", err)
	}

	return nil
}

func computeMetrics(graph *da.Graph, groundTruthEdgeIDs []int64, matchedPoints []*da.MatchedGPSPoint, edgeLengths map[int64]float64) (float64, float64) {
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

	crp := 0.0 // section V Accuracy: https://mod.wict.pku.edu.cn/docs/20240422170836017278.pdf
	if len(matchedEdgeSet) > 0 {
		crp = correctMatched / float64(len(matchedEdgeSet))
	}
	if util.Eq(lengthOfCorrectRoute, 0) {
		return crp, math.Inf(1)
	}

	lengthOfErrorneouslyAdded := 0.0
	for edgeID, length := range matchedEdgeSet {
		if !groundTruthSet[edgeID] {
			lengthOfErrorneouslyAdded += length
		}
	}

	lengthOfErrorneouslySubtracted := 0.0
	for edgeID := range groundTruthSet {
		if _, ok := matchedEdgeSet[edgeID]; !ok {
			lengthOfErrorneouslySubtracted += edgeLengths[edgeID]
		}
	}

	rmf := (lengthOfErrorneouslyAdded + lengthOfErrorneouslySubtracted) / lengthOfCorrectRoute
	return crp, rmf
}

func evaluateCase(tc trajectoryCase, graph *da.Graph, rtree *spatialindex.Rtree, transitionMatrix *da.SparseMatrix[int], edgeLengths map[int64]float64,
	getSegmentLength func(da.Index) float64) (evalMetrics, error) {
	trackPoints, err := readTrackFile(tc.inputFilePath)
	if err != nil {
		return evalMetrics{}, err
	}
	groundTruthEdgeIDs, err := readGroundTruthFile(tc.outputFilePath)
	if err != nil {
		return evalMetrics{}, err
	}

	onlineMM := online.NewOnlineMapMatchMHT(
		graph,
		rtree,
		defaultSpeedMPS, defaultSpeedMPS,
		0.001, 5.0, 0.000001, 0.06, 3,
		transitionMatrix,
		getSegmentLength,
	)

	var (
		prevLat, prevLon float64
		prevTimeSec      float64
		hasPrev          bool
		candidates       []*ma.Candidate
		speedMeanK       = defaultSpeedMPS
		speedStdK        = defaultSpeedMPS
		lastBearing      = 0.0
		k                = 1
	)

	matchedPoints := make([]*da.MatchedGPSPoint, 0, len(trackPoints))
	caseStart := time.Now()
	pointRuntimeMic := 0.0
	for _, point := range trackPoints {
		deltaTime := 1.0
		speed := defaultSpeedMPS
		if hasPrev {
			deltaTime = point.timeSec - prevTimeSec
			if util.Gt(deltaTime, 0) {
				distanceKM := geo.CalculateGreatCircleDistance(prevLat, prevLon, point.lat, point.lon)
				speed = util.KilometerToMeter(distanceKM) / deltaTime
			} else {
				deltaTime = 1.0
			}
		} else {
			hasPrev = true
		}

		prevLat = point.lat
		prevLon = point.lon
		prevTimeSec = point.timeSec

		curGPSTime := time.Unix(0, 0).Add(time.Duration(point.timeSec * float64(time.Second)))
		curGPS := da.NewGPSPoint(point.lat, point.lon, curGPSTime, speed, deltaTime)

		start := time.Now()
		matchedPoint, nextCandidates, nextSpeedMeanK, nextSpeedStdK := onlineMM.OnlineMapMatch(
			curGPS, k, candidates, speedMeanK, speedStdK, lastBearing,
		)

		pointRuntimeMic += float64(time.Since(start).Microseconds())

		candidates = nextCandidates
		speedMeanK = nextSpeedMeanK
		speedStdK = nextSpeedStdK
		lastBearing = matchedPoint.GetBearing()
		matchedPoints = append(matchedPoints, matchedPoint)
		k++
	}

	if err := writePredictionFile(tc.id, trackPoints, matchedPoints, graph); err != nil {
		return evalMetrics{}, err
	}
	if err := writePolylineFiles(tc.id, trackPoints, matchedPoints); err != nil {
		return evalMetrics{}, err
	}

	crp, rmf := computeMetrics(graph, groundTruthEdgeIDs, matchedPoints, edgeLengths)
	return evalMetrics{
		caseID:          tc.id,
		crp:             crp,
		rmf:             rmf,
		points:          len(trackPoints),
		runtimeMS:       float64(time.Since(caseStart).Milliseconds()),
		pointRuntimeMic: pointRuntimeMic,
	}, nil
}

func ensureTrainingDataExists() error {
	if _, err := os.Stat(giscupTrainingRoot); err != nil {
		return fmt.Errorf("ensureTrainingDataExists: GIS Cup training data not found at %s", giscupTrainingRoot)
	}
	return nil
}

func main() {
	re, graph, logger, transitionMatrix, edgeLengths, err := buildCRPGraph()
	if err != nil {
		panic(err)
	}

	if err := ensureTrainingDataExists(); err != nil {
		panic(err)
	}

	trajectoryCases, err := listTrajectoryCases()
	if err != nil {
		panic(err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)

	results := make([]evalMetrics, 0, len(trajectoryCases))
	for _, tc := range trajectoryCases {
		res, err := evaluateCase(tc, graph, rtree, transitionMatrix, edgeLengths, func(eID da.Index) float64 {
			return re.GetRoutingEngine().GetSegmentLength(eID, true)
		})
		if err != nil {
			logger.Sugar().Warnf("skip trajectory %s: %v", tc.id, err)
			continue
		}
		fmt.Printf("trajectory %s completed: CRP=%.6f RMF=%.6f points=%d runtime=%.2f ms\n",
			res.caseID, res.crp, res.rmf, res.points, res.runtimeMS)
		results = append(results, res)
	}

	if len(results) == 0 {
		panic(fmt.Errorf("all GIS Cup trajectory evaluations failed"))
	}

	sumCRP := 0.0
	sumRMF := 0.0
	totalPoints := 0.0
	totalRuntimeMS := 0.0
	totalPointRuntimeMic := 0.0
	for _, res := range results {
		sumCRP += res.crp
		sumRMF += res.rmf
		totalPoints += float64(res.points)
		totalRuntimeMS += res.runtimeMS
		totalPointRuntimeMic += res.pointRuntimeMic
	}

	meanCRP := sumCRP / float64(len(results))
	meanRMF := sumRMF / float64(len(results))
	avgRuntimePerGPSPoint := totalPointRuntimeMic / totalPoints
	matchingEfficiency := totalPoints / totalRuntimeMS

	fmt.Printf("\nGIS Cup evaluated trajectories: %d\n", len(results))
	fmt.Printf("Average CRP (accuracy): %.6f\n", meanCRP)
	fmt.Printf("Average RMF: %.6f\n", meanRMF)
	fmt.Printf("Average runtime per GPS point: %.6f microseconds\n", avgRuntimePerGPSPoint)
	fmt.Printf("Matching efficiency: %.6f points/ms\n", matchingEfficiency)
}

package main

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"math"
	"math/rand"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

// go run eval/crp_alt/online_map_matching/newsonkrumm/main.go

func parseLineString(s string, vertexCount int) ([]da.Coordinate, error) {
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

type krummEdge struct {
	eId         int64
	fromId      uint32
	toId        uint32
	speed       float64
	twoWay      bool
	vertexCount int
	geometry    []da.Coordinate
}

func newKrummEdge(eid int64, fromId, toId uint32, speed float64, twoWay bool, vertexCount int, geometry []da.Coordinate) krummEdge {
	return krummEdge{eId: eid, fromId: fromId, toId: toId, speed: speed, twoWay: twoWay, vertexCount: vertexCount, geometry: geometry}
}

type query struct {
	s, t da.Index
}

func newQuery(s, t da.Index) query {
	return query{
		s: s,
		t: t,
	}
}

const (
	transitionMatrixFilepath        = "./data/eval/mapmatching/omm_transition_history_id.mm"
	roadnetworkFilepath             = "./data/eval/mapmatching/road_network.txt"
	graphFile                string = "./data/original_eval_mm.graph"
	overlayGraphFile         string = "./data/overlay_graph_eval_mm.graph"
	metricsFile              string = "./data/metrics_eval_mm.txt"
	roadnetworkDriveFile            = "https://drive.google.com/uc?export=download&id=1ba1CcLbTRerbDVNN91wTNfrS85EJGhG6"
)

func download(filePath, url string, logger *zap.Logger) {
	logger.Sugar().Infof("downloading evaluation road network dataset.....")

	if _, err := os.Stat(filePath); os.IsNotExist(err) {

		dir := filepath.Dir(filePath)
		if err := os.MkdirAll(dir, os.ModePerm); err != nil {
			panic(err)
		}

		output, err := os.Create(filePath)
		if err != nil {
			panic(err)
		}
		defer output.Close()

		logger.Sugar().Infof("downloading file......")
		response, err := http.Get(url)
		if err != nil {
			panic(err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			panic(err)
		}

		logger.Sugar().Infof("download complete")
	}

	logger.Sugar().Infof("download completed")
}

func buildRoadNetworkCRPGraph(filepath string) (*engine.Engine, *da.Graph, *zap.Logger, *da.SparseMatrix[int], map[int64]float64, error) {
	logger, err := logger.New()
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	logger.Sugar().Infof("downloading evaluation road network dataset.....")
	download(roadnetworkFilepath, roadnetworkDriveFile, logger)

	logger.Sugar().Infof("download completed")

	logger.Sugar().Infof("building road network graph & running preprocessing, customization phase of Customizable Route Planning CRP....")

	f, err := os.OpenFile(filepath, os.O_RDONLY, 0644)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}
	defer f.Close()

	br := bufio.NewReader(f)

	// Edge ID	From Node ID	To Node ID	Two Way	 Speed (m/s)	Vertex Count	LINESTRING()

	edges := make([]krummEdge, 0)
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

		edgeId, err := strconv.ParseInt(ff[0], 10, 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		fromId, err := strconv.ParseInt(ff[1], 10, 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}

		toId, err := strconv.ParseInt(ff[2], 10, 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		twoWayInt, err := strconv.ParseInt(ff[3], 10, 32)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		var twoWay bool
		if twoWayInt == 1 {
			twoWay = true
		} else {
			twoWay = false
		}

		speed, err := strconv.ParseFloat(ff[4], 64)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}

		vertexCount, err := strconv.ParseInt(ff[5], 10, 32)
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}

		const key = "LINESTRING("

		idx := strings.Index(line, key)
		if idx == -1 {
			return nil, nil, nil, nil, nil, fmt.Errorf("LINESTRING not found")
		}

		lineString := line[idx:]
		edgeGeometry, err := parseLineString(lineString, int(vertexCount))
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}

		fromNId, ok1 := nodeIdMap[fromId]
		if !ok1 {
			fromNId = uint32(len(nodeIdMap))
			nodeIdMap[fromId] = fromNId
			nodeCoords = append(nodeCoords, edgeGeometry[0])
		}

		toNId, ok2 := nodeIdMap[toId]
		if !ok2 {
			toNId = uint32(len(nodeIdMap))
			nodeIdMap[toId] = toNId
			nodeCoords = append(nodeCoords, edgeGeometry[len(edgeGeometry)-1])
		}

		e := newKrummEdge(edgeId, fromNId, toNId, speed, twoWay, int(vertexCount), edgeGeometry)
		edges = append(edges, e)

	}
	graphStorage := da.NewGraphStorage()

	graphEdges := make([]osmparser.Edge, 0, len(edges))

	edgeLength := make(map[int64]float64)

	for _, e := range edges {

		from := e.fromId
		to := e.toId

		if from == to {
			continue
		}
		eGeometry := e.geometry

		distance := 0.0
		for i := 0; i < len(eGeometry); i++ {

			if i > 0 {
				distance += geo.CalculateHaversineDistance(eGeometry[i-1].GetLat(), eGeometry[i-1].GetLon(),
					eGeometry[i].GetLat(), eGeometry[i].GetLon())
			}
		}

		distanceInMeter := util.KilometerToMeter(distance)
		speed := e.speed
		travelTimeWeight := distanceInMeter / util.KMHToMMin(speed) // in minutes

		edgeLength[e.eId] = distanceInMeter
		if e.twoWay {

			// forward edge (two way)
			startPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendOsmNodePoints(eGeometry)
			endPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendMapEdgeInfo(da.NewEdgeExtraInfo(
				0, 0, 0,
				1,
				da.Index(startPointsIndex), da.Index(endPointsIndex),
				int64(e.eId),
			))

			graphEdge := osmparser.NewEdge(
				e.fromId,
				e.toId,
				travelTimeWeight,
				distanceInMeter,
				distanceInMeter,
				0,
			)

			graphEdges = append(graphEdges, graphEdge)

			// reverse edge (two way)
			graphStorage.AppendMapEdgeInfo(da.NewEdgeExtraInfo(
				0, 0, 0,
				1,
				da.Index(endPointsIndex), da.Index(startPointsIndex),
				int64(e.eId),
			))

			graphRevE := osmparser.NewEdge(
				e.toId,
				e.fromId,
				travelTimeWeight,
				distanceInMeter,
				distanceInMeter,
				0,
			)

			graphEdges = append(graphEdges, graphRevE)

		} else {
			// one way
			// only add forward edge
			startPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendOsmNodePoints(eGeometry)
			endPointsIndex := graphStorage.GetOsmNodePointsCount()

			graphStorage.AppendMapEdgeInfo(da.NewEdgeExtraInfo(
				0, 0, 0,
				1,
				da.Index(startPointsIndex), da.Index(endPointsIndex),
				int64(e.eId),
			))

			graphEdge := osmparser.NewEdge(
				e.fromId,
				e.toId,
				travelTimeWeight,
				distanceInMeter,
				distanceInMeter,
				0,
			)
			graphEdges = append(graphEdges, graphEdge)
		}
	}

	op := osmparser.NewOSMParserV2()
	n := len(nodeIdMap)

	acceptedNodeMap := make(map[int64]osmparser.NodeCoord, n)
	nodeToOsmId := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = osmparser.NewNodeCoord(nodeCoords[i].GetLat(), nodeCoords[i].GetLon())
		nodeToOsmId[da.Index(i)] = int64(i)
	}

	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmId)
	g := op.BuildGraph(graphEdges, graphStorage, uint32(len(nodeIdMap)), true)
	g.SetGraphStorage(graphStorage)

	us := []int{8, 11, 14, 16}

	ps := make([]int, len(us))

	for i := 0; i < len(ps); i++ {
		pow := us[i]
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps), 1,
		g, logger, true, false, true,
	)
	mp.RunMultilevelPartitioning()

	mlp := mp.BuildMLP()

	prep := preprocesser.NewPreprocessor(g, mlp, logger, graphFile, overlayGraphFile)
	err = prep.PreProcessing(true)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	cust := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)
	m, err := cust.Customize()
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(2, m, cust, logger)
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	logger.Sugar().Infof("building road network graph & running preprocessing, customization phase of Customizable Route Planning CRP done....")

	n = g.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))

	logger.Sugar().Infof("building transition matrix....")
	numQueries := math.Pow(10, 3) * 5
	i := 0
	queries := make([]query, 0, n)

	for i < int(numQueries) {
		s := da.Index(rd.Intn(n))
		t := da.Index(rd.Intn(n))
		if s == t {
			continue
		}
		if !g.VerticeUToVConnected(s, t) {
			continue
		}

		queries = append(queries, newQuery(s, t))
		i++
	}

	computeRoute := func(q query) []da.OutEdge {
		s, t := q.s, q.t
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)
		as := g.GetDummyOutEdgeId(s)
		at := g.GetDummyInEdgeId(t)

		_, _, _, edges, _ := crpQuery.ShortestPathSearch(as, at)
		return edges
	}

	workers := concurrent.NewWorkerPool[query, []da.OutEdge](100, len(queries))

	for _, qq := range queries {
		workers.AddJob(qq)
	}
	workers.Close()
	workers.Start(computeRoute)
	workers.Wait()

	var N *da.SparseMatrix[int]
	_, err = os.Stat(transitionMatrixFilepath)
	if err == nil {
		logger.Info("reading transition matrix from file...")

		N, err = da.ReadSparseMatrixFromFile[int](transitionMatrixFilepath, int(0),
			func(a, b int) bool { return a == b })
		if err != nil {
			panic(err)
		}
	} else {
		N = da.NewSparseMatrix[int](g.NumberOfEdges(), g.NumberOfEdges(),
			0, func(a, b int) bool { return a == b })
	}

	counter := 0

	for spEdges := range workers.CollectResults() {
		if len(spEdges) == 0 {
			continue
		}

		for j := 0; j < len(spEdges)-1; j++ {
			e := int(spEdges[j].GetEdgeId())
			eNext := int(spEdges[j+1].GetEdgeId())
			N.Set(N.Get(e, eNext)+1, e, eNext)
		}
		counter++

		if counter%1e2 == 0 {
			fmt.Printf("completed query: %v\n", counter)
			N.WriteToFile(transitionMatrixFilepath)
		}
	}

	logger.Sugar().Infof(" transition matrix built....")

	return re, g, logger, N, edgeLength, nil
}

const (
	gpsDataFilepath          = "./data/eval/mapmatching/gps_data.txt"
	groundTruthDataFilepath  = "./data/eval/mapmatching/ground_truth.txt"
	gpsDataDriveFile         = "https://drive.google.com/uc?export=download&id=1QCrMnchOjCfOMQet9Oon-dmZ36MasTjA"
	groundTruthDataDriveFile = "https://drive.google.com/uc?export=download&id=11LxzpV-VDCImDq3OWN3m3tukFKmwl9Fn"
)

func main() {
	_, g, logger, N, edgeLength, err := buildRoadNetworkCRPGraph(roadnetworkFilepath)
	if err != nil {
		panic(err)
	}

	download(gpsDataFilepath, gpsDataDriveFile, logger)
	download(groundTruthDataFilepath, groundTruthDataDriveFile, logger)

	rtree := spatialindex.NewRtree()
	rtree.Build(g, 0.05, logger)
	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(g, rtree, 8.33333, 8.3333, 0.0001, 4.07, 1.0, 0.0000001,
		0.06, 3, N) // speed in meter/s, default sampling interval 1.0 seconds (using seatle dataset)

	f, err := os.OpenFile(gpsDataFilepath, os.O_RDONLY, 0644)
	if err != nil {
		panic(err)
	}
	defer f.Close()

	br := bufio.NewReader(f)
	var (
		prevLat, prevLon float64
		prevTime         time.Time
		hasPrev          bool
		candidates       []*ma.Candidate
		speedMeanK       float64 = 8.333
		speedStdK        float64 = 8.333
		lastBearing      float64 = 0.0
		k                        = 1
		matchedPoint     *da.MatchedGPSPoint
	)

	mapMatchPointResult := make([]*da.MatchedGPSPoint, 0)
	avgRuntimePerGpsPoint := 0.0

	for {
		line, err := util.ReadLine(br)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			panic(err)
		}

		ff := util.Fields(line)
		if ff[0] == "Date" {
			// skip header
			continue
		}
		dateTime := ff[0] + " " + ff[1]
		lat, err := strconv.ParseFloat(ff[2], 64)
		if err != nil {
			panic(err)
		}
		lon, err := strconv.ParseFloat(ff[3], 64)
		if err != nil {
			panic(err)
		}

		curGpsTime, err := time.Parse("02-Jan-2006 15:04:05", dateTime)
		if err != nil {
			panic(err)
		}
		var deltaTime float64
		var speed float64

		if hasPrev {
			deltaTime = curGpsTime.Sub(prevTime).Seconds()

			if da.Gt(deltaTime, 0) {
				dist := geo.CalculateHaversineDistance(prevLat, prevLon, lat, lon)
				speed = util.KilometerToMeter(dist) / deltaTime
			}
		} else {
			hasPrev = true
		}

		prevLat, prevLon = lat, lon
		prevTime = curGpsTime

		now := time.Now()
		curGps := da.NewGPSPoint(lat, lon, curGpsTime, speed, deltaTime, false)
		matchedPoint, candidates, speedMeanK, speedStdK = onlineMapMatcherEngine.OnlineMapMatch(curGps, k, candidates, speedMeanK, speedStdK, lastBearing)
		k++
		lastBearing = matchedPoint.GetBearing()
		mapMatchPointResult = append(mapMatchPointResult, matchedPoint)
		avgRuntimePerGpsPoint += float64(time.Now().Sub(now).Microseconds())
	}

	// mteric pertama:
	// https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf
	// figure 6, RMF metric
	// RMF=(lengthOfErrorneouslyAdded+lengthOfErrorneouslySubtracted)/lengthOfCorrectRoute
	// metric kedua:
	// https://ieeexplore-ieee-org.ezproxy.ugm.ac.id/document/9831841/
	// Correct Road Percentage = # corrected matched roads / # roads of matched trips
	groundTruthFile, err := os.Open(groundTruthDataFilepath)
	if err != nil {
		panic(err)
	}
	brGroundTruth := bufio.NewReader(groundTruthFile)

	traversedEdges := make(map[int64]float64, 576)
	for {
		line, err := util.ReadLine(brGroundTruth)
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			panic(err)
		}
		ff := util.Fields(line)
		if ff[0] == "Edge" {
			// skip header
			continue
		}

		edgeId, err := strconv.ParseInt(ff[0], 10, 64)
		if err != nil {
			panic(err)
		}

		traversedEdges[edgeId] = edgeLength[edgeId]
	}

	lengthOfCorrectRoute := 0.0

	for _, eLength := range traversedEdges {
		lengthOfCorrectRoute += eLength
	}

	lengthOfErrorneouslyAdded := 0.0
	lengthOfErrorneouslySubtracted := 0.0

	numOfCorrectMatchedRoads := 0.0
	numberOfRoadsOfMatchedTrips := 0.0
	matchedEdgeSet := make(map[int64]float64)

	matchedCoords := make([]da.Coordinate, 0, len(mapMatchPointResult))
	for i := 0; i < len(mapMatchPointResult); i++ {

		curMatchedEId := mapMatchPointResult[i].GetEdgeId()
		if curMatchedEId == da.INVALID_EDGE_ID {
			continue
		}
		matchedDataEId := g.GetOsmWayId(curMatchedEId)

		matchedEdgeSet[matchedDataEId] = g.GetOutEdge(curMatchedEId).GetLength()
		_, inGroundTruth := traversedEdges[matchedDataEId]
		if inGroundTruth {
			numOfCorrectMatchedRoads++
		}
		matchedCoords = append(matchedCoords, mapMatchPointResult[i].GetMatchedCoord())
		numberOfRoadsOfMatchedTrips++
	}

	for matchedDataEId, eLength := range matchedEdgeSet {
		_, inGroundTruth := traversedEdges[matchedDataEId]
		if !inGroundTruth {
			lengthOfErrorneouslyAdded += eLength
		}
	}

	for eId, _ := range traversedEdges {
		eLength, inMapMatchResult := matchedEdgeSet[eId]
		if !inMapMatchResult {
			lengthOfErrorneouslySubtracted += eLength
		}
	}

	avgRuntimePerGpsPoint /= float64(k - 1)
	rmf := (lengthOfErrorneouslyAdded + lengthOfErrorneouslySubtracted) / lengthOfCorrectRoute
	crp := numOfCorrectMatchedRoads / numberOfRoadsOfMatchedTrips
	fmt.Printf("Route Mismatch Fraction (RMF): %v\n", rmf)
	fmt.Printf("Correct Road Percentage (CRP) or accuracy: %v\n", crp)
	fmt.Printf("avg runtime per gpt point: %v microseconds\n", avgRuntimePerGpsPoint)

	polyline := geo.PoylineFromCoords(datastructure.NewGeoCoordinates(matchedCoords))
	fmt.Printf("polyline of map matching result: %s\n", polyline)

	// todo: upload dataset ke drive, disini kita download file dataset dari drivenya (DONE)
	// todo2: benerin build graph roadnetwork untuk dataset ini,  masih ada yang salah (DONE)
	// todo3: benerin offline map matching pakai hidden markov model
	// todo4: add evaluasi online map matching pakai dataset: https://github.com/Hanwen-Hu/AMM/tree/main/MatchData/Shanghai
	// todo5: benerin frontend online map matching https://github.com/lintang-b-s/navigatorx-crp-fe
	// todo6: add reroute feature, pas user keluar dari rute pilihan (dari hasil online map matching), reroute -> diassalow u turn
	// todo7: deploy navigatorx ke vps lagi & coba fitur turn-by-turn navigation,online map  matching & reroute
	//

	/*
	   	Route Mismatch Fraction (RMF): 0.06885194640640466

	   Correct Road Percentage (CRP) or accuracy: 0.9563139025361838
	   avg runtime per gpt point: 8.464612933209402 microseconds
	*/
}

package onlinemapmatching

import (
	"bufio"
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"math/rand"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

const (
	nkRoadNetworkDriveFile = "https://drive.google.com/uc?export=download&id=1ba1CcLbTRerbDVNN91wTNfrS85EJGhG6"
	nkGPSDataDriveFile     = "https://drive.google.com/uc?export=download&id=1QCrMnchOjCfOMQet9Oon-dmZ36MasTjA"
	nkGroundTruthDriveFile = "https://drive.google.com/uc?export=download&id=11LxzpV-VDCImDq3OWN3m3tukFKmwl9Fn"
	nkExpectedMaxRMF       = 0.001
	nkExpectedMaxRMFOnline = 0.09
)

type nkEdge struct {
	eId         uint64
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

func nkParseLineString(s []byte, vertexCount int) ([]da.Coordinate, error) {
	const prefixLength = len("LINESTRING(")
	content := s[prefixLength : len(s)-1]
	parts := bytes.Split(content, []byte(","))
	coords := make([]da.Coordinate, 0, vertexCount)
	for _, p := range parts {
		xy := bytes.Fields(bytes.TrimSpace(p))
		if len(xy) < 2 {
			return nil, fmt.Errorf("invalid coordinates")
		}
		lon, err1 := util.ParseTextFloat64(string(xy[0]))
		lat, err2 := util.ParseTextFloat64(string(xy[1]))
		if err1 != nil || err2 != nil {
			return nil, fmt.Errorf("%v %w", err1, err2)
		}
		coords = append(coords, da.NewCoordinate(lat, lon))
	}
	return coords, nil
}

func nkDownload(filePath, url string, zlog *zap.Logger, t *testing.T, name string) error {
	if _, err := os.Stat(filePath); os.IsNotExist(err) {
		t.Logf("downloading evaluation %s dataset.....", name)
		zlog.Sugar().Infof("downloading evaluation %s dataset.....", name)

		if err := util.EnsureDirExists(filePath); err != nil {
			return fmt.Errorf("download: %w", err)
		}

		output, err := os.Create(filePath)
		if err != nil {
			return fmt.Errorf("download: Create failed %w", err)
		}
		defer output.Close()

		t.Logf("downloading file......")
		zlog.Sugar().Infof("downloading file......")
		response, err := http.Get(url)
		if err != nil {
			return fmt.Errorf("download: http.Get failed %w", err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		if err != nil {
			return fmt.Errorf("download: io.Copy failed %w", err)
		}

		t.Logf("download complete")
		zlog.Sugar().Infof("download complete")
	}
	return nil
}

// https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/
func nkBuildRoadNetworkCRPGraph(t *testing.T, workingDir string) (*engine.Engine[int32], *da.Graph, *zap.Logger, *da.SparseMatrix, map[uint64]float64, error) {
	zlog, err := logger.New()
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	config.InitProfileConfig("car", "newsonkrumm", pkg.TEST)

	roadnetworkFilepath := filepath.Join(workingDir, "data/eval/mapmatching/road_network.txt")
	if err := nkDownload(roadnetworkFilepath, nkRoadNetworkDriveFile, zlog, t, "road network"); err != nil {
		return nil, nil, nil, nil, nil, err
	}

	t.Logf("building road network graph & running preprocessing, customization phase of Customizable Route Planning CRP....")
	zlog.Sugar().Infof("building road network graph & running preprocessing, customization phase of Customizable Route Planning CRP....")

	f, err := os.OpenFile(roadnetworkFilepath, os.O_RDONLY, 0600)
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
		edgeID, err := util.ParseTextInt64(ff[0])
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		fromID, err := util.ParseTextInt64(ff[1])
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		toID, err := util.ParseTextInt64(ff[2])
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		twoWayInt, err := util.ParseTextInt32(ff[3])
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		twoWay := twoWayInt == 1
		speed, err := util.ParseTextFloat64(ff[4])
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}
		vertexCount, err := util.ParseTextInt32(ff[5])
		if err != nil {
			return nil, nil, nil, nil, nil, err
		}

		idx := strings.Index(line, "LINESTRING(")
		if idx == -1 {
			return nil, nil, nil, nil, nil, fmt.Errorf("LINESTRING not found")
		}
		edgeGeometry, err := nkParseLineString([]byte(line[idx:]), int(vertexCount))
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

		edges = append(edges, nkEdge{eId: uint64(edgeID), fromId: fromNID, toId: toNID, speed: speed, twoWay: twoWay, vertexCount: int(vertexCount), geometry: edgeGeometry})
	}

	rn := da.NewRoadNetworkDataContainer(54)
	graphEdges := make([]extractor.Edge[int32], 0, len(edges))
	edgeLength := make(map[uint64]float64)

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

		startPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendOsmNodePoints(e.geometry, make([]uint64, len(e.geometry)))
		endPointsIndex := rn.GetOsmNodePointsCount()
		rn.AppendSegmentData(int64(e.eId), da.Index(startPointsIndex), da.Index(endPointsIndex), 0, 0, 0, 1, da.NewEmptyTurnLanesData())
		eId := len(graphEdges)
		rn.SetSegmentFlag(da.Index(eId), 0)

		graphEdge := extractor.NewEdge[int32](
			e.fromId, e.toId, int32(util.RoundCentiseconds(travelTimeWeight)), util.RoundCentimeters(distanceInMeter),
		)
		graphEdges = append(graphEdges, graphEdge)
		if e.twoWay {
			rn.AppendSegmentData(int64(e.eId)*2, da.Index(endPointsIndex), da.Index(startPointsIndex), 0, 0, 0, 1, da.NewEmptyTurnLanesData())
			reverseEdge := extractor.NewEdge[int32](
				e.toId, e.fromId, int32(util.RoundCentiseconds(travelTimeWeight)), util.RoundCentimeters(distanceInMeter),
			)
			eId := len(graphEdges)
			rn.SetSegmentFlag(da.Index(eId), 0)

			graphEdges = append(graphEdges, reverseEdge)
		}
	}

	op := extractor.NewExtractor[int32]()
	n := len(nodeIdMap)
	acceptedNodeMap := make(map[int64]extractor.NodeCoord, n)
	nodeToOsmID := make(map[da.Index]int64, n)
	for i := 0; i < n; i++ {
		acceptedNodeMap[int64(i)] = extractor.NewNodeCoord(nodeCoords[i].GetLat(), nodeCoords[i].GetLon())
		nodeToOsmID[da.Index(i)] = int64(i)
	}
	op.SetAcceptedNodeMap(acceptedNodeMap)
	op.SetNodeToOsmId(nodeToOsmID)
	g, timeFunction, vertexTurnTablePtr, flattenTurnMatrices := op.BuildGraph(graphEdges, rn, uint32(len(nodeIdMap)), true)
	rn.BuildNameTable(map[uint32]string{0: ""})
	g, timeFunction = extractor.BuildEdgeBasedGraph(g, timeFunction, vertexTurnTablePtr, flattenTurnMatrices, rn)

	us := []int{8, 11, 14, 16}
	ps := make([]int, len(us))
	for i := range ps {
		ps[i] = 1 << us[i]
	}
	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 1, g, zlog, false)
	mp.RunMultilevelPartitioning()
	if err := mp.SaveToFile(); err != nil {
		return nil, nil, nil, nil, nil, err
	}
	mlp := da.NewPlainMLP()
	if err := mlp.ReadMlpFile(); err != nil {
		return nil, nil, nil, nil, nil, err
	}
	prep := preprocesser.NewPreprocessor(g, rn, timeFunction, mlp, zlog)
	if err := prep.PreProcessing(false); err != nil {
		return nil, nil, nil, nil, nil, err
	}
	g = prep.GetGraph()
	og := prep.GetOverlayGraph()
	ptf := prep.GetTimeFunction()
	cust := customizer.NewCustomizerDirect[int32](g, og, ptf, zlog)
	met, err := cust.CustomizeDirect()
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}
	re, err := engine.NewEngineDirect[int32](g, rn, og, met, zlog, "")
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	t.Logf("customization phase of Customizable Route Planning CRP done....")
	zlog.Sugar().Infof("customization phase of Customizable Route Planning CRP done....")

	n = g.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	t.Logf("building transition matrix....")
	zlog.Sugar().Infof("building transition matrix....")

	numQueries := 5_00
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
		crpQuery := routing.NewCRPALTQuery(re.GetRoutingEngine())

		_, routeSegments, _ := crpQuery.ShortestPathSearch(q.s, q.t)
		return routeSegments
	}

	workers := concurrent.NewWorkerPool[nkQuery, []da.Index](100, 25_000)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	var N *da.SparseMatrix
	N = da.NewSparseMatrix(g.NumberOfEdges(), g.NumberOfEdges(), 0, func(a, b uint32) bool { return a == b })

	wg := sync.WaitGroup{}
	wg.Add(1)
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
			}
		}
		wg.Done()
	}()

	for _, q := range queries {
		workers.AddJob(q)
	}
	workers.Close()
	workers.Wait()
	cancel()

	wg.Wait()

	t.Logf(" transition matrix built....")
	zlog.Sugar().Infof(" transition matrix built....")

	return re, g, zlog, N, edgeLength, nil
}

func nkReadGPSTrajectory(t *testing.T, gpsDataFilepath string) []*da.GPSPoint {
	t.Helper()

	f, err := os.OpenFile(gpsDataFilepath, os.O_RDONLY, 0600)
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

		dateTime := string(ff[0]) + " " + string(ff[1])
		lat, err := util.ParseTextFloat64(ff[2])
		if err != nil {
			t.Fatalf("parse lat failed: %v", err)
		}
		lon, err := util.ParseTextFloat64(ff[3])
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

func nkEvaluateMatchedRoute(t *testing.T, g *da.Graph, rn *da.RoadNetworkDataContainer, groundTruthDataFilepath string, edgeLength map[uint64]float64,
	mapMatchPointResult []*da.MatchedGPSPoint) float64 {
	t.Helper()

	groundTruthFile, err := os.Open(groundTruthDataFilepath)
	if err != nil {
		t.Fatalf("open ground truth failed: %v", err)
	}
	defer groundTruthFile.Close()
	brGroundTruth := bufio.NewReader(groundTruthFile)

	traversedEdgesLength := make(map[uint64]float64, 576)
	traversedEdges := make(map[uint64]bool, 576)
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
		edgeID, err := util.ParseTextUInt64(ff[0])
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
	matchedEdgeSet := make(map[uint64]float64)

	for _, point := range mapMatchPointResult {
		curMatchedEID := point.GetSegmentId()
		if curMatchedEID == da.INVALID_SEGMENT_ID {
			continue
		}
		matchedDataEID := rn.GetOsmWayId(curMatchedEID)
		length, ok := edgeLength[matchedDataEID]
		if !ok {
			continue
		}
		matchedEdgeSet[matchedDataEID] = length
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
		eLength := max(eLengthForward, eLengthReversed)
		if !inMapMatchResultForward && !inMapMatchResultReversed {
			lengthOfErrSubtracted += eLength
		}
	}
	// section 6  route mismatch fraction (rmf): https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf
	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute

	return rmf
}

// todo: update kode ini

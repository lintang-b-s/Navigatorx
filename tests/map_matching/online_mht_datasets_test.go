package onlinemapmatching

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"io"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"sort"
	"testing"
	"time"

	"strings"

	"github.com/golang/geo/s2"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapattributes"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	ommInitialSpeedMean   = 8.33333
	ommInitialSpeedStd    = 8.3333
	ommPosteriorThreshold = 0.001
	ommGPSStd             = 8.0
	ommLP                 = 0.000001
	ommLC                 = 0.05
	ommAccelerationStd    = 3.0

	ommExpectedMaxGisCupRMF      = 0.25
	ommExpectedMinGiscupAccuracy = 0.87
	ommExpectedMaxMelbRMF        = 0.15
	ommExpectedMaxHanwenhuRMF    = 0.12

	melbourneResultPolyline = "data/eval/mapmatching/melbourne/result_polyline.txt"
	melbourneGPSPolyline    = "data/eval/mapmatching/melbourne/gps_track_polyline.txt"
)

var (
	giscupResultPolyline = "data/eval/mapmatching/giscup/result_polyline_%s.txt"
	giscupGPSPolyline    = "data/eval/mapmatching/giscup/gps_track_polyline_%s.txt"
)

type ommTransitionQuery struct {
	s da.Index
	t da.Index
}

func ommBuildOrReadTransitionMatrix(t *testing.T, re *engine.Engine[int32], graph *da.Graph, matrixPath string,
	numQueries int) *da.SparseMatrix {
	t.Helper()

	if _, err := os.Stat(matrixPath); err == nil {
		matrix, err := da.ReadSparseMatrixFromFile(matrixPath, 0, func(a, b uint32) bool { return a == b })
		if err != nil {
			t.Fatalf("read transition matrix %s failed: %v", matrixPath, err)
		}
		return matrix
	} else if !os.IsNotExist(err) {
		t.Fatalf("stat transition matrix %s failed: %v", matrixPath, err)
	}

	matrix := da.NewSparseMatrix(graph.NumberOfVertices(), graph.NumberOfVertices(), 0, func(a, b uint32) bool { return a == b })
	rd := rand.New(rand.NewSource(1))
	queries := make([]ommTransitionQuery, 0, numQueries)
	for i := 0; len(queries) < numQueries && i < numQueries*100; i++ {
		s := da.Index(rd.Intn(graph.NumberOfVertices()))
		dst := da.Index(rd.Intn(graph.NumberOfVertices()))
		if s == dst || !graph.PathExists(s, dst) {
			continue
		}
		queries = append(queries, ommTransitionQuery{s: s, t: dst})
	}

	computeRoute := func(q ommTransitionQuery) []da.Index {
		crpQuery := routing.NewCRPALTQuery(re.GetRoutingEngine())

		_, spPath, _ := crpQuery.ShortestPathSearch(q.s, q.t)
		return spPath
	}

	workers := concurrent.NewWorkerPool[ommTransitionQuery, []da.Index](100, 25_000)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	done := make(chan struct{})
	go func() {
		defer close(done)
		for spPath := range workers.CollectResults() {
			for i := 0; i < len(spPath)-1; i++ {
				e := int(spPath[i])
				eNext := int(spPath[i+1])
				matrix.Set(matrix.Get(e, eNext)+1, e, eNext)
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

	if err := util.EnsureDirExists(matrixPath); err != nil {
		t.Fatalf("create transition matrix directory failed: %v", err)
	}
	if err := matrix.WriteToFile(matrixPath); err != nil {
		t.Fatalf("write transition matrix %s failed: %v", matrixPath, err)
	}
	return matrix
}

// go test ./tests/map_matching  -run TestNewsonKrummOnlineMapMatching  -v -timeout=0  -count=1
func TestNewsonKrummOnlineMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)

	eng, g, zlog, N, edgeLength, err := nkBuildRoadNetworkCRPGraph(t, workingDir)
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
	met := re.GetMetrics()
	rn := re.GetRoadNetworkContainer()
	sidx := spatialindex.NewS2RoadSegmentsIndex(g, rn, zlog)
	mapAttributesEngine := mapattributes.NewMapAttributesEngine(g, rn, zlog, met, sidx)

	gpsTraj := nkReadGPSTrajectory(t, gpsDataFilepath)
	matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(t, N, mapAttributesEngine, gpsTraj)
	if len(matchedPoints) == 0 {
		t.Fatalf("Newson-Krumm online MHT produced no matched points")
	}

	groundTruthFile, err := os.Open(groundTruthDataFilepath)
	if err != nil {
		t.Fatalf("open ground truth failed: %v", err)
	}
	defer groundTruthFile.Close()
	brGroundTruth := bufio.NewReader(groundTruthFile)

	groundTruthSegmentsLength := make(map[uint64]float64, 576)
	groundTruthSegments := make(map[uint64]bool, 576)
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
		segmentId, err := util.ParseTextUInt64(ff[0])
		if err != nil {
			t.Fatalf("parse edge id failed: %v", err)
		}
		groundTruthSegmentsLength[segmentId] = edgeLength[segmentId]
		groundTruthSegments[segmentId] = ff[1] == "1"
	}

	lengthOfCorrectRoute := 0.0
	for _, eLength := range groundTruthSegmentsLength {
		lengthOfCorrectRoute += eLength
	}
	lengthOfErrAdded := 0.0
	lengthOfErrSubtracted := 0.0
	matchedEdgeSet := make(map[uint64]float64)
	matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))

	for _, point := range matchedPoints {
		mSegId := point.GetSegmentId()
		if mSegId == da.INVALID_SEGMENT_ID {
			continue
		}
		mOsmWayId := rn.GetOsmWayId(mSegId)
		length, ok := edgeLength[mOsmWayId]
		if !ok {
			continue
		}
		matchedEdgeSet[mOsmWayId] = length
		matchedCoords = append(matchedCoords, point.GetMatchedCoord())
	}

	for mOsmWayId, eLength := range matchedEdgeSet {
		_, inGroundTruthReversed := groundTruthSegments[mOsmWayId]
		_, inGroundTruthForward := groundTruthSegments[mOsmWayId/2]
		if !inGroundTruthForward && !inGroundTruthReversed {
			lengthOfErrAdded += eLength
		}
	}

	for eID := range groundTruthSegments {
		eLengthForward, inMapMatchResultForward := matchedEdgeSet[eID]
		eLengthReversed, inMapMatchResultReversed := matchedEdgeSet[eID*2]
		eLength := max(eLengthForward, eLengthReversed)
		if !inMapMatchResultForward && !inMapMatchResultReversed {
			lengthOfErrSubtracted += eLength
		}
	}

	// Route Mismatch Fraction (RMF):  https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf
	// or Route Mismatch Fraction (RMF):  https://dl.acm.org/doi/epdf/10.1145/2666310.2666383
	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	t.Logf("Route Mismatch Fraction (RMF): %v", rmf)
	t.Logf("avg runtime per gps point: %v microseconds/gps point", avgRuntimeMicros)
	t.Logf("total matched points: %v/%v", len(matchedPoints), len(gpsTraj))

	polyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
	polyPath := filepath.Join(workingDir, "./data/eval/mapmatching/newsonkrumm/polyline.txt")
	if err := util.EnsureDirExists(polyPath); err != nil {
		t.Fatalf("ensure polyline dir failed: %v", err)
	}
	polyFile, err := os.Create(polyPath)
	if err != nil {
		t.Fatalf("create polyline file failed: %v", err)
	}
	defer polyFile.Close()
	if _, err := polyFile.Write([]byte(polyline)); err != nil {
		t.Fatalf("write polyline file failed: %v", err)
	}
	if rmf > nkExpectedMaxRMFOnline {
		t.Fatalf("RMF above threshold: got %v, want <= %v", rmf, nkExpectedMaxRMFOnline)
	}
}

func ommRunOnlineMHT(t *testing.T,
	transitionMatrix *da.SparseMatrix, mapAttributesEngine *mapattributes.MapAttributesEngine[int32],
	gpsTraj []*da.GPSPoint) ([]*da.MatchedGPSPoint, float64) {

	var (
		candidates     []*ma.Candidate
		speedMeanK     = ommInitialSpeedMean
		speedStdK      = ommInitialSpeedStd
		lastBearing    = 0.0
		centerS2CellId = s2.SentinelCellID
	)

	matchedPoints := make([]*da.MatchedGPSPoint, 0, len(gpsTraj))
	totalRuntimeMicros := 0.0
	prevGps := da.NewGPSPoint(0, 0, time.Now(), 0, 0)

	rtree := spatialindex.NewDynamicRtree()
	dg := da.NewDynamicGraph()
	onlineMM := online.NewOnlineMapMatchMHTClient(
		dg,
		rtree,
		ommInitialSpeedMean,
		ommInitialSpeedStd,
		ommPosteriorThreshold,
		ommGPSStd,
		ommLP,
		ommLC,
		ommAccelerationStd,
		transitionMatrix,
	)

	for i, gps := range gpsTraj {
		deltaDist := 0.0
		heading := 0.0
		if i > 0 {
			heading = geo.BearingTo(prevGps.Lat(), prevGps.Lon(), gps.Lat(), gps.Lon())
			deltaDist = geo.CalculateGreatCircleDistance(prevGps.Lat(), prevGps.Lon(), gps.Lat(), gps.Lon())
		}
		gps.SetDirectionAngle(heading)
		start := time.Now()

		currS2CellId := s2.CellIDFromLatLng(s2.LatLngFromDegrees(gps.Lat(), gps.Lon())).Parent(15)
		if centerS2CellId != currS2CellId {
			buf, err := mapAttributesEngine.GetMapAttributes(currS2CellId, deltaDist)
			if err != nil {
				t.Fatalf("GetMapAttributes failed: %v", err)
			}
			err = dg.Rebuild(buf)
			if err != nil {
				t.Fatalf("dg.Rebuild failed: %v", err)
			}
			rtree.Reset()
			rtree.Rebuild(dg)

			updatedCands := make([]*ma.Candidate, 0, len(candidates))
			for _, c := range candidates {
				segId := dg.GetGraphSegmentId(c.GetRoadNetworkId())
				newC := ma.NewCandidate(segId, c.GetWeight(), c.GetLength())
				newC.SetRoadNetworkId(c.GetRoadNetworkId())
				updatedCands = append(updatedCands, newC)
			}
			candidates = updatedCands
			centerS2CellId = currS2CellId
		}

		matchedPoint, currCandidates, nextSpeedMeanK, nextSpeedStdK := onlineMM.OnlineMapMatch(
			gps,
			i+1,
			candidates,
			speedMeanK,
			speedStdK,
			lastBearing,
		)
		totalRuntimeMicros += float64(time.Since(start).Microseconds())

		candidates = currCandidates
		speedMeanK = nextSpeedMeanK
		speedStdK = nextSpeedStdK
		lastBearing = matchedPoint.GetBearing()
		matchedPoints = append(matchedPoints, matchedPoint)
		prevGps = gps
	}

	if len(gpsTraj) == 0 {
		return matchedPoints, 0
	}
	return matchedPoints, totalRuntimeMicros / float64(len(gpsTraj))
}

func ommComputeGisCupOnlineMetrics(graph *da.Graph, rn *da.RoadNetworkDataContainer, groundTruthEdgeIDs []uint64, matchedPoints []*da.MatchedGPSPoint,
	edgeLengths map[uint64]float64) (float64, float64) {
	groundTruthSet := make(map[uint64]bool)
	lengthOfCorrectRoute := 0.0
	for _, segmentId := range groundTruthEdgeIDs {
		if groundTruthSet[segmentId] {
			continue
		}
		groundTruthSet[segmentId] = true
		lengthOfCorrectRoute += edgeLengths[segmentId]
	}

	correct := 0.0
	matchedEdgeSet := make(map[uint64]float64)
	for i, point := range matchedPoints {
		dataEdgeID := rn.GetOsmWayId(point.GetSegmentId())
		length, ok := edgeLengths[dataEdgeID]
		if !ok {
			continue
		}
		matchedEdgeSet[dataEdgeID] = length
		if groundTruthEdgeIDs[i] == dataEdgeID {
			correct++
		}
	}

	lengthOfErrAdded := 0.0
	for segmentId, length := range matchedEdgeSet {
		if !groundTruthSet[segmentId] {
			lengthOfErrAdded += length
		}
	}

	lengthOfErrSubtracted := 0.0
	for segmentId := range groundTruthSet {
		if _, ok := matchedEdgeSet[segmentId]; !ok {
			lengthOfErrSubtracted += edgeLengths[segmentId]
		}
	}

	// Route Mismatch Fraction (RMF):  https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf
	// or Route Mismatch Fraction (RMF):  https://dl.acm.org/doi/epdf/10.1145/2666310.2666383
	// section 6.1 accuracy: https://dl.acm.org/doi/epdf/10.1145/3725346
	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	n := float64(len(groundTruthEdgeIDs))
	m := float64(len(matchedPoints))
	if n != m {
		return -1, rmf
	}
	acc := correct / n
	return acc, rmf
}

// https://web.archive.org/web/20130127211936/http://depts.washington.edu/giscup/home
// go test ./tests/map_matching  -run TestGisCupOnlineMHTMapMatching  -v -timeout=0  -count=1
func TestGisCupOnlineMHTMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	eng, graph, zlog, edgeLengths := ohmmBuildGisCupCRPGraph(t, workingDir)
	transitionMatrix := ommBuildOrReadTransitionMatrix(
		t,
		eng,
		graph,
		filepath.Join(workingDir, "data/eval/mapmatching/giscup/omm_transition_history_giscup.ntm"),
		5000,
	)

	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	cases, err := ohmmListGisCupCases(workingDir)
	if err != nil {
		t.Fatalf("list GIS Cup cases failed: %v", err)
	}

	met := re.GetMetrics()
	sidx := spatialindex.NewS2RoadSegmentsIndex(graph, rn, zlog)
	mapAttributesEngine := mapattributes.NewMapAttributesEngine(graph, rn, zlog, met, sidx)

	totalRMF := 0.0
	totalAcc := 0.0
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
		matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(t, transitionMatrix, mapAttributesEngine, gpsTraj)
		if len(matchedPoints) == 0 {
			t.Fatalf("GIS Cup case %s produced no matched points", tc.id)
		}

		acc, rmf := ommComputeGisCupOnlineMetrics(graph, rn, groundTruthEdgeIDs, matchedPoints, edgeLengths)
		t.Logf("GIS Cup online MHT case %s: Accuracy=%v RMF=%v matched=%d/%d avg_runtime=%v microseconds/gps point",
			tc.id, acc, rmf, len(matchedPoints), len(gpsTraj), avgRuntimeMicros)

		totalAcc += acc
		totalRMF += rmf
		totalPoints += len(gpsTraj)

		gpsCoords := make([]da.Coordinate, 0, len(gpsTraj))
		matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))
		for i, p := range matchedPoints {
			gpsCoords = append(gpsCoords, gpsTraj[i].GetCoordinate())
			matchedCoords = append(matchedCoords, p.GetMatchedCoord())
		}

		if err := util.EnsureDirExists(fmt.Sprintf(giscupGPSPolyline, tc.id)); err != nil {
			t.Fatalf("create polyline directory failed: %v", err)
		}

		gpsTrackPolyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(gpsCoords))
		if err := os.WriteFile(fmt.Sprintf(giscupGPSPolyline, tc.id), []byte(gpsTrackPolyline), 0600); err != nil {
			panic(err)
		}

		matchedPolyline := ""
		if len(matchedCoords) > 0 {
			matchedPolyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
		}
		if err := os.WriteFile(fmt.Sprintf(giscupResultPolyline, tc.id), []byte(matchedPolyline), 0600); err != nil {
			panic(err)
		}
		fmt.Printf("wrote matched polyline to %s\n", fmt.Sprintf(giscupResultPolyline, tc.id))
		fmt.Printf("wrote gps trajectory polyline to %s\n", fmt.Sprintf(giscupGPSPolyline, tc.id))
	}
	avgAcc := totalAcc / float64(len(cases))
	avgRMF := totalRMF / float64(len(cases))
	t.Logf("GIS Cup online MHT aggregate: cases=%d points=%d avg_Accuracy=%v avg_RMF=%v ",
		len(cases), totalPoints, avgAcc, avgRMF)

	if avgRMF > ommExpectedMaxGisCupRMF {
		t.Fatalf("GIS Cup online MHT RMF above threshold: got %v, want <= %v", avgRMF, ommExpectedMaxGisCupRMF)
	}

	if util.Lt(avgAcc, ommExpectedMinGiscupAccuracy) {
		t.Fatalf("GIS Cup online MHT Average Accuracy below threshold: got %v, want >= %v", avgAcc, ommExpectedMinGiscupAccuracy)
	}
}

// go test ./tests/map_matching  -run TestHengfengLiOnlineMHTMapMatching  -v -timeout=0  -count=1
func TestHengfengLiOnlineMHTMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	eng, graph, logger, graphEdgeIDToMelbourneEdgeID, edgeLengthByID := ohmmBuildMelbourneCRPGraph(t, workingDir)
	transitionMatrix := ommBuildOrReadTransitionMatrix(
		t,
		eng,
		graph,
		filepath.Join(workingDir, "data/eval/mapmatching/melbourne/online_mht_transition_hl.ntm"),
		1000,
	)

	gpsPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/gps_track.txt")
	groundTruthPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/groundtruth.txt")
	points, err := ohmmReadMelbourneGPSTrack(gpsPath)
	if err != nil {
		t.Fatalf("read Melbourne GPS track failed: %v", err)
	}
	groundTruthEdgeIDs, err := ohmmReadMelbourneGroundTruthSegments(groundTruthPath)
	if err != nil {
		t.Fatalf("read Melbourne ground truth failed: %v", err)
	}

	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	rtree := spatialindex.NewRtree()
	rtree.Build(graph, rn, logger)
	gpsTraj := ohmmMelbourneGPSTrajectory(points)
	met := re.GetMetrics()
	sidx := spatialindex.NewS2RoadSegmentsIndex(graph, rn, logger)
	mapAttributesEngine := mapattributes.NewMapAttributesEngine(graph, rn, logger, met, sidx)

	matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(t, transitionMatrix, mapAttributesEngine, gpsTraj)
	if len(matchedPoints) == 0 {
		t.Fatalf("Hengfeng Li online MHT produced no matched points")
	}

	rmf := ohmmComputeMelbourneMetrics(matchedPoints, groundTruthEdgeIDs, graphEdgeIDToMelbourneEdgeID, edgeLengthByID)

	t.Logf("Hengfeng Li online MHT: RMF=%v matched=%d/%d avg_runtime=%v microseconds/gps point",
		rmf, len(matchedPoints), len(gpsTraj), avgRuntimeMicros)

	if rmf > ommExpectedMaxMelbRMF {
		t.Fatalf("Hengfeng Li online MHT RMF above threshold: got %v, want <= %v", rmf, ommExpectedMaxMelbRMF)
	}

	gpsCoords := make([]da.Coordinate, 0, len(gpsTraj))
	matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))
	for i, p := range matchedPoints {
		gpsCoords = append(gpsCoords, gpsTraj[i].GetCoordinate())
		matchedCoords = append(matchedCoords, p.GetMatchedCoord())
	}

	if err := os.MkdirAll(filepath.Dir(melbourneGPSPolyline), 0700); err != nil {
		t.Fatalf("create polyline directory failed: %v", err)
	}

	gpsTrackPolyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(gpsCoords))
	if err := os.WriteFile(melbourneGPSPolyline, []byte(gpsTrackPolyline), 0600); err != nil {
		panic(err)
	}

	matchedPolyline := ""
	if len(matchedCoords) > 0 {
		matchedPolyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
	}
	if err := os.WriteFile(melbourneResultPolyline, []byte(matchedPolyline), 0600); err != nil {
		panic(err)
	}
	fmt.Printf("wrote matched polyline to %s\n", melbourneResultPolyline)
	fmt.Printf("wrote gps trajectory polyline to %s\n", melbourneGPSPolyline)
}

// go test ./tests/map_matching  -run TestHanwenhuOnlineMapMatching  -v -timeout=0  -count=1
func TestHanwenhuOnlineMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	eng, graph, logger := ohmmBuildHanwenHuCRPGraph(t)
	transitionMatrix := ommBuildOrReadTransitionMatrix(
		t,
		eng,
		graph,
		filepath.Join(workingDir, "data/eval/mapmatching/Shanghai/online_mht_transition_hh.ntm"),
		1000,
	)

	projectPath := func(path string) string {
		return filepath.Join(workingDir, strings.TrimPrefix(path, "./"))
	}

	shanghaiDataFilePath := projectPath(hhShanghaiDataFilePath)
	shanghaiTestDataPath := projectPath(hhShanghaiTestDataPath)
	shanghaiGroundTruthPath := projectPath(hhShanghaiGroundTruthPath)
	shanghaiPolylinesPath := projectPath(hhShanghaiPolylinesPath)

	if err := hhDownload(shanghaiDataFilePath, hhShanghaiDatasetDriveFile, logger, "shanghai dataset"); err != nil {
		t.Fatalf("download dataset failed: %v", err)
	}
	gzFile, err := os.Open(shanghaiDataFilePath)
	if err != nil {
		t.Fatalf("open shanghai tar.gz failed: %v", err)
	}
	defer gzFile.Close()
	if _, err := os.Stat(shanghaiTestDataPath); err != nil || func() bool { _, err := os.Stat(shanghaiGroundTruthPath); return err != nil }() {
		if err := hhExtractTarGz(gzFile, filepath.Join(workingDir, "data/eval/mapmatching")); err != nil {
			t.Fatalf("extract tar.gz failed: %v", err)
		}
	}
	gpsTrajectories, err := hhReadAllCSVInDir(shanghaiTestDataPath)
	if err != nil {
		t.Fatalf("read trajectories failed: %v", err)
	}

	re := eng.GetRoutingEngine()
	rn := re.GetRoadNetworkContainer()
	met := re.GetMetrics()
	sidx := spatialindex.NewS2RoadSegmentsIndex(graph, rn, logger)
	mapAttributesEngine := mapattributes.NewMapAttributesEngine(graph, rn, logger, met, sidx)

	trajectoryNames := make([]string, 0, len(gpsTrajectories))
	for trajName := range gpsTrajectories {
		trajectoryNames = append(trajectoryNames, trajName)
	}
	sort.Strings(trajectoryNames)

	matchingErrors := make([]float64, 0, len(trajectoryNames))
	totalPoints := 0
	totalRuntimeMicros := 0.0
	totalRMF := 0.0

	for _, trajName := range trajectoryNames {
		gpsTraj := gpsTrajectories[trajName]
		gpsPoints := ohmmHanwenHuGPSTrajectory(t, gpsTraj)

		matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(t, transitionMatrix, mapAttributesEngine, gpsPoints)
		if len(matchedPoints) == 0 {
			t.Fatalf("Hanwen-Hu trajectory %s produced no matched points", trajName)
		}

		trackID := hhTrackIDFromName(trajName)
		resultPolylinePath := filepath.Join(shanghaiPolylinesPath, fmt.Sprintf("online_mht_result_polyline_%s.txt", trackID))
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

		rmf := math.Abs(matchLength-groundTruthLength) / groundTruthLength
		matchingErrors = append(matchingErrors, rmf)
		totalRMF += rmf
		totalPoints += len(gpsPoints)
		totalRuntimeMicros += avgRuntimeMicros * float64(len(gpsPoints))

		t.Logf("Hanwen-Hu trajectory %s: RMF=%v matched=%d/%d avg_runtime=%v microseconds/gps point",
			trajName, rmf, len(matchedPoints), len(gpsPoints), avgRuntimeMicros)
	}

	sort.Float64s(matchingErrors)
	avgRMF := totalRMF / float64(len(trajectoryNames))
	avgRuntimePerPoint := totalRuntimeMicros / float64(totalPoints)
	cdfAt014 := ohmmEmpiricalCDF(matchingErrors, 0.14)
	cdfAt040 := ohmmEmpiricalCDF(matchingErrors, 0.40)

	t.Logf("Hanwen-Hu online MHT aggregate: trajectories=%d, points=%d, avg_runtime=%v microseconds/gps point, avg_RMF=%v, CDF(<=0.14)=%v, CDF(<=0.40)=%v",
		len(trajectoryNames), totalPoints, avgRuntimePerPoint, avgRMF, cdfAt014, cdfAt040)

	if avgRMF > ommExpectedMaxHanwenhuRMF {
		t.Fatalf("Hanwen-Hu online MHT avg RMF above threshold: got %v, want <= %v", avgRMF, ommExpectedMaxHanwenhuRMF)
	}
	if cdfAt014 < ohmmExpectedMinHHCDFAt014 {
		t.Fatalf("Hanwen-Hu CDF(<=0.14) below threshold: got %v, want >= %v", cdfAt014, ohmmExpectedMinHHCDFAt014)
	}
	if cdfAt040 < ohmmExpectedMinHHCDFAt040 {
		t.Fatalf("Hanwen-Hu CDF(<=0.40) below threshold: got %v, want >= %v", cdfAt040, ohmmExpectedMinHHCDFAt040)
	}
}

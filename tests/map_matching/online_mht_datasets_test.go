package onlinemapmatching

import (
	"bufio"
	"context"
	"errors"
	"io"
	"math/rand"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/golang/geo/s2"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapattributes"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
)

const (
	ommInitialSpeedMean   = 8.33333
	ommInitialSpeedStd    = 8.3333
	ommPosteriorThreshold = 0.001
	ommGPSStd             = 11.0
	ommLP                 = 0.000001
	ommLC                 = 0.06
	ommAccelerationStd    = 3.0

	ommExpectedMaxGisCupRMF = 0.25
	ommExpectedMaxMelbRMF   = 0.11

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
	numQueries int) *da.SparseMatrix[int] {
	t.Helper()

	matrix := da.NewSparseMatrix[int](graph.NumberOfVertices(), graph.NumberOfVertices(), 0, func(a, b int) bool { return a == b })
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

	if err := os.MkdirAll(filepath.Dir(matrixPath), 0755); err != nil {
		t.Fatalf("create transition matrix directory failed: %v", err)
	}
	if err := matrix.WriteToFile(matrixPath); err != nil {
		t.Fatalf("write transition matrix %s failed: %v", matrixPath, err)
	}
	return matrix
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

	rtree := spatialindex.NewDynamicRtree()

	dg := da.NewDynamicGraph()
	onlineMM := online.NewOnlineMapMatchMHT(dg, rtree, 8.33333, 8.3333, 0.0001, 9.0, 0.0000001, 0.04, 3, N)

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
	mapMatchRes := make([]*da.MatchedGPSPoint, 0)
	avgRuntimePerGPSPoint := 0.0
	nowDataset := time.Now()

	centerS2CellId := s2.SentinelCellID
	re := eng.GetRoutingEngine()
	met := re.GetMetrics()
	rn := re.GetRoadNetworkContainer()
	sidx := spatialindex.NewS2RoadSegmentsIndex(g, rn, zlog)
	mapAttributesEngine := mapattributes.NewMapAttributesEngine(g, rn, zlog, met, sidx)

	prevGps := da.NewGPSPoint(0, 0, time.Now(), 0, 0)
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
		heading := 0.0
		if hasPrev {
			deltaTime = curGPSTime.Sub(prevTime).Seconds()
			if util.Gt(deltaTime, 0) {
				dist := geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon)
				speed = util.KilometerToMeter(dist) / deltaTime
			}
			heading = geo.BearingTo(prevLat, prevLon, lat, lon)
		} else {
			hasPrev = true
			heading = 0.0
		}
		prevLat, prevLon, prevTime = lat, lon, curGPSTime

		now := time.Now()
		curGPS := da.NewGPSPoint(lat, lon, curGPSTime, speed, deltaTime)
		curGPS.SetDirectionAngle(heading)

		currS2CellId := s2.CellIDFromLatLng(s2.LatLngFromDegrees(lat, lon)).Parent(15)

		if centerS2CellId != currS2CellId {

			buf, err := mapAttributesEngine.GetMapAttributes(currS2CellId)
			err = dg.Rebuild(buf)
			if err != nil {
				if errors.Is(err, os.ErrNotExist) {
					centerS2CellId = currS2CellId
					candidates = candidates[:0]
					dg.Reset()
					rtree.Reset()
					continue
				}
				t.Fatal(err)
			}

			// Rebuild the R-tree with the new tile data
			rtree.Reset()
			rtree.Rebuild(dg)

			updatedCands := make([]*ma.Candidate, 0, len(candidates))
			for _, c := range candidates {
				segId := dg.GetGraphSegmentId(c.GetRoadNetworkId())
				updatedCands = append(updatedCands, ma.NewCandidate(segId, c.GetWeight(), c.GetLength()))
			}
			candidates = updatedCands
			centerS2CellId = currS2CellId
		}

		matchedPoint, nextCandidates, nextSpeedMeanK, nextSpeedStdK := onlineMM.OnlineMapMatch(prevGps, curGPS, k, candidates, speedMeanK, speedStdK, lastBearing)
		candidates, speedMeanK, speedStdK = nextCandidates, nextSpeedMeanK, nextSpeedStdK
		k++

		if matchedPoint.GetSegmentId() != da.INVALID_SEGMENT_ID {
			rnEdgeId := dg.GetRoadNetworkSegmentId(matchedPoint.GetSegmentId())
			matchedPoint.SetSegmentId(rnEdgeId)
		}

		mapMatchRes = append(mapMatchRes, matchedPoint)
		avgRuntimePerGPSPoint += float64(time.Since(now).Microseconds())
		prevGps = curGPS
	}

	totalPoints := float64(k - 1)
	totalRuntime := float64(time.Since(nowDataset).Milliseconds())

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
	matchedCoords := make([]da.Coordinate, 0, len(mapMatchRes))

	for _, point := range mapMatchRes {
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

	avgRuntimePerGPSPoint /= float64(k - 1)
	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
	t.Logf("Route Mismatch Fraction (RMF): %v", rmf)
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
	if rmf > nkExpectedMaxRMFOnline {
		t.Fatalf("RMF above threshold: got %v, want <= %v", rmf, nkExpectedMaxRMFOnline)
	}
}

// func ommRunOnlineMHT(graph *da.Graph, rtree *spatialindex.Rtree, transitionMatrix *da.SparseMatrix[int],
// 	gpsTraj []*da.GPSPoint, getSegmentLength func(da.Index) float64) ([]*da.MatchedGPSPoint, float64) {
// 	onlineMM := online.NewOnlineMapMatchMHT(
// 		graph,
// 		rtree,
// 		ommInitialSpeedMean,
// 		ommInitialSpeedStd,
// 		ommPosteriorThreshold,
// 		ommGPSStd,
// 		ommLP,
// 		ommLC,
// 		ommAccelerationStd,
// 		transitionMatrix,
// 		getSegmentLength,
// 	)

// 	var (
// 		candidates  []*ma.Candidate
// 		speedMeanK  = ommInitialSpeedMean
// 		speedStdK   = ommInitialSpeedStd
// 		lastBearing = 0.0
// 	)

// 	matchedPoints := make([]*da.MatchedGPSPoint, 0, len(gpsTraj))
// 	totalRuntimeMicros := 0.0
// 	prevGps := da.NewGPSPoint(0, 0, time.Now(), 0, 0)

// 	for i, gps := range gpsTraj {
// 		heading := 0.0
// 		if i > 0 {
// 			heading = geo.BearingTo(prevGps.Lat(), prevGps.Lon(), gps.Lat(), gps.Lon())
// 		}
// 		gps.SetDirectionAngle(heading)
// 		start := time.Now()
// 		matchedPoint, nextCandidates, nextSpeedMeanK, nextSpeedStdK := onlineMM.OnlineMapMatch(
// 			prevGps,
// 			gps,
// 			i+1,
// 			candidates,
// 			speedMeanK,
// 			speedStdK,
// 			lastBearing,
// 		)
// 		totalRuntimeMicros += float64(time.Since(start).Microseconds())

// 		candidates = nextCandidates
// 		speedMeanK = nextSpeedMeanK
// 		speedStdK = nextSpeedStdK
// 		lastBearing = matchedPoint.GetBearing()
// 		matchedPoints = append(matchedPoints, matchedPoint)
// 		prevGps = gps
// 	}

// 	if len(gpsTraj) == 0 {
// 		return matchedPoints, 0
// 	}
// 	return matchedPoints, totalRuntimeMicros / float64(len(gpsTraj))
// }

// func ommComputeGisCupEdgeSetMetrics(graph *da.Graph, rn *da.RoadNetworkDataContainer, groundTruthEdgeIDs []uint64, matchedPoints []*da.MatchedGPSPoint,
// 	edgeLengths map[uint64]float64) (float64, float64) {
// 	groundTruthSet := make(map[uint64]bool)
// 	lengthOfCorrectRoute := 0.0
// 	for _, segmentId := range groundTruthEdgeIDs {
// 		if groundTruthSet[segmentId] {
// 			continue
// 		}
// 		groundTruthSet[segmentId] = true
// 		lengthOfCorrectRoute += edgeLengths[segmentId]
// 	}

// 	matchedEdgeSet := make(map[uint64]float64)
// 	for _, point := range matchedPoints {
// 		if point.GetSegmentId() == da.INVALID_SEGMENT_ID {
// 			continue
// 		}
// 		dataEdgeID := rn.GetOsmWayId(point.GetSegmentId())
// 		length, ok := edgeLengths[dataEdgeID]
// 		if !ok {
// 			continue
// 		}
// 		matchedEdgeSet[dataEdgeID] = length
// 	}

// 	correctMatched := 0.0
// 	for segmentId := range matchedEdgeSet {
// 		if groundTruthSet[segmentId] {
// 			correctMatched++
// 		}
// 	}

// 	crp := 0.0
// 	if len(matchedEdgeSet) > 0 {
// 		crp = correctMatched / float64(len(matchedEdgeSet))
// 	}
// 	if util.Eq(lengthOfCorrectRoute, 0) {
// 		return crp, math.Inf(1)
// 	}

// 	lengthOfErrAdded := 0.0
// 	for segmentId, length := range matchedEdgeSet {
// 		if !groundTruthSet[segmentId] {
// 			lengthOfErrAdded += length
// 		}
// 	}

// 	lengthOfErrSubtracted := 0.0
// 	for segmentId := range groundTruthSet {
// 		if _, ok := matchedEdgeSet[segmentId]; !ok {
// 			lengthOfErrSubtracted += edgeLengths[segmentId]
// 		}
// 	}

// 	rmf := (lengthOfErrAdded + lengthOfErrSubtracted) / lengthOfCorrectRoute
// 	return crp, rmf
// }

// func ommComputeGisCupPointAccuracy(graph *da.Graph, rn *da.RoadNetworkDataContainer, groundTruthEdgeIDs []uint64, matchedPoints []*da.MatchedGPSPoint) float64 {
// 	if len(groundTruthEdgeIDs) == 0 {
// 		return 0
// 	}

// 	correct := 0
// 	limit := len(groundTruthEdgeIDs)
// 	if len(matchedPoints) < limit {
// 		limit = len(matchedPoints)
// 	}
// 	for i := 0; i < limit; i++ {
// 		if matchedPoints[i].GetSegmentId() == da.INVALID_SEGMENT_ID {
// 			continue
// 		}
// 		matchedEdgeId := rn.GetOsmWayId(matchedPoints[i].GetSegmentId())
// 		if matchedEdgeId == groundTruthEdgeIDs[i] {
// 			correct++
// 		}
// 	}
// 	return float64(correct) / float64(len(groundTruthEdgeIDs))
// }

// // todo: update kode ini, adjust setelah pakai edge-based graph
// // https://web.archive.org/web/20130127211936/http://depts.washington.edu/giscup/home
// // go test ./tests/map_matching  -run TestGisCupOnlineMHTMapMatching  -v -timeout=0  -count=1
// func TestGisCupOnlineMHTMapMatching(t *testing.T) {
// 	workingDir := ohmmEnsureConfig(t)
// 	eng, graph, logger, edgeLengths := ohmmBuildGisCupCRPGraph(t, workingDir)
// 	transitionMatrix := ommBuildOrReadTransitionMatrix(
// 		t,
// 		eng,
// 		graph,
// 		filepath.Join(workingDir, "data/eval/mapmatching/giscup/omm_transition_history_giscup.ntm"),
// 		5000,
// 	)

// 	re := eng.GetRoutingEngine()
// 	rn := re.GetRoadNetworkContainer()
// 	cases, err := ohmmListGisCupCases(workingDir)
// 	if err != nil {
// 		t.Fatalf("list GIS Cup cases failed: %v", err)
// 	}

// 	rtree := spatialindex.NewRtree()
// 	rtree.Build(graph, rn, logger)

// 	totalCRP := 0.0
// 	totalRMF := 0.0
// 	totalPointAccuracy := 0.0
// 	totalPoints := 0
// 	for _, tc := range cases {
// 		points, err := ohmmReadGisCupTrack(tc.inputFilePath)
// 		if err != nil {
// 			t.Fatalf("read GIS Cup track %s failed: %v", tc.id, err)
// 		}
// 		groundTruthEdgeIDs, err := ohmmReadGisCupGroundTruth(tc.outputFilePath)
// 		if err != nil {
// 			t.Fatalf("read GIS Cup ground truth %s failed: %v", tc.id, err)
// 		}

// 		gpsTraj := ohmmGisCupGPSTrajectory(points)
// 		matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(graph, rtree, transitionMatrix, gpsTraj, func(eID da.Index) float64 {
// 			return re.GetSegmentLength(eID)
// 		})
// 		if len(matchedPoints) == 0 {
// 			t.Fatalf("GIS Cup case %s produced no matched points", tc.id)
// 		}

// 		crp, rmf := ommComputeGisCupEdgeSetMetrics(graph, rn, groundTruthEdgeIDs, matchedPoints, edgeLengths)
// 		pointAccuracy := ommComputeGisCupPointAccuracy(graph, rn, groundTruthEdgeIDs, matchedPoints)
// 		t.Logf("GIS Cup online MHT case %s: Accuracy=%v RMF=%v point_accuracy=%v matched=%d/%d avg_runtime=%v microseconds/gps point",
// 			tc.id, crp, rmf, pointAccuracy, len(matchedPoints), len(gpsTraj), avgRuntimeMicros)

// 		totalCRP += crp
// 		totalRMF += rmf
// 		totalPointAccuracy += pointAccuracy
// 		totalPoints += len(gpsTraj)

// 		gpsCoords := make([]da.Coordinate, 0, len(gpsTraj))
// 		matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))
// 		for i, p := range matchedPoints {
// 			gpsCoords = append(gpsCoords, gpsTraj[i].GetCoordinate())
// 			matchedCoords = append(matchedCoords, p.GetMatchedCoord())
// 		}

// 		if err := os.MkdirAll(filepath.Dir(fmt.Sprintf(giscupGPSPolyline, tc.id)), 0755); err != nil {
// 			t.Fatalf("create polyline directory failed: %v", err)
// 		}

// 		gpsTrackPolyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(gpsCoords))
// 		if err := os.WriteFile(fmt.Sprintf(giscupGPSPolyline, tc.id), []byte(gpsTrackPolyline), 0644); err != nil {
// 			panic(err)
// 		}

// 		matchedPolyline := ""
// 		if len(matchedCoords) > 0 {
// 			matchedPolyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
// 		}
// 		if err := os.WriteFile(fmt.Sprintf(giscupResultPolyline, tc.id), []byte(matchedPolyline), 0644); err != nil {
// 			panic(err)
// 		}
// 		fmt.Printf("wrote matched polyline to %s\n", fmt.Sprintf(giscupResultPolyline, tc.id))
// 		fmt.Printf("wrote gps trajectory polyline to %s\n", fmt.Sprintf(giscupGPSPolyline, tc.id))
// 	}

// 	avgRMF := totalRMF / float64(len(cases))
// 	avgPointAccuracy := totalPointAccuracy / float64(len(cases))
// 	t.Logf("GIS Cup online MHT aggregate: cases=%d points=%d avg_RMF=%v avg_point_accuracy=%v",
// 		len(cases), totalPoints, avgRMF, avgPointAccuracy)

// 	if avgRMF > ommExpectedMaxGisCupRMF {
// 		t.Fatalf("GIS Cup online MHT RMF above threshold: got %v, want <= %v", avgRMF, ommExpectedMaxGisCupRMF)
// 	}

// }

// // go test ./tests/map_matching  -run TestHengfengLiOnlineMHTMapMatching  -v -timeout=0  -count=1
// func TestHengfengLiOnlineMHTMapMatching(t *testing.T) {
// 	workingDir := ohmmEnsureConfig(t)
// 	eng, graph, logger, graphEdgeIDToMelbourneEdgeID, edgeLengthByID := ohmmBuildMelbourneCRPGraph(t, workingDir)
// 	transitionMatrix := ommBuildOrReadTransitionMatrix(
// 		t,
// 		eng,
// 		graph,
// 		filepath.Join(workingDir, "data/eval/mapmatching/melbourne/online_mht_transition_hl.ntm"),
// 		1000,
// 	)

// 	gpsPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/gps_track.txt")
// 	groundTruthPath := filepath.Join(workingDir, "data/eval/mapmatching/melbourne/groundtruth.txt")
// 	points, err := ohmmReadMelbourneGPSTrack(gpsPath)
// 	if err != nil {
// 		t.Fatalf("read Melbourne GPS track failed: %v", err)
// 	}
// 	groundTruthEdgeIDs, err := ohmmReadMelbourneGroundTruthSegments(groundTruthPath)
// 	if err != nil {
// 		t.Fatalf("read Melbourne ground truth failed: %v", err)
// 	}

// 	re := eng.GetRoutingEngine()
// 	rn := re.GetRoadNetworkContainer()
// 	rtree := spatialindex.NewRtree()
// 	rtree.Build(graph, rn, logger)
// 	gpsTraj := ohmmMelbourneGPSTrajectory(points)
// 	matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(graph, rtree, transitionMatrix, gpsTraj, func(eID da.Index) float64 {
// 		return re.GetSegmentLength(eID)
// 	})
// 	if len(matchedPoints) == 0 {
// 		t.Fatalf("Hengfeng Li online MHT produced no matched points")
// 	}

// 	rmf := ohmmComputeMelbourneMetrics(matchedPoints, groundTruthEdgeIDs, graphEdgeIDToMelbourneEdgeID, edgeLengthByID)
// 	t.Logf("Hengfeng Li online MHT: RMF=%v matched=%d/%d avg_runtime=%v microseconds/gps point",
// 		rmf, len(matchedPoints), len(gpsTraj), avgRuntimeMicros)

// 	if rmf > ommExpectedMaxMelbRMF {
// 		t.Fatalf("Hengfeng Li online MHT RMF above threshold: got %v, want <= %v", rmf, ommExpectedMaxMelbRMF)
// 	}

// 	gpsCoords := make([]da.Coordinate, 0, len(gpsTraj))
// 	matchedCoords := make([]da.Coordinate, 0, len(matchedPoints))
// 	for i, p := range matchedPoints {
// 		gpsCoords = append(gpsCoords, gpsTraj[i].GetCoordinate())
// 		matchedCoords = append(matchedCoords, p.GetMatchedCoord())
// 	}

// 	if err := os.MkdirAll(filepath.Dir(melbourneGPSPolyline), 0755); err != nil {
// 		t.Fatalf("create polyline directory failed: %v", err)
// 	}

// 	gpsTrackPolyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(gpsCoords))
// 	if err := os.WriteFile(melbourneGPSPolyline, []byte(gpsTrackPolyline), 0644); err != nil {
// 		panic(err)
// 	}

// 	matchedPolyline := ""
// 	if len(matchedCoords) > 0 {
// 		matchedPolyline = da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(matchedCoords))
// 	}
// 	if err := os.WriteFile(melbourneResultPolyline, []byte(matchedPolyline), 0644); err != nil {
// 		panic(err)
// 	}
// 	fmt.Printf("wrote matched polyline to %s\n", melbourneResultPolyline)
// 	fmt.Printf("wrote gps trajectory polyline to %s\n", melbourneGPSPolyline)
// }

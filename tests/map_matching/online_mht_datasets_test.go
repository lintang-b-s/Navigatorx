package onlinemapmatching

import (
	"context"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

const (
	ommInitialSpeedMean   = 8.33333
	ommInitialSpeedStd    = 8.3333
	ommPosteriorThreshold = 0.001
	ommGPSStd             = 5.0
	ommLP                 = 0.000001
	ommLC                 = 0.06
	ommAccelerationStd    = 3.0

	ommExpectedMinGisCupAccuracy = 0.83
	ommExpectedMaxGisCupRMF      = 0.25
	ommExpectedMinMelbAccuracy   = 0.92
	ommExpectedMaxMelbRMF        = 0.11
)

type ommTransitionQuery struct {
	s da.Index
	t da.Index
}

func ommBuildOrReadTransitionMatrix(t *testing.T, re *engine.Engine[int32], graph *da.Graph, matrixPath string,
	numQueries int) *da.SparseMatrix[int] {
	t.Helper()

	if _, err := os.Stat(matrixPath); err == nil {
		matrix, err := da.ReadSparseMatrixFromFile[int](matrixPath, 0, func(a, b int) bool { return a == b })
		if err != nil {
			t.Fatalf("read transition matrix %s failed: %v", matrixPath, err)
		}
		return matrix
	} else if !os.IsNotExist(err) {
		t.Fatalf("stat transition matrix %s failed: %v", matrixPath, err)
	}

	matrix := da.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(), 0, func(a, b int) bool { return a == b })
	rd := rand.New(rand.NewSource(1))
	queries := make([]ommTransitionQuery, 0, numQueries)
	for attempts := 0; len(queries) < numQueries && attempts < numQueries*100; attempts++ {
		s := da.Index(rd.Intn(graph.NumberOfVertices()))
		dst := da.Index(rd.Intn(graph.NumberOfVertices()))
		if s == dst || !graph.PathExists(s, dst) {
			continue
		}
		queries = append(queries, ommTransitionQuery{s: s, t: dst})
	}
	if len(queries) == 0 {
		t.Fatalf("failed to generate transition matrix queries for %s", matrixPath)
	}

	computeRoute := func(q ommTransitionQuery) []da.Index {
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

	workers := concurrent.NewWorkerPool[ommTransitionQuery, []da.Index](100, 25_000)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	done := make(chan struct{})
	go func() {
		defer close(done)
		for spEdges := range workers.CollectResults() {
			for i := 0; i < len(spEdges)-1; i++ {
				e := int(spEdges[i])
				next := int(spEdges[i+1])
				matrix.Set(matrix.Get(e, next)+1, e, next)
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

func ommRunOnlineMHT(graph *da.Graph, rtree *spatialindex.Rtree, transitionMatrix *da.SparseMatrix[int],
	gpsTraj []*da.GPSPoint, getSegmentLength func(da.Index) float64) ([]*da.MatchedGPSPoint, float64) {
	onlineMM := online.NewOnlineMapMatchMHT(
		graph,
		rtree,
		ommInitialSpeedMean,
		ommInitialSpeedStd,
		ommPosteriorThreshold,
		ommGPSStd,
		ommLP,
		ommLC,
		ommAccelerationStd,
		transitionMatrix,
		getSegmentLength,
	)

	var (
		candidates  []*ma.Candidate
		speedMeanK  = ommInitialSpeedMean
		speedStdK   = ommInitialSpeedStd
		lastBearing = 0.0
	)

	matchedPoints := make([]*da.MatchedGPSPoint, 0, len(gpsTraj))
	totalRuntimeMicros := 0.0
	for i, gps := range gpsTraj {
		start := time.Now()
		matchedPoint, nextCandidates, nextSpeedMeanK, nextSpeedStdK := onlineMM.OnlineMapMatch(
			gps,
			i+1,
			candidates,
			speedMeanK,
			speedStdK,
			lastBearing,
		)
		totalRuntimeMicros += float64(time.Since(start).Microseconds())

		candidates = nextCandidates
		speedMeanK = nextSpeedMeanK
		speedStdK = nextSpeedStdK
		lastBearing = matchedPoint.GetBearing()
		matchedPoints = append(matchedPoints, matchedPoint)
	}

	if len(gpsTraj) == 0 {
		return matchedPoints, 0
	}
	return matchedPoints, totalRuntimeMicros / float64(len(gpsTraj))
}

func ommComputeGisCupEdgeSetMetrics(graph *da.Graph, groundTruthEdgeIDs []int64, matchedPoints []*da.MatchedGPSPoint,
	edgeLengths map[int64]float64) (float64, float64) {
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

func ommComputeGisCupPointAccuracy(graph *da.Graph, groundTruthEdgeIDs []int64, matchedPoints []*da.MatchedGPSPoint) float64 {
	if len(groundTruthEdgeIDs) == 0 {
		return 0
	}

	correct := 0
	limit := len(groundTruthEdgeIDs)
	if len(matchedPoints) < limit {
		limit = len(matchedPoints)
	}
	for i := 0; i < limit; i++ {
		if matchedPoints[i].GetEdgeId() == da.INVALID_EDGE_ID {
			continue
		}
		if graph.GetOsmWayId(matchedPoints[i].GetEdgeId()) == groundTruthEdgeIDs[i] {
			correct++
		}
	}
	return float64(correct) / float64(len(groundTruthEdgeIDs))
}

// go test ./tests/map_matching  -run TestGisCupOnlineMHTMapMatching  -v -timeout=0  -count=1
func TestGisCupOnlineMHTMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	re, graph, logger, edgeLengths := ohmmBuildGisCupCRPGraph(t, workingDir)
	transitionMatrix := ommBuildOrReadTransitionMatrix(
		t,
		re,
		graph,
		filepath.Join(workingDir, "data/eval/mapmatching/giscup/omm_transition_history_giscup.ntm"),
		5000,
	)

	cases, err := ohmmListGisCupCases(workingDir)
	if err != nil {
		t.Fatalf("list GIS Cup cases failed: %v", err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)

	totalCRP := 0.0
	totalRMF := 0.0
	totalPointAccuracy := 0.0
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
		matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(graph, rtree, transitionMatrix, gpsTraj, func(eID da.Index) float64 {
			return re.GetRoutingEngine().GetSegmentLength(eID, true)
		})
		if len(matchedPoints) == 0 {
			t.Fatalf("GIS Cup case %s produced no matched points", tc.id)
		}

		crp, rmf := ommComputeGisCupEdgeSetMetrics(graph, groundTruthEdgeIDs, matchedPoints, edgeLengths)
		pointAccuracy := ommComputeGisCupPointAccuracy(graph, groundTruthEdgeIDs, matchedPoints)
		t.Logf("GIS Cup online MHT case %s: CRP=%v RMF=%v point_accuracy=%v matched=%d/%d avg_runtime=%v microseconds/gps point",
			tc.id, crp, rmf, pointAccuracy, len(matchedPoints), len(gpsTraj), avgRuntimeMicros)

		totalCRP += crp
		totalRMF += rmf
		totalPointAccuracy += pointAccuracy
		totalPoints += len(gpsTraj)
	}

	avgCRP := totalCRP / float64(len(cases))
	avgRMF := totalRMF / float64(len(cases))
	avgPointAccuracy := totalPointAccuracy / float64(len(cases))
	t.Logf("GIS Cup online MHT aggregate: cases=%d points=%d avg_CRP=%v avg_RMF=%v avg_point_accuracy=%v",
		len(cases), totalPoints, avgCRP, avgRMF, avgPointAccuracy)

	if avgCRP < ommExpectedMinGisCupAccuracy {
		t.Fatalf("GIS Cup online MHT CRP below threshold: got %v, want >= %v", avgCRP, ommExpectedMinGisCupAccuracy)
	}
	if avgRMF > ommExpectedMaxGisCupRMF {
		t.Fatalf("GIS Cup online MHT RMF above threshold: got %v, want <= %v", avgRMF, ommExpectedMaxGisCupRMF)
	}
}

// go test ./tests/map_matching  -run TestHengfengLiOnlineMHTMapMatching  -v -timeout=0  -count=1
func TestHengfengLiOnlineMHTMapMatching(t *testing.T) {
	workingDir := ohmmEnsureConfig(t)
	re, graph, logger, graphEdgeIDToMelbourneEdgeID, edgeLengthByID := ohmmBuildMelbourneCRPGraph(t, workingDir)
	transitionMatrix := ommBuildOrReadTransitionMatrix(
		t,
		re,
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
	groundTruthEdgeIDs, err := ohmmReadMelbourneGroundTruthEdges(groundTruthPath)
	if err != nil {
		t.Fatalf("read Melbourne ground truth failed: %v", err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)
	gpsTraj := ohmmMelbourneGPSTrajectory(points)
	matchedPoints, avgRuntimeMicros := ommRunOnlineMHT(graph, rtree, transitionMatrix, gpsTraj, func(eID da.Index) float64 {
		return re.GetRoutingEngine().GetSegmentLength(eID, true)
	})
	if len(matchedPoints) == 0 {
		t.Fatalf("Hengfeng Li online MHT produced no matched points")
	}

	crp, rmf, err := ohmmComputeMelbourneMetrics(matchedPoints, groundTruthEdgeIDs, graphEdgeIDToMelbourneEdgeID, edgeLengthByID)
	if err != nil {
		t.Fatalf("compute Melbourne metrics failed: %v", err)
	}
	t.Logf("Hengfeng Li online MHT: CRP=%v RMF=%v matched=%d/%d avg_runtime=%v microseconds/gps point",
		crp, rmf, len(matchedPoints), len(gpsTraj), avgRuntimeMicros)

	if crp < ommExpectedMinMelbAccuracy {
		t.Fatalf("Hengfeng Li online MHT CRP below threshold: got %v, want >= %v", crp, ommExpectedMinMelbAccuracy)
	}
	if rmf > ommExpectedMaxMelbRMF {
		t.Fatalf("Hengfeng Li online MHT RMF above threshold: got %v, want <= %v", rmf, ommExpectedMaxMelbRMF)
	}
}

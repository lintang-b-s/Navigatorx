package onlinemapmatching

import (
	"archive/tar"
	"compress/gzip"
	"context"
	"encoding/csv"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"math/rand"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strconv"
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
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/tiler"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/mmcloughlin/geohash"
	"github.com/spf13/viper"
	"go.uber.org/zap"
	"gonum.org/v1/gonum/stat"
)

const (
	hhOsmFile                         = "./data/eval/mapmatching/shanghai.osm.pbf"
	hhGraphFile                string = "./data/original_eval_mm_hh.graph"
	hhOverlayGraphFile         string = "./data/overlay_graph_eval_mm_hh.graph"
	hhMetricsFile              string = "./data/metrics_eval_mm_hh.txt"
	hhTransitionMatrixFile            = "./data/eval/mapmatching/omm_transition_history_id_hh.txt"
	hhShanghaiDatasetDriveFile        = "https://drive.google.com/uc?export=download&id=1Ecaabtah1TXyx5T-QqAngPPSEMhUQwaV"
	hhShanghaiOsmDriveFile            = "https://drive.google.com/uc?export=download&id=1cWnidrIprbzHiNxEq1zVgIDiawInqlVj"
	hhShanghaiDataFilePath            = "./data/eval/mapmatching/shanghai.tar.gz"
	hhShanghaiTestDataPath            = "./data/eval/mapmatching/Shanghai/track"
	hhShanghaiGroundTruthPath         = "./data/eval/mapmatching/Shanghai/ground"
	hhShanghaiPolylinesPath           = "./data/eval/mapmatching/Shanghai/polylines"
	hhMlpFile                         = "./data/eval/mapmatching/online_map_match_mlp_hanwenhu.mlp"
	hhLandmarkFile                    = "./data/eval/mapmatching/landmark_hh.lm"
	hhTimeFunctionFile         string = "./data/timefunction_eval_mm_hh.txt"
)

var (
	hhPartitionSizes = []int{8, 11, 13, 14, 15}
	hhInitOnce       sync.Once
	hhInitErr        error
)

type hhQuery struct {
	s, t da.Index
}

func newHHQuery(s, t da.Index) hhQuery {
	return hhQuery{s: s, t: t}
}

func ensureHHConfig(t *testing.T) {
	t.Helper()
	hhInitOnce.Do(func() {
		flag.Parse()
		workingDir, err := config.FindProjectWorkingDir()
		if err != nil {
			hhInitErr = err
			return
		}
		err = config.ReadConfig(workingDir)
		if err != nil {
			hhInitErr = err
			return
		}
		vehicleType := viper.GetString("vehicle_type")
		pkg.VehicleType = pkg.GetVehicleType(vehicleType)
		pkg.DoubleTrackedVehicleEnabled = pkg.GetIsDoubleTrackedVehicle()
		pkg.IsVehicleEnabled = pkg.GetIsVehicle()
		pkg.MotorizedVehicleEnabled = pkg.GetIsMotorizedVehicle()
	})
	if hhInitErr != nil {
		t.Fatalf("failed init config: %v", hhInitErr)
	}
}

func hhDownload(filePath, url string, logger *zap.Logger, name string) error {
	if _, err := os.Stat(filePath); os.IsNotExist(err) {
		logger.Sugar().Infof("downloading evaluation %s dataset.....", name)
		dir := filepath.Dir(filePath)
		if err := os.MkdirAll(dir, os.ModePerm); err != nil {
			return fmt.Errorf("download: MkdirAll failed %v", err)
		}
		output, err := os.Create(filePath)
		if err != nil {
			return fmt.Errorf("download: Create failed %v", err)
		}
		defer output.Close()
		logger.Sugar().Infof("downloading file......")
		response, err := http.Get(url)
		if err != nil {
			return fmt.Errorf("download: http.Get failed %v", err)
		}
		defer response.Body.Close()
		if _, err = io.Copy(output, response.Body); err != nil {
			return fmt.Errorf("download: io.Copy failed %v", err)
		}
		logger.Sugar().Infof("download complete")
	}
	return nil
}

func hhBuildCRPGraph(t *testing.T) (*engine.Engine, *da.Graph, *zap.Logger, *da.SparseMatrix[int]) {
	t.Helper()
	pkg.RegionName = "hanwenhu"

	logger, err := log.New()
	if err != nil {
		t.Fatalf("log.New failed: %v", err)
	}
	op := osmparser.NewOSMParserV2()
	err = hhDownload(hhOsmFile, hhShanghaiOsmDriveFile, logger, "shanghai openstreetmap file")
	if err != nil {
		t.Fatalf("download osm failed: %v", err)
	}
	graph, edgeInfoIds, err := op.Parse(hhOsmFile, logger)
	if err != nil {
		t.Fatalf("osm parse failed: %v", err)
	}

	ps := make([]int, len(hhPartitionSizes))
	for i := 0; i < len(ps); i++ {
		ps[i] = 1 << hhPartitionSizes[i]
	}
	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 5, graph, logger, false, false)
	mp.RunMultilevelPartitioning()
	if err = mp.SaveToFile(hhMlpFile); err != nil {
		t.Fatalf("save mlp failed: %v", err)
	}
	mlp := da.NewPlainMLP()
	if err = mlp.ReadMlpFile(hhMlpFile); err != nil {
		t.Fatalf("read mlp failed: %v", err)
	}
	prep := prepo.NewPreprocessor(graph, mlp, logger, hhGraphFile, hhOverlayGraphFile, edgeInfoIds)
	if err = prep.PreProcessing(true); err != nil {
		t.Fatalf("preprocessing failed: %v", err)
	}
	cust := customizer.NewCustomizer(hhGraphFile, hhOverlayGraphFile, hhMetricsFile, hhTimeFunctionFile, hhLandmarkFile, logger)
	if _, err = cust.Customize(); err != nil {
		t.Fatalf("customize failed: %v", err)
	}
	re, err := engine.NewEngine(hhGraphFile, hhOverlayGraphFile, hhMetricsFile, hhLandmarkFile, hhTimeFunctionFile, logger)
	if err != nil {
		t.Fatalf("new engine failed: %v", err)
	}

	logger.Sugar().Infof("customization phase of Customizable Route Planning (CRP) done....")
	t.Logf("customization phase of Customizable Route Planning (CRP) done....")

	n := graph.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))
	logger.Sugar().Infof("building transition matrix....")
	t.Logf("building transition matrix....")
	numQueries := 1000
	i := 0
	queries := make([]hhQuery, 0, n)
	for i < numQueries {
		s := da.Index(rd.Intn(n))
		tt := da.Index(rd.Intn(n))
		if s == tt || !graph.PathExists(s, tt) {
			continue
		}
		queries = append(queries, newHHQuery(s, tt))
		i++
	}

	computeRoute := func(q hhQuery) []da.Index {
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

	workers := concurrent.NewWorkerPool[hhQuery, []da.Index](100, 5)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	var N *da.SparseMatrix[int]
	if _, err = os.Stat(hhTransitionMatrixFile); err == nil {
		logger.Info("reading transition matrix from file...")
		N, err = da.ReadSparseMatrixFromFile[int](hhTransitionMatrixFile, int(0), func(a, b int) bool { return a == b })
		if err != nil {
			t.Fatalf("read sparse matrix failed: %v", err)
		}
	} else {
		N = da.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(), 0, func(a, b int) bool { return a == b })
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
				if err := N.WriteToFile(hhTransitionMatrixFile); err != nil {
					t.Logf("error writing transition matrix: %v", err)
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
	t.Logf("transition matrix built....")
	return re, graph, logger, N
}

func hhReadCSV(filePath string) ([]map[string]string, error) {
	file, err := os.Open(filePath)
	if err != nil {
		return nil, fmt.Errorf("failed to open file: %w", err)
	}
	defer file.Close()
	reader := csv.NewReader(file)
	reader.TrimLeadingSpace = true
	headers, err := reader.Read()
	if err != nil {
		return nil, fmt.Errorf("failed to read headers: %w", err)
	}
	var records []map[string]string
	for {
		row, err := reader.Read()
		if err != nil && errors.Is(err, io.EOF) {
			break
		} else if err != nil {
			return nil, fmt.Errorf("error read csv: %w", err)
		}
		record := make(map[string]string, len(headers))
		for i, value := range row {
			record[headers[i]] = value
		}
		records = append(records, record)
	}
	return records, nil
}

func hhReadAllCSVInDir(dirPath string) (map[string][]map[string]string, error) {
	matches, err := filepath.Glob(filepath.Join(dirPath, "*.csv"))
	if err != nil {
		return nil, fmt.Errorf("failed to glob directory: %w", err)
	}
	if len(matches) == 0 {
		return nil, fmt.Errorf("no CSV files found in: %s", dirPath)
	}
	results := make(map[string][]map[string]string)
	for _, filePath := range matches {
		fileName := filepath.Base(filePath)
		records, err := hhReadCSV(filePath)
		if err != nil {
			return nil, err
		}
		results[fileName] = records
	}
	return results, nil
}

func hhUnixTimestampToTime(ut int64) (time.Time, error) {
	return time.UnixMilli(ut), nil
}

func hhExtractTarGz(gzipStream io.Reader, destDir string) error {
	if _, err := os.Stat(filepath.Join(destDir, "Shanghai")); err == nil {
		return nil
	}
	uncompressedStream, err := gzip.NewReader(gzipStream)
	if err != nil {
		return fmt.Errorf("extractTarGz: gzip NewReader failed: %v", err)
	}
	tarReader := tar.NewReader(uncompressedStream)
	for {
		header, err := tarReader.Next()
		if err == io.EOF {
			break
		}
		if err != nil {
			return fmt.Errorf("extractTarGz: Next() failed: %v", err)
		}
		targetPath := filepath.Join(destDir, header.Name)
		switch header.Typeflag {
		case tar.TypeDir:
			if err := os.Mkdir(targetPath, 0755); err != nil {
				return fmt.Errorf("extractTarGz: mkdir failed: %v", err)
			}
		case tar.TypeReg:
			if err := os.MkdirAll(filepath.Dir(targetPath), 0755); err != nil {
				return fmt.Errorf("extractTarGz: mkdir for file failed: %v", err)
			}
			outFile, err := os.Create(targetPath)
			if err != nil {
				return fmt.Errorf("extractTarGz: Create failed: %v", err)
			}
			if _, err := io.Copy(outFile, tarReader); err != nil {
				return fmt.Errorf("extractTarGz: Copy failed: %v", err)
			}
			outFile.Close()
		default:
			return fmt.Errorf("extractTarGz: unknown type: %v in %s", header.Typeflag, header.Name)
		}
	}
	return nil
}

func hhTrackIDFromName(trajName string) string {
	base := filepath.Base(trajName)
	ext := filepath.Ext(base)
	return strings.TrimSuffix(base, ext)
}

func hhWritePolyline(filePath string, points []da.Coordinate) error {
	if err := os.MkdirAll(filepath.Dir(filePath), 0755); err != nil {
		return fmt.Errorf("write polyline: mkdir failed: %v", err)
	}
	polyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(points))
	if err := os.WriteFile(filePath, []byte(polyline), 0644); err != nil {
		return fmt.Errorf("write polyline: write file failed: %v", err)
	}
	return nil
}

// go test ./tests/map_matching  -run TestHanwenhuOnlineMapMatching  -v -timeout=0  -count=1
func TestHanwenhuOnlineMapMatching(t *testing.T) {
	ensureHHConfig(t)
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		t.Fatalf("find project working dir failed: %v", err)
	}

	_, graph, logger, N := hhBuildCRPGraph(t)
	shanghaiDataFilePath := ohmmProjectPath(workingDir, hhShanghaiDataFilePath)
	if err := hhDownload(shanghaiDataFilePath, hhShanghaiDatasetDriveFile, logger, "shanghai dataset"); err != nil {
		t.Fatalf("download dataset failed: %v", err)
	}
	gzFile, err := os.Open(shanghaiDataFilePath)
	if err != nil {
		t.Fatalf("open shanghai tar.gz failed: %v", err)
	}
	defer gzFile.Close()
	if err := hhExtractTarGz(gzFile, filepath.Join(workingDir, "data/eval/mapmatching")); err != nil {
		t.Fatalf("extract tar.gz failed: %v", err)
	}
	gpsTrajectories, err := hhReadAllCSVInDir(ohmmProjectPath(workingDir, hhShanghaiTestDataPath))
	if err != nil {
		t.Fatalf("read trajectories failed: %v", err)
	}

	rtree := spatialindex.NewRtree()

	mg := da.InitializeMapMatchingGraph(graph.NumberOfVertices())
	rtree.BuildMapMatch(mg, logger)
	onlineMapMatcherEngine := online.NewOnlineMapMatchMHTClient(mg, rtree, 8.33333, 8.3333, 0.001, 5.0, 0.000001, 0.04, 3, N)
	avgRuntimePerGpsPointAll := 0.0
	totalPoints := 0.0
	totalRuntime := 0.0
	matchingErrors := make([]float64, 0, len(gpsTrajectories))

	tilingEngine := tiler.NewTilingEngine(graph, logger)
	centerGeohash := uint64(0)

	for trajName, gpsTraj := range gpsTrajectories {
		var (
			prevLat, prevLon float64
			hasPrev          bool
			candidates       []*ma.Candidate
			speedMeanK       = 8.333
			speedStdK        = 8.333
			lastBearing      = 0.0
			k                = 1
			matchedPoint     *da.MatchedGPSPoint
		)
		mapMatchPointResult := make([]*da.MatchedGPSPoint, 0)
		gpsTrackPolyline := make([]da.Coordinate, 0, len(gpsTraj))
		matchResultPolyline := make([]da.Coordinate, 0, len(gpsTraj))
		avgRuntimePerGpsPoint := 0.0
		nowDataset := time.Now()

		locatetime, err := strconv.ParseInt(gpsTraj[0]["locatetime"], 10, 64)
		if err != nil {
			t.Fatalf("parse locatetime failed: %v", err)
		}
		startTime, err := hhUnixTimestampToTime(locatetime)
		if err != nil {
			t.Fatalf("convert unix time failed: %v", err)
		}
		curGpsTime := startTime

		for i := 0; i < len(gpsTraj); i++ {
			gps := gpsTraj[i]
			lat, err := strconv.ParseFloat(gps["lat"], 64)
			if err != nil {
				t.Fatalf("parse lat failed: %v", err)
			}
			lon, err := strconv.ParseFloat(gps["lon"], 64)
			if err != nil {
				t.Fatalf("parse lon failed: %v", err)
			}
			deltaTime := 2.0
			speed := 8.333
			if hasPrev {
				if util.Gt(deltaTime, 0) {
					dist := geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon)
					speed = util.KilometerToMeter(dist) / deltaTime
				}
			} else {
				hasPrev = true
			}
			prevLat, prevLon = lat, lon
			gpsTrackPolyline = append(gpsTrackPolyline, da.NewCoordinate(lat, lon))
			now := time.Now()
			curGps := da.NewGPSPoint(lat, lon, curGpsTime, speed, deltaTime)

			currGeohash := geohash.EncodeIntWithPrecision(curGps.Lat(), curGps.Lon(), tiler.GeohashBits)
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
					t.Fatal(err)
				}

				// Rebuild the R-tree with the new tile data
				rtree.Reset()
				rtree.BuildMapMatch(mg, logger)

				onlineMapMatcherEngine = online.NewOnlineMapMatchMHTClient(
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

			matchedPoint, candidates, speedMeanK, speedStdK = onlineMapMatcherEngine.OnlineMapMatch(curGps, k, candidates, speedMeanK, speedStdK, lastBearing)
			k++
			lastBearing = matchedPoint.GetBearing()
			mapMatchPointResult = append(mapMatchPointResult, matchedPoint)
			if matchedPoint.GetEdgeId() != da.INVALID_EDGE_ID {
				matchResultPolyline = append(matchResultPolyline, matchedPoint.GetMatchedCoord())
			}
			avgRuntimePerGpsPoint += float64(time.Since(now).Microseconds())
			curGpsTime = curGpsTime.Add(2)
		}

		trackID := hhTrackIDFromName(trajName)
		resultPolylinePath := filepath.Join(hhShanghaiPolylinesPath, fmt.Sprintf("result_polyline_%s.txt", trackID))
		if err := hhWritePolyline(resultPolylinePath, matchResultPolyline); err != nil {
			t.Fatalf("write result polyline failed for %s: %v", trajName, err)
		}
		gpsTrackPath := filepath.Join(hhShanghaiPolylinesPath, fmt.Sprintf("gps_track_%s.txt", trackID))
		if err := hhWritePolyline(gpsTrackPath, gpsTrackPolyline); err != nil {
			t.Fatalf("write gps polyline failed for %s: %v", trajName, err)
		}

		avgRuntimePerGpsPointAll += avgRuntimePerGpsPoint / float64(len(gpsTraj))
		runtimeDataset := time.Since(nowDataset).Milliseconds()
		totalRuntime += float64(runtimeDataset)
		totalPoints += float64(len(gpsTraj))

		shanghaiGroundTruthPath := ohmmProjectPath(workingDir, hhShanghaiGroundTruthPath)
		groundTruth, err := hhReadCSV(filepath.Join(shanghaiGroundTruthPath, trajName))
		if err != nil {
			t.Fatalf("read ground truth failed: %v", err)
		}
		groundTruthLength := 0.0
		for j := 1; j < len(groundTruth); j++ {
			prevGt := groundTruth[j-1]
			prevLat, err := strconv.ParseFloat(prevGt["lat"], 64)
			if err != nil {
				t.Fatalf("parse gt prev lat failed: %v", err)
			}
			prevLon, err := strconv.ParseFloat(prevGt["lon"], 64)
			if err != nil {
				t.Fatalf("parse gt prev lon failed: %v", err)
			}
			gt := groundTruth[j]
			lat, err := strconv.ParseFloat(gt["lat"], 64)
			if err != nil {
				t.Fatalf("parse gt lat failed: %v", err)
			}
			lon, err := strconv.ParseFloat(gt["lon"], 64)
			if err != nil {
				t.Fatalf("parse gt lon failed: %v", err)
			}
			dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon))
			groundTruthLength += dist
		}

		start := 0
		prevMp := mapMatchPointResult[start]
		if prevMp.GetEdgeId() == da.INVALID_EDGE_ID {
			for q := start + 1; q < len(mapMatchPointResult); q++ {
				if mapMatchPointResult[q].GetEdgeId() != da.INVALID_EDGE_ID {
					prevMp = mapMatchPointResult[q]
					start = q
					break
				}
			}
		}

		matchLength := 0.0
		for j := start + 1; j < len(mapMatchPointResult); j++ {
			prevLat := prevMp.GetMatchedCoord().GetLat()
			prevLon := prevMp.GetMatchedCoord().GetLon()
			mp := mapMatchPointResult[j]
			lat := mp.GetMatchedCoord().GetLat()
			lon := mp.GetMatchedCoord().GetLon()
			if mp.GetEdgeId() != da.INVALID_EDGE_ID {
				dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon))
				matchLength += dist
				prevMp = mp
			}
		}

		matchingError := math.Abs(matchLength-groundTruthLength) / groundTruthLength
		matchingErrors = append(matchingErrors, matchingError)
		t.Logf("trajectory %v completed", trajName)
	}

	avgRuntimePerGpsPointAll /= float64(len(gpsTrajectories))
	t.Logf("avg runtime per gpt point: %v microseconds", avgRuntimePerGpsPointAll)
	t.Logf("matching efficiency: %v points/ms", totalPoints/totalRuntime)

	sort.Float64s(matchingErrors)
	t.Logf("%-15s %-10s", "Error", "CDF P(X<=x)")
	t.Logf("-------------------------")

	for x := 0.02; util.Le(x, 0.4); x += 0.01 {
		y := stat.CDF(x, stat.Empirical, matchingErrors, nil)
		t.Logf("%-15.4f %-10.4f", x, y)
	}
	cdfAtPointFourteen := stat.CDF(0.14, stat.Empirical, matchingErrors, nil)
	if util.Lt(cdfAtPointFourteen, 0.95) {
		t.Fatalf("expected CDF P(X<=0.14) to be at least 0.95, got %v", cdfAtPointFourteen)
	}

	cdfAtPointFourty := stat.CDF(0.4, stat.Empirical, matchingErrors, nil)
	if util.Lt(cdfAtPointFourty, 0.995) {
		t.Fatalf("expected CDF P(X<=0.4) to be at least 0.995, got %v", cdfAtPointFourty)
	}
}

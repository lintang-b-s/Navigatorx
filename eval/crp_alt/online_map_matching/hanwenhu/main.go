package main

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
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	onlinemapmatching "github.com/lintang-b-s/Navigatorx/eval/crp_alt/online_map_matching"
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
	"gonum.org/v1/gonum/stat"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

/*
go run eval/crp_alt/online_map_matching/hanwenhu/main.go

pastikan downloaded file tidak corrupt.

[1] Taguchi, S., Koide, S. and Yoshimura, T. (2019) “Online Map Matching With Route
Prediction,” IEEE Transactions on Intelligent Transportation Systems, 20(1), pp.
338–347. Available at: https://doi.org/10.1109/TITS.2018.2812147.

[2] Hu, H. et al. (2023) “AMM: An Adaptive Online Map Matching Algorithm,”
IEEE Transactions on Intelligent Transportation Systems, 24(5), pp. 5039–5051.
Available at: https://doi.org/10.1109/TITS.2023.3237519.


*/

// ini evaluasi implementasi online map matching pakai multiple hypothesis technique [1] (mapmatch_mht.go)
// pakai dataset dari ref[2]: https://github.com/Hanwen-Hu/AMM/tree/main/MatchData/Shanghai

const (
	osmFile                            = "./data/eval/mapmatching/shanghai.osm.pbf"
	graphFile                   string = "./data/original_eval_mm_hh.ngraph"
	overlayGraphFile            string = "./data/overlay_graph_eval_mm_hh.ngraph"
	metricsFile                 string = "./data/metrics_eval_mm_hh.nmt"
	transitionMatrixFilepath           = "./data/eval/mapmatching/omm_transition_history_id_hh.ntm"
	shanghaiDatasetDriveFile           = "https://drive.google.com/uc?export=download&id=1Ecaabtah1TXyx5T-QqAngPPSEMhUQwaV"
	shanghaiOsmDriveFile               = "https://drive.google.com/uc?export=download&id=1cWnidrIprbzHiNxEq1zVgIDiawInqlVj"
	shanghaiDataFilePath               = "./data/eval/mapmatching/shanghai.tar.gz"
	shanghaiTestDataFilePath           = "./data/eval/mapmatching/Shanghai/track"
	shanghaiGroundTruthFilepath        = "./data/eval/mapmatching/Shanghai/ground"
	shanghaiPolylinesPath              = "./data/eval/mapmatching/Shanghai/polylines"
	mlpFile                            = "./data/eval/mapmatching/online_map_match_mlp_hanwenhu.mlp"
	landmarkFile                       = "./data/eval/mapmatching/landmark_hh.nlm"
	timeFunctionFile            string = "./data/timefunction_eval_mm_hh.ntf"
)

var (
	partitionSizes = []int{8, 11, 13, 14, 15}
)

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

type query struct {
	s, t da.Index
}

func newQuery(s, t da.Index) query {
	return query{
		s: s,
		t: t,
	}
}

func buildCRPGraph() (*engine.Engine[int32], *da.Graph, *zap.Logger, *da.SparseMatrix[int], error) {

	flag.Parse()
	logger, err := log.New()
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: log.New() failed %w", err)
	}
	op := osmparser.NewOSMParserV2[int32]()
	err = onlinemapmatching.Download(osmFile, shanghaiOsmDriveFile, logger, "shanghai openstreetmap file")
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: download() failed %w", err)
	}
	graph, timeFunction, edgeInfoIds, err := op.Parse(osmFile, logger)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: osmparse.Parse() failed: %v", err)
	}

	ps := make([]int, len(partitionSizes))
	for i := 0; i < len(ps); i++ {
		pow := partitionSizes[i]
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger,
		false,
		false,
	)
	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: mp.SaveToFile() failed: %v", err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpFile)
	if err != nil {
		panic(err)
	}

	prep := prepo.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: prep.PreProcessing() failed: %v", err)
	}

	cust := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)
	_, err = cust.Customize()
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: cust.Customize() failed: %v", err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: engine.NewEngine() failed: %v", err)
	}

	logger.Sugar().Infof("customization phase of Customizable Route Planning (CRP) done....")

	n := graph.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))

	logger.Sugar().Infof("building transition matrix....")
	numQueries := 1000.0
	i := 0
	queries := make([]query, 0, n)

	for i < int(numQueries) {
		s := da.Index(rd.Intn(n))
		t := da.Index(rd.Intn(n))
		if s == t {
			continue
		}
		if !graph.PathExists(s, t) {
			continue
		}

		queries = append(queries, newQuery(s, t))
		i++
	}

	computeRoute := func(q query) []da.Index {
		s, t := q.s, q.t
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0)
		as := graph.GetDummyOutEdgeId(s)
		at := graph.GetDummyInEdgeId(t)

		sVertex := graph.GetVertex(s)
		tVertex := graph.GetVertex(t)
		emptyCoords := make([]da.Coordinate, 0)
		sPhantomNode := da.NewPhantomNode(sVertex.GetCoordinate(), 0, 0, as, sVertex.GetFirstIn(), 0, 0, emptyCoords, emptyCoords)
		tPhantomNode := da.NewPhantomNode(tVertex.GetCoordinate(), 0, 0, tVertex.GetFirstOut(), at, 0, 0, emptyCoords, emptyCoords)

		_, _, _, edges, _ := crpQuery.ShortestPathSearch(sPhantomNode, tPhantomNode)
		return edges
	}

	workers := concurrent.NewWorkerPool[query, []da.Index](100, 25_000)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	var N *da.SparseMatrix[int]
	_, err = os.Stat(transitionMatrixFilepath)
	if err == nil {
		logger.Info("reading transition matrix from file...")

		N, err = da.ReadSparseMatrixFromFile[int](transitionMatrixFilepath, int(0),
			func(a, b int) bool { return a == b })
		if err != nil {
			return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph:  da.ReadSparseMatrixFromFile() failed: %v", err)
		}
	} else {
		N = da.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(),
			0, func(a, b int) bool { return a == b })
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

			if counter%1e2 == 0 {
				fmt.Printf("completed query: %v\n", counter)
				if err := N.WriteToFile(transitionMatrixFilepath); err != nil {
					fmt.Printf("error writing transition matrix: %v\n", err)
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

	return re, graph, logger, N, nil
}

func readCsv(filePath string) ([]map[string]string, error) {
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

func readAllCSVInDir(dirPath string) (map[string][]map[string]string, error) {
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
		records, err := readCsv(filePath)
		if err != nil {
			return nil, err
		}
		results[fileName] = records
	}

	return results, nil
}

func unixTimestampToTime(ut int64) (time.Time, error) {
	tm := time.UnixMilli(ut)
	return tm, nil
}

// Source - https://stackoverflow.com/a/57640231
// Posted by Victor Rusnac
// Retrieved 2026-03-25, License - CC BY-SA 4.0
func extractTarGz(gzipStream io.Reader) error {
	const destDir = "./data/eval/mapmatching/"
	if _, err := os.Stat(destDir + "Shanghai"); err == nil {
		// already extracted
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
			return fmt.Errorf("extractTarGz:  Next() failed: %v", err)

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
			return fmt.Errorf("extractTarGz: uknown type: %v in %s",
				header.Typeflag,
				header.Name)
		}
	}

	return nil
}

func trackIDFromName(trajName string) string {
	base := filepath.Base(trajName)
	ext := filepath.Ext(base)
	return strings.TrimSuffix(base, ext)
}

func writePolyline(filePath string, points []da.Coordinate) error {
	if err := os.MkdirAll(filepath.Dir(filePath), 0755); err != nil {
		return fmt.Errorf("write polyline: mkdir failed: %v", err)
	}
	polyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(points))
	if err := os.WriteFile(filePath, []byte(polyline), 0644); err != nil {
		return fmt.Errorf("write polyline: write file failed: %v", err)
	}
	return nil
}

func main() {
	re, graph, logger, N, err := buildCRPGraph()
	if err != nil {
		panic(err)
	}

	err = onlinemapmatching.Download(shanghaiDataFilePath, shanghaiDatasetDriveFile, logger, "shanghai dataset")
	if err != nil {
		panic(err)
	}

	gzFile, err := os.Open(shanghaiDataFilePath)
	if err != nil {
		panic(err)
	}
	err = extractTarGz(gzFile)
	if err != nil {
		panic(err)
	}

	gpsTrajectories, err := readAllCSVInDir(shanghaiTestDataFilePath)
	if err != nil {
		panic(err)
	}

	rtree := spatialindex.NewRtree()
	rtree.Build(graph, logger)
	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(graph, rtree, 8.33333, 8.3333, 0.001, 5.0, 0.000001,
		0.06, 3, N, func(eID da.Index) float64 {
			return re.GetRoutingEngine().GetSegmentLength(eID, true)
		}) // speed in meter/s,
	avgRuntimePerGpsPointAll := 0.0

	totalPoints := 0.0
	totalRuntime := 0.0
	matchingErrors := make([]float64, 0, len(gpsTrajectories))

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

		locatetime, err := util.ParseTextInt64(gpsTraj[0]["locatetime"])
		if err != nil {
			panic(err)
		}
		startTime, err := unixTimestampToTime(locatetime)
		if err != nil {
			panic(err)
		}
		curGpsTime := startTime

		for i := 0; i < len(gpsTraj); i++ {
			gps := gpsTraj[i]

			lat, err := util.ParseTextFloat64(gps["lat"])
			if err != nil {
				panic(err)
			}

			lon, err := util.ParseTextFloat64(gps["lon"])
			if err != nil {
				panic(err)
			}

			var deltaTime = 2.0 // in seconds
			var speed = 8.333

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

		trackID := trackIDFromName(trajName)
		resultPolylinePath := filepath.Join(shanghaiPolylinesPath, fmt.Sprintf("result_polyline_%s.txt", trackID))
		if err := writePolyline(resultPolylinePath, matchResultPolyline); err != nil {
			panic(err)
		}
		gpsTrackPath := filepath.Join(shanghaiPolylinesPath, fmt.Sprintf("gps_track_%s.txt", trackID))
		if err := writePolyline(gpsTrackPath, gpsTrackPolyline); err != nil {
			panic(err)
		}

		avgRuntimePerGpsPointAll += avgRuntimePerGpsPoint / float64(len(gpsTraj))

		runtimeDataset := time.Since(nowDataset).Milliseconds()
		totalRuntime += float64(runtimeDataset)
		totalPoints += float64(len(gpsTraj))

		// calc rmf
		// buat rmf/matching error kita ngikut definisi matching error di implementasi paper [2]: https://github.com/Hanwen-Hu/AMM/blob/main/Algorithm/src/main/java/Matching.java
		groundTruth, err := readCsv(shanghaiGroundTruthFilepath + "/" + trajName)
		if err != nil {
			panic(err)
		}
		groundTruthLength := 0.0
		for j := 1; j < len(groundTruth); j++ {
			prevGt := groundTruth[j-1]
			prevLat, err := util.ParseTextFloat64(prevGt["lat"])
			if err != nil {
				panic(err)
			}

			prevLon, err := util.ParseTextFloat64(prevGt["lon"])
			if err != nil {
				panic(err)
			}

			gt := groundTruth[j]
			lat, err := util.ParseTextFloat64(gt["lat"])
			if err != nil {
				panic(err)
			}

			lon, err := util.ParseTextFloat64(gt["lon"])
			if err != nil {
				panic(err)
			}

			dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon))
			groundTruthLength += dist
		}

		start := 0
		var prevMp = mapMatchPointResult[start]
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
				dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(
					prevLat, prevLon, lat, lon,
				))
				matchLength += dist
				prevMp = mp
			}
		}

		matchingError := math.Abs(matchLength-groundTruthLength) / groundTruthLength
		matchingErrors = append(matchingErrors, matchingError)
		fmt.Printf("trajectory %v completed\n", trajName)
	}

	avgRuntimePerGpsPointAll /= float64(len(gpsTrajectories))
	fmt.Printf("avg runtime per gpt point: %v microseconds\n", avgRuntimePerGpsPointAll)

	fmt.Printf("matching efficiency: %v points/ms\n", totalPoints/totalRuntime)

	sort.Float64s(matchingErrors)

	fmt.Printf("%-15s %-10s\n", "Error", "CDF P(X<=x)")
	fmt.Println("-------------------------")
	for x := 0.02; util.Le(x, 0.4); x += 0.01 {
		y := stat.CDF(x, stat.Empirical, matchingErrors, nil)
		fmt.Printf("%-15.4f %-10.4f\n", x, y)
	}

	/*
		avg runtime per gpt point: 12.199466181923274 microseconds
		matching efficiency: 83.44114219114219 points/ms
		Error           CDF P(X<=x)
		-------------------------
		0.0200          0.7000
		0.0300          0.7500
		0.0400          0.7800
		0.0500          0.8350
		0.0600          0.8800
		0.0700          0.8900
		0.0800          0.9050
		0.0900          0.9200
		0.1000          0.9250
		0.1100          0.9350
		0.1200          0.9500
		0.1300          0.9550
		0.1400          0.9600
		0.1500          0.9650
		0.1600          0.9650
		0.1700          0.9650
		0.1800          0.9650
		0.1900          0.9750
		0.2000          0.9750
		0.2100          0.9800
		0.2200          0.9800
		0.2300          0.9850
		0.2400          0.9850
		0.2500          0.9850
		0.2600          0.9850
		0.2700          0.9850
		0.2800          0.9850
		0.2900          0.9850
		0.3000          0.9850
		0.3100          0.9900
		0.3200          0.9900
		0.3300          0.9900
		0.3400          0.9900
		0.3500          0.9950
		0.3600          0.9950
		0.3700          0.9950
		0.3800          0.9950
		0.3900          1.0000
		0.4000          1.0000

		buat perbandingan, see fig. 12 paper [2]
	*/

}

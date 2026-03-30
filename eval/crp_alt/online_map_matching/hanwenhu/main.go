package main

import (
	"archive/tar"
	"compress/gzip"
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
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/online"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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

// ini evaluasi implementasi online map matching pakai multiple hypothesis techinque [1] (mapmatch_mht.go)
// pakai dataset dari ref[2]: https://github.com/Hanwen-Hu/AMM/tree/main/MatchData/Shanghai

const (
	osmFile                            = "./data/eval/mapmatching/shanghai.osm.pbf"
	graphFile                   string = "./data/original_eval_mm_hh.graph"
	overlayGraphFile            string = "./data/overlay_graph_eval_mm_hh.graph"
	metricsFile                 string = "./data/metrics_eval_mm_hh.txt"
	transitionMatrixFilepath           = "./data/eval/mapmatching/omm_transition_history_id_hh.mm"
	shanghaiDatasetDriveFile           = "https://drive.google.com/uc?export=download&id=1Ecaabtah1TXyx5T-QqAngPPSEMhUQwaV"
	shanghaiOsmDriveFile               = "https://drive.google.com/uc?export=download&id=1cWnidrIprbzHiNxEq1zVgIDiawInqlVj"
	shanghaiDataFilePath               = "./data/eval/mapmatching/shanghai.tar.gz"
	shanghaiTestDataFilePath           = "./data/eval/mapmatching/Shanghai/track"
	shanghaiGroundTruthFilepath        = "./data/eval/mapmatching/Shanghai/ground"
	mlpFile                            = "online_map_match_mlp_hanwenhu"
)

var (
	partitionSizes = []int{8, 11, 13, 14, 15}
)

type query struct {
	s, t da.Index
}

func newQuery(s, t da.Index) query {
	return query{
		s: s,
		t: t,
	}
}

func buildCRPGraph() (*engine.Engine, *da.Graph, *zap.Logger, *da.SparseMatrix[int], error) {

	flag.Parse()
	logger, err := log.New()
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: log.New() failed %w", err)
	}
	op := osmparser.NewOSMParserV2()
	err = download(osmFile, shanghaiOsmDriveFile, logger, "shanghai openstreetmap file")
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: download() failed %w", err)
	}
	graph, err := op.Parse(fmt.Sprintf(osmFile), logger, false)
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
		true,
		false,
		true,
	) // i recommend u to use unit-capacity, because it faster, less shorctuts created, faster p2p query runtime

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: mp.SaveToFile() failed: %v", err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s", "crp_inertial_flow_"+mlpFile+".mlp"))
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: mlp.ReadMlpFile() failed: %v", err)
	}

	prep := prepo.NewPreprocessor(graph, mlp, logger, graphFile, overlayGraphFile)
	err = prep.PreProcessing(true)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: prep.PreProcessing() failed: %v", err)
	}

	cust := customizer.NewCustomizer(graphFile, overlayGraphFile, metricsFile, logger)
	m, err := cust.Customize()
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: cust.Customize() failed: %v", err)
	}

	re, err := engine.NewEngine(graphFile, overlayGraphFile, metricsFile, logger)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph: engine.NewEngine() failed: %v", err)
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(2, m, cust, logger)
	if err != nil {
		return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph:  lm.PreprocessALT() failed: %v", err)
	}

	logger.Sugar().Infof("customization phase of Customizable Route Planning (CRP) done....")

	n := graph.NumberOfVertices()
	rd := rand.New(rand.NewSource(time.Now().UnixNano()))

	logger.Sugar().Infof("building transition matrix....")
	numQueries := math.Pow(10, 3)
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

	computeRoute := func(q query) []da.OutEdge {
		s, t := q.s, q.t
		crpQuery := routing.NewCRPALTBidirectionalSearch(re.GetRoutingEngine(), 1.0, lm)
		as := graph.GetDummyOutEdgeId(s)
		at := graph.GetDummyInEdgeId(t)

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
			return nil, nil, nil, nil, fmt.Errorf("buildCRPGraph:  da.ReadSparseMatrixFromFile() failed: %v", err)
		}
	} else {
		N = da.NewSparseMatrix[int](graph.NumberOfEdges(), graph.NumberOfEdges(),
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

	return re, graph, logger, N, nil
}

func download(filePath, url string, logger *zap.Logger, name string) error {

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

		_, err = io.Copy(output, response.Body)
		if err != nil {
			return fmt.Errorf("download: io.Copy failed %v", err)
		}

		logger.Sugar().Infof("download complete")
	}

	return nil
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

	for true {
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
			return fmt.Errorf("extractTarGz: uknown type: %s in %s",
				header.Typeflag,
				header.Name)
		}
	}

	return nil
}

func main() {
	_, graph, logger, N, err := buildCRPGraph()
	if err != nil {
		panic(err)
	}

	err = download(shanghaiDataFilePath, shanghaiDatasetDriveFile, logger, "shanghai dataset")
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
	rtree.Build(graph, 0.07, logger)
	onlineMapMatcherEngine := online.NewOnlineMapMatchMHT(graph, rtree, 8.33333, 8.3333, 0.001, 4.07, 1.0, 0.000001,
		0.06, 3, N) // speed in meter/s, default sampling interval 1.0 seconds (using seatle dataset)

	avgRuntimePerGpsPointAll := 0.0

	totalPoints := 0.0
	totalRuntime := 0.0
	matchingErrors := make([]float64, 0, len(gpsTrajectories))

	for trajName, gpsTraj := range gpsTrajectories {
		var (
			prevLat, prevLon float64
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

		nowDataset := time.Now()

		locatetime, err := strconv.ParseInt(gpsTraj[0]["locatetime"], 10, 64)
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

			lat, err := strconv.ParseFloat(gps["lat"], 64)
			if err != nil {
				panic(err)
			}

			lon, err := strconv.ParseFloat(gps["lon"], 64)
			if err != nil {
				panic(err)
			}

			var deltaTime float64 = 2 // in seconds
			var speed float64 = 8.333

			if hasPrev {

				if util.Gt(deltaTime, 0) {
					dist := geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon)
					speed = util.KilometerToMeter(dist) / deltaTime
				}
			} else {
				hasPrev = true
			}

			prevLat, prevLon = lat, lon

			now := time.Now()
			curGps := da.NewGPSPoint(lat, lon, curGpsTime, speed, deltaTime, false)
			matchedPoint, candidates, speedMeanK, speedStdK = onlineMapMatcherEngine.OnlineMapMatch(curGps, k, candidates, speedMeanK, speedStdK, lastBearing)
			k++
			lastBearing = matchedPoint.GetBearing()
			mapMatchPointResult = append(mapMatchPointResult, matchedPoint)
			avgRuntimePerGpsPoint += float64(time.Now().Sub(now).Microseconds())

			curGpsTime = curGpsTime.Add(2)
		}

		avgRuntimePerGpsPointAll += avgRuntimePerGpsPoint / float64(len(gpsTraj))

		runtimeDataset := time.Now().Sub(nowDataset).Milliseconds()
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
			prevLat, err := strconv.ParseFloat(prevGt["lat"], 64)
			if err != nil {
				panic(err)
			}

			prevLon, err := strconv.ParseFloat(prevGt["lon"], 64)
			if err != nil {
				panic(err)
			}

			gt := groundTruth[j]
			lat, err := strconv.ParseFloat(gt["lat"], 64)
			if err != nil {
				panic(err)
			}

			lon, err := strconv.ParseFloat(gt["lon"], 64)
			if err != nil {
				panic(err)
			}

			dist := util.KilometerToMeter(geo.CalculateGreatCircleDistance(prevLat, prevLon, lat, lon))
			groundTruthLength += dist
		}

		start := 0
		var prevMp *da.MatchedGPSPoint = mapMatchPointResult[start]
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

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
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	prepo "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
	"go.uber.org/zap"
)

const (
	hhOsmFile                  = "./data/eval/mapmatching/shanghai.osm.pbf"
	hhShanghaiDatasetDriveFile = "https://drive.google.com/uc?export=download&id=1Ecaabtah1TXyx5T-QqAngPPSEMhUQwaV"
	hhShanghaiOsmDriveFile     = "https://drive.google.com/uc?export=download&id=1cWnidrIprbzHiNxEq1zVgIDiawInqlVj"
	hhShanghaiDataFilePath     = "./data/eval/mapmatching/shanghai.tar.gz"
	hhShanghaiTestDataPath     = "./data/eval/mapmatching/Shanghai/track"
	hhShanghaiGroundTruthPath  = "./data/eval/mapmatching/Shanghai/ground"
	hhShanghaiPolylinesPath    = "./data/eval/mapmatching/Shanghai/polylines"
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
		if err := util.EnsureDirExists(filePath); err != nil {
			return fmt.Errorf("download: %w", err)
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

// https://github.com/Hanwen-Hu/AMM/tree/main/MatchData/Shanghai
func hhBuildCRPGraph(t *testing.T) (*engine.Engine[int32], *da.Graph, *zap.Logger, *da.SparseMatrix) {
	t.Helper()
	config.InitRegionName("hanwenhu", pkg.TEST)

	logger, err := log.New()
	if err != nil {
		t.Fatalf("log.New failed: %v", err)
	}
	op := extractor.NewExtractor[int32]()
	err = hhDownload(hhOsmFile, hhShanghaiOsmDriveFile, logger, "shanghai openstreetmap file")
	if err != nil {
		t.Fatalf("download osm failed: %v", err)
	}
	graph, rn, timeFunction, err := op.Extract(hhOsmFile, logger)
	if err != nil {
		t.Fatalf("osm parse failed: %v", err)
	}

	ps := make([]int, len(hhPartitionSizes))
	for i := 0; i < len(ps); i++ {
		ps[i] = 1 << hhPartitionSizes[i]
	}
	mp := partitioner.NewMultilevelPartitioner(ps, len(ps), 5, graph, logger, false)
	mp.RunMultilevelPartitioning()
	if err = mp.SaveToFile(); err != nil {
		t.Fatalf("save mlp failed: %v", err)
	}
	mlp := da.NewPlainMLP()
	if err = mlp.ReadMlpFile(); err != nil {
		t.Fatalf("read mlp failed: %v", err)
	}
	prep := prepo.NewPreprocessor(graph, rn, timeFunction, mlp, logger)
	if err = prep.PreProcessing(true); err != nil {
		t.Fatalf("preprocessing failed: %v", err)
	}
	cust := customizer.NewCustomizer[int32](logger)
	if _, err = cust.Customize(); err != nil {
		t.Fatalf("customize failed: %v", err)
	}
	re, err := engine.NewEngine[int32](logger)
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
		crpQuery := routing.NewCRPALTQuery(re.GetRoutingEngine())
		_, spPath, _ := crpQuery.ShortestPathSearch(q.s, q.t)
		return spPath
	}

	workers := concurrent.NewWorkerPool[hhQuery, []da.Index](100, 25_000)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	workers.StartWithContext(ctx, computeRoute)

	var N *da.SparseMatrix
	N = da.NewSparseMatrix(graph.NumberOfEdges(), graph.NumberOfEdges(), 0, func(a, b uint32) bool { return a == b })

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
	trackDir := filepath.Join(destDir, "Shanghai/track")
	groundDir := filepath.Join(destDir, "Shanghai/ground")
	if _, err := os.Stat(trackDir); err == nil {
		if _, err := os.Stat(groundDir); err == nil {
			return nil
		}
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
			if err := os.MkdirAll(targetPath, 0700); err != nil {
				return fmt.Errorf("extractTarGz: mkdir failed: %v", err)
			}
		case tar.TypeReg:
			if err := util.EnsureDirExists(targetPath); err != nil {
				return fmt.Errorf("extractTarGz: mkdir for file failed: %v", err)
			}
			outFile, err := os.Create(targetPath)
			if err != nil {
				return fmt.Errorf("extractTarGz: Create failed: %v", err)
			}
			if _, err := io.Copy(outFile, tarReader); err != nil {
				outFile.Close()
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
	if err := util.EnsureDirExists(filePath); err != nil {
		return fmt.Errorf("write polyline: mkdir failed: %v", err)
	}
	polyline := da.GooglePoylineFromCoords(*da.NewCoordinatesWithInitialValues(points))
	if err := os.WriteFile(filePath, []byte(polyline), 0600); err != nil {
		return fmt.Errorf("write polyline: write file failed: %v", err)
	}
	return nil
}

// todo: update kode ini

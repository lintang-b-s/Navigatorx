// Package tests provides integration and unit tests for the routing engine.
package tests

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"

	"archive/zip"
	"bytes"
	"errors"
	"io"
	"net/http"
	"net/url"
	"os/exec"
	"regexp"

	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

func init() {
	config.InitProfileConfig("car", "test_region")
}

func Setup(t *testing.T, fileName string) (*engine.Engine[int32], *zap.Logger, *customizer.Customizer[int32]) {
	var (
		mlpFile          = fmt.Sprintf("./data/stress_test_%s.mlp", fileName)
		osmfFile         = fmt.Sprintf("./data/%s.osm.pbf", fileName)
		graphFile        = fmt.Sprintf("./data/original_%s_test.ngraph", fileName)
		overlayGraphFile = fmt.Sprintf("./data/overlay_graph_%s_test.ngraph", fileName)
		metricsFile      = fmt.Sprintf("./data/metrics_%s_test.nmt", fileName)
		landmarkFile     = fmt.Sprintf("./data/landmark_%s_test.nlm", fileName)
		timeFunctionFile = fmt.Sprintf("./data/timefunction_%s_test.ntf", fileName)
	)

	if err := os.MkdirAll("./data", 0755); err != nil {
		t.Fatal(err)
	}

	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	op := osmparser.NewOSMParserV2[int32]()

	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		t.Fatal(err)
	}
	graph, timeFunction, edgeInfoIds, err := op.Parse(filepath.Join(workingDir, osmfFile), logger)
	if err != nil {
		t.Fatal(err)
	}

	pss := strings.Split("8,10,11,12,15", ",")
	ps := make([]int, len(pss))
	for i := 0; i < len(ps); i++ {
		pow, err := util.ParseTextInt(pss[i])
		if err != nil {
			t.Fatal(err)
		}
		ps[i] = 1 << pow // 2^pow
	}

	mp := partitioner.NewMultilevelPartitioner(
		ps,
		len(ps),
		5,
		graph, logger, false, true,
	)

	mp.RunMultilevelPartitioning()

	err = mp.SaveToFile(mlpFile)
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile(mlpFile)
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, timeFunction, mlp, logger, graphFile, overlayGraphFile, edgeInfoIds)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer[int32](graphFile, overlayGraphFile, metricsFile, timeFunctionFile, landmarkFile, logger)

	_, err = custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine[int32](graphFile, overlayGraphFile, metricsFile, landmarkFile, timeFunctionFile, logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, logger, custom
}

type PairEdge struct {
	To     int
	Weight float64
}

func NewPairEdge(to int, weight float64) PairEdge {
	return PairEdge{To: to, Weight: weight}
}

func FlattenEdges(es [][]PairEdge) []osmparser.Edge[float64] {
	flatten := make([]osmparser.Edge[float64], 0, len(es))

	eid := 0

	for from, edges := range es {
		for _, e := range edges {
			length := uint32(math.Round(e.Weight * 100))
			flatten = append(flatten, osmparser.NewEdge(uint32(from), uint32(e.To), e.Weight, length, false, 0))
			eid++
		}
	}

	return flatten
}

func Download(filePath, url string, logger *zap.Logger, name string) error {
	if _, err := os.Stat(filePath); os.IsNotExist(err) {
		logger.Sugar().Infof("downloading evaluation %s dataset.....", name)

		dir := filepath.Dir(filePath)
		if err := os.MkdirAll(dir, os.ModePerm); err != nil {
			return fmt.Errorf("download: MkdirAll failed %v", err)
		}

		tmpFilePath := filePath + ".tmp"
		output, err := os.Create(tmpFilePath)
		if err != nil {
			return fmt.Errorf("download: Create failed %v", err)
		}

		logger.Sugar().Infof("downloading file......")
		response, err := http.Get(url)
		if err != nil {
			_ = output.Close()
			_ = os.Remove(tmpFilePath)
			return fmt.Errorf("download: http.Get failed %v", err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		closeErr := output.Close()
		if err != nil {
			_ = os.Remove(tmpFilePath)
			return fmt.Errorf("download: io.Copy failed %v", err)
		}
		if closeErr != nil {
			_ = os.Remove(tmpFilePath)
			return fmt.Errorf("download: output.Close failed %v", closeErr)
		}

		isHTML := strings.Contains(strings.ToLower(response.Header.Get("Content-Type")), "text/html")
		if !isHTML {
			isHTML, err = isHTMLDocument(tmpFilePath)
			if err != nil {
				_ = os.Remove(tmpFilePath)
				return fmt.Errorf("download: detect html failed %v", err)
			}
		}

		if isHTML {
			logger.Sugar().Infof("direct download returned HTML, switching to gdown fallback...")
			_ = os.Remove(tmpFilePath)

			fileID, err := extractGoogleDriveFileID(url)
			if err != nil {
				return fmt.Errorf("download: failed to parse Google Drive file id from url %q: %w", url, err)
			}

			if err := downloadWithGDown(filePath, fileID); err != nil {
				return fmt.Errorf("download: gdown fallback failed: %w", err)
			}
		} else {
			if err := os.Rename(tmpFilePath, filePath); err != nil {
				_ = os.Remove(tmpFilePath)
				return fmt.Errorf("download: rename temp file failed %v", err)
			}
		}

		logger.Sugar().Infof("download complete")
	}

	return nil
}

func isHTMLDocument(path string) (bool, error) {
	f, err := os.OpenFile(path, os.O_RDONLY, 0644)
	if err != nil {
		return false, err
	}
	defer f.Close()

	buf := make([]byte, 2048)
	n, err := f.Read(buf)
	if err != nil && !errors.Is(err, io.EOF) {
		return false, err
	}
	snippet := strings.ToLower(strings.TrimSpace(string(buf[:n])))
	return strings.HasPrefix(snippet, "<!doctype html") || strings.HasPrefix(snippet, "<html"), nil
}

func extractGoogleDriveFileID(rawURL string) (string, error) {
	if !strings.Contains(rawURL, "://") && !strings.Contains(rawURL, "/") {
		return rawURL, nil
	}

	u, err := url.Parse(rawURL)
	if err != nil {
		return "", err
	}
	if id := strings.TrimSpace(u.Query().Get("id")); id != "" {
		return id, nil
	}

	re := regexp.MustCompile(`/d/([a-zA-Z0-9_-]+)`)
	m := re.FindStringSubmatch(rawURL)
	if len(m) > 1 {
		return m[1], nil
	}
	return "", errors.New("google drive id not found")
}

func downloadWithGDown(filePath, fileID string) error {
	if err := ensureGDownInstalled(); err != nil {
		return err
	}

	tmpFilePath := filePath + ".gdown.tmp"
	gdownURL := fmt.Sprintf("https://drive.google.com/uc?id=%s", fileID)
	cmd := exec.Command("gdown", gdownURL, "-O", tmpFilePath)

	var stderr bytes.Buffer
	cmd.Stdout = os.Stdout
	cmd.Stderr = io.MultiWriter(os.Stderr, &stderr)

	if err := cmd.Run(); err != nil {
		_ = os.Remove(tmpFilePath)
		msg := strings.TrimSpace(stderr.String())
		if msg == "" {
			return fmt.Errorf("gdown command failed: %w", err)
		}
		return fmt.Errorf("gdown command failed: %w: %s", err, msg)
	}

	if err := os.Rename(tmpFilePath, filePath); err != nil {
		_ = os.Remove(tmpFilePath)
		return fmt.Errorf("rename temp file failed: %w", err)
	}
	return nil
}

func ensureGDownInstalled() error {
	if _, err := exec.LookPath("gdown"); err == nil {
		return nil
	}

	installCmd := exec.Command("pip", "install", "gdown")
	var stderr bytes.Buffer
	installCmd.Stderr = &stderr
	if err := installCmd.Run(); err != nil {
		msg := strings.TrimSpace(stderr.String())
		if msg == "" {
			return fmt.Errorf("failed to install gdown: %w", err)
		}
		return fmt.Errorf("failed to install gdown: %w: %s", err, msg)
	}

	if _, err := exec.LookPath("gdown"); err != nil {
		return fmt.Errorf("gdown installed but not found in PATH: %w", err)
	}
	return nil
}

func ExtractZip(zipPath, destDir string) error {
	reader, err := zip.OpenReader(zipPath)
	if err != nil {
		return fmt.Errorf("extractZip: OpenReader failed %v", err)
	}
	defer reader.Close()

	if err := os.MkdirAll(destDir, os.ModePerm); err != nil {
		return fmt.Errorf("extractZip: MkdirAll failed %v", err)
	}

	destDir = filepath.Clean(destDir)
	prefix := destDir + string(os.PathSeparator)

	for _, file := range reader.File {
		targetPath := filepath.Join(destDir, file.Name)
		cleanTargetPath := filepath.Clean(targetPath)

		if cleanTargetPath != destDir && !strings.HasPrefix(cleanTargetPath, prefix) {
			return fmt.Errorf("extractZip: invalid path %s", file.Name)
		}

		if file.FileInfo().IsDir() {
			if err := os.MkdirAll(cleanTargetPath, file.Mode()); err != nil {
				return fmt.Errorf("extractZip: MkdirAll failed %v", err)
			}
			continue
		}

		parentDir := filepath.Dir(cleanTargetPath)
		if err := os.MkdirAll(parentDir, os.ModePerm); err != nil {
			return fmt.Errorf("extractZip: MkdirAll parent failed %v", err)
		}

		src, err := file.Open()
		if err != nil {
			return fmt.Errorf("extractZip: File.Open failed %v", err)
		}

		dst, err := os.OpenFile(cleanTargetPath, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, file.Mode())
		if err != nil {
			src.Close()
			return fmt.Errorf("extractZip: OpenFile failed %v", err)
		}

		_, err = io.Copy(dst, src)
		closeErr := dst.Close()
		srcCloseErr := src.Close()
		if err != nil {
			return fmt.Errorf("extractZip: io.Copy failed %v", err)
		}
		if closeErr != nil {
			return fmt.Errorf("extractZip: dst.Close failed %v", closeErr)
		}
		if srcCloseErr != nil {
			return fmt.Errorf("extractZip: src.Close failed %v", srcCloseErr)
		}
	}

	return nil
}

// least significant bit
func LSOne(S int) int {
	return S & -S
}

func Density(n, m int, directed bool) float64 {
	nf := float64(n)
	mf := float64(m)
	if !directed {
		return 2 * mf / (nf * (nf - 1))
	}

	return mf / (nf * (nf - 1))
}

// Package tests provides integration and unit tests for the routing engine.
package tests

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/extractor"
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
	"github.com/lintang-b-s/Navigatorx/pkg/partitioner"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

func init() {
	config.InitProfileConfig("car", "test_region", pkg.TEST)
}

// Setup initializes the test engine, logger, and customizer.
func Setup(t *testing.T, fileName string) (*engine.Engine[int32], *zap.Logger, *customizer.Customizer[int32]) {
	logger, err := log.New()
	if err != nil {
		t.Fatal(err)
	}

	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		logger.Fatal(err.Error())
	}
	outputDir := filepath.Join(workingDir, "data")

	osmfFile := fmt.Sprintf("%s/%s.osm.pbf", outputDir, fileName)

	if err := util.EnsureDirExists(outputDir); err != nil {
		logger.Fatal(err.Error())
	}

	config.InitRegionName(fileName, pkg.TEST)

	op := extractor.NewExtractor[int32](logger)
	_, ebg, rn, _, timeFunction, err := op.Extract(osmfFile)
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
		ebg, logger,
	)

	mp.RunMultilevelPartitioning()
	err = mp.SaveToFile()
	if err != nil {
		t.Fatal(err)
	}

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile()
	if err != nil {
		logger.Fatal(err.Error())
	}
	prep := preprocessor.NewPreprocessor(ebg, rn, timeFunction, mlp, logger)
	err = prep.PreProcessing(true)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("Preprocessing completed successfully.")

	custom := customizer.NewCustomizer[int32](logger)
	_, err = custom.Customize()
	if err != nil {
		t.Fatal(err)
	}

	re, err := engine.NewEngine[int32](logger)
	if err != nil {
		t.Fatal(err)
	}

	return re, logger, custom
}

// Download downloads a remote dataset if it does not already exist.
func Download(filePath, url string, logger *zap.Logger, name string) error {
	if _, err := os.Stat(filePath); os.IsNotExist(err) {
		logger.Sugar().Infof("downloading evaluation %s dataset.....", name)

		if err := util.EnsureDirExists(filePath); err != nil {
			return fmt.Errorf("download: EnsureDirExists failed %w", err)
		}

		tmpFilePath := filePath + ".tmp"
		output, err := os.Create(tmpFilePath) //nolint:gosec // temporary file creation for tests
		if err != nil {
			return fmt.Errorf("download: Create failed %w", err)
		}

		logger.Sugar().Infof("downloading file......")
		response, err := http.Get(url) //nolint:gosec // test helper downloading test fixtures
		if err != nil {
			_ = output.Close()
			_ = os.Remove(tmpFilePath)
			return fmt.Errorf("download: http.Get failed %w", err)
		}
		defer response.Body.Close()

		_, err = io.Copy(output, response.Body)
		closeErr := output.Close()
		if err != nil {
			_ = os.Remove(tmpFilePath)
			return fmt.Errorf("download: io.Copy failed %w", err)
		}
		if closeErr != nil {
			_ = os.Remove(tmpFilePath)
			return fmt.Errorf("download: output.Close failed %w", closeErr)
		}

		isHTML := strings.Contains(strings.ToLower(response.Header.Get("Content-Type")), "text/html")
		if !isHTML {
			isHTML, err = isHTMLDocument(tmpFilePath)
			if err != nil {
				_ = os.Remove(tmpFilePath)
				return fmt.Errorf("download: detect html failed %w", err)
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
				return fmt.Errorf("download: rename temp file failed %w", err)
			}
		}

		logger.Sugar().Infof("download complete")
	}

	return nil
}

func isHTMLDocument(path string) (bool, error) {
	f, err := os.OpenFile(path, os.O_RDONLY, 0600) //nolint:gosec // inspecting downloaded test file
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

func downloadWithGDown(filePath, fileId string) error {
	if err := ensureGDownInstalled(); err != nil {
		return err
	}

	tmpFilePath := filePath + ".gdown.tmp"
	gdownURL := fmt.Sprintf("https://drive.google.com/uc?id=%s", fileId)
	cmd := exec.Command("gdown", gdownURL, "-O", tmpFilePath) //nolint:gosec // test helper invokes gdown

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

// ExtractZip extracts a zip archive to destDir.
func ExtractZip(zipPath, destDir string) error {
	reader, err := zip.OpenReader(zipPath)
	if err != nil {
		return fmt.Errorf("extractZip: OpenReader failed %w", err)
	}
	defer func() {
		_ = reader.Close()
	}()

	if err := util.EnsureDirExists(destDir); err != nil {
		return fmt.Errorf("extractZip: MkdirAll failed %w", err)
	}

	destDir = filepath.Clean(destDir)
	prefix := destDir + string(os.PathSeparator)

	for _, file := range reader.File {
		targetPath := filepath.Join(destDir, file.Name) //nolint:gosec // target path validated below
		cleanTargetPath := filepath.Clean(targetPath)

		if cleanTargetPath != destDir && !strings.HasPrefix(cleanTargetPath, prefix) {
			return fmt.Errorf("extractZip: invalid path %s", file.Name)
		}

		if file.FileInfo().IsDir() {
			if err := util.EnsureDirExists(targetPath); err != nil {
				return fmt.Errorf("extractZip: MkdirAll failed %w", err)
			}
			continue
		}

		if err := util.EnsureDirExists(cleanTargetPath); err != nil {
			return fmt.Errorf("extractZip: EnsureDirExists failed %w", err)
		}

		src, err := file.Open()
		if err != nil {
			return fmt.Errorf("extractZip: File.Open failed %w", err)
		}

		dst, err := os.OpenFile(cleanTargetPath, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0600)
		if err != nil {
			err = src.Close()
			if err != nil {
				return err
			}
			return fmt.Errorf("extractZip: OpenFile failed %w", err)
		}

		_, err = io.Copy(dst, src) //nolint:gosec // decompression in test archive
		closeErr := dst.Close()
		srcCloseErr := src.Close()
		if err != nil {
			return fmt.Errorf("extractZip: io.Copy failed %w", err)
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

// LSOne least significant bit
func LSOne(s int) int {
	return s & -s
}

// Density calculates graph density.
func Density(n, m int, directed bool) float64 {
	nf := float64(n)
	mf := float64(m)
	if !directed {
		return 2 * mf / (nf * (nf - 1))
	}

	return mf / (nf * (nf - 1))
}

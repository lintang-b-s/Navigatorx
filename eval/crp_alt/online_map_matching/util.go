// onlinemapmatching package provides utilization
package onlinemapmatching

import (
	"archive/zip"
	"bytes"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strings"

	"go.uber.org/zap"
)

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

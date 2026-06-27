package onlinemapmatching

import (
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"github.com/bytedance/gopkg/util/gopool"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	evalutil "github.com/lintang-b-s/Navigatorx/tests"
)

func TestMain(m *testing.M) {
	gopool.SetCap(int32(runtime.NumCPU()))
	if err := ensureMapMatchingDatasetBundle(); err != nil {
		fmt.Fprintf(os.Stderr, "prepare map matching dataset bundle failed: %v\n", err)
		os.Exit(1)
	}

	os.Exit(m.Run())
}

func ensureMapMatchingDatasetBundle() error {
	workingDir, err := config.FindProjectWorkingDir()
	if err != nil {
		return err
	}

	logger, err := log.New()
	if err != nil {
		return err
	}

	zipPath := filepath.Join(workingDir, "data/eval/mapmatching/dataset_bundle.zip")
	if err := evalutil.Download(zipPath, ohmmMelbourneDatasetDriveFile, logger, "map matching dataset bundle file"); err != nil {
		return err
	}
	if err := evalutil.ExtractZip(zipPath, filepath.Join(workingDir, "data/eval/mapmatching")); err != nil {
		return err
	}
	return nil
}

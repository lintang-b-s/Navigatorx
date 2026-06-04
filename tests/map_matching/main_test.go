package onlinemapmatching

import (
	"fmt"
	"os"
	"path/filepath"
	"testing"

	evalutil "github.com/lintang-b-s/Navigatorx/eval/crp_alt/online_map_matching"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
)

func TestMain(m *testing.M) {
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

package main

import (
	"path/filepath"
	"runtime"
	"strings"

	"github.com/bytedance/gopkg/util/gopool"
	flag "github.com/spf13/pflag"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
)

var (
	profileName       string
	profileFilePath   = flag.String("profile", "./data/car.yaml", "profile file path")
	regionName        = flag.String("region", "diy_solo_semarang", "region name")
	edgeSpeedsFile    = flag.StringSlice("segment-speed-file", []string{}, "segment speed csv file. example usage --segment-speed-file=blokade.csv,traffic_solo.csv")
	turnPenaltiesFile = flag.StringSlice("turn-penalty-file", []string{}, "turn penaltiy csv file. example usage --turn-penalty-file=tutup_portal.csv")
)

func init() {
	flag.Parse()
	profileName = strings.ReplaceAll(filepath.Base(*profileFilePath), ".yaml", "")
	config.InitProfileConfig(profileName, *regionName, pkg.ROUTER)
	gopool.SetCap(int32(runtime.NumCPU()))
}

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}

	custom := customizer.NewCustomizer[int32](logger, pkg.ROUTER)
	custom.SetEdgeSpeedsFilePath(*edgeSpeedsFile)
	custom.SetTurnPenaltiesFilePath(*turnPenaltiesFile)

	_, err = custom.Customize()
	if err != nil {
		panic(err)
	}
}

package main

import (
	"flag"
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

var (
	osmFile = flag.String("osm_file", "diy_solo_semarang.osm.pbf", "Openstreetmap .pbf filename")
	mlpFile = flag.String("mlp_file", "crp_inertial_flow_diy_solo_semarang.mlp", "Multilevel partition filename")
)

func main() {
	flag.Parse()
	logger, err := log.New()
	if err != nil {
		panic(err)
	}
	osmParser := osmparser.NewOSMParserV2()
	graph, err := osmParser.Parse(fmt.Sprintf("./data/%s", *osmFile), logger, false)
	if err != nil {
		panic(err)
	}

	mlp := datastructure.NewPlainMLP()
	err = mlp.ReadMlpFile(fmt.Sprintf("./data/%s", *mlpFile))
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger)
	err = prep.PreProcessing(true)
	if err != nil {
		panic(err)
	}

	logger.Sugar().Infof("Preprocessing completed successfully.")

}

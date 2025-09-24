package main

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

func main() {
	logger, err := logger.New()
	if err != nil {
		panic(err)
	}
	osmParser := osmparser.NewOSMParserV2()
	graph := osmParser.Parse("./data/solo_jogja.osm.pbf", logger)

	mlp := datastructure.NewPlainMLP()
	err = mlp.ReadMlpFile("./data/kaffpa_kaffpa_solo_jogja.mlp")
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp, logger)
	err = prep.PreProcessing()
	if err != nil {
		panic(err)
	}

	logger.Sugar().Infof("Preprocessing completed successfully.")

}

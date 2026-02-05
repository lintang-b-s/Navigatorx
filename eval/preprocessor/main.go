package main

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	log "github.com/lintang-b-s/Navigatorx/pkg/logger"
	"github.com/lintang-b-s/Navigatorx/pkg/osmparser"
	preprocessor "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

func main() {
	logger, err := log.New()
	if err != nil {
		panic(err)
	}
	tgprParser := osmparser.NewTPGRParser()
	graph, _, err := tgprParser.ParseTGPRFile("./data/NY/NY.coordinate", "./data/NY/NY.tpgr",
		logger)

	mlp := da.NewPlainMLP()
	err = mlp.ReadMlpFile("./data/crp_inertial_flow_new_york_eval.mlp")
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

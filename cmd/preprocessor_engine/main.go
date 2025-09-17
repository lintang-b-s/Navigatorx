package main

import (
	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx-crp/pkg/osmparser"
	preprocessor "github.com/lintang-b-s/navigatorx-crp/pkg/preprocessor"
)

func main() {
	osmParser := osmparser.NewOSMParserV2()
	graph := osmParser.Parse("./data/solo_jogja.osm.pbf")

	mlp := datastructure.NewPlainMLP()
	err := mlp.ReadMlpFile("./data/multilevel_partitioning_solo_jogja_best_param_crp.mlp")
	if err != nil {
		panic(err)
	}
	prep := preprocessor.NewPreprocessor(graph, mlp)
	err = prep.PreProcessing()
	if err != nil {
		panic(err)
	}

}

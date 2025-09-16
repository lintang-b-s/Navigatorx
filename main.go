package main

import (
	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx-crp/pkg/osmparser"
)

func main() {
	osmParser := osmparser.NewOSMParserV2()
	graph := osmParser.Parse("./data/solo_jogja.osm.pbf")
	err := graph.WriteGraph("./data/solo_jogja.graph")
	if err != nil {
		panic(err)
	}
	_, err = datastructure.ReadGraph("./data/solo_jogja.graph")
	if err != nil {
		panic(err)
	}
	mlp := datastructure.NewPlainMLP()
	err = mlp.ReadMlpFile("./data/multilevel_partitioning_solo_jogja_best_param_crp.mlp")
	if err != nil {
		panic(err)
	}

}

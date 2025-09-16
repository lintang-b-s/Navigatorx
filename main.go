package main

import "github.com/lintang-b-s/navigatorx-crp/pkg/osmparser"

func main() {
	osmParser := osmparser.NewOSMParserV2()
	graph := osmParser.Parse("solo_jogja.osm.pbf")
	_ = graph
}

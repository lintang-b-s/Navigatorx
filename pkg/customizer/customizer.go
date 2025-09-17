package customizer

import (
	"log"

	"github.com/lintang-b-s/navigatorx-crp/pkg/costfunction"
	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx-crp/pkg/metrics"
)

type Customizer struct {
	graphFilePath        string
	overlayGraphFilePath string
	metricOutputFilePath string
}

func NewCustomizer(graphFilePath, overlayGraphFilePath, metricOutputFilePath string) *Customizer {
	return &Customizer{
		graphFilePath:        graphFilePath,
		overlayGraphFilePath: overlayGraphFilePath,
		metricOutputFilePath: metricOutputFilePath,
	}
}

func (c *Customizer) Customize() error {
	log.Printf("Starting customization step of Customizable Route Planning...")

	log.Printf("Reading graph from %s", c.graphFilePath)
	graph, err := datastructure.ReadGraph(c.graphFilePath)
	if err != nil {
		return err
	}

	log.Printf("Reading overlay graph from %s", c.overlayGraphFilePath)
	overlayGraph, err := datastructure.ReadOverlayGraph(c.overlayGraphFilePath)
	if err != nil {
		return err
	}

	costFunction := costfunction.NewTimeCostFunction()
	log.Printf("Building cliques for each cell for each overlay graph level...")
	overlayWeight := datastructure.NewOverlayWeights(overlayGraph.GetWeightVectorSize())
	overlayWeight.Build(graph, overlayGraph, costFunction)

	log.Printf("Building stalling tables...")
	metrics := metrics.NewMetric(graph, costFunction, overlayWeight)
	metrics.BuildStallingTables(overlayGraph, graph)
	err = metrics.WriteToFile(c.metricOutputFilePath)
	if err != nil {
		return err
	}
	log.Printf("Customization step completed successfully.")
	return nil
}

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
	overlayWeight := datastructure.NewOverlayWeights(overlayGraph.GetWeightVectorSize())

	log.Printf("Building overlay weights and stalling tables...")
	overlayWeight.Build(graph, overlayGraph, costFunction)
	metrics := metrics.NewMetric(graph, costFunction, overlayWeight)
	metrics.BuildStallingTables(overlayGraph, graph)
	err = metrics.WriteToFile(c.metricOutputFilePath)
	if err != nil {
		return err
	}
	log.Printf("Customization step completed successfully.")
	return nil
}

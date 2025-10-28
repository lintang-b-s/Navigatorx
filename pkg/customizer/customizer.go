package customizer

import (
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type Customizer struct {
	logger               *zap.Logger
	graphFilePath        string
	overlayGraphFilePath string
	metricOutputFilePath string
}

func NewCustomizer(graphFilePath, overlayGraphFilePath, metricOutputFilePath string,
	logger *zap.Logger) *Customizer {
	return &Customizer{
		graphFilePath:        graphFilePath,
		overlayGraphFilePath: overlayGraphFilePath,
		metricOutputFilePath: metricOutputFilePath,
		logger:               logger,
	}
}

func (c *Customizer) Customize(visualize bool) error {
	c.logger.Sugar().Infof("Starting customization step of Customizable Route Planning...")

	c.logger.Sugar().Infof("Reading graph from %s", c.graphFilePath)
	graph, err := datastructure.ReadGraph(c.graphFilePath)
	if err != nil {
		return err
	}

	c.logger.Sugar().Infof("Reading overlay graph from %s", c.overlayGraphFilePath)
	overlayGraph, err := datastructure.ReadOverlayGraph(c.overlayGraphFilePath)
	if err != nil {
		return err
	}

	costFunction := costfunction.NewTimeCostFunction()
	c.logger.Sugar().Infof("Building cliques for each cell for each overlay graph level...")
	overlayWeight := datastructure.NewOverlayWeights(overlayGraph.GetWeightVectorSize())
	if !visualize {
		overlayWeight.Build(graph, overlayGraph, costFunction)
	} else {
		overlayWeight.VisualizeBuild(graph, overlayGraph, costFunction)
	}

	c.logger.Sugar().Infof("Building stalling tables...")
	metrics := metrics.NewMetric(graph, costFunction, overlayWeight)
	metrics.BuildStallingTables(overlayGraph, graph)
	err = metrics.WriteToFile(c.metricOutputFilePath)
	if err != nil {
		return err
	}
	c.logger.Sugar().Infof("Customization step completed successfully.")
	return nil
}

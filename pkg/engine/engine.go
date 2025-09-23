package engine

import (
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type Engine struct {
	crpRoutingEngine *routing.CRPRoutingEngine
}

func (e *Engine) GetRoutingEngine() *routing.CRPRoutingEngine {
	return e.crpRoutingEngine
}

func NewEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string, logger *zap.Logger) (*Engine, error) {
	initializeRoutingEngine, err := initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath,
		logger)
	if err != nil {
		return nil, err
	}
	return &Engine{
		crpRoutingEngine: initializeRoutingEngine,
	}, nil
}

func initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string, logger *zap.Logger) (*routing.CRPRoutingEngine,
	error) {

	logger.Info("Starting query engine of Customizable Route Planning...")

	logger.Info("Reading graph from ", zap.String("graphFilePath", graphFilePath))
	graph, err := datastructure.ReadGraph(graphFilePath)
	if err != nil {
		return nil, err
	}

	logger.Info("Reading overlay graph from ", zap.String("overlayGraphFilePath", overlayGraphFilePath))
	overlayGraph, err := datastructure.ReadOverlayGraph(overlayGraphFilePath)
	if err != nil {
		return nil, err
	}
	costFunction := costfunction.NewTimeCostFunction()
	logger.Info("Reading stalling tables & time-dependent metrics...")
	metrics, err := metrics.ReadFromFile(metricsFilePath, costFunction)
	if err != nil {
		return nil, err
	}

	return routing.NewCRPRoutingEngine(graph, overlayGraph, metrics, logger), nil
}

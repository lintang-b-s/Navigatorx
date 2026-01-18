package engine

import (
	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
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

func NewEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string, logger *zap.Logger, td bool) (*Engine, error) {
	initializeRoutingEngine, err := initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath,
		logger, td)
	if err != nil {
		return nil, err
	}
	return &Engine{
		crpRoutingEngine: initializeRoutingEngine,
	}, nil
}

func initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string, logger *zap.Logger, td bool,
) (*routing.CRPRoutingEngine,
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
	metrics, err := metrics.ReadFromFile(metricsFilePath, costFunction, td)
	if err != nil {
		return nil, err
	}
	customizer := customizer.NewCustomizer(graphFilePath, overlayGraphFilePath, metricsFilePath, logger)
	customizer.SetGraph(graph)
	customizer.SetOverlayGraph(overlayGraph)
	customizer.SetOverlayWeight(metrics.GetWeights())

	// customizable route planning in road networks section 7.2 (path retrieval)
	puCache, _ := lru.New[routing.PUCacheKey, []datastructure.Index](1 << 20) // 1048576

	return routing.NewCRPRoutingEngine(graph, overlayGraph, metrics, logger, puCache, customizer, costFunction), nil
}

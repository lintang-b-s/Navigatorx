package engine

import (
	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
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

func NewEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string, logger *zap.Logger, td bool, osmwayPWL map[int64]*da.PWL) (*Engine, error) {
	initializeRoutingEngine, err := initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath,
		logger, td, osmwayPWL)
	if err != nil {
		return nil, err
	}
	return &Engine{
		crpRoutingEngine: initializeRoutingEngine,
	}, nil
}

func NewEngineDirect(graph *da.Graph, overlayGraph *da.OverlayGraph, m *metrics.Metric,
	logger *zap.Logger, cst routing.Customizer, cf routing.CostFunction,
	td bool, day string) (*Engine, error) {
	// customizable route planning in road networks section 7.2 (path retrieval)
	puCache, _ := lru.New[routing.PUCacheKey, []da.Index](1 << 20) // 1048576

	re := routing.NewCRPRoutingEngine(graph, overlayGraph, m, logger, puCache, cst, cf)

	return &Engine{
		crpRoutingEngine: re,
	}, nil
}

func initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string, logger *zap.Logger, td bool, osmwayPWL map[int64]*da.PWL,
) (*routing.CRPRoutingEngine,
	error) {

	logger.Info("Starting query engine of Customizable Route Planning...")

	logger.Info("Reading graph from ", zap.String("graphFilePath", graphFilePath))
	graph, err := da.ReadGraph(graphFilePath)
	if err != nil {
		return nil, err
	}

	logger.Info("Reading overlay graph from ", zap.String("overlayGraphFilePath", overlayGraphFilePath))
	overlayGraph, err := da.ReadOverlayGraph(overlayGraphFilePath)
	if err != nil {
		return nil, err
	}
	var m *metrics.Metric
	var cst *customizer.Customizer
	var cf routing.CostFunction
	if td {
		logger.Info("Reading stalling tables & time-dependent metrics...")

		cf = costfunction.NewTimeDependentCostFunction(graph, osmwayPWL)
		m, err = metrics.ReadFromFile(metricsFilePath, td, graph, cf)
		if err != nil {
			return nil, err
		}

	} else {
		logger.Info("Reading stalling tables & time-dependent metrics...")
		cf = costfunction.NewTimeCostFunction()
		m, err = metrics.ReadFromFile(metricsFilePath, td, graph, cf)
		if err != nil {
			return nil, err
		}
	}
	cst = customizer.NewCustomizer(graphFilePath, overlayGraphFilePath, metricsFilePath, logger)
	cst.SetGraph(graph)
	cst.SetOverlayGraph(overlayGraph)
	cst.SetOverlayWeight(m.GetWeights())

	// customizable route planning in road networks section 7.2 (path retrieval)
	puCache, _ := lru.New[routing.PUCacheKey, []da.Index](1 << 20) // 1048576

	return routing.NewCRPRoutingEngine(graph, overlayGraph, m, logger, puCache, cst, cf), nil
}

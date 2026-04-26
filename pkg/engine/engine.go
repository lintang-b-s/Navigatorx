package engine

import (
	"context"

	"github.com/cockroachdb/errors"
	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"go.uber.org/zap"
)

type Engine struct {
	crpRoutingEngine *routing.CRPRoutingEngine
}

func (e *Engine) GetRoutingEngine() *routing.CRPRoutingEngine {
	return e.crpRoutingEngine
}

func NewEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath string, logger *zap.Logger) (*Engine, error) {
	re, err := initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath,
		logger)
	if err != nil {
		return nil, err
	}
	return &Engine{
		crpRoutingEngine: re,
	}, nil
}

const keyValByteApproxSize = 9 + 4*5

func NewEngineDirect(graph *da.Graph, overlayGraph *da.OverlayGraph, m *metrics.Metric,
	logger *zap.Logger, cst routing.Customizer, cf routing.CostFunction, landmarkFile, timeFunctionFilePath string) (*Engine, error) {
	// customizable route planning in road networks section 7.2 (path retrieval)
	// puCache, _ := lru.New[routing.PUCacheKey, []da.Index](1 << 21) // 524288

	const maxCost = int64(1) << 27
	puCache, err := ristretto.NewCache(&ristretto.Config[[]byte, []da.Index]{
		NumCounters: (maxCost / keyValByteApproxSize) * 5, // number of keys to track frequency of .
		MaxCost:     maxCost,                              // maximum cost of cache .
		BufferItems: 64,                                   // number of keys per Get buffer.
	})
	if err != nil {
		return nil, errors.Wrapf(err, "NewEngineDirect: failed to create new ristretto cache with capacity: %v")
	}

	lm := landmark.NewLandmark()
	err = lm.PreprocessALT(4, m, graph, logger)
	if err != nil {
		panic(err)
	}

	re := routing.NewCRPRoutingEngine(graph, overlayGraph, m, logger, puCache, cf, landmarkFile)

	return &Engine{
		crpRoutingEngine: re,
	}, nil
}

func initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath string, logger *zap.Logger,
) (*routing.CRPRoutingEngine,
	error) {

	logger.Info("Starting query engine....")

	logger.Info("Reading graph....")
	graph, err := da.ReadGraph(graphFilePath)
	if err != nil {
		return nil, err
	}

	logger.Info("Reading overlay graph....")
	overlayGraph, err := da.ReadOverlayGraph(overlayGraphFilePath)
	if err != nil {
		return nil, err
	}

	logger.Info("Reading stalling tables & metrics...")

	cf, err := costfunction.ReadFromFile(timeFunctionFilePath)
	if err != nil {
		return nil, err
	}
	m, err := metrics.ReadFromFile(metricsFilePath, timeFunctionFilePath)
	if err != nil {
		return nil, err
	}

	// customizable route planning in road networks section 7.2 (path retrieval)

	const maxCost = int64(1) << 26
	const maxItems = (maxCost / keyValByteApproxSize)
	const numCounters = maxItems * 3
	puCache, err := ristretto.NewCache(&ristretto.Config[[]byte, []da.Index]{
		NumCounters: numCounters, // number of keys to track frequency of .
		MaxCost:     maxCost,     // maximum cost of cache .
		BufferItems: 64,          // number of keys per Get buffer.
	})
	if err != nil {
		return nil, errors.Wrapf(err, "initializeRoutingEngine: failed to create new ristretto cache with capacity: %v")
	}

	re := routing.NewCRPRoutingEngine(graph, overlayGraph, m, logger, puCache, cf, landmarkFile)

	return re, nil
}

func (e *Engine) InitBackgroundWorker(ctx context.Context) {
	e.crpRoutingEngine.InitBackgroundWorker(ctx)
}


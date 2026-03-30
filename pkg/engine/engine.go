package engine

import (
	"github.com/cockroachdb/errors"
	"github.com/dgraph-io/ristretto/v2"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
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

func NewEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile string, logger *zap.Logger) (*Engine, error) {
	initializeRoutingEngine, err := initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile,
		logger)
	if err != nil {
		return nil, err
	}
	return &Engine{
		crpRoutingEngine: initializeRoutingEngine,
	}, nil
}

const keyValByteApproxSize = 9 + 4*5

func NewEngineDirect(graph *da.Graph, overlayGraph *da.OverlayGraph, m *metrics.Metric,
	logger *zap.Logger, cst routing.Customizer, cf routing.CostFunction, landmarkFile string) (*Engine, error) {
	// customizable route planning in road networks section 7.2 (path retrieval)
	// puCache, _ := lru.New[routing.PUCacheKey, []da.Index](1 << 21) // 524288

	maxCost := int64(1) << 15
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

	re := routing.NewCRPRoutingEngine(graph, overlayGraph, m, logger, puCache, cst, cf, lm)

	return &Engine{
		crpRoutingEngine: re,
	}, nil
}

func initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile string, logger *zap.Logger,
) (*routing.CRPRoutingEngine,
	error) {

	logger.Info("Starting query engine of Customizable Route Planning...")

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
	cf := costfunction.NewTimeCostFunction()
	m, err := metrics.ReadFromFile(metricsFilePath, graph, cf)
	if err != nil {
		return nil, err
	}

	cst := customizer.NewCustomizer(graphFilePath, overlayGraphFilePath, metricsFilePath, logger)
	cst.SetGraph(graph)
	cst.SetOverlayGraph(overlayGraph)
	cst.SetOverlayWeight(m.GetWeights())

	// customizable route planning in road networks section 7.2 (path retrieval)

	maxCost := int64(1) << 27 // kalo ristretto ukurannya mb? 134.217728 MB
	// max items in cache ~ 1.5jt
	puCache, err := ristretto.NewCache(&ristretto.Config[[]byte, []da.Index]{
		NumCounters: (maxCost / keyValByteApproxSize) * 4, // number of keys to track frequency of .
		MaxCost:     maxCost,                              // maximum cost of cache .
		BufferItems: 64,                                   // number of keys per Get buffer.
	})
	if err != nil {
		return nil, errors.Wrapf(err, "initializeRoutingEngine: failed to create new ristretto cache with capacity: %v")
	}

	lm, err := landmark.ReadLandmark(landmarkFile)
	if err != nil {
		panic(err)
	}

	return routing.NewCRPRoutingEngine(graph, overlayGraph, m, logger, puCache, cst, cf, lm), nil
}

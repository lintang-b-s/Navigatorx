// Package engine contains the core routing engine logic.
package engine

import (
	"bufio"
	"context"
	"fmt"

	"github.com/dgraph-io/ristretto/v2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Engine[W util.RoutingNumber] struct {
	crpRoutingEngine *routing.CRPRoutingEngine[W]
}

func (e *Engine[W]) GetRoutingEngine() *routing.CRPRoutingEngine[W] {
	return e.crpRoutingEngine
}

func NewEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath string, logger *zap.Logger) (*Engine[int32], error) {
	util.USE_INT32 = true
	re, err := initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath,
		logger)
	if err != nil {
		return nil, fmt.Errorf("NewEngine: failed to initialize routing engine: %w", err)
	}
	return &Engine[int32]{
		crpRoutingEngine: re,
	}, nil
}

const keyValByteApproxSize = 8 + 4*5

func NewEngineDirect[W util.RoutingNumber](
	graph *da.Graph,
	overlayGraph *da.OverlayGraph,
	m *metrics.Metric[W],
	logger *zap.Logger,
	landmarkFile string,
) (*Engine[W], error) {
	// customizable route planning in road networks section 7.2 (path retrieval)
	var zero W
	switch any(zero).(type) {
	case float64:
		util.USE_INT32 = false
	default:
		util.USE_INT32 = true
	}
	const maxCost = int64(1) << 20
	puCache, err := ristretto.NewCache(&ristretto.Config[[]byte, []da.Index]{
		NumCounters: (maxCost / keyValByteApproxSize) * 5, // number of keys to track frequency of .
		MaxCost:     maxCost,                              // maximum cost of cache .
		BufferItems: 64,                                   // number of keys per Get buffer.
	})
	if err != nil {
		return nil, fmt.Errorf("NewEngineDirect: failed to create new ristretto cache with capacity: %v: %w", maxCost, err)
	}

	lm := landmark.NewLandmark[W]()
	err = lm.PreprocessALT(4, m, graph, logger)
	if err != nil {
		panic(err)
	}

	readBuf := bufio.NewReaderSize(nil, util.BUFIO_SIZE)
	re := routing.NewCRPRoutingEngine(
		graph, overlayGraph, m, logger, puCache, landmarkFile, readBuf,
	)

	return &Engine[W]{
		crpRoutingEngine: re,
	}, nil
}

func initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath string, logger *zap.Logger,
) (*routing.CRPRoutingEngine[int32],
	error) {

	logger.Info("Starting query engine....")
	readBuf := bufio.NewReaderSize(nil, util.BUFIO_SIZE)

	logger.Info("Reading graph....")
	graph, err := da.ReadGraph(graphFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read graph from %s: %w", graphFilePath, err)
	}

	logger.Info("Reading overlay graph....")
	overlayGraph, err := da.ReadOverlayGraph(overlayGraphFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read overlay graph from %s: %w", overlayGraphFilePath, err)
	}

	logger.Info("Reading stalling tables & metrics...")

	m, err := metrics.ReadFromFile[int32](metricsFilePath, timeFunctionFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read metrics from %s (timeFunction=%s): %w", metricsFilePath, timeFunctionFilePath, err)
	}

	// customizable route planning in road networks section 7.2 (path retrieval)

	const maxCost = int64(1) << 28
	const maxItems = (maxCost / keyValByteApproxSize)
	const numCounters = maxItems * 3
	puCache, err := ristretto.NewCache(&ristretto.Config[[]byte, []da.Index]{
		NumCounters: numCounters, // number of keys to track frequency of .
		MaxCost:     maxCost,     // maximum cost of cache .
		BufferItems: 64,          // number of keys per Get buffer.
	})
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to create new ristretto cache with capacity: %v: %w", maxCost, err)
	}

	re := routing.NewCRPRoutingEngine(
		graph, overlayGraph, m, logger, puCache, landmarkFile, readBuf,
	)

	return re, nil
}

func (e *Engine[W]) InitBackgroundWorker(ctx context.Context) {
	e.crpRoutingEngine.InitBackgroundWorker(ctx)
}

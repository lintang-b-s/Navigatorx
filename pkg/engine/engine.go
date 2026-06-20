// Package engine contains the core routing engine logic.
package engine

import (
	"bufio"
	"context"
	"fmt"

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

func NewEngine[W util.RoutingNumber](graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath string, logger *zap.Logger) (*Engine[W], error) {
	util.ActivateMode[W]()
	re, err := initializeRoutingEngine[W](graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath,
		logger)
	if err != nil {
		return nil, fmt.Errorf("NewEngine: failed to initialize routing engine: %w", err)
	}
	return &Engine[W]{
		crpRoutingEngine: re,
	}, nil
}

// NewEngineDirect yang ini gak perlu read dari file, karena pakai CustomizeDirect
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
	puCache := da.NewPuCache()

	lm := landmark.NewLandmark[W]()
	err := lm.PreprocessALT(4, m, graph, logger)
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

func initializeRoutingEngine[W util.RoutingNumber](graphFilePath, overlayGraphFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath string, logger *zap.Logger,
) (*routing.CRPRoutingEngine[W],
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

	m, err := metrics.ReadFromFile[W](metricsFilePath, timeFunctionFilePath, readBuf)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read metrics from %s (timeFunction=%s): %w", metricsFilePath, timeFunctionFilePath, err)
	}

	// customizable route planning in road networks section 7.2 (path retrieval)

	puCache := da.NewPuCache()

	re := routing.NewCRPRoutingEngine(
		graph, overlayGraph, m, logger, puCache, landmarkFile, readBuf,
	)

	return re, nil
}

func (e *Engine[W]) InitBackgroundWorker(ctx context.Context) {
	e.crpRoutingEngine.InitBackgroundWorker(ctx)
}

// Package engine contains the core routing engine logic.
package engine

import (
	"context"
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Engine[W util.RoutingNumber] struct {
	re *routing.CRPRoutingEngine[W]
}

func (e *Engine[W]) GetRoutingEngine() *routing.CRPRoutingEngine[W] {
	return e.re
}

func getEngineFilePath(fileType pkg.FILE_TYPE) (
	graph, overlayGraph, landmark, metrics, tf, roadNetwork string,
) {
	root := config.ProfilesRoot()
	base := fmt.Sprintf("%s/%s/%s", root, pkg.ProfileName, pkg.RegionName)
	return base + ".ngraph",
		base + "_overlay_graph.ngraph",
		base + "_landmark.nlm",
		base + "_metrics.nmt",
		base + ".ntf",
		base + "_road_network.ndata"
}

func NewEngine[W util.RoutingNumber](logger *zap.Logger, fileType pkg.FILE_TYPE) (*Engine[W], error) {
	util.ActivateMode[W]()
	gf, ogf, lmf, metf, tff, rnf := getEngineFilePath(fileType)
	re, err := initializeRoutingEngine[W](gf, ogf, rnf, metf, lmf, tff,
		logger)
	if err != nil {
		return nil, fmt.Errorf("NewEngine: failed to initialize routing engine: %w", err)
	}
	return &Engine[W]{
		re: re,
	}, nil
}

// NewEngineDirect yang ini gak perlu read dari file, karena pakai CustomizeDirect

func initializeRoutingEngine[W util.RoutingNumber](graphFilePath, overlayGraphFilePath, rndContainerFilePath, metricsFilePath, landmarkFile, timeFunctionFilePath string, logger *zap.Logger,
) (*routing.CRPRoutingEngine[W],
	error) {

	logger.Info("Starting query engine....")

	logger.Info("Reading graph....")
	graph, err := da.ReadGraph(graphFilePath)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read graph from %s: %w", graphFilePath, err)
	}

	logger.Info("Reading overlay graph....")
	overlayGraph, err := da.ReadOverlayGraph(overlayGraphFilePath)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read overlay graph from %s: %w", overlayGraphFilePath, err)
	}

	logger.Info("Reading stalling tables & metrics...")

	m, err := metrics.ReadFromFile[W](metricsFilePath, timeFunctionFilePath)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read metrics from %s (timeFunction=%s): %w", metricsFilePath, timeFunctionFilePath, err)
	}

	rn, err := da.ReadRoadNetworkDataContainer(rndContainerFilePath)
	if err != nil {
		return nil, fmt.Errorf("initializeRoutingEngine: failed to read road network container from %s: %w", rndContainerFilePath, err)
	}
	// customizable route planning in road networks section 7.2 (path retrieval)

	puCache := da.NewPuCache()

	re := routing.NewCRPRoutingEngine(
		graph, overlayGraph, m, logger, puCache, landmarkFile, rn,
	)

	return re, nil
}

func (e *Engine[W]) InitBackgroundWorker(ctx context.Context) {
	e.re.InitBackgroundWorker(ctx)
}

func NewEngineDirect[W util.RoutingNumber](
	graph *da.Graph,
	rn *da.RoadNetworkDataContainer,
	overlayGraph *da.OverlayGraph,
	m *metrics.Metric[W],
	logger *zap.Logger,
	landmarkFile string,
) (*Engine[W], error) {
	util.ActivateMode[W]()
	puCache := da.NewPuCache()

	re := routing.NewCRPRoutingEngine(
		graph, overlayGraph, m, logger, puCache, landmarkFile, rn,
	)

	return &Engine[W]{
		re: re,
	}, nil
}

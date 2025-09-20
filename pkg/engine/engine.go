package engine

import (
	"log"

	"github.com/lintang-b-s/navigatorx-crp/pkg/costfunction"
	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx-crp/pkg/engine/routing"
	"github.com/lintang-b-s/navigatorx-crp/pkg/metrics"
)

type Engine struct {
	crpRoutingEngine *routing.CRPRoutingEngine
}

func (e *Engine) GetRoutingEngine() *routing.CRPRoutingEngine {
	return e.crpRoutingEngine
}

func NewEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string) (*Engine, error) {
	initializeRoutingEngine, err := initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath)
	if err != nil {
		return nil, err
	}
	return &Engine{
		crpRoutingEngine: initializeRoutingEngine,
	}, nil
}

func initializeRoutingEngine(graphFilePath, overlayGraphFilePath, metricsFilePath string) (*routing.CRPRoutingEngine,
	error) {
	log.Printf("Starting query step of Customizable Route Planning...")

	log.Printf("Reading graph from %s", graphFilePath)
	graph, err := datastructure.ReadGraph(graphFilePath)
	if err != nil {
		return nil, err
	}

	log.Printf("Reading overlay graph from %s", overlayGraphFilePath)
	overlayGraph, err := datastructure.ReadOverlayGraph(overlayGraphFilePath)
	if err != nil {
		return nil, err
	}
	costFunction := costfunction.NewTimeCostFunction()
	log.Printf("Building stalling tables...")
	metrics, err := metrics.ReadFromFile(metricsFilePath, costFunction)
	if err != nil {
		return nil, err
	}

	return routing.NewCRPRoutingEngine(graph, overlayGraph, metrics), nil
}

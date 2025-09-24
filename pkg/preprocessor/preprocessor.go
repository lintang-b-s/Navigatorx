package preprocesser

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"go.uber.org/zap"
)

type Preprocessor struct {
	graph  *datastructure.Graph
	mlp    *datastructure.MultilevelPartition
	logger *zap.Logger
}

func NewPreprocessor(graph *datastructure.Graph, mlp *datastructure.MultilevelPartition,
	logger *zap.Logger) *Preprocessor {
	return &Preprocessor{
		graph:  graph,
		mlp:    mlp,
		logger: logger,
	}
}

func (p *Preprocessor) PreProcessing() error {
	p.logger.Sugar().Infof("Starting preprocessing step of Customizable Route Planning...")

	p.logger.Sugar().Infof("Building Overlay Graph of each levels...")
	p.BuildCellNumber()
	p.graph.SortByCellNumber()
	p.logger.Sugar().Infof("After setting out/in edge cell offset")
	p.graph.ForOutEdgesOf(0, 0, func(e *datastructure.OutEdge, exitPoint datastructure.Index, turn pkg.TurnType) {
		vId := e.GetHead()
		p.logger.Sugar().Infof("outEdge from 0 to %v with turn type %v\n", vId, turn)
	})

	overlayGraph := datastructure.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.graph")
	err := overlayGraph.WriteToFile("./data/overlay_graph.graph")
	if err != nil {
		return err
	}

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.graph.RunKosaraju()

	p.logger.Sugar().Infof("Writing graph to ./data/original.graph")

	return p.graph.WriteGraph("./data/original.graph")
}

func (p *Preprocessor) BuildCellNumber() {
	cellNumbers := make([]datastructure.Pv, 0, p.mlp.GetNumberOfCellsInLevel(0))
	pvMap := make(map[datastructure.Pv]datastructure.Index, p.mlp.GetNumberOfCellsInLevel(0))

	for _, v := range p.graph.GetVertices() {
		cellNumber := p.mlp.GetCellNumber(v.GetID())
		if _, exists := pvMap[cellNumber]; !exists {
			cellNumbers = append(cellNumbers, cellNumber)
			cellPvPtr := len(cellNumbers) - 1
			pvMap[cellNumber] = datastructure.Index(cellPvPtr)
			v.SetPvPtr(datastructure.Index(cellPvPtr)) // set pointer to the index in cellNumbers slice
		} else {
			v.SetPvPtr(pvMap[cellNumber])
		}
	}

	// cellNumbers contains all unique bitpacked cell numbers from level 0->L.
	p.graph.SetCellNumbers(cellNumbers)
}

func (p *Preprocessor) PreProcessingNotSorted() error {
	p.logger.Sugar().Infof("Starting preprocessing step of Customizable Route Planning...")

	p.logger.Sugar().Infof("Building Overlay Graph of each levels...")
	p.BuildCellNumber()
	p.graph.SetOutInEdgeCellOffset()
	p.logger.Sugar().Infof("After setting out/in edge cell offset")
	p.graph.ForOutEdgesOf(0, 0, func(e *datastructure.OutEdge, exitPoint datastructure.Index, turn pkg.TurnType) {
		vId := e.GetHead()
		p.logger.Sugar().Infof("outEdge from 0 to %v with turn type %v\n", vId, turn)
	})

	overlayGraph := datastructure.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.graph")
	err := overlayGraph.WriteToFile("./data/overlay_graph.graph")
	if err != nil {
		return err
	}

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.graph.RunKosaraju()

	p.logger.Sugar().Infof("Writing graph to ./data/original.graph")

	return p.graph.WriteGraph("./data/original.graph")
}

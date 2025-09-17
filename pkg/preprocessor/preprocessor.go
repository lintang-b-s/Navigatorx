package preprocesser

import "github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"

type Preprocessor struct {
	graph *datastructure.Graph
	mlp   *datastructure.MultilevelPartition
}

func NewPreprocessor(graph *datastructure.Graph, mlp *datastructure.MultilevelPartition) *Preprocessor {
	return &Preprocessor{
		graph: graph,
		mlp:   mlp,
	}
}

func (p *Preprocessor) PreProcessing() error {
	p.BuildCellNumber()
	p.graph.SortVerticesByCellNumber()
	overlayGraph := datastructure.NewOverlayGraph(p.graph, p.mlp)
	err := overlayGraph.WriteToFile("./data/overlay_graph.graph")
	if err != nil {
		return err 
	}
	return p.graph.WriteGraph("./data/solo_jogja.graph")
}

func (p *Preprocessor) BuildCellNumber() {
	cellNumbers := make([]datastructure.Pv, 0, p.mlp.GetNumberOfCellsInLevel(0))
	pvMap := make(map[datastructure.Pv]datastructure.Index, p.mlp.GetNumberOfCellsInLevel(0))

	for _, v := range p.graph.GetVertices() {
		cellNumber := p.mlp.GetCellNumber(v.GetID())
		if _, exists := pvMap[cellNumber]; !exists {
			cellNumbers = append(cellNumbers, cellNumber)
			pvMap[cellNumber] = datastructure.Index(len(cellNumbers) - 1)
			v.SetPvPtr(pvMap[cellNumber]) // set pointer to the index in cellNumbers slice
		} else {
			v.SetPvPtr(pvMap[cellNumber])
		}
	}

	// cellNumbers contains all unique bitpacked cell numbers from level 0->L.
	p.graph.SetCellNumbers(cellNumbers)
}

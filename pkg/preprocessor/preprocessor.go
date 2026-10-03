// Package preprocessor handles the preprocessing phase of the Customizable Route Planning (CRP) by delling et al. (2015).
package preprocessor

import (
	"fmt"
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/config"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Preprocessor[W util.RoutingNumber] struct {
	graph                                                                              *da.Graph
	rn                                                                                 *da.RoadNetworkDataContainer
	mlp                                                                                *da.MultilevelPartition
	overlayGraph                                                                       *da.OverlayGraph
	logger                                                                             *zap.Logger
	oToNewVId                                                                          []da.Index
	nToOldVId                                                                          map[da.Index]da.Index
	timeFunction                                                                       *met.TimeFunction[W]
	graphFilename, overlayGraphFilename, prepCostFunctionFilename, rnContainerFilename string
	writeTiles                                                                         bool
}

func getPrepFilePath(fileType pkg.FILE_TYPE) (graph, overlayGraph, roadNetwork string) {
	root := config.ProfilesRoot()
	base := fmt.Sprintf("%s/%s/%s", root, pkg.ProfileName, pkg.RegionName)
	return base + ".ngraph",
		base + "_overlay_graph.ngraph",
		base + "_road_network.ndata"
}

func NewPreprocessor[W util.RoutingNumber](graph *da.Graph, rn *da.RoadNetworkDataContainer, timeFunction *met.TimeFunction[W], mlp *da.MultilevelPartition,
	logger *zap.Logger,
) *Preprocessor[W] {
	gf, ogf, rnf := getPrepFilePath(pkg.TIPE)
	return &Preprocessor[W]{
		graph:                    graph,
		mlp:                      mlp,
		rn:                       rn,
		logger:                   logger,
		oToNewVId:                make([]da.Index, graph.NumberOfVertices()),
		nToOldVId:                make(map[da.Index]da.Index, graph.NumberOfVertices()),
		graphFilename:            gf,
		overlayGraphFilename:     ogf,
		prepCostFunctionFilename: met.PrepTimeFunctionPath(),
		timeFunction:             timeFunction,
		rnContainerFilename:      rnf,
		writeTiles:               true,
	}
}

func (p *Preprocessor[W]) SetWriteTiles(writeTiles bool) {
	p.writeTiles = writeTiles
}

// Preprocesssing. Preprocessing (building Overlay Graph) phase. see section 5.1 Metric Independent Preprocessing (Overlay Topology) :  https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
func (p *Preprocessor[W]) PreProcessing(writefile bool) error {
	p.logger.Sugar().Infof("Starting building overlay graph preprocessing step of Customizable Route Planning...")

	p.logger.Sugar().Infof("Assign each vertices cell numbers....")
	p.BuildCellNumber()
	p.logger.Sugar().Infof("Sort vertices by its level-1 cell....")
	if err := p.SortByCellNumber(); err != nil {
		return err
	}

	p.logger.Sugar().Infof("Building Overlay Graph of each levels....")
	p.overlayGraph = da.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.ngraph")
	for l := p.overlayGraph.GetLevelData().GetLevelCount(); l >= 1; l-- {
		p.logger.Sugar().Infof("overlay graph level %v: number of overlay vertices %v", l, p.overlayGraph.NumberOfVerticesInLevel(l))
	}

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.graph.RunKosaraju()
	p.logger.Sugar().Infof("Writing graph to ./data/original.ngraph")

	if p.graph.IsRoadNetworkGraph() {
		err := p.buildLookupTable()
		if err != nil {
			return err
		}
	}

	if writefile {
		err := p.overlayGraph.WriteToFile(p.overlayGraphFilename)
		if err != nil {
			return err
		}

		if err := p.graph.WriteGraph(p.graphFilename); err != nil {
			return err
		}

		if err := p.rn.WriteToFile(p.rnContainerFilename); err != nil {
			return err
		}

		return p.timeFunction.WriteToFile(p.prepCostFunctionFilename)
	}

	return nil
}

func (p *Preprocessor[W]) buildLookupTable() error {
	ebgvNum := p.graph.NumberOfVertices()
	segmentKVs := make([]*da.SegmentKV, 0, ebgvNum)
	segmentTurnKVs := make([]*da.TurnKV, 0, ebgvNum)

	p.graph.ForVertices(func(_ da.Vertex, u da.Index) {
		uOsmId, vOsmId := p.rn.GetTailHeadOsmNodeId(u)
		p.graph.ForOutEdgesOf(u, func(eId, v, entryPoint da.Index) {
			_, wOsmId := p.rn.GetTailHeadOsmNodeId(v)
			segmentTurnKVs = append(segmentTurnKVs, da.NewTurnKV(uint64(uOsmId), uint64(vOsmId), uint64(wOsmId), eId))
		})
		segmentKVs = append(segmentKVs, da.NewSegmentKV(uint64(uOsmId), uint64(vOsmId), u))
	})

	p.logger.Sugar().Infof("writing segments & turn lookup table...")
	segmentLookupTable := da.NewLookupTable[*da.SegmentKV](segmentKVs)
	turnLookupTable := da.NewLookupTable[*da.TurnKV](segmentTurnKVs)
	rf := config.ProfilesRoot()
	seglkFilename := fmt.Sprintf("%s/%s/%s_segment.nlk", rf, pkg.ProfileName, pkg.RegionName)
	turnlkFilename := fmt.Sprintf("%s/%s/%s_turn.nlk", rf, pkg.ProfileName, pkg.RegionName)
	err := segmentLookupTable.WriteToFile(seglkFilename)
	if err != nil {
		return fmt.Errorf("preprocessor.buildLookupTable: failed to write segmentkvs lookup table: %w", err)
	}
	err = turnLookupTable.WriteToFile(turnlkFilename)
	if err != nil {
		return fmt.Errorf("preprocessor.buildLookupTable: failed to write turnkvs lookup table: %w", err)
	}
	return nil
}

func (p *Preprocessor[W]) BuildCellNumber() {
	cellNumbers := make([]da.Pv, 0, p.mlp.GetNumberOfCellsInLevel(0))
	pvMap := make(map[da.Pv]da.Index, p.mlp.GetNumberOfCellsInLevel(0))
	p.graph.ForVertices(func(_ da.Vertex, id da.Index) {

		cellNumber := p.mlp.GetCellNumber(id)
		if _, exists := pvMap[cellNumber]; !exists {
			cellNumbers = append(cellNumbers, cellNumber)
			cellPvPtr := len(cellNumbers) - 1
			pvMap[cellNumber] = da.Index(cellPvPtr)
			p.graph.SetVertexPvPtr(id, da.Index(cellPvPtr)) // set pointer to the index in cellNumbers slice
		} else {
			p.graph.SetVertexPvPtr(id, pvMap[cellNumber])
		}
	})

	// cellNumbers contains all unique bitpacked cell numbers from level 0->L.
	p.graph.SetCellNumbers(cellNumbers)
}

/*
SortByCellNumber. group vertices s.t. vertices within the same cell are adjacent to each other
adapted from https://github.com/michaelwegner/CRP/blob/master/datastructures/Graph.cpp
*/
func (p *Preprocessor[W]) SortByCellNumber() error {
	cellVertices := make([][]struct {
		vertex        da.Vertex
		originalIndex da.Index
	}, p.graph.GetNumberOfCellsNumbers()) // slice of slice of vertices in each cell

	minLat, minLon := math.MaxFloat64, math.MaxFloat64
	maxLat, maxLon := math.Inf(-1), math.Inf(-1)

	numVerticesInCell := make([]da.Index, p.graph.GetNumberOfCellsNumbers()) // number of outEdges in each cell

	type oldEdge struct {
		id da.Index
		v  da.Index // head if outgoing Edge. tail if incoming edge.
	}

	oEdges := make([][]oldEdge, p.graph.NumberOfVertices()) //
	iEdges := make([][]oldEdge, p.graph.NumberOfVertices())

	p.graph.SetMaxVerticesInCell(da.Index(0)) // maximum number of edges in any cell

	for i := da.Index(0); i < da.Index(p.graph.NumberOfVertices()); i++ {
		cell := p.graph.GetVertexPvPtr(i) // cellNumber

		vertex := p.graph.GetVertex(i)
		cellVertices[cell] = append(cellVertices[cell], struct {
			vertex        da.Vertex
			originalIndex da.Index
		}{vertex: vertex, originalIndex: i})

		oEdges[i] = make([]oldEdge, p.graph.GetOutDegree(i))
		iEdges[i] = make([]oldEdge, p.graph.GetInDegree(i))

		k := da.Index(0)
		eOut := p.graph.GetVertexFirstOut(i)
		for eOut < p.graph.GetVertexFirstOut(i+1) {
			head := p.graph.GetHead(eOut)

			oEdges[i][k] = oldEdge{id: eOut, v: head}
			eOut++
			k++
		}

		k = da.Index(0)
		eIn := p.graph.GetVertexFirstIn(i)
		for eIn < p.graph.GetVertexFirstIn(i+1) {
			tail := p.graph.GetTail(eIn)

			iEdges[i][k] = oldEdge{id: eIn, v: tail}
			eIn++
			k++
		}

		numVerticesInCell[cell] += 1

		vCoord := p.graph.GetVertexCoordinate(i)
		minLat = min(minLat, vCoord.GetLat())
		minLon = min(minLon, vCoord.GetLon())
		maxLat = max(maxLat, vCoord.GetLat())
		maxLon = max(maxLon, vCoord.GetLon())
	}

	for _, nv := range numVerticesInCell {
		if nv > p.graph.GetMaxVerticesInCell() {
			p.graph.SetMaxVerticesInCell(nv)
		}
	}

	p.rn.SetBoundingBox(da.NewBoundingBox(minLat, minLon, maxLat, maxLon))

	p.oToNewVId = make([]da.Index, p.graph.NumberOfVertices()+1) // new vertex id after sorting by cell number
	newVid := da.Index(0)                                        // new vertex id after sorting by cell number
	for i := 0; i < len(cellVertices); i++ {
		for v := 0; v < len(cellVertices[i]); v++ {
			p.oToNewVId[cellVertices[i][v].originalIndex] = newVid
			p.nToOldVId[newVid] = cellVertices[i][v].originalIndex
			newVid++
		}
	}

	noeId := da.Index(0)                                             // new id for outEdges for each vertex for each cell
	p.graph.MakeOutEdgeCellOffset(p.graph.GetNumberOfCellsNumbers()) // offset of first outEdge for each cell
	nieId := da.Index(0)                                             // new id for inEdges for each vertex for each cell
	p.graph.MakeInEdgeCellOffset(p.graph.GetNumberOfCellsNumbers())  // offset of first inEdge for each cell

	vId := da.Index(0)

	ePerm := make([]int, p.graph.NumberOfEdges()) // permutation that maps new edge id to old edge id
	eRevPerm := make([]int, p.graph.NumberOfEdges())
	nPerm := make([]int, p.graph.NumberOfVertices()+1)
	nPerm[len(nPerm)-1] = len(nPerm) - 1

	for i := da.Index(0); i < da.Index(p.graph.GetNumberOfCellsNumbers()); i++ {
		p.graph.SetHeadCellOffset(i, noeId)
		p.graph.SetTailCellOffset(i, nieId)

		for v := da.Index(0); v < da.Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number

			vOldId := cellVertices[i][v].originalIndex
			nPerm[vId] = int(vOldId)

			p.graph.SetFirstOut(vOldId, noeId)
			p.graph.SetFirstIn(vOldId, nieId)
			p.graph.SetVId(vOldId, vId)

			// update outedges & inedges
			for k := da.Index(0); k < da.Index(len(oEdges[vOldId])); k++ {

				oe := oEdges[vOldId][k]
				nHead := p.oToNewVId[oe.v]
				p.graph.SetHead(noeId, nHead)
				ePerm[noeId] = int(oe.id)

				noeId++
			}

			for k := da.Index(0); k < da.Index(len(iEdges[vOldId])); k++ {
				oie := iEdges[vOldId][k]
				nTail := p.oToNewVId[oie.v]
				p.graph.SetTail(nieId, nTail)
				eRevPerm[nieId] = int(oie.id)
				nieId++
			}

			vId++
		}
	}

	isRn := p.graph.IsRoadNetworkGraph()
	p.graph.ApplyGraphPermutation(nPerm, ePerm, eRevPerm)
	if isRn {
		p.rn.ApplySegmentsPermutation(nPerm)
		p.timeFunction.ApplySegmentsPermutation(ePerm, nPerm, isRn)
	} else {
		p.timeFunction.ApplySegmentsPermutation(ePerm, ePerm, isRn)
	}

	return nil
}

func (p *Preprocessor[W]) GetOldToNewVId() []da.Index {
	return p.oToNewVId
}

func (p *Preprocessor[W]) GetNewToOldVId() map[da.Index]da.Index {
	return p.nToOldVId
}

func (p *Preprocessor[W]) GetOverlayGraph() *da.OverlayGraph {
	return p.overlayGraph
}

func (p *Preprocessor[W]) GetGraph() *da.Graph {
	return p.graph
}

func (p *Preprocessor[W]) GetTimeFunction() *met.TimeFunction[W] {
	return p.timeFunction
}

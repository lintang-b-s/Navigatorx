// Package preprocessor handles the preprocessing phase of the Customizable Route Planning (CRP) by delling et al. (2015).
package preprocessor

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/tiler"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Preprocessor[W util.RoutingNumber] struct {
	graph                                                                  *da.Graph
	mlp                                                                    *da.MultilevelPartition
	overlayGraph                                                           *da.OverlayGraph
	logger                                                                 *zap.Logger
	newVIdMap                                                              []da.Index
	newToOldVIdMap                                                         map[da.Index]da.Index
	edgeInfoIds                                                            [][]da.Index
	timeFunction                                                           *costfunction.TimeFunction[W]
	graphFilename, overlayGraphFilename, preprocessingTimeFunctionFilename string
	writeTiles                                                             bool
}

func NewPreprocessor[W util.RoutingNumber](graph *da.Graph, timeFunction *costfunction.TimeFunction[W], mlp *da.MultilevelPartition,
	logger *zap.Logger, gFilename string, ogFilename string, edgeInfoIds [][]da.Index,
) *Preprocessor[W] {
	return &Preprocessor[W]{
		graph:                             graph,
		mlp:                               mlp,
		logger:                            logger,
		newVIdMap:                         make([]da.Index, graph.NumberOfVertices()),
		newToOldVIdMap:                    make(map[da.Index]da.Index, graph.NumberOfVertices()),
		graphFilename:                     gFilename,
		overlayGraphFilename:              ogFilename,
		preprocessingTimeFunctionFilename: costfunction.PreprocessingTimeFunctionPath(gFilename),
		edgeInfoIds:                       edgeInfoIds,
		timeFunction:                      timeFunction,
		writeTiles:                        true,
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
	p.SortByCellNumber()

	p.logger.Sugar().Infof("Building Overlay Graph of each levels....")
	p.overlayGraph = da.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.ngraph")
	for l := p.overlayGraph.GetLevelInfo().GetLevelCount(); l >= 1; l-- {
		p.logger.Sugar().Infof("overlay graph level %v: number of overlay vertices %v", l, p.overlayGraph.NumberOfVerticesInLevel(l))
	}

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.graph.RunKosaraju()

	p.logger.Sugar().Infof("Writing graph to ./data/original.ngraph")

	if writefile {
		err := p.overlayGraph.WriteToFile(p.overlayGraphFilename)
		if err != nil {
			return err
		}

		// write graph tiles
		if p.writeTiles {
			tilingEngine := tiler.NewTilingEngine(p.graph, p.logger, p.timeFunction)
			if err := tilingEngine.PreprocessTiles(); err != nil {
				return err
			}
		}

		if err := p.graph.WriteGraph(p.graphFilename); err != nil {
			return err
		}
		return p.timeFunction.WritePreprocessingToFile(p.preprocessingTimeFunctionFilename)
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
group vertices s.t. vertices within the same cell are adjacent to each other
*/
func (p *Preprocessor[W]) SortByCellNumber() {
	cellVertices := make([][]struct {
		vertex        da.Vertex
		originalIndex da.Index
	}, p.graph.GetNumberOfCellsNumbers()) // slice of slice of vertices in each cell

	minLat, minLon := math.MaxFloat64, math.MaxFloat64
	maxLat, maxLon := math.Inf(-1), math.Inf(-1)

	numOutEdgesInCell := make([]da.Index, p.graph.GetNumberOfCellsNumbers()) // number of outEdges in each cell
	numInEdgesInCell := make([]da.Index, p.graph.GetNumberOfCellsNumbers())

	oEdges := make([][]da.OutEdge, p.graph.NumberOfVertices()) // copy of original outEdges of each vertex
	iEdges := make([][]da.InEdge, p.graph.NumberOfVertices())

	p.graph.SetMaxEdgesInCell(da.Index(0)) // maximum number of edges in any cell
	for i := da.Index(0); i < da.Index(p.graph.NumberOfVertices()); i++ {
		cell := p.graph.GetVertexPvPtr(i) // cellNumber

		vertex := p.graph.GetVertex(i)
		cellVertices[cell] = append(cellVertices[cell], struct {
			vertex        da.Vertex
			originalIndex da.Index
		}{vertex: vertex, originalIndex: i})

		oEdges[i] = make([]da.OutEdge, p.graph.GetOutDegree(i))
		iEdges[i] = make([]da.InEdge, p.graph.GetInDegree(i))

		k := da.Index(0)
		eOut := p.graph.GetVertexFirstOut(i)
		for eOut < p.graph.GetVertexFirstOut(i+1) {
			oEdge := p.graph.GetOutEdge(eOut)
			newOEdge := da.NewOutEdge(
				oEdge.GetEdgeId(),
				oEdge.GetHead(),
				oEdge.GetEntryPoint(),
				oEdge.GetHighwayType(),
			)
			newOEdge.SetFlag(oEdge.GetFlag())
			oEdges[i][k] = newOEdge
			eOut++
			k++
		}

		k = da.Index(0)
		eIn := p.graph.GetVertexFirstIn(i)
		for eIn < p.graph.GetVertexFirstIn(i+1) {
			inEdge := p.graph.GetInEdge(eIn)
			newInEdge := da.NewInEdge(
				inEdge.GetEdgeId(),
				inEdge.GetTail(),
				inEdge.GetExitPoint(),
				inEdge.GetHighwayType(),
			)
			newInEdge.SetFlag(inEdge.GetFlag())
			iEdges[i][k] = newInEdge
			eIn++
			k++
		}

		numOutEdgesInCell[cell] += p.graph.GetOutDegree(i)
		numInEdgesInCell[cell] += p.graph.GetInDegree(i)

		if p.graph.GetMaxEdgesInCell() < numOutEdgesInCell[cell] {
			p.graph.SetMaxEdgesInCell(numOutEdgesInCell[cell])
		}

		if p.graph.GetMaxEdgesInCell() < numInEdgesInCell[cell] {
			p.graph.SetMaxEdgesInCell(numInEdgesInCell[cell])
		}

		vCoord := p.graph.GetVertexCoordinate(i)
		minLat = min(minLat, vCoord.GetLat())
		minLon = min(minLon, vCoord.GetLon())
		maxLat = max(maxLat, vCoord.GetLat())
		maxLon = max(maxLon, vCoord.GetLon())
	}

	p.graph.SetBoundingBox(da.NewBoundingBox(minLat, minLon, maxLat, maxLon))

	p.newVIdMap = make([]da.Index, p.graph.NumberOfVertices()) // new vertex id after sorting by cell number
	newVid := da.Index(0)                                      // new vertex id after sorting by cell number
	for i := 0; i < len(cellVertices); i++ {
		for v := 0; v < len(cellVertices[i]); v++ {
			p.newVIdMap[cellVertices[i][v].originalIndex] = newVid
			p.newToOldVIdMap[newVid] = cellVertices[i][v].originalIndex
			newVid++
		}
	}

	newOutEdgeId := da.Index(0)                                      // new id for outEdges for each vertex for each cell
	p.graph.MakeOutEdgeCellOffset(p.graph.GetNumberOfCellsNumbers()) // offset of first outEdge for each cell
	newInEdgeId := da.Index(0)                                       // new id for inEdges for each vertex for each cell
	p.graph.MakeInEdgeCellOffset(p.graph.GetNumberOfCellsNumbers())  // offset of first inEdge for each cell

	vId := da.Index(0)

	edgeIdsPerm := make([]int, p.graph.NumberOfOutEdges())
	edgeMetaIdsPerm := make([]int, p.graph.NumberOfOutEdges())
	vertexIdsPerm := make([]int, p.graph.GetNumberOfVerticesWithDummyVertex())
	vertexIdsPerm[len(vertexIdsPerm)-1] = len(vertexIdsPerm) - 1

	for i := da.Index(0); i < da.Index(p.graph.GetNumberOfCellsNumbers()); i++ {
		p.graph.SetOutEdgeCellOffset(i, newOutEdgeId)
		p.graph.SetInEdgeCellOffset(i, newInEdgeId)

		for v := da.Index(0); v < da.Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number

			vOldId := cellVertices[i][v].originalIndex
			vertexIdsPerm[vId] = int(vOldId)

			p.graph.SetFirstOut(vOldId, newOutEdgeId)
			p.graph.SetFirstIn(vOldId, newInEdgeId)
			p.graph.SetVId(vOldId, vId)

			// update outedges & inedges
			for k := da.Index(0); k < da.Index(len(oEdges[vOldId])); k++ {

				oldOutEdge := oEdges[vOldId][k]
				newOutEdgeHead := p.newVIdMap[oldOutEdge.GetHead()]
				newOutEdge := da.NewOutEdge(
					newOutEdgeId, newOutEdgeHead, oldOutEdge.GetEntryPoint(), oldOutEdge.GetHighwayType(),
				)
				newOutEdge.SetFlag(oldOutEdge.GetFlag())
				p.graph.SetOutEdge(newOutEdgeId, newOutEdge)

				vExitPoint := oldOutEdge.GetEdgeId() - cellVertices[i][v].vertex.GetFirstOut()
				oldEdgeMetaId := p.edgeInfoIds[vOldId][vExitPoint]
				edgeMetaIdsPerm[newOutEdgeId] = int(oldEdgeMetaId)

				edgeIdsPerm[newOutEdgeId] = int(oldOutEdge.GetEdgeId())
				newOutEdgeId++
			}

			for k := da.Index(0); k < da.Index(len(iEdges[vOldId])); k++ {
				oldInEdge := iEdges[vOldId][k]
				newInEdgeTail := p.newVIdMap[oldInEdge.GetTail()]
				newInEdge := da.NewInEdge(
					newInEdgeId, newInEdgeTail, oldInEdge.GetExitPoint(),
					oldInEdge.GetHighwayType(),
				)
				newInEdge.SetFlag(oldInEdge.GetFlag())
				p.graph.SetInEdge(newInEdgeId, newInEdge)
				newInEdgeId++
			}

			vId++
		}
	}

	p.graph.ApplyVerticesPermutation(vertexIdsPerm)
	p.graph.ApplyEdgesMetadataPermutation(edgeMetaIdsPerm, edgeIdsPerm)
	p.timeFunction.ApplyEdgesPermutation(edgeIdsPerm)
}

func (p *Preprocessor[W]) GetOldToNewVIdMap() []da.Index {
	return p.newVIdMap
}

func (p *Preprocessor[W]) GetNewToOldVIdMap() map[da.Index]da.Index {
	return p.newToOldVIdMap
}

func (p *Preprocessor[W]) GetOverlayGraph() *da.OverlayGraph {
	return p.overlayGraph
}

func (p *Preprocessor[W]) GetGraph() *da.Graph {
	return p.graph
}

func (p *Preprocessor[W]) GetTimeFunction() *costfunction.TimeFunction[W] {
	return p.timeFunction
}

package preprocesser

import (
	"math"

	"github.com/bits-and-blooms/bitset"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Preprocessor struct {
	graph                               *da.Graph
	mlp                                 *da.MultilevelPartition
	overlayGraph                        *da.OverlayGraph
	logger                              *zap.Logger
	newVIdMap                           []da.Index
	newToOldVIdMap                      map[da.Index]da.Index
	edgeInfoIds                         [][]da.Index
	graphFilename, overlayGraphFilename string
}

func NewPreprocessor(graph *da.Graph, mlp *da.MultilevelPartition,
	logger *zap.Logger, gFilename string, ogFilename string, edgeInfoIds [][]da.Index,
) *Preprocessor {
	return &Preprocessor{
		graph:                graph,
		mlp:                  mlp,
		logger:               logger,
		newVIdMap:            make([]da.Index, graph.NumberOfVertices()),
		newToOldVIdMap:       make(map[da.Index]da.Index, graph.NumberOfVertices()),
		graphFilename:        gFilename,
		overlayGraphFilename: ogFilename,
		edgeInfoIds:          edgeInfoIds,
	}
}

func (p *Preprocessor) PreProcessing(writefile bool) error {
	p.logger.Sugar().Infof("Starting preprocessing step of Customizable Route Planning...")

	p.logger.Sugar().Infof("Building Overlay Graph of each levels...")
	p.BuildCellNumber()
	p.SortByCellNumber()

	p.overlayGraph = da.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.graph")
	for l := p.overlayGraph.GetLevelInfo().GetLevelCount(); l >= 1; l-- {
		p.logger.Sugar().Infof("overlay graph level %v: number of overlay vertices %v", l, p.overlayGraph.NumberOfVerticesInLevel(l))
	}

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.graph.RunKosaraju()

	p.logger.Sugar().Infof("Writing graph to ./data/original.graph")

	if writefile {
		err := p.overlayGraph.WriteToFile(p.overlayGraphFilename)
		if err != nil {
			return err
		}

		return p.graph.WriteGraph(p.graphFilename)
	}
	return nil
}

func (p *Preprocessor) BuildCellNumber() {
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
func (p *Preprocessor) SortByCellNumber() {
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
		v := p.graph.GetVertex(i)

		cellVertices[cell] = append(cellVertices[cell], struct {
			vertex        da.Vertex
			originalIndex da.Index
		}{vertex: v, originalIndex: i})

		oEdges[i] = make([]da.OutEdge, p.graph.GetOutDegree(i))
		iEdges[i] = make([]da.InEdge, p.graph.GetInDegree(i))

		k := da.Index(0)
		e := p.graph.GetVertexFirstOut(i)
		for e < p.graph.GetVertexFirstOut(i+1) {
			oEdge := p.graph.GetOutEdge(e)
			newOEdge := da.NewOutEdge(
				oEdge.GetEdgeId(),
				oEdge.GetHead(),
				oEdge.GetWeight(),
				oEdge.GetLength(),
				oEdge.GetEntryPoint(),
				oEdge.GetHighwayType(),
			)

			oEdges[i][k] = newOEdge
			e++
			k++
		}

		k = da.Index(0)
		e = p.graph.GetVertexFirstIn(i)
		for e < p.graph.GetVertexFirstIn(i+1) {
			inEdge := p.graph.GetInEdge(e)
			newInEdge := da.NewInEdge(
				inEdge.GetEdgeId(),
				inEdge.GetTail(),
				inEdge.GetWeight(),
				inEdge.GetLength(),
				inEdge.GetExitPoint(),
				inEdge.GetHighwayType(),
			)
			iEdges[i][k] = newInEdge
			e++
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

		minLat = util.MinFloat(minLat, p.graph.GetVertexCoordinate(i).GetLat())
		minLon = util.MinFloat(minLon, p.graph.GetVertexCoordinate(i).GetLon())
		maxLat = util.MaxFloat(maxLat, p.graph.GetVertexCoordinate(i).GetLat())
		maxLon = util.MaxFloat(maxLon, p.graph.GetVertexCoordinate(i).GetLon())
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

	// create new edge infos
	oldEdgeInfos := p.graph.GetEdgeInfos()
	newEdgeInfos := make([]da.EdgeExtraInfo, len(oldEdgeInfos))

	// create new roundabout flag
	oldRoundaboutFlag := p.graph.GetRoundaboutFlag()
	newRoundaboutFlags := bitset.New(oldRoundaboutFlag.Len())

	// create new traffic light flag
	oldGraphTrafficLight := p.graph.GetNodeTrafficLight()
	newNodeTrafficLight := bitset.New(oldGraphTrafficLight.Len())

	// create new street direction flags
	newStreetDirectionForward := bitset.New(uint(p.graph.NumberOfEdges()))
	newStreetDirectionBackward := bitset.New(uint(p.graph.NumberOfEdges()))

	vId := da.Index(0)

	lastVertex := p.graph.GetVertex(da.Index(p.graph.GetNumberOfVerticesWithDummyVertex() - 1))
	newVertices := make([]da.Vertex, p.graph.GetNumberOfVerticesWithDummyVertex())
	for i := da.Index(0); i < da.Index(p.graph.GetNumberOfCellsNumbers()); i++ {
		p.graph.SetOutEdgeCellOffset(i, newOutEdgeId)
		p.graph.SetInEdgeCellOffset(i, newInEdgeId)

		for v := da.Index(0); v < da.Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number

			newVertices[vId] = cellVertices[i][v].vertex

			vOldId := cellVertices[i][v].originalIndex
			newVertices[vId].SetFirstOut(newOutEdgeId)
			newVertices[vId].SetFirstIn(newInEdgeId)
			newVertices[vId].SetId(vId)

			// update trafic light flag
			isTraficLight := oldGraphTrafficLight.Test(uint(vOldId))
			if isTraficLight {
				newNodeTrafficLight.Set(uint(vId))
			}

			// update outedges & inedges
			for k := da.Index(0); k < da.Index(len(oEdges[vOldId])); k++ {

				oldOutEdge := oEdges[vOldId][k]

				newOutEdgeHead := p.newVIdMap[oldOutEdge.GetHead()]

				newOutEdge := da.NewOutEdge(
					newOutEdgeId, newOutEdgeHead, oldOutEdge.GetWeight(),
					oldOutEdge.GetLength(), oldOutEdge.GetEntryPoint(), oldOutEdge.GetHighwayType(),
				)

				p.graph.SetOutEdge(newOutEdgeId, newOutEdge)

				// update edge metadata

				if oldOutEdge.GetHead() != vOldId { // skip dummy edge (vOldI, vOldId)
					vExitPoint := p.graph.GetExitOrder(vOldId, oldOutEdge.GetEdgeId())
					oldEdgeInfoId := p.edgeInfoIds[vOldId][vExitPoint]
					oldEdgeInfo := oldEdgeInfos[oldEdgeInfoId]

					newEdgeInfos[newOutEdgeId] = oldEdgeInfo

					// update roundabout flag
					isRoundabout := oldRoundaboutFlag.Test(uint(oldEdgeInfoId))
					if isRoundabout {
						newRoundaboutFlags.Set(uint(newOutEdgeId))
					}

					// update street direction flag
					streetdir := p.graph.GetStreetDirection(oldOutEdge.GetEdgeId())
					if streetdir[0] {
						newStreetDirectionForward.Set(uint(newOutEdgeId))
					}

					if streetdir[1] {
						newStreetDirectionBackward.Set(uint(newOutEdgeId))
					}
				}

				newOutEdgeId++
			}

			for k := da.Index(0); k < da.Index(len(iEdges[vOldId])); k++ {
				oldInEdge := iEdges[vOldId][k]

				newInEdgeTail := p.newVIdMap[oldInEdge.GetTail()]
				newInEdge := da.NewInEdge(
					newInEdgeId, newInEdgeTail, oldInEdge.GetWeight(),
					oldInEdge.GetLength(), oldInEdge.GetExitPoint(),
					oldInEdge.GetHighwayType(),
				)

				p.graph.SetInEdge(newInEdgeId, newInEdge)

				newInEdgeId++
			}

			vId++
		}
	}

	newVertices[len(newVertices)-1] = lastVertex
	p.graph.SetVertices(newVertices)
	p.graph.SetRoundaboutFlags(newRoundaboutFlags)
	p.graph.SetTrafficLightFlags(newNodeTrafficLight)
	p.graph.SetStreetDirection(newStreetDirectionForward, newStreetDirectionBackward)
	p.graph.SetEdgeInfos(newEdgeInfos)
}

func (p *Preprocessor) GetOldToNewVIdMap() []da.Index {
	return p.newVIdMap
}

func (p *Preprocessor) GetNewToOldVIdMap() map[da.Index]da.Index {
	return p.newToOldVIdMap
}

func (p *Preprocessor) GetOverlayGraph() *da.OverlayGraph {
	return p.overlayGraph
}

func (p *Preprocessor) GetGraph() *da.Graph {
	return p.graph
}

package preprocesser

import (
	"math"

	"github.com/bits-and-blooms/bitset"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type Preprocessor struct {
	graph                               *datastructure.Graph
	mlp                                 *datastructure.MultilevelPartition
	overlayGraph                        *datastructure.OverlayGraph
	logger                              *zap.Logger
	newVIdMap                           []datastructure.Index
	newToOldVIdMap                      map[datastructure.Index]datastructure.Index
	graphFilename, overlayGraphFilename string
}

func NewPreprocessor(graph *datastructure.Graph, mlp *datastructure.MultilevelPartition,
	logger *zap.Logger, gFilename string, ogFilename string) *Preprocessor {
	return &Preprocessor{
		graph:                graph,
		mlp:                  mlp,
		logger:               logger,
		newVIdMap:            make([]datastructure.Index, graph.NumberOfVertices()),
		newToOldVIdMap:       make(map[datastructure.Index]datastructure.Index, graph.NumberOfVertices()),
		graphFilename:        gFilename,
		overlayGraphFilename: ogFilename,
	}
}

func (p *Preprocessor) PreProcessing(writefile bool) error {
	p.logger.Sugar().Infof("Starting preprocessing step of Customizable Route Planning...")

	p.logger.Sugar().Infof("Building Overlay Graph of each levels...")
	p.BuildCellNumber()
	p.SortByCellNumber()

	p.overlayGraph = datastructure.NewOverlayGraph(p.graph, p.mlp)
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

/*
group vertices s.t. vertices within the same cell are adjacent to each other
*/
func (p *Preprocessor) SortByCellNumber() {
	cellVertices := make([][]struct {
		vertex        *datastructure.Vertex
		originalIndex datastructure.Index
	}, p.graph.GetNumberOfCellsNumbers()) // slice of slice of vertices in each cell

	minLat, minLon := math.MaxFloat64, math.MaxFloat64
	maxLat, maxLon := math.Inf(-1), math.Inf(-1)

	numOutEdgesInCell := make([]datastructure.Index, p.graph.GetNumberOfCellsNumbers()) // number of outEdges in each cell
	numInEdgesInCell := make([]datastructure.Index, p.graph.GetNumberOfCellsNumbers())

	oEdges := make([][]*datastructure.OutEdge, p.graph.NumberOfVertices()) // copy of original outEdges of each vertex
	iEdges := make([][]*datastructure.InEdge, p.graph.NumberOfVertices())

	p.graph.SetMaxEdgesInCell(datastructure.Index(0)) // maximum number of edges in any cell
	for i := datastructure.Index(0); i < datastructure.Index(p.graph.NumberOfVertices()); i++ {
		cell := p.graph.GetVertex(i).GetPvPtr() // cellNumber
		v := p.graph.GetVertex(i)

		cellVertices[cell] = append(cellVertices[cell], struct {
			vertex        *datastructure.Vertex
			originalIndex datastructure.Index
		}{vertex: v, originalIndex: i})

		oEdges[i] = make([]*datastructure.OutEdge, p.graph.GetOutDegree(i))
		iEdges[i] = make([]*datastructure.InEdge, p.graph.GetInDegree(i))

		k := datastructure.Index(0)
		e := p.graph.GetVertex(i).GetFirstOut()
		for e < p.graph.GetVertex(i+1).GetFirstOut() {
			oEdge := p.graph.GetOutEdge(e)
			newOEdge := datastructure.NewOutEdge(
				oEdge.GetEdgeId(),
				oEdge.GetHead(),
				oEdge.GetWeight(),
				oEdge.GetLength(),
				oEdge.GetEntryPoint(),
				oEdge.GetHighwayType(),
			)
			newOEdge.SetInfoEdgeId(oEdge.GetEdgeInfoId())
			newOEdge.SetSimplifiedLength(oEdge.GetSimplifiedLength())
			oEdges[i][k] = newOEdge
			e++
			k++
		}

		k = datastructure.Index(0)
		e = p.graph.GetVertex(i).GetFirstIn()
		for e < p.graph.GetVertex(i+1).GetFirstIn() {
			inEdge := p.graph.GetInEdge(e)
			newInEdge := datastructure.NewInEdge(
				inEdge.GetEdgeId(),
				inEdge.GetTail(),
				inEdge.GetWeight(),
				inEdge.GetLength(),
				inEdge.GetExitPoint(),
				inEdge.GetHighwayType(),
			)
			newInEdge.SetInfoEdgeId(inEdge.GetEdgeInfoId())
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

		minLat = math.Min(minLat, p.graph.GetVertex(i).GetLat())
		minLon = math.Min(minLon, p.graph.GetVertex(i).GetLon())
		maxLat = util.MaxFloat(maxLat, p.graph.GetVertex(i).GetLat())
		maxLon = util.MaxFloat(maxLon, p.graph.GetVertex(i).GetLon())
	}

	p.graph.SetBoundingBox(datastructure.NewBoundingBox(minLat, minLon, maxLat, maxLon))

	p.newVIdMap = make([]datastructure.Index, p.graph.NumberOfVertices()) // new vertex id after sorting by cell number
	newVid := datastructure.Index(0)                                      // new vertex id after sorting by cell number
	for i := 0; i < len(cellVertices); i++ {
		for v := 0; v < len(cellVertices[i]); v++ {
			p.newVIdMap[cellVertices[i][v].originalIndex] = newVid
			p.newToOldVIdMap[newVid] = cellVertices[i][v].originalIndex
			newVid++
		}
	}

	newOutEdgeId := datastructure.Index(0)                           // new id for outEdges for each vertex for each cell
	p.graph.MakeOutEdgeCellOffset(p.graph.GetNumberOfCellsNumbers()) // offset of first outEdge for each cell
	newInEdgeId := datastructure.Index(0)                            // new id for inEdges for each vertex for each cell
	p.graph.MakeInEdgeCellOffset(p.graph.GetNumberOfCellsNumbers())  // offset of first inEdge for each cell

	graphEdgeInfo := p.graph.GetEdgeInfos()
	gsEdgeExtraInfos := make([]datastructure.EdgeExtraInfo, len(graphEdgeInfo))
	copy(gsEdgeExtraInfos, graphEdgeInfo)

	// create new roundabout flag
	graphRoundaboutFlag := p.graph.GetRoundaboutFlag()
	roundaboutFlags := bitset.New(graphRoundaboutFlag.Len())

	// create new traffic light flag
	graphTrafficLight := p.graph.GetNodeTrafficLight()
	nodeTrafficLight := bitset.New(graphTrafficLight.Len())

	// create new street direction flags

	streetDirectionForward := bitset.New(uint(p.graph.NumberOfEdges()))
	streetDirectionBackward := bitset.New(uint(p.graph.NumberOfEdges()))

	vId := datastructure.Index(0)

	lastVertex := p.graph.GetVertex(datastructure.Index(p.graph.GetNumberOfVerticesWithDummyVertex() - 1))
	newVertices := make([]*datastructure.Vertex, p.graph.GetNumberOfVerticesWithDummyVertex())
	for i := datastructure.Index(0); i < datastructure.Index(p.graph.GetNumberOfCellsNumbers()); i++ {
		p.graph.SetOutEdgeCellOffset(i, newOutEdgeId)
		p.graph.SetInEdgeCellOffset(i, newInEdgeId)

		for v := datastructure.Index(0); v < datastructure.Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number

			newVertices[vId] = cellVertices[i][v].vertex

			vOldId := cellVertices[i][v].originalIndex
			newVertices[vId].SetFirstOut(newOutEdgeId)
			newVertices[vId].SetFirstIn(newInEdgeId)
			newVertices[vId].SetId(vId)

			// update trafic light flag
			isTraficLight := graphTrafficLight.Test(uint(vOldId))
			if isTraficLight {
				nodeTrafficLight.Set(uint(vId))
			}

			// update outedges & inedges
			for k := datastructure.Index(0); k < datastructure.Index(len(oEdges[vOldId])); k++ {

				oldOutEdge := oEdges[vOldId][k]
				newOutEdge := datastructure.NewOutEdge(
					newOutEdgeId, oldOutEdge.GetHead(), oldOutEdge.GetWeight(),
					oldOutEdge.GetLength(), oldOutEdge.GetEntryPoint(), oldOutEdge.GetHighwayType(),
				)
				newOutEdge.SetSimplifiedLength(oldOutEdge.GetSimplifiedLength())

				p.graph.SetOutEdge(newOutEdgeId, newOutEdge)
				p.graph.SetEdgeInfo(newOutEdgeId, gsEdgeExtraInfos[oldOutEdge.GetEdgeInfoId()]) // update edge extra info storage

				// update roundabout flag
				isRoundabout := graphRoundaboutFlag.Test(uint(oldOutEdge.GetEdgeInfoId()))
				if isRoundabout {
					roundaboutFlags.Set(uint(newOutEdgeId))
				}

				// update street direction flag
				streetdir := p.graph.GetStreetDirection(oldOutEdge.GetEdgeId())
				if streetdir[0] {
					streetDirectionForward.Set(uint(newOutEdgeId))
				}

				if streetdir[1] {
					streetDirectionBackward.Set(uint(newOutEdgeId))
				}

				outEdge := p.graph.GetOutEdge(newOutEdgeId)
				outEdge.SetEdgeId(newOutEdgeId)
				outEdge.SetHead(p.newVIdMap[oldOutEdge.GetHead()])

				newOutEdgeId++
			}

			for k := datastructure.Index(0); k < datastructure.Index(len(iEdges[vOldId])); k++ {
				oldInEdge := iEdges[vOldId][k]
				newInEdge := datastructure.NewInEdge(
					newInEdgeId, oldInEdge.GetTail(), oldInEdge.GetWeight(),
					oldInEdge.GetLength(), oldInEdge.GetExitPoint(),
					oldInEdge.GetHighwayType(),
				)

				p.graph.SetInEdge(newInEdgeId, newInEdge)

				inEdge := p.graph.GetInEdge(newInEdgeId)
				inEdge.SetEdgeId(newInEdgeId)
				inEdge.SetTailId(p.newVIdMap[oldInEdge.GetTail()])

				newInEdgeId++
			}

			vId++
		}
	}

	newVertices[len(newVertices)-1] = lastVertex
	p.graph.SetVertices(newVertices)
	p.graph.SetRoundaboutFlags(roundaboutFlags)
	p.graph.SetTrafficLightFlags(nodeTrafficLight)
	p.graph.SetStreetDirection(streetDirectionForward, streetDirectionBackward)
}

func (p *Preprocessor) GetOldToNewVIdMap() []datastructure.Index {
	return p.newVIdMap
}

func (p *Preprocessor) GetNewToOldVIdMap() map[datastructure.Index]datastructure.Index {
	return p.newToOldVIdMap
}

func (p *Preprocessor) GetOverlayGraph() *datastructure.OverlayGraph {
	return p.overlayGraph
}

func (p *Preprocessor) GetGraph() *datastructure.Graph {
	return p.graph
}

package preprocesser

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"go.uber.org/zap"
)

type Preprocessor struct {
	graph          *datastructure.Graph
	mlp            *datastructure.MultilevelPartition
	overlayGraph   *datastructure.OverlayGraph
	logger         *zap.Logger
	newVIdMap      []datastructure.Index
	newToOldVIdMap map[datastructure.Index]datastructure.Index
}

func NewPreprocessor(graph *datastructure.Graph, mlp *datastructure.MultilevelPartition,
	logger *zap.Logger) *Preprocessor {
	return &Preprocessor{
		graph:          graph,
		mlp:            mlp,
		logger:         logger,
		newVIdMap:      make([]datastructure.Index, graph.NumberOfVertices()),
		newToOldVIdMap: make(map[datastructure.Index]datastructure.Index, graph.NumberOfVertices()),
	}
}

func (p *Preprocessor) PreProcessing(writefile bool) error {
	p.logger.Sugar().Infof("Starting preprocessing step of Customizable Route Planning...")

	p.logger.Sugar().Infof("Building Overlay Graph of each levels...")
	p.BuildCellNumber()
	p.SortByCellNumber()
	p.logger.Sugar().Infof("After setting out/in edge cell offset")

	p.overlayGraph = datastructure.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.graph")

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.graph.RunKosaraju()

	p.logger.Sugar().Infof("Writing graph to ./data/original.graph")

	if writefile {
		err := p.overlayGraph.WriteToFile("./data/overlay_graph.graph")
		if err != nil {
			return err
		}

		return p.graph.WriteGraph("./data/original.graph")
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
To improve the spatial locality of process memory, group vertices and edges such that vertices with the same cell number are adjacent to each other in memory layout of the process.

The Linux Programming interface page 118-120:

Spatial locality is the tendency of a program to reference memory addresses
that are near those that were recently accessed (because of sequential process-
ing of instructions, and, sometimes, sequential processing of data structures).
A virtual memory scheme splits the memory used by each program into small,
fixed-size units called pages. Correspondingly, RAM is divided into a series of page
frames of the same size. At any one time, only some of the pages of a program need
to be resident in physical memory page frames; these pages form the so-called
resident set. Copies of the unused pages of a program are maintained in the swap
area—a reserved area of disk space used to supplement the computer’s RAM—and
loaded into physical memory only as required. When a process references a page
that is not currently resident in physical memory, a page fault occurs, at which point
the kernel suspends execution of the process while the page is loaded from disk
into memory.
so if we improve spatial locality -> number of page faults is minimized
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
			oEdges[i][k] = datastructure.NewOutEdge(
				oEdge.GetEdgeId(),
				oEdge.GetHead(),
				oEdge.GetWeight(),
				oEdge.GetLength(),
				oEdge.GetEntryPoint(),
			)
			e++
			k++
		}

		k = datastructure.Index(0)
		e = p.graph.GetVertex(i).GetFirstIn()
		for e < p.graph.GetVertex(i+1).GetFirstIn() {
			inEdge := p.graph.GetInEdge(e)
			iEdges[i][k] = datastructure.NewInEdge(
				inEdge.GetEdgeId(),
				inEdge.GetTail(),
				inEdge.GetWeight(),
				inEdge.GetLength(),
				inEdge.GetExitPoint(),
			)
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
		maxLat = math.Max(maxLat, p.graph.GetVertex(i).GetLat())
		maxLon = math.Max(maxLon, p.graph.GetVertex(i).GetLon())
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

	outOffset := datastructure.Index(0)                              // new offset for outEdges for each vertex for each cell
	p.graph.MakeOutEdgeCellOffset(p.graph.GetNumberOfCellsNumbers()) // offset of first outEdge for each cell
	inOffset := datastructure.Index(0)                               // new offset for inEdges for each vertex for each cell
	p.graph.MakeInEdgeCellOffset(p.graph.GetNumberOfCellsNumbers())  // offset of first inEdge for each cell

	graphMapEdgeInfo := p.graph.GetMapEdgeInfo()
	gsEdgeExtraInfos := make([]datastructure.EdgeExtraInfo, len(graphMapEdgeInfo))
	copy(gsEdgeExtraInfos, graphMapEdgeInfo)

	graphRoundaboutFlag := p.graph.GetRoundaboutFlag()
	roundaboutFlags := make([]datastructure.Index, len(graphRoundaboutFlag))
	copy(roundaboutFlags, graphRoundaboutFlag)

	graphTrafficLight := p.graph.GetNodeTrafficLight()
	nodeTrafficLight := make([]datastructure.Index, len(graphTrafficLight))
	copy(nodeTrafficLight, graphTrafficLight)
	vId := datastructure.Index(0)

	lastVertex := p.graph.GetVertex(datastructure.Index(p.graph.GetNumberOfVerticesWithDummyVertex() - 1))
	newVertices := make([]*datastructure.Vertex, p.graph.GetNumberOfVerticesWithDummyVertex())
	for i := datastructure.Index(0); i < datastructure.Index(p.graph.GetNumberOfCellsNumbers()); i++ {
		p.graph.SetOutEdgeCellOffset(i, outOffset)
		p.graph.SetInEdgeCellOffset(i, inOffset)

		for v := datastructure.Index(0); v < datastructure.Index(len(cellVertices[i])); v++ {
			// update vertex to use new vId
			// in the end of the outer loop, graph vertices are sorted by cell number

			newVertices[vId] = cellVertices[i][v].vertex

			vOldId := cellVertices[i][v].originalIndex
			newVertices[vId].SetFirstOut(outOffset)
			newVertices[vId].SetFirstIn(inOffset)
			newVertices[vId].SetId(vId)

			index := int(math.Floor(float64(vOldId) / 32))

			if len(nodeTrafficLight) > 0 {
				isTraficLight := (nodeTrafficLight[index] & (1 << (vOldId % 32))) != 0
				if isTraficLight {
					p.graph.SetNodeTrafficLight(vId)
				}
			}

			// update outedges & inedges
			for k := datastructure.Index(0); k < datastructure.Index(len(oEdges[vOldId])); k++ {

				oldOutEdge := oEdges[vOldId][k]
				p.graph.SetOutEdge(outOffset, datastructure.NewOutEdge(
					oldOutEdge.GetEdgeId(), oldOutEdge.GetHead(), oldOutEdge.GetWeight(),
					oldOutEdge.GetLength(), oldOutEdge.GetEntryPoint(),
				))
				p.graph.SetEdgeInfo(outOffset, gsEdgeExtraInfos[oldOutEdge.GetEdgeId()]) // update edge extra info storage

				indexRoundabout := int(math.Floor(float64(oldOutEdge.GetEdgeId()) / 32)) // update roundabout edge info
				if len(roundaboutFlags) > 0 {
					isRoundabout := (roundaboutFlags[indexRoundabout] & (1 << (oldOutEdge.GetEdgeId() % 32))) != 0
					p.graph.SetRoundabout(outOffset, isRoundabout)
				}

				outEdge := p.graph.GetOutEdge(outOffset)
				outEdge.SetEdgeId(outOffset)
				outEdge.SetHead(p.newVIdMap[oldOutEdge.GetHead()])

				outOffset++
			}

			for k := datastructure.Index(0); k < datastructure.Index(len(iEdges[vOldId])); k++ {
				oldInEdge := iEdges[vOldId][k]
				p.graph.SetInEdge(inOffset, datastructure.NewInEdge(
					oldInEdge.GetEdgeId(), oldInEdge.GetTail(), oldInEdge.GetWeight(),
					oldInEdge.GetLength(), oldInEdge.GetExitPoint(),
				))

				inEdge := p.graph.GetInEdge(inOffset)
				inEdge.SetEdgeId(inOffset)
				inEdge.SetTailId(p.newVIdMap[oldInEdge.GetTail()])
				inOffset++
			}

			vId++
		}
	}

	newVertices[len(newVertices)-1] = lastVertex
	p.graph.SetVertices(newVertices)
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

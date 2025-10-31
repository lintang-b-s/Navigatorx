package preprocesser

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
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
	p.SortByCellNumber()
	p.logger.Sugar().Infof("After setting out/in edge cell offset")

	overlayGraph := datastructure.NewOverlayGraph(p.graph, p.mlp)
	p.logger.Sugar().Infof("Overlay graph built and written to ./data/overlay_graph.graph")
	err := overlayGraph.WriteToFile("./data/overlay_graph.graph")
	if err != nil {
		return err
	}

	p.logger.Sugar().Infof("Running Kosaraju's algorithm to find strongly connected components (SCCs)...")
	p.RunKosaraju()

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

	newIds := make([]datastructure.Index, p.graph.NumberOfVertices()) // new vertex id after sorting by cell number
	newVid := datastructure.Index(0)                                  // new vertex id after sorting by cell number
	for i := 0; i < len(cellVertices); i++ {
		for v := 0; v < len(cellVertices[i]); v++ {
			newIds[cellVertices[i][v].originalIndex] = newVid
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
	roundabout := make([]datastructure.Index, len(graphRoundaboutFlag))
	copy(roundabout, graphRoundaboutFlag)

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

			isTraficLight := (nodeTrafficLight[index] & (1 << (vOldId % 32))) != 0
			if isTraficLight {
				p.graph.SetNodeTrafficLight(vId)
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
				roundabout := (roundabout[indexRoundabout] & (1 << (oldOutEdge.GetEdgeId() % 32))) != 0
				p.graph.SetRoundabout(outOffset, roundabout)

				outEdge := p.graph.GetOutEdge(outOffset)
				outEdge.SetEdgeId(outOffset)
				outEdge.SetHead(newIds[oldOutEdge.GetHead()])

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
				inEdge.SetTailId(newIds[oldInEdge.GetTail()])
				inOffset++
			}

			vId++
		}
	}

	newVertices[len(newVertices)-1] = lastVertex
	p.graph.SetVertices(newVertices)

}

// RunKosaraju. runs kosaraju's algorithm to find strongly connected components (SCCs) in the graph considering turn-restrictions of road networks.
func (p *Preprocessor) RunKosaraju() {
	m := datastructure.Index(p.graph.NumberOfEdges())
	components := make([][]datastructure.Index, 0, 10) // k component, with each component has arbritrary number of edges

	// remember this project use turn-based/edge-based graph
	// so the vertex in this graph is actually an edge in node-based graph
	// the index of inEdge is the same as the index of outEdge

	order := make([]datastructure.Index, 0, m)
	visited := make([]bool, m)
	timeCostFunction := costfunction.NewTimeCostFunction()
	for v := datastructure.Index(0); v < m; v++ {
		if !visited[v] {
			p.dfs(datastructure.Index(v), &order, visited, false, timeCostFunction)
		}
	}

	order = util.ReverseG[datastructure.Index](order)

	// reset visited
	visited = make([]bool, m)
	roots := make([]datastructure.Index, m)

	for _, v := range order {
		if !visited[v] {
			component := make([]datastructure.Index, 0, 10)
			p.dfs(v, &component, visited, true, timeCostFunction)
			components = append(components, component)
			root := datastructure.Index(math.MaxInt32)
			for _, node := range component {
				if node < root {
					root = node
				}
			}

			for _, node := range component {
				roots[node] = root
			}
		}
	}
	sccs := make([]datastructure.Index, m)

	for i, component := range components {
		for _, v := range component {
			sccs[v] = datastructure.Index(i)
		}
	}
	p.graph.SetSCCs(sccs)

	condAdj := make([][]datastructure.Index, m)
	for v := datastructure.Index(0); v < m; v++ {
		head := p.graph.GetHeadOfInedge(v)
		p.graph.ForOutEdgesOf(head, v-p.graph.GetEntryOffset(head), func(e *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
			eEntryPoint := datastructure.Index(e.GetEntryPoint()) + p.graph.GetInEdgeCellOffset(e.GetHead())
			if roots[eEntryPoint] != roots[v] {
				condAdj[roots[v]] = append(condAdj[roots[v]], roots[eEntryPoint])
			}
		})
	}

	scccCondAdj := make([][]datastructure.Index, len(components))
	for fromRootId, adjRootIds := range condAdj {
		sccOfV := sccs[fromRootId]
		for _, adjRootID := range adjRootIds {
			sccOfAdjRootId := sccs[adjRootID]
			scccCondAdj[sccOfV] = append(scccCondAdj[sccOfV], sccOfAdjRootId)
		}
	}

	p.graph.SetSCCCondensationAdj(scccCondAdj)
}

func (p *Preprocessor) dfs(v datastructure.Index, output *[]datastructure.Index, visited []bool,
	reversed bool, costFunction costfunction.CostFunction) {

	visited[v] = true

	if !reversed {
		// for forward dfs, first we find the head of the outdge
		// then we traverse outedges of the head
		// in here, v is entryPoint
		head := p.graph.GetHeadOfInedge(v)
		p.graph.ForOutEdgesOf(head, v-p.graph.GetEntryOffset(head), func(e *datastructure.OutEdge, exitPoint datastructure.Index, turnType pkg.TurnType) {
			turnCost := costFunction.GetTurnCost(turnType)
			if turnCost == pkg.INF_WEIGHT {
				return
			}
			eEntryPoint := datastructure.Index(e.GetEntryPoint()) + p.graph.GetInEdgeCellOffset(e.GetHead())
			if !visited[eEntryPoint] {
				p.dfs(eEntryPoint, output, visited, reversed, costFunction)
			}
		})
	} else {
		// for reversed dfs, first we find the tail of the edge
		// then we traverse inedges of the tail
		tail := p.graph.GetTailOfOutedge(v)
		p.graph.ForInEdgesOf(tail, v-p.graph.GetExitOffset(tail), func(e *datastructure.InEdge, entryPoint datastructure.Index, turnType pkg.TurnType) {
			turnCost := costFunction.GetTurnCost(turnType)
			if turnCost == pkg.INF_WEIGHT {
				return
			}
			eExitPoint := datastructure.Index(e.GetExitPoint()) + p.graph.GetOutEdgeCellOffset(e.GetTail())
			if !visited[eExitPoint] {
				p.dfs(eExitPoint, output, visited, reversed, costFunction)
			}
		})
	}

	*output = append(*output, v)
}

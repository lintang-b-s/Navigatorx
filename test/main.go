package main

import (
	"fmt"
	"log"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/customizer"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/logger"
	preprocesser "github.com/lintang-b-s/Navigatorx/pkg/preprocessor"
)

type edge struct {
	u      datastructure.Index
	v      datastructure.Index
	weight float64
	dist   float64
}

func main() {
	vertices := make([]*datastructure.Vertex, 9)
	for i := 0; i < 9; i++ {
		vertices[i] = &datastructure.Vertex{}
		vertices[i].SetId(datastructure.Index(i))
	}
	edges := make([]edge, 13)
	nodeOutEdges := make([][]*datastructure.OutEdge, 9)
	nodeInEdges := make([][]*datastructure.InEdge, 9) // inEdge is just edge with head at a node
	// example: u->v, u is tail, v is head.

	// in backward search in bidirectional dijksra, we traverse though inEdge (reverse of outEdge)
	// but inEdge not represent real road segment in road networks.
	// if road segment is bidirectional, there are two outEdge and two inEdge, one for each direction.
	// but if road segment is unidirectional, there is only one outEdge and one inEdge.

	// not bidirectional
	edges[0] = edge{0, 4, 8.0, 192.0}
	edges[1] = edge{1, 0, 8.0, 192.0}
	edges[2] = edge{1, 2, 8.0, 192.0}
	edges[3] = edge{1, 4, 8.0, 192.0}
	edges[4] = edge{1, 6, 8.0, 192.0}
	edges[5] = edge{2, 3, 8.0, 192.0}
	edges[6] = edge{2, 5, 8.0, 192.0}
	edges[7] = edge{4, 6, 8.0, 192.0}
	edges[8] = edge{5, 3, 8.0, 192.0}
	edges[9] = edge{5, 7, 8.0, 192.0}
	edges[10] = edge{6, 1, 8.0, 192.0}
	edges[11] = edge{6, 3, 8.0, 192.0}
	edges[12] = edge{7, 2, 8.0, 192.0}
	lastEdgeId := 13

	for i := 0; i < len(edges); i++ {
		outEdge := datastructure.NewOutEdge(datastructure.Index(i), edges[i].v, edges[i].weight, edges[i].dist, uint8(len(nodeInEdges[edges[i].v])))
		inEdge := datastructure.NewInEdge(datastructure.Index(i), edges[i].u, edges[i].weight, edges[i].dist, uint8(len(nodeOutEdges[edges[i].u])))
		nodeOutEdges[edges[i].u] = append(nodeOutEdges[edges[i].u], outEdge)
		nodeInEdges[edges[i].v] = append(nodeInEdges[edges[i].v], inEdge)
	}

	for v := 0; v < 8; v++ {
		// we need to do this because crp query assume all vertex have at least one outEdge (at for target) and one inEdge (as for source)
		if len(nodeOutEdges[v]) == 0 {

			dummyID := datastructure.Index(lastEdgeId)
			dummyOut := datastructure.NewOutEdge(dummyID, datastructure.Index(v),
				0, 0, uint8(len(nodeInEdges[v])))
			nodeOutEdges[v] = append(nodeOutEdges[v], dummyOut)

			dummyIn := datastructure.NewInEdge(dummyID, datastructure.Index(v),
				0, 0, uint8(len(nodeOutEdges[v])-1))
			nodeInEdges[v] = append(nodeInEdges[v], dummyIn)
			lastEdgeId++
		}
	}

	outEdgeOffset := 0
	inEdgeOffset := 0
	for i := 0; i < 9; i++ {
		vertices[i].SetFirstOut(datastructure.Index(outEdgeOffset))
		vertices[i].SetFirstIn(datastructure.Index(inEdgeOffset))
		outEdgeOffset += len(nodeOutEdges[i])
		inEdgeOffset += len(nodeInEdges[i])
	}

	// flatten
	flattenOutEdges := make([]*datastructure.OutEdge, outEdgeOffset)
	flattenInEdges := make([]*datastructure.InEdge, inEdgeOffset)
	outEdgeOffset = 0
	inEdgeOffset = 0
	for i := 0; i < 9; i++ {

		for j := 0; j < len(nodeOutEdges[i]); j++ {
			flattenOutEdges[outEdgeOffset] = nodeOutEdges[i][j]
			outEdgeOffset++
		}

		for j := 0; j < len(nodeInEdges[i]); j++ {
			flattenInEdges[inEdgeOffset] = nodeInEdges[i][j]
			inEdgeOffset++
		}
	}

	turnTables := make([]pkg.TurnType, 4)
	for i := 0; i < 4; i++ {
		turnTables[i] = pkg.TurnType(5)
	}
	graph := datastructure.NewGraph(vertices, flattenOutEdges, flattenInEdges, turnTables)

	graph.ForOutEdgesOf(0, 0, func(e *datastructure.OutEdge, exitPoint datastructure.Index, turn pkg.TurnType) {
		vId := e.GetHead()
		log.Printf("outEdge from 0 to %v with turn type %v\n", vId, turn)
	})
	var mlp *datastructure.MultilevelPartition = &datastructure.MultilevelPartition{}
	mlp.SetNumberOflevels(2)
	mlp.SetNumberOfVertices(graph.NumberOfVertices())
	mlp.SetNumberOfCellsInLevel(0, 4)
	mlp.SetNumberOfCellsInLevel(1, 2)
	mlp.ComputeBitmap()

	cells := make([]int, 8)
	cells = []int{0, 0, 1, 3, 2, 1, 2, 1, 1}
	topLevelCell := make([]int, 4)
	topLevelCell = []int{0, 0, 1, 1}
	for v := 0; v < graph.NumberOfVertices(); v++ {
		mlp.SetCell(0, v, cells[v])
		mlp.SetCell(1, v, topLevelCell[cells[v]])
	}

	logger, err := logger.New()
	if err != nil {
		panic(err)
	}
	preprocessor := preprocesser.NewPreprocessor(graph, mlp, logger)
	err = preprocessor.PreProcessing()
	if err != nil {
		panic(err)
	}

	log.Printf("after preprocessing")
	graph.ForOutEdgesOf(0, 0, func(e *datastructure.OutEdge, exitPoint datastructure.Index, turn pkg.TurnType) {
		vId := e.GetHead()
		log.Printf("outEdge from 0 to %v with turn type %v\n", vId, turn)
	})

	custo := customizer.NewCustomizer("./data/original.graph", "./data/overlay_graph.graph", "./data/metrics.txt", logger)
	err = custo.Customize()
	if err != nil {
		panic(err)
	}

	routingEngine, err := engine.NewEngine("./data/original.graph", "./data/overlay_graph.graph", "./data/metrics.txt", logger)
	if err != nil {
		panic(err)
	}

	for i := 0; i < 8; i++ {
		for j := 0; j < 8; j++ {
			bidirSearch := routing.NewCRPBidirectionalSearch(routingEngine.GetRoutingEngine())
			if i == j {
				continue
			}
			shortestPath, _, path, found := bidirSearch.Search(datastructure.Index(i), datastructure.Index(j))
			if !found {
				fmt.Printf("no path found from %v to %v\n", i, j)
			} else {
				for k := 0; k < len(path); k++ {
					if k > 0 {
						fmt.Printf("->")
					}
					fmt.Printf("%v", path[k])
				}
				fmt.Printf("\n")
			}
			fmt.Printf("shortest path from %v to %v: ", i, j)
			fmt.Printf("%v\n", shortestPath)
		}
	}

}

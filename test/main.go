package main

import (
	"fmt"

	"github.com/lintang-b-s/navigatorx-crp/pkg"
	"github.com/lintang-b-s/navigatorx-crp/pkg/customizer"
	"github.com/lintang-b-s/navigatorx-crp/pkg/datastructure"
	"github.com/lintang-b-s/navigatorx-crp/pkg/engine"
	"github.com/lintang-b-s/navigatorx-crp/pkg/engine/routing"
	preprocesser "github.com/lintang-b-s/navigatorx-crp/pkg/preprocessor"
)

func main() {
	vertices := make([]*datastructure.Vertex, 9)
	for i := 0; i < 9; i++ {
		vertices[i] = &datastructure.Vertex{}
		vertices[i].SetId(datastructure.Index(i))
	}
	vertices[1].SetFirstOut(1)
	vertices[2].SetFirstOut(5)
	vertices[3].SetFirstOut(7)
	vertices[4].SetFirstOut(7)
	vertices[5].SetFirstOut(8)
	vertices[6].SetFirstOut(10)
	vertices[7].SetFirstOut(12)
	vertices[8].SetFirstOut(13)

	vertices[1].SetFirstIn(1)
	vertices[2].SetFirstIn(2)
	vertices[3].SetFirstIn(4)
	vertices[4].SetFirstIn(7)
	vertices[5].SetFirstIn(9)
	vertices[6].SetFirstIn(10)
	vertices[7].SetFirstIn(12)
	vertices[8].SetFirstIn(13)

	forwardEdges := make([]*datastructure.OutEdge, 13)

	weight := 8.0
	length := 192.0
	forwardEdges[0] = datastructure.NewOutEdge(0, 4, weight, length, 0) 
	forwardEdges[1] = datastructure.NewOutEdge(1, 0, weight, length, 0)
	forwardEdges[2] = datastructure.NewOutEdge(2, 2, weight, length, 0)
	forwardEdges[3] = datastructure.NewOutEdge(3, 4, weight, length, 1)
	forwardEdges[4] = datastructure.NewOutEdge(4, 6, weight, length, 0)
	forwardEdges[5] = datastructure.NewOutEdge(5, 3, weight, length, 0)
	forwardEdges[6] = datastructure.NewOutEdge(6, 5, weight, length, 0)
	forwardEdges[7] = datastructure.NewOutEdge(7, 6, weight, length, 1)
	forwardEdges[8] = datastructure.NewOutEdge(8, 3, weight, length, 1)
	forwardEdges[9] = datastructure.NewOutEdge(9, 7, weight, length, 0)
	forwardEdges[10] = datastructure.NewOutEdge(10, 1, weight, length, 0)
	forwardEdges[11] = datastructure.NewOutEdge(11, 3, weight, length, 2)
	forwardEdges[12] = datastructure.NewOutEdge(12, 2, weight, length, 1)

	backwardEdges := make([]*datastructure.InEdge, 13)
	backwardEdges[1] = datastructure.NewInEdge(1, 6, weight, length, 0)   
	backwardEdges[0] = datastructure.NewInEdge(0, 1, weight, length, 0)  
	backwardEdges[2] = datastructure.NewInEdge(2, 1, weight, length, 1)   
	backwardEdges[3] = datastructure.NewInEdge(3, 7, weight, length, 0)    
	backwardEdges[4] = datastructure.NewInEdge(4, 2, weight, length, 0)    
	backwardEdges[5] = datastructure.NewInEdge(5, 5, weight, length, 0)   
	backwardEdges[6] = datastructure.NewInEdge(6, 6, weight, length, 1)   
	backwardEdges[7] = datastructure.NewInEdge(7, 0, weight, length, 0)   
	backwardEdges[8] = datastructure.NewInEdge(8, 1, weight, length, 2)   
	backwardEdges[9] = datastructure.NewInEdge(9, 2, weight, length, 1)  
	backwardEdges[10] = datastructure.NewInEdge(10, 1, weight, length, 3)  
	backwardEdges[11] = datastructure.NewInEdge(11, 4, weight, length, 0)  
	backwardEdges[12] = datastructure.NewInEdge(12, 5, weight, length, 1)  

	turnTables := make([]pkg.TurnType, 4)
	for i := 0; i < 4; i++ {
		turnTables[i] = pkg.TurnType(5)
	}
	graph := datastructure.NewGraph(vertices, forwardEdges, backwardEdges, turnTables)

	var mlp *datastructure.MultilevelPartition = &datastructure.MultilevelPartition{}
	mlp.SetNumberOflevels(2)
	mlp.SetNumberOfVertices(graph.NumberOfVertices())
	mlp.SetNumberOfCellsInLevel(0, 4)
	mlp.SetNumberOfCellsInLevel(1, 2)
	mlp.ComputeBitmap()

	cells := make([]int, 8)
	cells = []int{0, 0, 1, 3, 2, 1, 2, 1}
	topLevelCell := make([]int, 4)
	topLevelCell = []int{0, 0, 1, 1}
	for v := 0; v < graph.NumberOfVertices(); v++ {
		mlp.SetCell(0, v, cells[v])
		mlp.SetCell(1, v, topLevelCell[cells[v]])
	}

	// preprocessor := preprocesser.NewPreprocessor(graph, mlp)
	// og := preprocessor.PreProcessing2()

	// og.ForVertices(func(v *datastructure.OverlayVertex) {
	// 	fmt.Printf("(%v, %v)", v.GetEntryPointSize(), v.GetCellNumber())
	// })

	// costFunction := costfunction.NewTimeCostFunction()
	// log.Printf("Building cliques for each cell for each overlay graph level...")
	// overlayWeight := datastructure.NewOverlayWeights(og.GetWeightVectorSize())
	// overlayWeight.Build(graph, og, costFunction)

	// log.Printf("Building stalling tables...")
	// metrics := metrics.NewMetric(graph, costFunction, overlayWeight)
	// metrics.BuildStallingTables(og, graph)

	preprocessor := preprocesser.NewPreprocessor(graph, mlp)
	err := preprocessor.PreProcessing()
	if err != nil {
		panic(err)
	}

	custo := customizer.NewCustomizer("./data/test.graph", "./data/overlay_graph.graph", "./data/metrics.txt")
	err = custo.Customize()
	if err != nil {
		panic(err)
	}

	routingEngine, err := engine.NewEngine("./data/test.graph", "./data/overlay_graph.graph", "./data/metrics.txt")
	if err != nil {
		panic(err)
	}

	for i := 0; i < 8; i++ {
		for j := 0; j < 8; j++ {

			bidirSearch := routing.NewCRPBidirectionalSearch(routingEngine.GetRoutingEngine())

			if i == j {
				continue
			}
			shortestPath, _, found := bidirSearch.Search(datastructure.Index(i), datastructure.Index(j))
			if !found {
				fmt.Printf("no path found from %v to %v\n", i, j)
			}

			fmt.Printf("shortest path from %v to %v: ", i, j)
			fmt.Printf("%v\n", shortestPath)

		}
	}
}

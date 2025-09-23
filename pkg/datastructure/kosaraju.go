package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// RunKosaraju. runs kosaraju's algorithm to find strongly connected components (SCCs) in the graph considering turn-restrictions of road networks.
func (g *Graph) RunKosaraju() {
	m := Index(g.NumberOfEdges())
	components := make([][]Index, 0, 10) // k component, with each component has arbritrary number of edges

	// remember this project use turn-based/edge-based graph
	// so the vertex in this graph is actually an edge in node-based graph
	// the index of inEdge is the same as the index of outEdge

	order := make([]Index, 0, m)
	visited := make([]bool, m)
	timeCostFunction := costfunction.NewTimeCostFunction()
	for v := Index(0); v < m; v++ {
		if !visited[v] {
			g.dfs(Index(v), &order, visited, false, timeCostFunction)
		}
	}

	order = util.ReverseG[Index](order)

	// reset visited
	visited = make([]bool, m)
	roots := make([]Index, m)

	for _, v := range order {
		if !visited[v] {
			component := make([]Index, 0, 10)
			g.dfs(v, &component, visited, true, timeCostFunction)
			components = append(components, component)
			root := Index(math.MaxInt32)
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
	sccs := make([]Index, m)

	for i, component := range components {
		for _, v := range component {
			sccs[v] = Index(i)
		}
	}
	g.setSCCs(sccs)

	condAdj := make([][]Index, m)
	for v := Index(0); v < m; v++ {
		head := g.GetHeadOfInedge(v)
		g.ForOutEdgesOf(head, v-g.GetEntryOffset(head), func(e *OutEdge, exitPoint Index, turnType pkg.TurnType) {
			eEntryPoint := Index(e.GetEntryPoint()) + g.GetInEdgeCellOffset(e.GetHead())
			if roots[eEntryPoint] != roots[v] {
				condAdj[roots[v]] = append(condAdj[roots[v]], roots[eEntryPoint])
			}
		})
	}

	scccCondAdj := make([][]Index, len(components))
	for fromRootId, adjRootIds := range condAdj {
		sccOfV := sccs[fromRootId]
		for _, adjRootID := range adjRootIds {
			sccOfAdjRootId := sccs[adjRootID]
			scccCondAdj[sccOfV] = append(scccCondAdj[sccOfV], sccOfAdjRootId)
		}
	}

	g.setSCCCondensationAdj(scccCondAdj)
}

func (g *Graph) dfs(v Index, output *[]Index, visited []bool,
	reversed bool, costFunction costfunction.CostFunction) {

	visited[v] = true

	if !reversed {
		// for forward dfs, first we find the head of the outdge
		// then we traverse outedges of the head
		// in here, v is entryPoint
		head := g.GetHeadOfInedge(v)
		g.ForOutEdgesOf(head, v-g.GetEntryOffset(head), func(e *OutEdge, exitPoint Index, turnType pkg.TurnType) {
			turnCost := costFunction.GetTurnCost(turnType)
			if turnCost == pkg.INF_WEIGHT {
				return
			}
			eEntryPoint := Index(e.GetEntryPoint()) + g.GetInEdgeCellOffset(e.GetHead())
			if !visited[eEntryPoint] {
				g.dfs(eEntryPoint, output, visited, reversed, costFunction)
			}
		})
	} else {
		// for reversed dfs, first we find the tail of the edge
		// then we traverse inedges of the tail
		tail := g.GetTailOfOutedge(v)
		g.ForInEdgesOf(tail, v-g.GetExitOffset(tail), func(e *InEdge, entryPoint Index, turnType pkg.TurnType) {
			turnCost := costFunction.GetTurnCost(turnType)
			if turnCost == pkg.INF_WEIGHT {
				return
			}
			eExitPoint := Index(e.GetExitPoint()) + g.GetOutEdgeCellOffset(e.GetTail())
			if !visited[eExitPoint] {
				g.dfs(eExitPoint, output, visited, reversed, costFunction)
			}
		})
	}

	*output = append(*output, v)
}

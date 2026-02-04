package datastructure

import (
	"math"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// https://cp-algorithms.com/graph/strongly-connected-components.html
func (g *Graph) RunKosaraju() {
	// O(V+E)
	n := Index(g.NumberOfVertices())
	components := make([][]Index, 0, 10)

	order := make([]Index, 0, n)
	visited := make([]bool, n)
	for v := Index(0); v < n; v++ {
		// v is index of vertice id
		if !visited[v] {
			g.dfs(Index(v), &order, visited, false)
		}
	}

	order = util.ReverseG[Index](order)

	// reset visited
	visited = make([]bool, n)
	roots := make([]Index, n)

	for _, v := range order {
		if !visited[v] {
			component := make([]Index, 0, 10)
			g.dfs(v, &component, visited, true)
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
	sccs := make([]Index, n)

	for i, component := range components {
		for _, v := range component {
			sccs[v] = Index(i)
		}
	}
	g.SetSCCs(sccs)

	condAdj := make([][]Index, n)
	for v := Index(0); v < n; v++ {
		g.ForOutEdgesOfWithId(v, func(e *OutEdge, id Index) {
			if roots[e.GetHead()] != roots[v] {
				condAdj[roots[v]] = append(condAdj[roots[v]], roots[e.GetHead()])
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

	g.SetSCCCondensationAdj(scccCondAdj)
}

func (g *Graph) dfs(v Index, output *[]Index, visited []bool,
	reversed bool) {

	if !reversed {
		g.ForOutEdgesOfWithId(v, func(e *OutEdge, id Index) {
			if !visited[e.GetHead()] {
				visited[e.GetHead()] = true
				g.dfs(e.GetHead(), output, visited, reversed)
			}
		})
	} else {
		g.ForInEdgesOfWithId(v, func(e *InEdge, id Index) {
			if !visited[e.GetTail()] {
				visited[e.GetTail()] = true
				g.dfs(e.GetTail(), output, visited, reversed)
			}
		})
	}

	*output = append(*output, v)
}

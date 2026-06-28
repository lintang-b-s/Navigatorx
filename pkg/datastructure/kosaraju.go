package datastructure

import (
	"github.com/bits-and-blooms/bitset"
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

	util.ReverseG[Index](order)

	// reset visited
	visited = make([]bool, n)
	roots := make([]Index, n)

	for _, v := range order {
		if !visited[v] {
			component := make([]Index, 0, 10)
			g.dfs(v, &component, visited, true)
			components = append(components, component)
			root := v
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
		g.ForOutEdgeIdsOf(v, func(id Index) {
			eHead := g.GetHeadOfOutEdge(id)
			if roots[eHead] != roots[v] {
				condAdj[roots[v]] = append(condAdj[roots[v]], roots[eHead])
			}
		})
	}

	sccCondAdj := make([][]Index, len(components))
	for fromRootId, adjRootIds := range condAdj {
		sccOfV := sccs[fromRootId]
		for _, adjRootID := range adjRootIds {
			sccOfAdjRootId := sccs[adjRootID]
			sccCondAdj[sccOfV] = append(sccCondAdj[sccOfV], sccOfAdjRootId)
		}

		sccAdjs := sccCondAdj[sccOfV]
		sccCondAdj[sccOfV] = util.RemoveDuplicates(sccAdjs)
	}

	topoSorted := g.topoSort(sccCondAdj)
	reach := g.buildReachabilityArr(sccCondAdj, topoSorted)

	g.SetSCCCondensationAdj(sccCondAdj)
	g.SetSccReach(reach)
}

func (g *Graph) buildReachabilityArr(sccCondAdjList [][]Index, topoSorted []Index) []*bitset.BitSet {
	n := len(sccCondAdjList)

	reach := make([]*bitset.BitSet, n) // sccId v -> bitset dari list dari other sccIds u yang dapat reach sccId v
	// sccCondAdjList adlh condensation graph (directed acyclic graph) hasil kosaraju SCC.

	for v := 0; v < n; v++ {
		bs := bitset.New(INITIAL_REACHIBILITY_BITSET_SIZE)
		bs.Set(uint(v))
		reach[v] = bs
	}

	for i := 0; i < n; i++ {
		u := topoSorted[i]
		for _, v := range sccCondAdjList[u] {
			reach[v].InPlaceUnion(reach[u])
		}
	}

	return reach
}

// topological sorting scc condensation graph pakai kahn's algorithm. sccCondAdjList adlh condensation graph (directed acyclic graph) hasil kosaraju SCC.
func (g *Graph) topoSort(sccCondAdjList [][]Index) []Index {
	n := len(sccCondAdjList)
	inDegree := make([]Index, n)
	for u := 0; u < n; u++ {
		for _, v := range sccCondAdjList[u] {
			inDegree[v]++
		}
	}

	topoSorted := make([]Index, 0)
	queue := make([]Index, 0)

	for u := 0; u < n; u++ {
		if inDegree[u] == 0 {
			queue = append(queue, Index(u))
		}
	}

	// O(V+E), v=number of sccs, E=number of edges that connect sccs in condensation graph
	for len(queue) > 0 {
		u := queue[0]
		queue = queue[1:]

		topoSorted = append(topoSorted, u)
		for _, v := range sccCondAdjList[u] {
			inDegree[v]--
			if inDegree[v] == 0 {
				queue = append(queue, v)
			}
		}
	}

	return topoSorted
}

func (g *Graph) dfs(v Index, output *[]Index, visited []bool,
	reversed bool) {
	// discovered v

	visited[v] = true

	if !reversed {
		g.ForOutEdgeIdsOf(v, func(id Index) {
			eHead := g.GetHeadOfOutEdge(id)

			if !visited[eHead] {
				g.dfs(eHead, output, visited, reversed)
			}
		})
	} else {
		g.ForInEdgeIdsOf(v, func(id Index) {
			eTail := g.GetTailOfInedge(id)
			if !visited[eTail] {
				g.dfs(eTail, output, visited, reversed)
			}
		})
	}

	// finished v
	*output = append(*output, v)
}

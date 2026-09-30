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
			g.Dfs(Index(v), &order, visited, false)
		}
	}

	util.ReverseG[Index](order)

	// reset visited
	visited = make([]bool, n)
	roots := make([]Index, n)

	for _, v := range order {
		if !visited[v] {
			component := make([]Index, 0, 10)
			g.Dfs(v, &component, visited, true)
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
			eHead := g.GetHead(id)
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

func (g *Graph) Dfs(v Index, output *[]Index, visited []bool,
	reversed bool) {
	// discovered v

	visited[v] = true

	if !reversed {
		g.ForOutEdgeIdsOf(v, func(id Index) {
			eHead := g.GetHead(id)
			if !visited[eHead] {
				g.Dfs(eHead, output, visited, reversed)
			}
		})
	} else {
		g.ForInEdgeIdsOf(v, func(id Index) {
			eTail := g.GetTail(id)
			if !visited[eTail] {
				g.Dfs(eTail, output, visited, reversed)
			}
		})
	}

	// finished v
	*output = append(*output, v)
}

// pathExistsCondensationGraph. cek apakah ada path (tanpa costs) dari u ke v
// O(1)
func (g *Graph) pathExistsCondensationGraph(u, v Index) bool {
	sccOfU := g.sccs[u]
	sccOfV := g.sccs[v]

	uvPathExists := g.sccReach[sccOfV].Test(uint(sccOfU))
	return uvPathExists
}

func (g *Graph) SccVCanBeReachedBySccU(sccu, sccv Index) bool {
	uvPathExists := g.sccReach[sccv].Test(uint(sccu))
	return uvPathExists
}

// dfsCondensationGraph. dfs di condesation graph
// O(V_G + E_G), V_G=number of sccs in graph/number of vertices in condensation graph, E_G=number of edges in condensation graph
func (g *Graph) DfsCondensationGraph(u Index, t Index, discovered []bool, uvPathExists *bool) {
	if u == t {
		*uvPathExists = true
		return // gak perlu discover out neighbor dari t. discover u = discover vertex u sebelum adjacency listnya examined
	}

	if discovered[u] {
		return
	}
	discovered[u] = true

	for _, v := range g.sccCondensationAdj[u] {
		if *uvPathExists {
			// kita bisa return early karena uvPathExists=true
			// gak perlu examine other out neighbor dari u
			return
		}
		g.DfsCondensationGraph(v, t, discovered, uvPathExists)
	}
}

// PathExists. cek apakah ada path (tanpa costs) dari u ke v .
// kalau u dan v terdapat dalam scc yang sama, then its strongly connected atau ada path dari u ke v dan sebaliknya
// kita sudah precompute condensation graph yang merupakan directed acyclic graph (DAG) dengan vertices nya adalah sccs dari graph
// dan terdapat edge dari scc c1 ke scc c2 jika pada graph terdapat simpul in c1 yang memiliki edge dengan head in c2.
// pas kita dfs di condensation graph dari c1, jika kita bisa reach/discover c2 maka terdapat path dari u ke v,
// hal ini karena all vertices in c2 strongly connected.
// note, kita udah precompute scc reachability (di kosaraju.go): untuk setiap scc v, g.sccreach[v] simpan semua other scc u yang dapat reach v.
// O(1)
func (g *Graph) PathExists(u, v Index) bool {
	sccOfU := g.GetSCCOfAVertex(u)
	sccOfV := g.GetSCCOfAVertex(v)
	if sccOfU == sccOfV {
		return true
	}

	return g.pathExistsCondensationGraph(u, v)
}

func (g *Graph) SetSCCs(sccs []Index) {
	g.sccs = sccs
}

func (g *Graph) SetSCCCondensationAdj(adj [][]Index) {
	g.sccCondensationAdj = adj
}

func (g *Graph) SetSccReach(sccReach []*bitset.BitSet) {
	g.sccReach = sccReach
}

func (g *Graph) GetSCCOfAVertex(u Index) Index {
	return g.sccs[u]
}

func (g *Graph) GetSCCS() []Index {
	return g.sccs
}
func (g *Graph) GetSCCCondensationAdjList() [][]Index {
	return g.sccCondensationAdj
}

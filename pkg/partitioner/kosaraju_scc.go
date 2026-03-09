package partitioner

import (
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type kosrajuGraph struct {
	vertices         []da.PartitionVertex
	adjacencyList    [][]da.MaxFlowEdge
	revAdjacencyList [][]da.MaxFlowEdge
}

func buildKosarajuGraph(n int, edges []da.MaxFlowEdge, vertices []da.PartitionVertex) *kosrajuGraph {
	adjacencyList := make([][]da.MaxFlowEdge, n)
	revAdjacencyList := make([][]da.MaxFlowEdge, n)
	for i := 0; i < len(edges); i++ {
		e := edges[i]
		u := e.GetFrom()
		v := e.GetTo()
		adjacencyList[u] = append(adjacencyList[u], e)

		eRev := da.NewMaxFlowEdge(e.GetID(), v, u, e.GetCapacity())
		revAdjacencyList[v] = append(revAdjacencyList[v], eRev)
	}
	return &kosrajuGraph{vertices, adjacencyList, revAdjacencyList}
}

func (g *kosrajuGraph) forOutEdgesOf(u da.Index, handle func(e da.MaxFlowEdge, id int)) {
	for id, e := range g.adjacencyList[u] {
		handle(e, id)
	}
}

func (g *kosrajuGraph) forInEdgesOf(u da.Index, handle func(e da.MaxFlowEdge, id int)) {
	for id, e := range g.revAdjacencyList[u] {
		handle(e, id)
	}
}

func (g *kosrajuGraph) getVertex(vid da.Index) da.PartitionVertex {
	return g.vertices[vid]
}

/*
kosaraju.
given directed graph partitionGraph, find strongly connected components
for sccs yang punya size >= maximumCellSize return as bigcomponents
else gabung semua sccs yang punya size < maximumCellSize jadi 1 smallComponent

return concatenation dari bigcomponents dan smallComponent & jadikan array of partitionGraph
dengan vertex id dari setiap partitionGraph sama dengan vertex id di pg

*/
func prePartitionWithSCC(pg *da.PartitionGraph, maximumCellSize int) []*da.PartitionGraph {
	// O(n+m), n,m= number of vertices and edges of pg

	edges := pg.GetDirectedEdges()
	n := da.Index(pg.NumberOfVertices())

	vertices := pg.GetVertices()
	g := buildKosarajuGraph(int(n), edges, vertices)
	components := make([][]da.PartitionVertex, 0, 10)

	order := make([]da.PartitionVertex, 0, n)
	visited := make([]bool, n)

	for v := da.Index(0); v < n; v++ {
		// v is index of vertice id
		vertex := g.getVertex(v)
		if !visited[v] {
			g.dfs(vertex, &order, visited, false)
		}
	}

	order = util.ReverseG[da.PartitionVertex](order)

	// reset visited
	visited = make([]bool, n)

	for _, v := range order {
		if !visited[v.GetID()] {
			component := make([]da.PartitionVertex, 0, 10)
			g.dfs(v, &component, visited, true)
			components = append(components, component)

		}
	}


	// pecah sccs
	bigComponents := make([][]da.PartitionVertex, 0, len(components)/2)
	smallComponent := make([]da.PartitionVertex, 0, maximumCellSize)

	for _, component := range components {
		if len(component) >= maximumCellSize {
			bigComponents = append(bigComponents, component)
		} else {
			smallComponent = append(smallComponent, component...)
		}
	}

	groupedComponents := append(bigComponents, smallComponent)

	pgComponents := make([]*da.PartitionGraph, 0, len(groupedComponents))

	// bikin id baru buat tiap vertices di setiap components
	for i := 0; i < len(groupedComponents); i++ {
		// O(n+m), number of vertices & edges dari union of all components equal to n dan m
		comp := groupedComponents[i]
		npg := len(comp)
		pgComponent := da.NewPartitionGraph(npg)

		compVIds := make([]da.Index, 0, len(comp))
		for _, vertex := range comp {
			compVIds = append(compVIds, vertex.GetID())
		}

		newVid := datastructure.Index(0)
		newMapVid := make(map[datastructure.Index]datastructure.Index, len(compVIds))
		for i := 0; i < len(compVIds); i++ {
			oldVertex := comp[i]
			vId := compVIds[i]
			lat, lon := oldVertex.GetVertexCoordinate()

			oriVId := oldVertex.GetOriginalVertexID()
			vertex := datastructure.NewPartitionVertex(newVid, oriVId, lat, lon)
			newMapVid[vId] = newVid
			pgComponent.AddVertex(vertex)
			newVid++
		}

		compVIdsSet := makeNodeSet(compVIds)

		for _, uVertex := range comp {
			u := uVertex.GetID()
			pg.ForEachDirectedEdgesOf(u, func(e da.MaxFlowEdge, eId int) {
				v := e.GetTo()

				if _, adjVertexInSet := compVIdsSet[v]; !adjVertexInSet {
					// skip arc that its head outside current cell
					return
				}

				newU := newMapVid[u]
				newV := newMapVid[v]
				pgComponent.AddEdge(newU, newV, e.GetCapacity(), false)
			})
		}

		pgComponents = append(pgComponents, pgComponent)
	}

	return pgComponents
}

func (g *kosrajuGraph) dfs(vertex da.PartitionVertex, output *[]da.PartitionVertex, visited []bool,
	reversed bool) {
	v := vertex.GetID()
	// discovered v
	visited[v] = true

	if !reversed {
		g.forOutEdgesOf(v, func(e da.MaxFlowEdge, id int) {
			if !visited[e.GetTo()] {
				g.dfs(g.getVertex(e.GetTo()), output, visited, reversed)
			}
		})
	} else {
		g.forInEdgesOf(v, func(e da.MaxFlowEdge, id int) {
			if !visited[e.GetTo()] {
				g.dfs(g.getVertex(e.GetTo()), output, visited, reversed)
			}
		})
	}

	// finished v
	*output = append(*output, vertex)
}


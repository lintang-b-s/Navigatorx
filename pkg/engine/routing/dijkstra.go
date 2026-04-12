package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Dijkstra struct {
	engine *CRPRoutingEngine

	pq *da.QueryHeap[da.CRPQueryKey]

	numSettledNodes  int
	useReversedEdges bool
}

func NewDijkstra(engine *CRPRoutingEngine, useReversedEdges bool) Dijkstra {
	dj := Dijkstra{
		engine:           engine,
		numSettledNodes:  0,
		useReversedEdges: useReversedEdges,
	}

	dj.Preallocate()
	return dj
}

/*
single-source shortest paths, from s to all other vertices.
with no turn costs.
ini implementasi dijkstra yang biasa anda lihat di internet / competitive programming template.

useReversedGraph = true -> buat cari sssp dari every vertices in graph to s
*/
func (us *Dijkstra) ShortestPath(s da.Index) ([]float64, [][]da.Index) {

	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	djKey := da.NewDijkstraKey(s, s)
	us.pq.Insert(s, 0, sVertexInfo, djKey)

	for !us.pq.IsEmpty() {

		us.graphSearchUni(s)
		us.numSettledNodes++
	}

	sps, spEdges := us.constructShortestPath(s)

	return sps, spEdges
}

func (us *Dijkstra) graphSearchUni(source da.Index) {

	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()

	if !us.useReversedEdges {

		// traverse outEdges of u
		us.engine.graph.ForOutEdgeIdsOf(uId, func(eId da.Index) {
			head := us.engine.graph.GetHeadOfOutEdge(eId)

			vId := head

			edgeWeight := us.engine.GetWeight(eId, true)

			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newTravelTime := us.pq.GetPriority(uId) + edgeWeight

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vId), pkg.INF_WEIGHT)
			if vAlreadyLabelled && util.Ge(newTravelTime, us.pq.GetPriority(vId)) {
				// newTravelTime is not better, do nothing

				return
			}

			// newTravelTime is better, update the forwardInfo

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, eId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vId, newTravelTime, newTravelTime, newPar)

			} else if !vAlreadyLabelled {
				queryKey := da.NewDijkstraKey(vId, vId)
				vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, eId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vId, newTravelTime, vertexInfo, queryKey)
			}
		})
	} else {
		// use reversed edges

		// traverse inEdges of u
		us.engine.graph.ForInEdgeIdsOf(uId, func(eId da.Index) {
			tail := us.engine.graph.GetTailOfInedge(eId)

			vId := tail

			edgeWeight := us.engine.GetWeight(eId, false)

			newTravelTime := us.pq.GetPriority(uId) + edgeWeight

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vId), pkg.INF_WEIGHT)
			if vAlreadyLabelled && util.Ge(newTravelTime, us.pq.GetPriority(vId)) {
				// newTravelTime is not better, do nothing
				return
			}

			// newTravelTime is better, update the forwardInfo

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, eId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vId, newTravelTime, newTravelTime, newPar)

			} else if !vAlreadyLabelled {
				queryKey := da.NewDijkstraKey(vId, vId)
				vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, eId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vId, newTravelTime, vertexInfo, queryKey)
			}
		})
	}

	return
}

func (us *Dijkstra) Preallocate() {
	numberOfVerties := us.engine.graph.NumberOfVertices()
	maxSearchSize := numberOfVerties

	maxEdgesInCell := us.engine.graph.GetMaxEdgesInCell()
	us.pq = da.NewQueryHeap[da.CRPQueryKey](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
	us.pq.PreallocateHeap(maxSearchSize)
}

func (us *Dijkstra) constructShortestPath(s da.Index) ([]float64, [][]da.Index) {
	n := us.engine.graph.NumberOfVertices()
	spEdges := make([][]da.Index, n)
	sps := make([]float64, n)
	if !us.useReversedEdges {

		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetPriority(t)

			if s == t {
				continue // sp == 0
			}

			sps[t] = sp
			if util.Ge(sp, pkg.INF_WEIGHT) {
				continue
			}

			curInfo := us.pq.Get(t)

			for curInfo.GetParent().GetVertex() != s {
				parent := curInfo.GetParent()

				spEdges[t] = append(spEdges[t], parent.GetEdge())

				curInfo = us.pq.Get(parent.GetVertex())
			}

			util.ReverseG(spEdges[t])
		}

	} else {
		//  use reversed edges
		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetPriority(t)

			if s == t {
				continue // sp == 0
			}
			sps[t] = sp
			if util.Ge(sp, pkg.INF_WEIGHT) {
				continue
			}

			curInfo := us.pq.Get(t)

			for curInfo.GetParent().GetVertex() != s {
				parent := curInfo.GetParent()
				parentEdge := parent.GetEdge() // in inEdgeId

				outEdgeId := us.engine.graph.GetExitIdOfInEdge(parentEdge)

				// jadiin outEdge semua
				spEdges[t] = append(spEdges[t], outEdgeId)

				curInfo = us.pq.Get(parent.GetVertex())
			}
		}

		// kita mau cari
		// dari v -> .... -> landmark, for all vertices v
		// karena pakai reversed edges: landmark -> .... -> v ,
		// setiap reversed edge (v,u) punya weight sama dengan weight edge (u,v).
		// shortest path dari landmark ke v pakai reversed edges equivalent to shortest path dari v ke landmark pakai original edges
		// kita dapet spEdges berupa list of outEdges dari v ,... ke landmark
		// gak perlu direverse
	}
	return sps, spEdges
}

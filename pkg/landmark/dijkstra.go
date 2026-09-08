package landmark

import (
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg/costfunction"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Dijkstra[W util.RoutingNumber] struct {
	graph *da.Graph
	cf    *costfunction.TimeFunction[W]

	pq *da.QueryHeap[da.CRPQueryKey, W]

	useReverseGraph bool

	numSettledNodes int
}

func NewDijkstra[W util.RoutingNumber](graph *da.Graph, cf *costfunction.TimeFunction[W], useReverseGraph bool) *Dijkstra[W] {
	dj := &Dijkstra[W]{
		graph:           graph,
		useReverseGraph: useReverseGraph,
		numSettledNodes: 0,

		cf: cf,
	}

	return dj
}

/*
dijkstra sssp.

useReversedGraph = true -> buat cari sssp dari every vertices in graph to s

Cormen, T.H. et al. (2022) Introduction to Algorithms. 4th ed. Cambridge, MA, USA:
MIT Press (CLRS).:
Single-destination shortest-paths problem: Find a shortest path to a given des-
tination vertex t from each vertex v. By reversing the direction of each edge in
the graph, we can reduce this problem to a single-source problem..

buat precalculated landmark sp distances (untuk potential/heuristic ALT (A*, landmarks, and triangle inequality) algorithm)
kita gak perlu pakai turn costs karena kalau misal pake turn costs di fase query Customizable Route Planning (CRP), potential/heuristic nya masih underestimate true sp dist dan masih memenuhi sifat konsisten/feasible...
*/
func (us *Dijkstra[W]) ShortestPath(s da.Index, heapPool *sync.Pool) []W {
	us.pq = heapPool.Get().(*da.QueryHeap[da.CRPQueryKey, W])
	us.pq.Clear()

	done := func() {
		heapPool.Put(us.pq)
	}
	defer done()

	noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)

	djKey := da.NewDijkstraKey(s, s)
	us.pq.Insert(s, 0, da.NewVertexData(W(0),
		noPar), djKey)

	for !us.pq.IsEmpty() {
		// search on graph level 1
		us.graphSearchUni(s)
		us.numSettledNodes++
	}

	sps := us.constructShortestPath(s)

	return sps
}

func (us *Dijkstra[W]) graphSearchUni(source da.Index) {
	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()

	if !us.useReverseGraph {

		// traverse outEdges of u
		us.graph.ForOutEdgeIdsOf(uId, func(eId da.Index) {
			head := us.graph.GetHeadOfOutEdge(eId)

			vId := head

			edgeWeight := us.cf.GetWeight(eId)

			// get cost to reach v through u
			newVCost := us.pq.GetCost(uId) + edgeWeight

			if util.Ge(newVCost, util.Infinity[W]()) {

				return
			}

			vAlreadyLabelled := util.Lt(us.pq.GetCost(vId), util.Infinity[W]())

			if vAlreadyLabelled && util.Ge(newVCost, us.pq.GetCost(vId)) {
				// newVCost is not better, do nothing

				return
			}

			// newVCost is better, update the forwardData

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, eId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vId, newVCost, newVCost, newPar)

			} else if !vAlreadyLabelled {
				queryKey := da.NewDijkstraKey(vId, vId)
				vData := da.NewVertexData(newVCost, da.NewVertexEdgePair(uId, eId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vId, newVCost, vData, queryKey)
			}
		})
	} else {
		// use reversed edges

		// traverse inEdges of u
		us.graph.ForInEdgeIdsOf(uId, func(eId da.Index) {
			tail := us.graph.GetTailOfInedge(eId)

			vId := tail

			eExitId := us.graph.GetOutIdOfInEdge(eId)
			edgeWeight := us.cf.GetWeight(eExitId)

			newVCost := us.pq.GetCost(uId) + edgeWeight

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			vAlreadyLabelled := util.Lt(us.pq.GetCost(vId), util.Infinity[W]())
			if vAlreadyLabelled && util.Ge(newVCost, us.pq.GetCost(vId)) {
				// newVCost is not better, do nothing
				return
			}

			// newVCost is better, update the forwardData
			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, eId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vId, newVCost, newVCost, newPar)

			} else if !vAlreadyLabelled {
				queryKey := da.NewDijkstraKey(vId, vId)
				vData := da.NewVertexData(newVCost, da.NewVertexEdgePair(uId, eId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vId, newVCost, vData, queryKey)
			}
		})
	}
}

func (us *Dijkstra[W]) constructShortestPath(s da.Index) []W {
	n := us.graph.NumberOfVertices()
	sps := make([]W, n)
	if !us.useReverseGraph {

		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetCost(t)

			if s == t {
				continue // sp == 0
			}

			sps[t] = sp

		}

	} else {
		//  use reversed edges
		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetCost(t)

			if s == t {
				continue // sp == 0
			}
			sps[t] = sp

		}

	}
	return sps
}

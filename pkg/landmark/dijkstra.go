package landmark

import (
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Dijkstra struct {
	graph   *da.Graph
	metrics *met.Metric

	pq *da.QueryHeap[da.CRPQueryKey]

	useReverseGraph bool

	numSettledNodes int
}

func NewDijkstra(graph *da.Graph, metrics *met.Metric, useReverseGraph bool) *Dijkstra {
	dj := &Dijkstra{
		graph:           graph,
		useReverseGraph: useReverseGraph,
		numSettledNodes: 0,

		metrics: metrics,
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
func (us *Dijkstra) ShortestPath(s da.Index, heapPool *sync.Pool) []float64 {
	us.pq = heapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	us.pq.Clear()

	done := func() {
		heapPool.Put(us.pq)
	}
	defer done()

	noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)

	djKey := da.NewDijkstraKey(s, s)
	us.pq.Insert(s, 0, da.NewVertexInfo(0,
		noPar), djKey)

	for !us.pq.IsEmpty() {
		// search on graph level 1
		us.graphSearchUni(s)
		us.numSettledNodes++
	}

	sps := us.constructShortestPath(s)

	return sps
}

func (us *Dijkstra) graphSearchUni(source da.Index) {
	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()

	if !us.useReverseGraph {

		// traverse outEdges of u
		us.graph.ForOutEdgeIdsOf(uId, func(eId da.Index) {
			head := us.graph.GetHeadOfOutEdge(eId)

			vId := head

			weight, length, _ := us.graph.GetOutEdgeTripleWeightKey(eId)

			edgeWeight := us.metrics.GetWeight(eId, weight, length)

			// get cost to reach v through u
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
		us.graph.ForInEdgeIdsOf(uId, func(eId da.Index) {
			tail := us.graph.GetTailOfInedge(eId)

			vId := tail

			weight, length, _ := us.graph.GetInEdgeTripleWeightKey(eId)

			eExitId := us.graph.GetExitIdOfInEdge(eId)
			edgeWeight := us.metrics.GetWeight(eExitId, weight, length)

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
}

func (us *Dijkstra) constructShortestPath(s da.Index) []float64 {
	n := us.graph.NumberOfVertices()
	sps := make([]float64, n)
	if !us.useReverseGraph {

		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetPriority(t)

			if s == t {
				continue // sp == 0
			}

			sps[t] = sp
			
		}

	} else {
		//  use reversed edges
		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetPriority(t)

			if s == t {
				continue // sp == 0
			}
			sps[t] = sp
			
		}

	}
	return sps
}

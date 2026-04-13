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

	shortestTravelTimes []float64

	pq *da.QueryHeap[da.CRPQueryKey]

	useReverseGraph bool

	numSettledNodes int
}

func NewDijkstra(graph *da.Graph, metrics *met.Metric, useReverseGraph bool) *Dijkstra {
	dj := &Dijkstra{
		graph:               graph,
		useReverseGraph:     useReverseGraph,
		numSettledNodes:     0,
		shortestTravelTimes: make([]float64, 0),
		metrics:             metrics,
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
the graph, we can reduce this problem to a single-source problem
*/
func (us *Dijkstra) ShortestPath(s da.Index, heapPool *sync.Pool) []float64 {
	us.pq = heapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	us.pq.Clear()

	done := func() {
		heapPool.Put(us.pq)
	}
	defer done()

	sForwardId := da.Index(0)
	if !us.useReverseGraph {
		// v - sInEdge -> s

		// for iterating outEdges, we need entryOffset.
		sForwardId = us.graph.GetEntryOffset(s) + us.graph.GetInDegree(s) - 1 // dummy edge
	} else {
		// s - sOutEdge -> w
		sForwardId = us.graph.GetExitOffset(s) + us.graph.GetOutDegree(s) - 1 // dummy edge
	}

	noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)

	us.pq.Insert(sForwardId, 0, da.NewVertexInfo(0,
		noPar), da.NewDijkstraKey(s, sForwardId))

	for !us.pq.IsEmpty() {
		// search on graph level 1
		us.graphSearchUni(s)
		us.numSettledNodes++
	}

	n := us.graph.NumberOfVertices()
	us.shortestTravelTimes = make([]float64, n)

	for v := 0; v < n; v++ {
		us.shortestTravelTimes[v] = pkg.INF_WEIGHT
	}

	if !us.useReverseGraph {
		us.graph.ForOutEdges(func(exitPoint, head, tail, entryId da.Index, percentage float64, idx da.Index) {

			// -vEntry->v
			v := us.graph.GetHeadOfInedge(da.Index(entryId))

			sp := us.pq.GetPriority(entryId)

			if util.Lt(sp, us.shortestTravelTimes[v]) {
				us.shortestTravelTimes[v] = sp
			}
		})
	} else {
		us.graph.ForInEdges(func(e da.InEdge, entryPoint, head, tail, exitId da.Index, percentage float64, idx da.Index) {

			// v-vExit->
			v := us.graph.GetTailOfOutedge(da.Index(exitId))

			sp := us.pq.GetPriority(exitId)

			if util.Lt(sp, us.shortestTravelTimes[v]) {
				us.shortestTravelTimes[v] = sp
			}
		})
	}

	us.shortestTravelTimes[s] = 0
	return us.shortestTravelTimes
}

func (us *Dijkstra) graphSearchUni(source da.Index) {
	// taken from Delling, D. et al. (2015) “Customizable Route Planning in Road Networks,” Transportation Science [Preprint]. Available at: https://doi.org/10.1287/trsc.2014.0579 :
	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()

	uId := uItem.GetNode()
	if !us.useReverseGraph {

		uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

		uEntryPoint := uEntryId - us.graph.GetEntryOffset(uId)

		// traverse outEdges of u
		us.graph.ForOutEdgesOf(uId, uEntryPoint, func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType,
			hwType pkg.OsmHighwayType) {

			vId := head

			edgeWeight := us.metrics.GetWeight(hwType, weight, length)

			turnCost := us.metrics.GetTurnCost(turnType)
			if uId == source {
				turnCost = 0
			}

			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newArrTime := us.pq.GetPriority(uEntryId) + edgeWeight + turnCost

			if util.Ge(newArrTime, pkg.INF_WEIGHT) {
				return
			}

			vEntryId := us.graph.GetEntryOffset(vId) + da.Index(entryPoint)

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vEntryId), pkg.INF_WEIGHT)
			if vAlreadyLabelled && util.Ge(newArrTime, us.pq.GetPriority(vEntryId)) {
				// newArrTime is not better, do nothing
				return
			}

			// newArrTime is better, update the forwardInfo

			noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)
			if vAlreadyLabelled {

				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vEntryId, newArrTime, newArrTime,
					noPar)
			} else if !vAlreadyLabelled {
				// is key not in the priority queue, insert it

				us.pq.Insert(vEntryId, newArrTime,
					da.NewVertexInfo(newArrTime, noPar), da.NewDijkstraKey(vId, vEntryId))
			}

		})
	} else {

		uExitId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

		uExitPoint := uExitId - us.graph.GetExitOffset(uId)

		// traverse outEdges of u
		us.graph.ForInEdgesOf(uId, uExitPoint, func(eId, tail da.Index, weight, length float64, exitPoint, entryPoint da.Index,
			turnType pkg.TurnType, hwType pkg.OsmHighwayType) {

			vId := tail

			edgeWeight := us.metrics.GetWeight(hwType, weight, length)

			turnCost := us.metrics.GetTurnCost(turnType)

			if uId == source {
				turnCost = 0
			}

			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newArrTime := us.pq.GetPriority(uExitId) + edgeWeight + turnCost

			if util.Ge(newArrTime, pkg.INF_WEIGHT) {
				return
			}

			vExitId := us.graph.GetExitOffset(vId) + da.Index(exitPoint)

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vExitId), pkg.INF_WEIGHT)
			if vAlreadyLabelled && util.Ge(newArrTime, us.pq.GetPriority(vExitId)) {
				// newArrTime is not better, do nothing

				return
			}

			// newArrTime is better, update the forwardInfo
			noPar := da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false)

			if vAlreadyLabelled {
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vExitId, newArrTime, newArrTime, noPar)
			} else if !vAlreadyLabelled {
				// is key not in the priority queue, insert it

				us.pq.Insert(vExitId, newArrTime, da.NewVertexInfo(newArrTime, noPar),
					da.NewDijkstraKey(vId, vExitId))
			}

		})
	}
}

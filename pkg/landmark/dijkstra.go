package landmark

import (
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
)

type Dijkstra struct {
	graph   *da.Graph
	metrics *met.Metric

	forwardInfo         []float64
	heapNodes           []*da.PriorityQueueNode[da.CRPQueryKey]
	shortestTravelTimes []float64

	pq *da.QueryHeap[da.CRPQueryKey]

	useReverseGraph bool

	numSettledNodes int
}

func NewDijkstra(graph *da.Graph, metrics *met.Metric, useReverseGraph bool) *Dijkstra {
	dj := &Dijkstra{
		graph:               graph,
		forwardInfo:         make([]float64, graph.NumberOfEdges()),
		heapNodes:           make([]*da.PriorityQueueNode[da.CRPQueryKey], graph.NumberOfEdges()),
		useReverseGraph:     useReverseGraph,
		numSettledNodes:     0,
		shortestTravelTimes: make([]float64, 0),
		metrics:             metrics,
	}

	return dj
}

func (us *Dijkstra) ShortestPath(asId da.Index, heapPool *sync.Pool) []float64 {
	us.pq = heapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	us.pq.Clear()

	done := func() {
		heapPool.Put(us.pq)
	}
	defer done()

	s := da.Index(0)
	sForwardId := da.Index(0)
	if !us.useReverseGraph {
		s = us.graph.GetOutEdge(asId).GetHead()

		// for iterating outEdges, we need entryOffset.
		sForwardId = us.graph.GetEntryOffset(s) + da.Index(us.graph.GetOutEdge(asId).GetEntryPoint())
	} else {
		s = us.graph.GetInEdge(asId).GetTail()

		sForwardId = us.graph.GetExitOffset(s) + da.Index(us.graph.GetInEdge(asId).GetExitPoint())
	}

	initInfWeightVertexInfo(us.forwardInfo)

	us.forwardInfo[sForwardId] = 0

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
		us.shortestTravelTimes[v] = 2 * pkg.INF_WEIGHT
	}

	for entryExitId, sp := range us.forwardInfo {

		v := da.Index(0)
		if !us.useReverseGraph {
			v = us.graph.GetHeadOfInedge(da.Index(entryExitId))
		} else {
			v = us.graph.GetTailOfOutedge(da.Index(entryExitId))
		}

		if da.Lt(sp, us.shortestTravelTimes[v]) {
			us.shortestTravelTimes[v] = sp
		}
	}
	us.shortestTravelTimes[s] = 0
	return us.shortestTravelTimes
}

func (us *Dijkstra) graphSearchUni(source da.Index) {

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
		us.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {

			vId := outArc.GetHead()

			edgeWeight := us.metrics.GetWeight(outArc)

			turnCost := us.metrics.GetTurnCost(turnType)
			if uId == source {
				turnCost = 0
			}

			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newArrTime := us.forwardInfo[uEntryId] + edgeWeight + turnCost

			if da.Ge(newArrTime, pkg.INF_WEIGHT) {
				return
			}

			vEntryId := us.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())

			vAlreadyLabelled := da.Lt(us.forwardInfo[vEntryId], pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newArrTime, us.forwardInfo[vEntryId]) {
				// newArrTime is not better, do nothing
				return
			}

			// newArrTime is better, update the forwardInfo
			us.forwardInfo[vEntryId] = newArrTime

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
		us.graph.ForInEdgesOf(uId, uExitPoint, func(inArc *da.InEdge, entryPoint da.Index, turnType pkg.TurnType) {

			vId := inArc.GetTail()

			edgeWeight := us.metrics.GetWeight(inArc)

			turnCost := us.metrics.GetTurnCost(turnType)
			if uId == source {
				turnCost = 0
			}

			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newArrTime := us.forwardInfo[uExitId] + edgeWeight + turnCost

			if da.Ge(newArrTime, pkg.INF_WEIGHT) {
				return
			}

			vExitId := us.graph.GetExitOffset(vId) + da.Index(inArc.GetExitPoint())

			vAlreadyLabelled := da.Lt(us.forwardInfo[vExitId], pkg.INF_WEIGHT)
			if vAlreadyLabelled && da.Ge(newArrTime, us.forwardInfo[vExitId]) {
				// newArrTime is not better, do nothing

				return
			}

			// newArrTime is better, update the forwardInfo
			us.forwardInfo[vExitId] = newArrTime
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

func initInfWeightVertexInfo(vs []float64) {
	for i := range vs {
		vs[i] = 2 * pkg.INF_WEIGHT
	}
}

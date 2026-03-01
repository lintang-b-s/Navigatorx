package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type Dijkstra struct {
	engine *CRPRoutingEngine

	finalDist           []da.VertexInfo[da.CRPQueryKey]
	finalEdge           []da.Index
	shortestTravelTimes []float64

	pq *da.QueryHeap[da.CRPQueryKey]

	numSettledNodes int
}

func NewDijkstra(engine *CRPRoutingEngine) Dijkstra {
	dj := Dijkstra{
		engine:              engine,
		finalDist:           make([]da.VertexInfo[da.CRPQueryKey], 0),
		numSettledNodes:     0,
		shortestTravelTimes: make([]float64, 0),
	}

	dj.Preallocate()
	return dj
}

// single-source shortest paths, from s to all other vertices
func (us *Dijkstra) ShortestPath(s da.Index) ([]float64, [][]da.OutEdge) {

	asId := us.engine.graph.GetExitOffset(s) + us.engine.graph.GetOutDegree(s) - 1
	// for iterating outEdges, we need entryOffset.
	sForwardId := us.engine.graph.GetEntryOffset(s) + da.Index(us.engine.graph.GetOutEdge(asId).GetEntryPoint())

	sVertexInfo := da.NewVertexInfo[da.CRPQueryKey](0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	djKey := da.NewDijkstraKey(s, sForwardId)
	us.pq.Insert(sForwardId, 0, sVertexInfo, djKey)

	finish := false
	for !us.pq.IsEmpty() {
		if finish {
			break
		}

		finish = us.graphSearchUni(s)
		us.numSettledNodes++
	}

	n := us.engine.graph.NumberOfVertices()

	spEdges := make([][]da.OutEdge, n)
	sps := make([]float64, n)
	for t := da.Index(0); t < da.Index(n); t++ {
		curInfo := us.finalDist[t]
		tEntryId := us.finalEdge[t]
		sp := curInfo.GetTravelTime()

		if s == t {
			continue // sp == 0
		}
		sps[t] = sp
		if sp == pkg.INF_WEIGHT {
			continue
		}

		// jadiin outEdge semua
		inEdge := us.engine.graph.GetInEdge(tEntryId)
		_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
		spEdges[t] = append(spEdges[t], *outEdge)

		for curInfo.GetParent().GetEdge() != sForwardId {
			parent := curInfo.GetParent()
			parentEdge := parent.GetEdge()
			parentCopy := parent

			// jadiin outEdge semua
			inEdge := us.engine.graph.GetInEdge(parentCopy.GetEdge())
			_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			spEdges[t] = append(spEdges[t], *outEdge)
			parentCopy.SetEdge(outEdge.GetEdgeId())

			curInfo = us.pq.Get(parentEdge)
		}

	}

	return sps, spEdges
}

func (us *Dijkstra) graphSearchUni(source da.Index) bool {

	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1
	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()

	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	if da.Eq(us.finalDist[uId].GetTravelTime(), pkg.INF_WEIGHT) {
		us.finalDist[uId] = us.pq.Get(uEntryId)
		us.finalEdge[uId] = uEntryId
	}

	uEntryPoint := uEntryId - us.engine.graph.GetEntryOffset(uId)

	// traverse outEdges of u
	us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {

		vId := outArc.GetHead()

		edgeWeight := us.engine.metrics.GetWeight(outArc)

		turnCost := us.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := us.pq.GetPriority(uEntryId) + edgeWeight + turnCost

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())

		vAlreadyLabelled := da.Lt(us.pq.GetPriority(vEntryId), pkg.INF_WEIGHT)
		if vAlreadyLabelled && da.Ge(newTravelTime, us.pq.GetPriority(vEntryId)) {
			// newTravelTime is not better, do nothing

			return
		}

		// newTravelTime is better, update the forwardInfo

		if vAlreadyLabelled {
			newPar := da.NewVertexEdgePair(uId, uEntryId, false)
			// is key already in the priority queue, decrease its key
			us.pq.DecreaseKey(vEntryId, newTravelTime, newTravelTime, newPar)

		} else if !vAlreadyLabelled {
			queryKey := da.NewDijkstraKey(vId, vEntryId)
			vertexInfo := da.NewVertexInfo[da.CRPQueryKey](newTravelTime, da.NewVertexEdgePair(uId, uEntryId, false))

			// is key not in the priority queue, insert it
			us.pq.Insert(vEntryId, newTravelTime, vertexInfo, queryKey)
		}
	})

	return false
}

func (us *Dijkstra) Preallocate() {
	numberOfEdges := us.engine.graph.NumberOfEdges()
	maxSearchSize := numberOfEdges
	numberOfVerties := us.engine.graph.NumberOfVertices()
	us.finalDist = make([]da.VertexInfo[da.CRPQueryKey], numberOfVerties)
	us.finalEdge = make([]da.Index, numberOfVerties)
	maxEdgesInCell := us.engine.graph.GetMaxEdgesInCell()
	us.pq = da.NewQueryHeap[da.CRPQueryKey](maxSearchSize, int(maxEdgesInCell), da.ARRAY_STORAGE)
	us.pq.PreallocateHeap(maxSearchSize)
}

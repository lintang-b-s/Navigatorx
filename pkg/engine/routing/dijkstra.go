package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Dijkstra struct {
	engine *CRPRoutingEngine

	finalDist           []da.VertexInfo
	finalEdge           []da.Index
	shortestTravelTimes []float64

	pq *da.QueryHeap[da.CRPQueryKey]

	numSettledNodes  int
	useReversedEdges bool
}

func NewDijkstra(engine *CRPRoutingEngine, useReversedEdges bool) Dijkstra {
	dj := Dijkstra{
		engine:              engine,
		finalDist:           make([]da.VertexInfo, 0),
		numSettledNodes:     0,
		shortestTravelTimes: make([]float64, 0),
		useReversedEdges:    useReversedEdges,
	}

	dj.Preallocate()
	return dj
}

/*
single-source shortest paths, from s to all other vertices
edge-based graph, support turn-costs

useReversedGraph = true -> buat cari sssp dari every vertices in graph to s
*/
func (us *Dijkstra) ShortestPath(s da.Index) ([]float64, [][]da.OutEdge) {
	var (
		sForwardId da.Index
	)

	if !us.useReversedEdges {
		sForwardId = us.engine.graph.GetEntryOffset(s) + us.engine.graph.GetInDegree(s) - 1
	} else {
		sForwardId = us.engine.graph.GetExitOffset(s) + us.engine.graph.GetOutDegree(s) - 1
	}

	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

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
	if !us.useReversedEdges {

		for t := da.Index(0); t < da.Index(n); t++ {
			curInfo := us.finalDist[t]
			tEntryId := us.finalEdge[t]
			sp := curInfo.GetTravelTime()

			if s == t {
				continue // sp == 0
			}
			sps[t] = sp
			if util.Ge(sp, pkg.INF_WEIGHT) {
				continue
			}

			// jadiin outEdge semua
			inEdge := us.engine.graph.GetInEdge(tEntryId)
			_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
			spEdges[t] = append(spEdges[t], *us.engine.graph.GetOutEdge(outEdge))

			for curInfo.GetParent().GetEdge() != sForwardId {
				parent := curInfo.GetParent()
				parentEdge := parent.GetEdge()

				// jadiin outEdge semua
				inEdge := us.engine.graph.GetInEdge(parentEdge)
				_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
				spEdges[t] = append(spEdges[t], *us.engine.graph.GetOutEdge(outEdge))

				curInfo = us.pq.Get(parentEdge)
			}

			util.ReverseG(spEdges[t])
		}

	} else {
		//  use reversed edges
		for t := da.Index(0); t < da.Index(n); t++ {
			curInfo := us.finalDist[t]
			tExitId := us.finalEdge[t]
			sp := curInfo.GetTravelTime()

			if s == t {
				continue // sp == 0
			}
			sps[t] = sp
			if util.Ge(sp, pkg.INF_WEIGHT) {
				continue
			}

			// jadiin outEdge semua
			outEdge := us.engine.graph.GetOutEdge(tExitId)
			spEdges[t] = append(spEdges[t], *outEdge)

			for curInfo.GetParent().GetEdge() != sForwardId {
				parent := curInfo.GetParent()
				parentEdge := parent.GetEdge()

				// jadiin outEdge semua
				outEdge := us.engine.graph.GetOutEdge(parentEdge)
				spEdges[t] = append(spEdges[t], *outEdge)

				curInfo = us.pq.Get(parentEdge)
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

func (us *Dijkstra) graphSearchUni(source da.Index) bool {

	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1
	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()

	if !us.useReversedEdges {
		uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId
		// -uEntry> u

		if util.Eq(us.finalDist[uId].GetTravelTime(), pkg.INF_WEIGHT) || util.Lt(us.pq.GetPriority(uEntryId), us.finalDist[uId].GetTravelTime()) {
			us.finalDist[uId] = us.pq.Get(uEntryId)
			us.finalEdge[uId] = uEntryId
		}

		uEntryPoint := uEntryId - us.engine.graph.GetEntryOffset(uId)

		// traverse outEdges of u
		us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(eId, head da.Index, weight, length float64, exitPoint, entryPoint da.Index, turnType pkg.TurnType,
			hwType pkg.OsmHighwayType) {

			vId := head

			edgeWeight := us.engine.metrics.GetWeight(hwType, weight, length)

			turnCost := us.engine.metrics.GetTurnCost(turnType)

			if uId == source {
				turnCost = 0
			}

			// get cost to reach v through u + turn cost from inEdge to outEdge of u
			newTravelTime := us.pq.GetPriority(uEntryId) + edgeWeight + turnCost

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(entryPoint)

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vEntryId), pkg.INF_WEIGHT)
			if vAlreadyLabelled && util.Ge(newTravelTime, us.pq.GetPriority(vEntryId)) {
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
				vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, uEntryId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vEntryId, newTravelTime, vertexInfo, queryKey)
			}
		})
	} else {
		// use reversed edges
		uExitId := uItem.GetEntryExitPoint()

		// u-uExit->
		if util.Eq(us.finalDist[uId].GetTravelTime(), pkg.INF_WEIGHT) || util.Lt(us.pq.GetPriority(uExitId), us.finalDist[uId].GetTravelTime()) {
			us.finalDist[uId] = us.pq.Get(uExitId)
			us.finalEdge[uId] = uExitId
		}

		uExitPoint := uExitId - us.engine.graph.GetExitOffset(uId)

		// traverse inEdges of u
		us.engine.graph.ForInEdgesOf(uId, uExitPoint, func(eId, tail da.Index, weight, length float64, exitPoint, entryPoint da.Index,
			turnType pkg.TurnType, hwType pkg.OsmHighwayType) {

			vId := tail

			edgeWeight := us.engine.metrics.GetWeight(hwType, weight, length)

			turnCost := us.engine.metrics.GetTurnCost(turnType)

			if uId == source {
				turnCost = 0
			}

			newTravelTime := us.pq.GetPriority(uExitId) + edgeWeight + turnCost

			if util.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			vExitId := us.engine.graph.GetExitOffset(vId) + da.Index(exitPoint)

			vAlreadyLabelled := util.Lt(us.pq.GetPriority(vExitId), pkg.INF_WEIGHT)
			if vAlreadyLabelled && util.Ge(newTravelTime, us.pq.GetPriority(vExitId)) {
				// newTravelTime is not better, do nothing

				return
			}

			// newTravelTime is better, update the forwardInfo

			if vAlreadyLabelled {
				newPar := da.NewVertexEdgePair(uId, uExitId, false)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(vExitId, newTravelTime, newTravelTime, newPar)

			} else if !vAlreadyLabelled {
				queryKey := da.NewDijkstraKey(vId, vExitId)
				vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, uExitId, false))

				// is key not in the priority queue, insert it
				us.pq.Insert(vExitId, newTravelTime, vertexInfo, queryKey)
			}
		})
	}

	return false
}

func (us *Dijkstra) Preallocate() {
	numberOfEdges := us.engine.graph.NumberOfEdges()
	maxSearchSize := numberOfEdges
	numberOfVerties := us.engine.graph.NumberOfVertices()
	us.finalDist = make([]da.VertexInfo, numberOfVerties)
	for i := 0; i < numberOfVerties; i++ {
		us.finalDist[i] = da.NewVertexInfo(pkg.INF_WEIGHT, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	}
	us.finalEdge = make([]da.Index, numberOfVerties)
	maxEdgesInCell := us.engine.graph.GetMaxEdgesInCell()
	us.pq = da.NewQueryHeap[da.CRPQueryKey](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
	us.pq.PreallocateHeap(maxSearchSize)
}

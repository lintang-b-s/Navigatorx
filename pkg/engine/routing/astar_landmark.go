package routing

import (
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
)

type AstarLandmark struct {
	engine *CRPRoutingEngine

	forwardInfo         map[da.Index]VertexInfo
	shortestTimeTravels []float64

	pq *da.MinHeap[da.CRPQueryKey]

	lm *landmark.Landmark

	targetEntryId da.Index

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numSettledNodes int
}

func NewAstarLandmark(engine *CRPRoutingEngine, lm *landmark.Landmark) AstarLandmark {
	return AstarLandmark{
		engine:              engine,
		forwardInfo:         make(map[da.Index]VertexInfo),
		pq:                  da.NewFourAryHeap[da.CRPQueryKey](),
		numSettledNodes:     0,
		shortestTimeTravels: make([]float64, 0),
		lm:                  lm,
	}
}

func (us *AstarLandmark) ShortestPath(asId, atId da.Index) float64 {

	s := us.engine.graph.GetOutEdge(asId).GetHead()

	// for iterating outEdges, we need entryOffset.
	sForwardId := us.engine.graph.GetEntryOffset(s) + da.Index(us.engine.graph.GetOutEdge(asId).GetEntryPoint())

	t := us.engine.graph.GetInEdge(atId).GetTail()

	us.forwardInfo[sForwardId] = NewVertexInfo(0, newVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	us.pq.Insert(da.NewPriorityQueueNode(0, da.NewDijkstraKey(s, sForwardId)))

	finish := false
	for !us.pq.IsEmpty() {
		if finish {
			break
		}
		// search on graph level 1
		finish = us.graphSearchUni(s, t)
		us.numSettledNodes++
	}

	curInfo := us.forwardInfo[us.targetEntryId]
	inEdge := us.engine.graph.GetInEdge(us.targetEntryId)
	_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
	sp := 0.0
	sp += us.engine.metrics.GetWeight(outEdge, 0)

	for curInfo.GetParent().edge != sForwardId {
		parent := curInfo.GetParent()
		parentEdge := parent.getEdge()
		parentCopy := parent

		// jadiin outEdge semua
		inEdge := us.engine.graph.GetInEdge(parentCopy.getEdge())
		_, outEdge := us.engine.graph.GetHeadOfInedgeWithOutEdge(inEdge.GetEdgeId())
		parentCopy.setEdge(outEdge.GetEdgeId())
		parentCopy.setisOutEdge(true)

		sp += us.engine.metrics.GetWeight(outEdge, 0)

		curInfo = us.forwardInfo[parentEdge]
	}

	return sp
}

func (us *AstarLandmark) graphSearchUni(source, target da.Index) bool {

	//The query algorithm maintains a distance label d(u) for each entry u which can either be a vertex on the overlay or a pair (u, i) corresponding to the i-th entry point of u in the original graph.
	// for forward search, we traverse outEdges of the graph and store (u, entryPoint of outEdge) to represent the key of the priority queue.
	// we need to store entryPoint because we need to know turnType & turn cost when traversing from inEdge to outEdge of vertex u.
	// forward search  on graph level 1
	queryKey, _ := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()

	if uId == target {
		us.targetEntryId = uItem.GetEntryExitPoint()
		return true
	}

	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	uEntryPoint := uEntryId - us.engine.graph.GetEntryOffset(uId)

	// traverse outEdges of u
	us.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {

		vId := outArc.GetHead()
		if vId == uId {
			return
		}

		edgeWeight := us.engine.metrics.GetWeight(outArc, 0)

		turnCost := us.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}

		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
		lb := us.lm.FindTighestLowerBound(vId, target)

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := us.forwardInfo[uEntryId].GetTravelTime() + edgeWeight + turnCost

		if newTravelTime >= pkg.INF_WEIGHT {
			return
		}

		vEntryId := us.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())

		_, vAlreadyVisited := us.forwardInfo[vEntryId]
		if vAlreadyVisited && newTravelTime >= us.forwardInfo[vEntryId].GetTravelTime() {
			// newTravelTime is not better, do nothing

			return
		}

		// newTravelTime is better, update the forwardInfo
		us.forwardInfo[vEntryId] = NewVertexInfo(newTravelTime, newVertexEdgePair(uId, uEntryId, false))

		priority := newTravelTime + lb
		if vAlreadyVisited {
			// is key already in the priority queue, decrease its key
			us.pq.DecreaseKey(da.NewPriorityQueueNode(
				priority, da.NewDijkstraKey(vId, vEntryId)),
			)
		} else if !vAlreadyVisited {
			// is key not in the priority queue, insert it
			us.pq.Insert(da.NewPriorityQueueNode(
				priority, da.NewDijkstraKey(vId, vEntryId)),
			)
		}

	})

	return false
}

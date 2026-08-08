package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type ALTP2P[W util.RoutingNumber] struct {
	engine *CRPRoutingEngine[W]

	pq              *da.QueryHeap[da.CRPQueryKey, W]
	activeLandmarks []da.Index
	runtime         int64
	numSettledNodes int
}

func NewALTP2P[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) ALTP2P[W] {
	dj := ALTP2P[W]{
		engine:          engine,
		numSettledNodes: 0,
	}

	dj.Preallocate()
	return dj
}

func (us *ALTP2P[W]) ShortestPath(s, t da.Index) (W, []da.Index) {

	us.activeLandmarks = us.engine.lm.SelectBestQueryLandmarks(s, t)
	sVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	now := time.Now()

	djKey := da.NewDijkstraKey(s, s)
	us.pq.Insert(s, 0, sVertexInfo, djKey)

	for !us.pq.IsEmpty() {
		finish := us.graphSearchUni(s, t)
		us.numSettledNodes++
		if finish {
			break
		}
	}

	sp, spPath := us.constructShortestPath(s, t)

	dur := time.Since(now).Milliseconds()
	us.runtime = dur

	return sp, spPath
}

func (us *ALTP2P[W]) graphSearchUni(source, target da.Index) bool {

	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()
	if uId == target {
		return true
	}

	// traverse outEdges of u
	us.engine.graph.ForOutEdgeIdsOf(uId, func(eId da.Index) {
		head := us.engine.graph.GetHeadOfOutEdge(eId)
		vId := head
		edgeWeight := us.engine.getWeight(eId, true)
		// get cost to reach v through u
		newTravelTime := us.pq.GetPriority(uId) + edgeWeight

		if util.Ge(newTravelTime, util.Infinity[W]()) {
			return
		}

		vAlreadyLabelled := util.Lt(us.pq.GetPriority(vId), util.Infinity[W]())
		if vAlreadyLabelled && util.Ge(newTravelTime, us.pq.GetPriority(vId)) {
			// newTravelTime is not better, do nothing
			return
		}

		pfv := us.engine.lm.FindTighestLowerBound(vId, target, us.activeLandmarks)
		priority := newTravelTime + pfv

		// newTravelTime is better, update the forwardInfo
		if vAlreadyLabelled {
			newPar := da.NewVertexEdgePair(uId, eId, false)
			// is key already in the priority queue, decrease its key
			us.pq.DecreaseKey(vId, priority, newTravelTime, newPar)
		} else if !vAlreadyLabelled {
			queryKey := da.NewDijkstraKey(vId, vId)
			vertexInfo := da.NewVertexInfo(newTravelTime, da.NewVertexEdgePair(uId, eId, false))
			// is key not in the priority queue, insert it
			us.pq.Insert(vId, priority, vertexInfo, queryKey)
		}
	})

	return false
}

func (us *ALTP2P[W]) Preallocate() {
	numberOfVerties := us.engine.graph.NumberOfVertices()
	maxSearchSize := numberOfVerties

	maxEdgesInCell := us.engine.graph.GetMaxEdgesInCell()
	us.pq = da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
	us.pq.PreallocateHeap(maxSearchSize)
}

func (us *ALTP2P[W]) constructShortestPath(s, t da.Index) (W, []da.Index) {
	spPath := make([]da.Index, 0)

	sp := us.pq.GetPriority(t)

	if s == t || sp == util.Infinity[W]() {
		return sp, spPath
	}

	curInfo := us.pq.Get(t)
	spPath = append(spPath, t)

	for curInfo.GetParent().GetVertex() != s {
		parent := curInfo.GetParent()

		spPath = append(spPath, parent.GetVertex())

		curInfo = us.pq.Get(parent.GetVertex())
	}
	spPath = append(spPath, s)

	util.ReverseG(spPath)
	return sp, spPath
}

func (us *ALTP2P[W]) GetStats(n int) (float64, int, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(us.numSettledNodes)
	return efficiency, us.numSettledNodes, us.runtime
}

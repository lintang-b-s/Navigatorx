package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type DijkstraP2P[W util.RoutingNumber] struct {
	engine *CRPRoutingEngine[W]

	pq              *da.QueryHeap[da.CRPQueryKey, W]
	runtime         int64
	numSettledNodes int
}

func NewDijkstraP2P[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) DijkstraP2P[W] {
	dj := DijkstraP2P[W]{
		engine:          engine,
		numSettledNodes: 0,
	}

	dj.Preallocate()
	return dj
}

/*
point-to-point shortest path.
with no turn costs.
ini implementasi dijkstra point-to-point shortest path (p2psp)
*/
func (us *DijkstraP2P[W]) ShortestPath(s, t da.Index) (W, []da.Index) {

	sVertexData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))

	now := time.Now()

	djKey := da.NewDijkstraKey(s, s)
	us.pq.Insert(s, 0, sVertexData, djKey)

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

func (us *DijkstraP2P[W]) graphSearchUni(source, target da.Index) bool {

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
		eWeight := us.engine.getWeight(eId, true)
		// get cost to reach v through u
		newTT := us.pq.GetPriority(uId) + eWeight

		if util.Ge(newTT, util.Infinity[W]()) {
			return
		}

		vLabelled := util.Lt(us.pq.GetPriority(vId), util.Infinity[W]())
		if vLabelled && util.Ge(newTT, us.pq.GetPriority(vId)) {
			// newTT is not better, do nothing
			return
		}

		// newTT is better, update the forwardData
		if vLabelled {
			newPar := da.NewVertexEdgePair(uId, eId, false)
			// is key already in the priority queue, decrease its key
			us.pq.DecreaseKey(vId, newTT, newTT, newPar)
		} else if !vLabelled {
			queryKey := da.NewDijkstraKey(vId, vId)
			vData := da.NewVertexData(newTT, da.NewVertexEdgePair(uId, eId, false))
			// is key not in the priority queue, insert it
			us.pq.Insert(vId, newTT, vData, queryKey)
		}
	})

	return false
}

func (us *DijkstraP2P[W]) Preallocate() {
	numberOfVerties := us.engine.graph.NumberOfVertices()
	maxSearchSize := numberOfVerties

	maxEdgesInCell := us.engine.graph.GetMaxEdgesInCell()
	us.pq = da.NewQueryHeap[da.CRPQueryKey, W](uint32(maxSearchSize), uint32(maxEdgesInCell), da.ARRAY_STORAGE, true)
	us.pq.PreallocateHeap(maxSearchSize)
}

func (us *DijkstraP2P[W]) constructShortestPath(s, t da.Index) (W, []da.Index) {
	spPath := make([]da.Index, 0)

	sp := us.pq.GetPriority(t)

	if s == t || sp == util.Infinity[W]() {
		return sp, spPath
	}

	vData := us.pq.Get(t)
	spPath = append(spPath, t)

	for vData.GetParent().GetVertex() != s {
		parent := vData.GetParent()

		spPath = append(spPath, parent.GetVertex())

		vData = us.pq.Get(parent.GetVertex())
	}
	spPath = append(spPath, s)

	util.ReverseG(spPath)
	return sp, spPath
}

func (us *DijkstraP2P[W]) GetStats(n int) (float64, int, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(us.numSettledNodes)
	return efficiency, us.numSettledNodes, us.runtime
}

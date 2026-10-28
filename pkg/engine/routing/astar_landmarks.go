package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// ALTP2P ALT (A*, Landmarks, and Triangle Inequality) Algorithm.
type ALTP2P[W util.Numeric] struct {
	engine          *CRPRoutingEngine[W]
	metrics         *metrics.Metric[W]
	lm              *landmark.Landmark[W]
	pq              *da.QueryHeap[da.QueryKey, W]
	activeLandmarks []da.Index
	runtime         int64
	numSettledNodes int
}

// NewALTP2P creates and returns altp2p.
func NewALTP2P[W util.Numeric](
	engine *CRPRoutingEngine[W],
) ALTP2P[W] {
	dj := ALTP2P[W]{
		engine:          engine,
		numSettledNodes: 0,
	}
	dj.metrics = engine.GetMetrics()
	dj.lm = engine.GetLandmark()
	dj.Preallocate()
	return dj
}

// ShortestPath compute point-to-point shortest path
func (us *ALTP2P[W]) ShortestPath(s, t da.Index) (W, []da.Index) {

	us.activeLandmarks = us.lm.SelectBestQueryLandmarks(s, t)
	sVertexData := da.NewVData(W(0), da.NewParentVertex(da.InvalidVertexId, false))

	now := time.Now()

	djKey := da.NewDijkstraKey(s)
	us.pq.Insert(s, 0, sVertexData, djKey)

	for !us.pq.IsEmpty() {
		finish := us.graphSearchUni(t)
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

func (us *ALTP2P[W]) graphSearchUni(target da.Index) bool {

	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()
	if uId == target {
		return true
	}

	// traverse outEdges of u
	us.engine.graph.ForOutEdgeIdsOf(uId, func(eId da.Index) {
		head := us.engine.graph.GetHead(eId)
		vId := head
		eWeight := us.metrics.GetWeight(eId)
		// get cost to reach v through u
		newVCost := us.pq.Cost(uId) + eWeight

		ovCost := us.pq.Cost(vId)
		if util.Ge(newVCost, ovCost) {
			// newVCost is not better, do nothing
			return
		}

		vLabelled := util.Lt(ovCost, util.Infinity[W]())
		hv := us.lm.ComputeTighestLowerBound(vId, target, us.activeLandmarks)
		priority := newVCost + hv
		vPar := da.NewParentVertex(uId, false)

		// newVCost is better, update the labelled vertex data
		if vLabelled {
			// is key already in the priority queue, decrease its key
			us.pq.DecreaseKey(vId, priority, newVCost, vPar)
		} else if !vLabelled {
			queryKey := da.NewDijkstraKey(vId)
			vData := da.NewVData(newVCost, vPar)
			// is key not in the priority queue, insert it
			us.pq.Insert(vId, priority, vData, queryKey)
		}
	})

	return false
}

// Preallocate handles preallocate priority queue.
func (us *ALTP2P[W]) Preallocate() {
	numberOfVerties := us.engine.graph.NumberOfVertices()
	maxSearchSize := numberOfVerties

	us.pq = da.NewQueryHeap[da.QueryKey, W](da.Index(maxSearchSize), da.ArrayStorageMode)
	us.pq.PreallocateHeap(maxSearchSize)
}

func (us *ALTP2P[W]) constructShortestPath(s, t da.Index) (W, []da.Index) {
	spPath := make([]da.Index, 0)

	sp := us.pq.Cost(t)

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

// GetStats returns stats.
func (us *ALTP2P[W]) GetStats(n int) (float64, int, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(us.numSettledNodes)
	return efficiency, us.numSettledNodes, us.runtime
}

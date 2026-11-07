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
	pq              *da.QueryHeap[da.Index, W]
	activeLandmarks [2]da.Index
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
	now := time.Now()

	us.activeLandmarks = us.lm.SelectBestQueryLandmarks(s, t)
	sVertexData := da.NewVData(W(0), s, da.InvalidVertexId)
	us.pq.Insert(s, 0, sVertexData)

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

	u, uCost := us.pq.ExtractMin()
	if u == target {
		return true
	}

	// traverse outEdges of u
	us.engine.graph.ForOutEdgeIdsOf(u, func(eId da.Index) {
		head := us.engine.graph.GetHead(eId)
		v := head
		eWeight := us.metrics.GetWeight(eId)
		// get cost to reach v through u
		newVCost := uCost + eWeight

		ovCost := us.pq.Cost(v)
		if util.Ge(newVCost, ovCost) {
			// newVCost is not better, do nothing
			return
		}
		vLabelled := util.Lt(ovCost, util.Infinity[W]())
		hv := us.lm.ComputeTighestLowerBound(v, target, us.activeLandmarks)
		priority := newVCost + hv
		// newVCost is better, update the labelled vertex data
		if vLabelled {
			// is key already in the priority queue, decrease its key
			us.pq.DecreaseKey(v, priority, newVCost, u)
		} else if !vLabelled {
			vData := da.NewVData(newVCost, v, u)
			// is key not in the priority queue, insert it
			us.pq.Insert(v, priority, vData)
		}
	})

	return false
}

// Preallocate handles preallocate priority queue.
func (us *ALTP2P[W]) Preallocate() {
	numberOfVerties := us.engine.graph.NumberOfVertices()
	maxSearchSize := numberOfVerties

	us.pq = da.NewQueryHeap[da.Index, W](da.Index(maxSearchSize), da.Index(maxSearchSize), da.ArrayStorageMode)
}

func (us *ALTP2P[W]) constructShortestPath(s, t da.Index) (W, []da.Index) {
	spPath := make([]da.Index, 0)

	sp := us.pq.Cost(t)

	if s == t || sp == util.Infinity[W]() {
		return sp, spPath
	}

	vData := us.pq.Get(t)
	spPath = append(spPath, t)

	for vData.GetParent() != s {
		par := vData.GetParent()
		spPath = append(spPath, par)
		vData = us.pq.Get(par)
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

package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// DijkstraP2P represents dijkstra point-to-point shortest path algorithm.
type DijkstraP2P[W util.Numeric] struct {
	engine          *CRPRoutingEngine[W]
	metrics         *met.Metric[W]
	pq              *da.QueryHeap[da.Index, W]
	runtime         int64
	numSettledNodes int
}

// NewDijkstraP2P creates and returns dijkstrap2p.
func NewDijkstraP2P[W util.Numeric](
	engine *CRPRoutingEngine[W],
) DijkstraP2P[W] {
	dj := DijkstraP2P[W]{
		engine:          engine,
		numSettledNodes: 0,
	}
	dj.metrics = engine.GetMetrics()
	dj.Preallocate()
	return dj
}

// ShortestPath compute point-to-point fastest path
// ini implementasi dijkstra point-to-point shortest path (p2psp)
func (us *DijkstraP2P[W]) ShortestPath(s, t da.Index) (W, []da.Index) {

	sVertexData := da.NewVData(W(0), s, da.InvalidVertexId)
	now := time.Now()
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

func (us *DijkstraP2P[W]) graphSearchUni(t da.Index) bool {

	u, uCost := us.pq.ExtractMin()

	if u == t {
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
		if util.Lt(newVCost, ovCost) {
			vLabelled := util.Lt(ovCost, util.Infinity[W]())
			// newVCost is better, update the v cost

			if vLabelled {
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(v, newVCost, newVCost, u)
			} else if !vLabelled {
				vData := da.NewVData(newVCost, v, u)
				// is key not in the priority queue, insert it
				us.pq.Insert(v, newVCost, vData)
			}
		}
	})

	return false
}

// Preallocate handles preallocate priority queue.
func (us *DijkstraP2P[W]) Preallocate() {
	numberOfVerties := us.engine.graph.NumberOfVertices()
	maxSearchSize := numberOfVerties

	us.pq = da.NewQueryHeap[da.Index, W](da.Index(maxSearchSize), da.Index(maxSearchSize), da.ArrayStorageMode)
}

func (us *DijkstraP2P[W]) constructShortestPath(s, t da.Index) (W, []da.Index) {
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
func (us *DijkstraP2P[W]) GetStats(n int) (float64, int, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(us.numSettledNodes)
	return efficiency, us.numSettledNodes, us.runtime
}

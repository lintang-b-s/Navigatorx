package landmark

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// Dijkstra represents dijkstra algorithm.
type Dijkstra[W util.Numeric] struct {
	graph *da.Graph
	cf    *met.TimeFunction[W]

	inf             W
	useReverseGraph bool

	numExploredNodes int
}

// NewDijkstra creates and returns dijkstra.
func NewDijkstra[W util.Numeric](graph *da.Graph, cf *met.TimeFunction[W], useReverseGraph bool) *Dijkstra[W] {
	dj := &Dijkstra[W]{
		graph:            graph,
		useReverseGraph:  useReverseGraph,
		numExploredNodes: 0,
		inf:              util.Infinity[W](),

		cf: cf,
	}

	return dj
}

// todo: ini kayake bisa dioptimize pakai radix-heap kalau inputnya osm.pbf karena kita pakai int32 buat weightnya routing engine, weight (duration) in centiseconds .
// pas di cek pprof profile dari cmd/customizer, ini sama SelectLandmarks() yang bikin lama
// baca O(m + n log C) dijkstra radix heap Ravindra K. Ahuja, Kurt Mehlhorn, James Orlin, and Robert E. Tarjan. Faster algorithms for the shortest path problem. J. ACM 37, 2 (April 1990), 213-223.
// referensi implementasi radix-heap https://github.com/iwiwi/radix-heap/tree/master dan benchmark dijkstra radix heap di dimacs road network graph data: https://github.com/iwiwi/radix-heap/tree/master/example
// di benchmark nya repo takuya akiba diatas dijkstra radix heap di road network dimacs sedikit lebih cepet
// mungkin di iterasi edges (sekalian weightnya) of vertex u nya bisa pakai https://pkg.go.dev/iter ? harus dibenchmark mana yang lebih cepet
// di table 3 https://sidsen.azurewebsites.net//papers/heaps-alenex14.pdf  gak bandingin sama radix heap sama sekali
// radix heap is faster than 4-ary heap confirmed by this latest experiment https://github.com/RagnarGrootKoerkamp/quickheap

/*
ShortestPath run dijkstara single source shortest path algorithm.
useReversedGraph = true -> buat cari sssp dari every vertices in graph to s

Cormen, T.H. et al. (2022) Introduction to Algorithms. 4th ed. Cambridge, MA, USA:
MIT Press (CLRS).:
Single-destination shortest-paths problem: Find a shortest path to a given des-
tination vertex t from each vertex v. By reversing the direction of each edge in
the graph, we can reduce this problem to a single-source problem..

buat precalculated landmark sp distances (untuk potential/heuristic ALT (A*, landmarks, and triangle inequality) algorithm)
*/
func (us *Dijkstra[W]) ShortestPath(s da.Index, pq *da.QueryHeap[da.Index, W], dist []W) {
	out := !us.useReverseGraph
	noPar := da.InvalidSegmentId
	vData := da.NewVData[da.Index, W](0, s, noPar)
	dist[s] = 0
	pq.Insert(s, 0, vData)
	for !pq.IsEmpty() {
		u, uCost := pq.ExtractMin()

		offset, eNum := us.graph.NumEdges(u, out)
		for eId := offset; eId < offset+eNum; eId++ {
			// get cost to reach v through u
			v, eWeight := us.getV(eId, out)
			newVCost := uCost + eWeight
			ovCost := dist[v]
			if util.Lt(newVCost, ovCost) {
				vData := da.NewVData[da.Index, W](newVCost, v, noPar)
				vLabelled := util.Lt(ovCost, us.inf)
				if vLabelled {
					pq.DecreaseKey(v, newVCost, newVCost, noPar)
				} else {
					pq.Insert(v, newVCost, vData)
				}
				dist[v] = newVCost
			}
		}
		us.numExploredNodes++
	}
}

func (us *Dijkstra[W]) getV(eId da.Index, out bool) (da.Index, W) {
	var v da.Index
	var weight W

	if out {
		v = us.graph.GetHead(eId)
		weight = us.cf.Weight(eId)
	} else {
		v = us.graph.GetTail(eId)
		reId := us.graph.GetOutId(eId)
		weight = us.cf.Weight(reId)
	}
	return v, weight
}

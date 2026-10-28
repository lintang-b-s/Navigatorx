package landmark

import (
	"sync"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// Dijkstra represents dijkstra algorithm.
type Dijkstra[W util.Numeric] struct {
	graph *da.Graph
	cf    *met.TimeFunction[W]

	pq *da.QueryHeap[da.QueryKey, W]

	useReverseGraph bool

	numSettledNodes int
}

// NewDijkstra creates and returns dijkstra.
func NewDijkstra[W util.Numeric](graph *da.Graph, cf *met.TimeFunction[W], useReverseGraph bool) *Dijkstra[W] {
	dj := &Dijkstra[W]{
		graph:           graph,
		useReverseGraph: useReverseGraph,
		numSettledNodes: 0,

		cf: cf,
	}

	return dj
}

/*
ShortestPath run dijkstara single source shortest path algorithm.
useReversedGraph = true -> buat cari sssp dari every vertices in graph to s

Cormen, T.H. et al. (2022) Introduction to Algorithms. 4th ed. Cambridge, MA, USA:
MIT Press (CLRS).:
Single-destination shortest-paths problem: Find a shortest path to a given des-
tination vertex t from each vertex v. By reversing the direction of each edge in
the graph, we can reduce this problem to a single-source problem..

buat precalculated landmark sp distances (untuk potential/heuristic ALT (A*, landmarks, and triangle inequality) algorithm)
kita gak perlu pakai turn costs karena kalau misal pake turn costs di fase query Customizable Route Planning (CRP), potential/heuristic nya masih underestimate true sp dist dan masih memenuhi sifat konsisten/feasible...
*/
func (us *Dijkstra[W]) ShortestPath(s da.Index, heapPool *sync.Pool) []W {
	sps := make([]W, us.graph.NumberOfVertices())
	us.ShortestPathInto(s, heapPool, sps, 1)
	return sps
}

// ShortestPathInto writes shortest path costs into dst with the given stride.
func (us *Dijkstra[W]) ShortestPathInto(s da.Index, heapPool *sync.Pool, dst []W, stride int) {
	us.pq = heapPool.Get().(*da.QueryHeap[da.QueryKey, W])
	us.pq.Clear()

	done := func() {
		heapPool.Put(us.pq)
	}
	defer done()

	noPar := da.NewParentVertex(da.InvalidVertexId, false)

	djKey := da.NewDijkstraKey(s)
	us.pq.Insert(s, 0, da.NewVData(W(0),
		noPar), djKey)

	for !us.pq.IsEmpty() {
		// search on graph level 1
		us.graphSearchUni()
		us.numSettledNodes++
	}

	us.constructShortestPath(s, dst, stride)
}

func (us *Dijkstra[W]) graphSearchUni() {
	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	uId := uItem.GetNode()

	out := !us.useReverseGraph
	offset, eNum := us.graph.NumEdges(uId, out)
	for eId := offset; eId < offset+eNum; eId++ {
		v := us.GetV(eId, out)
		eWeight := us.GetWeight(eId, out)

		// get cost to reach v through u
		newVCost := us.pq.Cost(uId) + eWeight

		ovCost := us.pq.Cost(v)
		vLabelled := util.Lt(ovCost, util.Infinity[W]())
		if util.Lt(newVCost, ovCost) {
			// newVCost is better, update the vertex v cost in pq
			vPar := da.NewParentVertex(uId, false)

			if vLabelled {
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(v, newVCost, newVCost, vPar)
			} else {
				queryKey := da.NewDijkstraKey(v)
				vData := da.NewVData(newVCost, vPar)
				// is key not in the priority queue, insert it
				us.pq.Insert(v, newVCost, vData, queryKey)
			}
		}
	}
}

// GetV returns the adjacent vertex Id and edge Id for the given offset and index.
func (us *Dijkstra[W]) GetV(eId da.Index, out bool) da.Index {

	if out {
		return us.graph.GetHead(eId)
	}
	return us.graph.GetTail(eId)
}

// GetWeight returns the edge weight according to search direction.
func (us *Dijkstra[W]) GetWeight(eId da.Index, out bool) W {
	if out {
		return us.cf.GetWeight(eId)
	}
	reId := us.graph.GetOutId(eId)
	return us.cf.GetWeight(reId)
}

func (us *Dijkstra[W]) constructShortestPath(s da.Index, sps []W, stride int) {
	n := us.graph.NumberOfVertices()
	for t := da.Index(0); t < da.Index(n); t++ {
		sp := us.pq.Cost(t)

		if s == t {
			sps[int(t)*stride] = 0
			continue
		}
		sps[int(t)*stride] = sp
	}
}

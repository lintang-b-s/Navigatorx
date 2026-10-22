package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type Dijkstra[W util.Numeric] struct {
	engine  *CRPRoutingEngine[W]
	metrics *met.Metric[W]

	pq *da.QueryHeap[da.QueryKey, W]

	numSettledNodes  int
	useReversedEdges bool
}

func NewDijkstra[W util.Numeric](
	engine *CRPRoutingEngine[W],
	useReversedEdges bool,
) Dijkstra[W] {
	dj := Dijkstra[W]{
		engine:           engine,
		numSettledNodes:  0,
		useReversedEdges: useReversedEdges,
	}
	dj.metrics = engine.GetMetrics()
	dj.Preallocate()
	return dj
}

/*
single-source shortest paths, from s to all other vertices.
with no turn costs.
ini implementasi dijkstra yang biasa anda lihat di internet / competitive programming template.

useReversedEdges = true -> buat cari sssp dari every vertices in graph to s
*/
func (us *Dijkstra[W]) ShortestPath(s da.Index) ([]W, [][]da.Index) {

	sVertexData := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID))

	djKey := da.NewDijkstraKey(s)
	us.pq.Insert(s, 0, sVertexData, djKey)

	for !us.pq.IsEmpty() {

		us.graphSearchUni(s)
		us.numSettledNodes++
	}

	sps, spPath := us.constructShortestPath(s)

	return sps, spPath
}

func (us *Dijkstra[W]) graphSearchUni(source da.Index) {

	queryKey := us.pq.ExtractMin()
	uItem := queryKey.GetItem()
	u := uItem.GetNode()
	out := !us.useReversedEdges
	offset, eNum := us.engine.graph.NumEdges(u, out)
	for eId := offset; eId < offset+eNum; eId++ {
		v := us.engine.GetV(eId, out)
		edgeWeight := us.GetWeight(eId, out)
		// get cost to reach v through u
		newVCost := us.pq.GetCost(u) + edgeWeight

		oldCost := us.pq.GetCost(v)
		vLabelled := util.Lt(oldCost, util.Infinity[W]())
		if util.Lt(newVCost, oldCost) {
			// newVCost is better, update the v cost
			if vLabelled {
				newPar := da.NewParentVertex(u)
				// is key already in the priority queue, decrease its key
				us.pq.DecreaseKey(v, newVCost, newVCost, newPar)
			} else if !vLabelled {
				queryKey := da.NewDijkstraKey(v)
				vData := da.NewVData(newVCost, da.NewParentVertex(u))
				// is key not in the priority queue, insert it
				us.pq.Insert(v, newVCost, vData, queryKey)
			}
		}
	}
}

func (us *Dijkstra[W]) Preallocate() {
	numberOfVerties := us.engine.graph.NumberOfVertices()
	maxSearchSize := numberOfVerties
	us.pq = da.NewQueryHeap[da.QueryKey, W](da.Index(maxSearchSize), da.ARRAY_STORAGE)
	us.pq.PreallocateHeap(maxSearchSize)
}

func (us *Dijkstra[W]) GetWeight(eId da.Index, out bool) W {
	if out {
		return us.metrics.GetWeight(eId)
	}
	reId := us.engine.graph.GetOutId(eId)
	return us.metrics.GetWeight(reId)
}

func (us *Dijkstra[W]) constructShortestPath(s da.Index) ([]W, [][]da.Index) {
	n := us.engine.graph.NumberOfVertices()
	spPath := make([][]da.Index, n)
	sps := make([]W, n)
	if !us.useReversedEdges {

		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetCost(t)

			if s == t {
				continue // sp == 0
			}

			sps[t] = sp
			if util.Ge(sp, util.Infinity[W]()) {
				continue
			}

			vData := us.pq.Get(t)

			for vData.GetParent().GetVertex() != s {
				parent := vData.GetParent()

				spPath[t] = append(spPath[t], parent.GetVertex())

				vData = us.pq.Get(parent.GetVertex())
			}

			util.ReverseG(spPath[t])
		}

	} else {
		//  use reversed edges
		for t := da.Index(0); t < da.Index(n); t++ {
			sp := us.pq.GetCost(t)

			if s == t {
				continue // sp == 0
			}
			sps[t] = sp
			if util.Ge(sp, util.Infinity[W]()) {
				continue
			}

			vData := us.pq.Get(t)

			for vData.GetParent().GetVertex() != s {
				parent := vData.GetParent()

				// jadiin outEdge semua
				spPath[t] = append(spPath[t], parent.GetVertex())

				vData = us.pq.Get(parent.GetVertex())
			}
		}

		// kita mau cari
		// dari v -> .... -> landmark, for all vertices v
		// karena pakai reversed edges: landmark -> .... -> v ,
		// setiap reversed edge (v,u) punya weight sama dengan weight edge (u,v).
		// shortest path dari landmark ke v pakai reversed edges equivalent to shortest path dari v ke landmark pakai original edges
		// kita dapet spPath berupa list of outEdges dari v ,... ke landmark
		// gak perlu direverse
	}
	return sps, spPath
}

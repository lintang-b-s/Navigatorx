package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPQuery[W util.Numeric] struct {
	engine               *CRPRoutingEngine[W]
	metrics              *met.Metric[W]
	lm                   *landmark.Landmark[W]
	shortestCost         W
	mid                  da.ParentVertex
	fpq                  *da.QueryHeap[da.QueryKey, W]
	bpq                  *da.QueryHeap[da.QueryKey, W]
	sCellNumber          da.Pv
	tCellNumber          da.Pv
	viaVertices          []*ViaVertex
	numExploredVertices  int
	runtime              int64
	pathUnpackingRuntime int64
	ub                   float64

	maxSearchRad W
	mapmatching  bool

	forAlternatives bool
	reroute         bool
}

func NewCRPQuery[W util.Numeric](
	engine *CRPRoutingEngine[W],
) *CRPQuery[W] {
	crpQuery := &CRPQuery[W]{
		engine:               engine,
		mid:                  da.NewParentVertex(0),
		viaVertices:          make([]*ViaVertex, 0, 5),
		numExploredVertices:  0,
		runtime:              0,
		pathUnpackingRuntime: 0,
		ub:                   1.0,
		maxSearchRad:         2 * util.Infinity[W](),
	}
	crpQuery.metrics = engine.GetMetrics()
	crpQuery.lm = engine.GetLandmark()
	crpQuery.Preallocate()
	return crpQuery
}

/*
references:
1. query phase: Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.
2. Sungwon Jung and S. Pramanik, "An efficient path computation model for hierarchically structured topographical road maps," in IEEE Transactions on Knowledge and Data Engineering, vol. 14, no. 5, pp. 1029-1046, Sept.-Oct. 2002, doi: 10.1109/TKDE.2002.1033772.
keywords: {Computational modeling;Roads;Navigation;Computational efficiency;Performance analysis;Shortest path problem;Concurrent computing;Cost function;Automobiles;Space exploration},
3. Haeupler, B. et al. (2025) “Bidirectional Dijkstra's Algorithm is Instance-Optimal,” in 2025 Symposium on Simplicity in Algorithms (SOSA). Society for Industrial and Applied Mathematics (Proceedings), pp. 202–215. Available at: https://doi.org/10.1137/1.9781611978315.16.
4. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
5. Towers, M. (2020). Bidirectional Dijkstra. https://www.homepages.ucl.ac.uk/~ucahmto/math/2020/05/30/bidirectional-dijkstra.html. Diakses tanggal: 5 Agustus 2026.
6. Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable Route Planning in Road Networks. Transportation Science, No. 2, Volume 51, pages 566-591.

ini adalah implementasi dari fase query dari Customizable Route Planning (CRP) [1, 6] (or sometimes called as Multi-Level Dijkstra like in https://github.com/Project-OSRM/osrm-backend).
read section 5.3 Query: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf dan read section 3 - Basic Algorithm in ref[1].
intinya cuma bidirectional dijkstra pada graf yang consisiting of overlay graph H, cell C_s, cell C_t. C_s adalah cell level 1 yang mengandung vertex s hasil multilevel partition (lihat package partitioner).
Setiap cell C_v memiliki vertices (all inside cell C_v) dan edges (semua endpoints nya inside C_v), vertices dan edges dari cell C_v adalah subset dari vertices dan edges dari graf G.
overlay graph H adalah graf yang mengandung all boundary/overlay vertices, all cut/boundary edges, all shortcut edges di setiap cells hasil multilevel partitioning.
boundary vertices adalah vertices yang punya setidaknya satu edge yang kedua endpoint nya (tail dan head) di cell yang berbeda, edge yang kedua endpointnya in different cell ini disebut cut/boundary edge.
entry boundary/overlay vertex adalah boundary/overlay vertex yang dia jadi head dari cut edge. exit boundary/overlay vertex adalah boundary/overlay vertex yang dia jadi tail dari cut edge.

shortcut dari setiap cell di overlay graph adalah shortest path dari entry boundary/overlay vertex ke exit boundary/overlay vertex yang dicompute dengan hanya menggunakan vertices dan edges inside that cell.

Customizable Route Planning (CRP) adalah extensi dari algoritma HiTi [2] yang diterapkan pada road network graph.
correctness dari algoritma ini dapat dilihat pada proof dari theorem 4.4 ref [2]. inti dari theorem 4.4 adalah shortest path dari simpul s ke simpul t pada graf yang terdiri dari overlay graph H, cell C_s, cell C_t ekuivalen
dengan s-t shortest path pada graf G.
untuk any s-t shortest path, kita bisa decompose edges penyusun s-t shortest path dengan edges inside cell C_s, edges inside C_t, cut edges in any cells, atau edges inside any other cell (selain C_s dan C_t).
tapi karena di fase kustomisasi CRP [1] dan HiTi [2], kita compute shortcut edges di setiap cell yang mana adalah shortest path dari entry boundary vertex ke exit boundary vertex dengan hanya menggunakan vertices and edges inside that cell.
dengan menggunakan sifat optimal substructure dari shortest path, bagian "edges inside any cell (selain C_s dan C_t)" bisa kita ganti dengan shortcut edges di overlay graph H yang udah kita precompute di fase kustomisasi.

proof of correctness bidirectional dijkstra bisa dilihat di proof of correctness Algorithm 2 di ref [5] dan [3]. di implementasi ini, kita apply bidirectional dijkstra pada graf consisting of overlay graph H, cell C_s, cell C_t yang mana s-t shortest path yang dihasilkan
ekuivalen dengan s-t shortest path pada graf G.


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p maximum number of nodes, edges within any level-1 cell
let \hat{m_p} denote the maximum number of shortcut edges (and cut edges) within any cell
let k, n_o denote the number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + n_p + m_p + k * \hat{m_p}) * log (n_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all vertices in lowest level (level-1) cell that containing s or t and all overlay vertices in all cell other than level 1 cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, di C_s/C_t kita masih relax all edges inside C_s/C_t yang mana at most m_p, ketika di overlay graph H, kita relax shortcut edges dan cut edges yang mana at most k * \hat{m_p}
extract-min at most O(n_p+n_o) operations, yang kita insert di pq adlaah vertices inside C_s/C_t yang mana at most n_p dan overlay vertices in overlay graph H yang mana at most n_o.

inspired by Customizable Route Planning (CRP) [1] query phase code implementation by michael wegner: https://github.com/michaelwegner/CRP/blob/master/algorithm/CRPQuery.cpp
*/

func (bs *CRPQuery[W]) ShortestPathSearch(s, t da.Index) (W, []da.Index, bool) {

	defer bs.Done()
	now := time.Now()

	if s == t {
		return 0, EmptyIndexSet, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	bs.shortestCost = 2 * util.Infinity[W]()

	sOffset, sSize := bs.engine.graph.GetVertexCellOffset(s)
	tOffset, tSize := bs.engine.graph.GetVertexCellOffset(t)
	maxV := bs.engine.graph.GetMaxVerticesInCell()
	nv := da.Index(bs.engine.numberOfVertices)
	bs.fpq.InitCRPQuery(sOffset, sSize, tOffset, tSize, maxV, nv)
	bs.bpq.InitCRPQuery(sOffset, sSize, tOffset, tSize, maxV, nv)

	svd := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID))
	tvd := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID))
	sQueryKey := da.NewQKey(s, 0, false)
	tQueryKey := da.NewQKey(t, 0, false)
	bs.fpq.Insert(s, 0, svd, sQueryKey)
	bs.bpq.Insert(t, 0, tvd, tQueryKey)

	for bs.fpq.Size() > 0 && bs.bpq.Size() > 0 {
		minForward := bs.fpq.GetMinrank()
		minBackward := bs.bpq.GetMinrank()

		if util.Ge(minForward+minBackward, W(float64(bs.shortestCost)*(bs.ub))) || (bs.mapmatching && util.Ge(minForward+minBackward, W(bs.maxSearchRad))) {
			break
		}

		queryKey := bs.fpq.ExtractMin()
		uItem := queryKey.GetItem()

		if !uItem.IsOverlay() {
			bs.fpq.Explore(uItem.GetNode())
			bs.graphSearch(uItem, s, t, true, bs.fpq, bs.bpq)
			bs.addViaVertex(uItem, true, false)
		} else {
			bs.fpq.Explore(uItem.GetNode())
			bs.overlayGraphSearch(uItem, s, t, true, bs.fpq, bs.bpq)
			bs.addViaVertex(uItem, true, true)
		}

		queryKey = bs.bpq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.bpq.Explore(uItem.GetNode())
			bs.graphSearch(uItem, s, t, false, bs.bpq, bs.fpq)
			bs.addViaVertex(uItem, false, false)
		} else {
			bs.bpq.Explore(uItem.GetNode())
			bs.overlayGraphSearch(uItem, s, t, false, bs.bpq, bs.fpq)
			bs.addViaVertex(uItem, false, true)
		}

		bs.numExploredVertices += 2
	}

	if util.Ge(bs.shortestCost, util.Infinity[W]()) {
		return util.Infinity[W](), EmptyIndexSet, false
	}

	packedPath := bs.engine.RetrievePackedPath(bs.mid,
		bs.fpq, bs.bpq, bs.sCellNumber, s, t)

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	unpacker := newPathUnpackerALT(bs.engine, bs.metrics, bs.lm)
	vPath := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)
	bs.pathUnpackingRuntime = unpacker.runtime

	return bs.shortestCost, vPath, true
}

/*
graphSearch. forward search & backward search dari bidirectional Dijkstra in sel c1(s) atau c1(t).
*/
func (bs *CRPQuery[W]) graphSearch(uKey da.QueryKey, s, t da.Index, forw bool, pq, rpq *da.QueryHeap[da.QueryKey, W]) {

	u := uKey.GetNode()
	uCost := pq.GetCost(u)

	offset, eNum := bs.engine.graph.NumEdges(u, forw)
	for eId := offset; eId < offset+eNum; eId++ {
		v := bs.engine.GetV(eId, forw)

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(v))

		eWeight := bs.GetWeight(eId, forw)

		// get cost to reach v through u
		newVCost := uCost + eWeight

		if vQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in dijkstra
			// relax edge
			oldVCost := pq.GetCost(v)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				// newVCost is better, update the vertex v cost in priority queue

				if vLabelled {
					// is key already in the priority queue, decrease its key
					newPar := da.NewParentVertex(u)
					pq.DecreaseKey(v, newVCost, newVCost, newPar)
				} else if !vLabelled {
					vData := da.NewVData(newVCost,
						da.NewParentVertex(u))
					queryKey := da.NewQKey(v, 0, false)
					// is key not in the priority queue, insert it
					pq.Insert(v, newVCost, vData, queryKey)
				}
			}

			if rpq.IsExplored(v) {
				newEstSPCost := pq.GetCost(v) + rpq.GetCost(v)
				if util.Lt(newEstSPCost, bs.shortestCost) {
					bs.shortestCost = newEstSPCost
					bs.mid = da.NewParentVertex(v)
				}
			}
		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			vOv := bs.engine.graph.GetOverlayVertex(v, eId, forw)
			vOv = bs.engine.offsetOverlay(vOv)
			oldVCost := pq.GetCost(vOv)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				newPar := da.NewParentVertex(u)
				if !vLabelled {
					vData := da.NewVData(newVCost,
						newPar)
					queryKey := da.NewQKey(vOv, vQueryLevel, true)
					pq.Insert(vOv, newVCost, vData, queryKey)
				} else {
					pq.DecreaseKey(vOv, newVCost, newVCost, newPar)
				}
			}

			if rpq.IsExplored(vOv) {
				// if v explored by other search, check whether we can improve the shortestPath
				newEstSPCost := pq.GetCost(vOv) + rpq.GetCost(vOv)
				if util.Lt(newEstSPCost, bs.shortestCost) {
					bs.shortestCost = newEstSPCost
					mid := da.NewParentVertex(vOv)
					mid.SetIsOverlayVertex()
					bs.mid = mid
				}
			}
		}
	}
}

/*
overlayGraphSearch. forward search & backward search dari bidirectional Dijkstra in overlay graph H.

reference: 1. Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable
Route Planning in Road Networks. Transportation Science, No. 2, Volume 51,
pages 566-591. Available at: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

the explanation below taken from Delling et al. (2015) ref [1]:
"Otherwise, we use the overlay graph at level lst(u), which does not have turns.
In either case, the neighbors v of u are added to the priority queue with the appropriate distance labels.
Note that a level transition occurs when u and v have different query levels; in particular,
for transitions from or to level 0, we must translate between the two vertex identifiers for v"

and applying optimization from Delling et al. (2015) below:

the explanation below taken from Delling et al. (2015) ref [1]:
"We apply several optimizations. First, by construction, each exit vertex u in the overlay has a single
outgoing arc (u, v). Therefore, during the search we do not add u to the priority queue; instead, we traverse
the arc (u, v) immediately and process v."
*/
func (bs *CRPQuery[W]) overlayGraphSearch(uKey da.QueryKey, s, t da.Index, forw bool, pq, rpq *da.QueryHeap[da.QueryKey, W]) {
	// search on overlay graph

	uOv := uKey.GetNode() // overlay vertex id
	uQueryLevel := int(uKey.GetQueryLevel())
	u := bs.engine.adjustoffsetOverlay(uOv)
	uCost := pq.GetCost(uOv)

	ovOffset, wOffset, eNum, numExit := bs.engine.overlayGraph.NumOfShortcuts(u, uQueryLevel, forw)

	for i := da.Index(0); i < eNum; i++ {
		v, ewOffset := bs.engine.GetOV(ovOffset, wOffset, numExit, i, forw)
		shortcutWeight := bs.engine.metrics.GetShortcutWeight(ewOffset)
		newVCost := uCost + shortcutWeight
		vOv := bs.engine.offsetOverlay(v)
		// relax edge
		oldVCost := pq.GetCost(vOv)
		if util.Lt(newVCost, oldVCost) {
			vPar := da.NewParentVertex(uOv)
			vPar.SetIsOverlayVertex()
			pq.Set(vOv, da.NewVData(newVCost,
				vPar), da.NewQKey(vOv, uint8(uQueryLevel), true))
			pq.Explore(vOv)

			// traverse cut edge (v,w), immediately process w
			vVertex := bs.engine.overlayGraph.GetVertex(v)
			vCutEdgeId := vVertex.GetCutEdge()
			eWeight := bs.GetWeight(vCutEdgeId, forw)
			wOv := vVertex.GetNeighborOverlayVertex()
			wVertex := bs.engine.overlayGraph.GetVertex(wOv)
			wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
				wVertex.GetCellNumber())
			w := wVertex.GetOrigVId()
			newWCost := newVCost + eWeight

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				oldWCost := pq.GetCost(w)
				wLabelled := util.Lt(oldWCost, util.Infinity[W]())
				if util.Lt(newWCost, oldWCost) {
					newPar := da.NewParentVertex(vOv)
					newPar.SetIsOverlayVertex()

					if wLabelled {
						pq.DecreaseKey(w, newWCost, newWCost, newPar)
					} else {
						vData := da.NewVData(newWCost, newPar)
						queryKey := da.NewQKey(w, 0, false)
						pq.Insert(w, newWCost, vData, queryKey)
					}
				}

				if rpq.IsExplored(w) {
					newEstSPCost := pq.GetCost(w) + rpq.GetCost(w)
					if util.Lt(newEstSPCost, bs.shortestCost) {
						bs.shortestCost = newEstSPCost
						bs.mid = da.NewParentVertex(w)
					}
				}
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				wOv = bs.engine.offsetOverlay(wOv)
				oldwOvCost := pq.GetCost(wOv)
				wLabelled := util.Lt(oldwOvCost, util.Infinity[W]())
				if util.Lt(newWCost, oldwOvCost) {
					newPar := da.NewParentVertex(vOv)
					newPar.SetIsOverlayVertex()
					if !wLabelled {
						vData := da.NewVData(newWCost, newPar)
						queryKey := da.NewQKey(wOv, wQueryLevel, true)
						pq.Insert(wOv, newWCost, vData, queryKey)
					} else {
						pq.DecreaseKey(wOv, newWCost, newWCost, newPar)
					}
				}

				if rpq.IsExplored(wOv) {
					// if overlay vertex w explored by other search, check whether we can improve the shortestPath
					newEstSPCost := pq.GetCost(wOv) + rpq.GetCost(wOv)
					if util.Lt(newEstSPCost, bs.shortestCost) {
						bs.shortestCost = newEstSPCost
						mid := da.NewParentVertex(wOv)
						mid.SetIsOverlayVertex()
						bs.mid = mid
					}
				}
			}
		}

		if rpq.IsExplored(vOv) {
			newEstSPCost := pq.GetCost(vOv) + rpq.GetCost(vOv)
			if util.Lt(newEstSPCost, bs.shortestCost) {
				bs.shortestCost = newEstSPCost
				mid := da.NewParentVertex(vOv)
				mid.SetIsOverlayVertex()
				bs.mid = mid
			}
		}
	}
}

func (bs *CRPQuery[W]) GetWeight(eId da.Index, out bool) W {
	if out {
		return bs.metrics.GetWeight(eId)
	}
	reId := bs.engine.graph.GetOutId(eId)
	return bs.metrics.GetWeight(reId)
}

func (bs *CRPQuery[W]) Preallocate() {
	bs.fpq = bs.engine.fHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
	bs.bpq = bs.engine.bHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
}

func (bs *CRPQuery[W]) Done() {
	if bs.forAlternatives {
		return
	}
	bs.fpq.Clear()
	bs.bpq.Clear()
	bs.engine.fHeapPool.Put(bs.fpq)
	bs.engine.bHeapPool.Put(bs.bpq)
}

func (bs *CRPQuery[W]) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(bs.numExploredVertices)
	return efficiency, bs.numExploredVertices, bs.runtime, bs.pathUnpackingRuntime
}

func (bs *CRPQuery[W]) addViaVertex(uqk da.QueryKey, forward bool, overlay bool) {
	u := uqk.GetNode()
	if forward {
		if bs.bpq.IsExplored(u) {
			bs.viaVertices = append(bs.viaVertices, NewViaVertex(u, overlay))
		}
	} else {
		if bs.fpq.IsExplored(u) {
			bs.viaVertices = append(bs.viaVertices, NewViaVertex(u, overlay))
		}
	}
}

func (bs *CRPQuery[W]) SetMaxSearchRadiusSecs(maxSearchRadiusSecs float64) {
	weight := util.WeightFromSeconds[W](maxSearchRadiusSecs)
	bs.maxSearchRad = weight
	bs.mapmatching = true
}

func (bs *CRPQuery[W]) SetUpperBound(ub float64) {
	bs.ub = ub
}

func (bs *CRPQuery[W]) ForAlternatives(val bool) {
	bs.forAlternatives = val
}

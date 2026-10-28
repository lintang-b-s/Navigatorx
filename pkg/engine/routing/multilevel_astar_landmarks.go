package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPALTQuery[W util.Numeric] struct {
	engine  *CRPRoutingEngine[W]
	metrics *met.Metric[W]
	lm      *landmark.Landmark[W]

	shortestCost W
	mid          da.ParentVertex

	fpq *da.QueryHeap[da.QueryKey, W]
	bpq *da.QueryHeap[da.QueryKey, W]

	activeLandmarks []da.Index
	pirt            W

	sCellNumber da.Pv
	tCellNumber da.Pv

	reroute bool

	numExploredVertices        int
	numExploredOverlayVertices int
	runtime                    int64
	pathUnpackingRuntime       int64
}

func NewCRPALTQuery[W util.Numeric](
	engine *CRPRoutingEngine[W],
) *CRPALTQuery[W] {
	crpQuery := &CRPALTQuery[W]{
		engine:                     engine,
		mid:                        da.NewParentVertex(0, false),
		numExploredVertices:        0,
		numExploredOverlayVertices: 0,
		runtime:                    0,
		pathUnpackingRuntime:       0,
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
4. query phase: Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable Route Planning in Road Networks. Transportation Science, No. 2, Volume 51, pages 566-591.
5. ALT query phase: Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
6. https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf
7. bidirectional A*: Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.
8. Cormen, T.H. et al. (2009) Introduction to Algorithms. 3th ed. Cambridge, MA, USA: MIT Press
9. Towers, M. (2020). Bidirectional Dijkstra. https://www.homepages.ucl.ac.uk/~ucahmto/math/2020/05/30/bidirectional-dijkstra.html. Diakses tanggal: 5 Agustus 2026.
10. Efentakis, A., Pfoser, D. dan Vassiliou, Y. (2015). SALT. A Unified Framework for All Shortest-Path Query Variants on Road Networks. In Proceedings of the 14th International Symposium on Experimental Algorithms, Volume 9125, pages 298–311, Paris.


ini adalah implementasi dari fase query dari Customizable Route Planning (CRP) [1,4] + Bidirectional ALT (A* search, landmarks, and triangle inequality) [5], similar to the one in [10]
intinya cuma bidirectional ALT  (A* search, landmarks, and triangle inequality)  pada graf yang consisiting of overlay graph H, cell C_s, cell C_t. C_s adalah cell level 1 yang mengandung vertex s hasil multilevel partition (lihat package partitioner).
Setiap cell C_v memiliki vertices (all inside cell C_v) dan edges (semua tail dan head dari edges nya inside C_v), vertices dan edges dari cell C_v adalah subset dari vertices dan edges dari graf G.
overlay graph H adalah graf yang mengandung all boundary/overlay vertices, all cut edges, all shortcut edges di setiap cells hasil multilevel partitioning.
boundary vertices adalah vertices yang punya setidaknya satu edge yang kedua endpoint nya (tail dan head) di cell yang berbeda, edge yang kedua endpointnya in different cell ini disebut cut/boundary edge.
entry boundary/overlay vertex adalah boundary/overlay vertex yang dia jadi head dari cut edge. exit boundary/overlay vertex adalah boundary/overlay vertex yang dia jadi tail dari cut edge.

shortcut dari setiap cell di overlay graph adalah shortest path dari entry boundary/overlay vertex ke exit boundary/overlay vertex yang dicompute dengan hanya menggunakan vertices dan edges inside that cell.

Customizable Route Planning (CRP) adalah extensi dari algoritma HiTi [2] yang diterapkan pada road network graph.
correctness dari algoritma ini dapat dilihat pada proof dari theorem 4.4 ref [2]. inti dari theorem 4.4 adalah shortest path dari simpul s ke simpul t pada graf yang terdiri dari overlay graph H, cell C_s, cell C_t ekuivalen
dengan s-t shortest path pada graf G.
untuk any s-t shortest path, kita bisa decompose edges penyusun s-t shortest path dengan edges inside cell C_s, edges inside C_t, cut edges in any cells, atau edges inside any cell (selain C_s dan C_t).
tapi karena di fase kustomisasi CRP [1] dan HiTi [2], kita compute shortcut edges di setiap cell yang mana adalah shortest path dari entry boundary vertex ke exit boundary vertex dengan hanya menggunakan vertices and edges inside that cell.
dengan menggunakan sifat optimal substructure dari shortest path, bagian "edges inside any cell (selain C_s dan C_t)" bisa kita ganti dengan shortcut edges di overlay graph H yang udah kita precompute di fase kustomisasi.
proof of correctness bidirectional dijkstra bisa dilihat di proof of correctness Algorithm 2 di ref [9] dan [3]

di implementasi ini, kita apply bidirectional ALT (A* search, landmarks, and triangle inequality) [5]  pada graf consisting of overlay graph H, cell C_s, cell C_t yang mana s-t shortest path yang dihasilkan
ekuivalen dengan s-t shortest path pada graf G.


di implementasi multilevel-alt ini, kita menggunakan Bidirectional ALT (A* search, landmarks, and triangle inequality) [5] instead of bidirectional dijkstra
Bidirectional A*, landmarks, and triangle inequality (ALT) [5] adalah algoritma bidirectional A* yang fungsi heuristik/potential nya memanfaatkan precomputed landmark shortest path distances (see ref [5] for the details)
fungsi heuristik/potential yang digunakan bidirectional ALT memiliki sifat konsisten/feasible
potential function adalah fungsi dari vertices ke bilangan real, fungsi potensial \pi_f(v) memberikan estimate sp distance dari v ke t
diberikan fungsi potensial \pi, kita mendefinisikan reduced cost dari sebuah edge dengan l_{\pi}(v,w)=l(v,w)-\pi(v)+\pi(w)
fungsi potensial \pi dikakan konsisten atau feasible jika l_{\pi} >= 0 untuk semua edges

pada bidirectional A*,kita perlu adjust fungsi potensial agar tetap bersifat konsisten. misal \pi_f(v) adalah estimate sp distance dari v ke t dan \pi_r(v) estimate sp distance dari s ke v
diadaptasi dari ref [5] dan [6], kita menggunakan fungsi potensial konsisten/feasible  pi_f(v)=max(h_f(v), h_r(t)-h_r(v)+beta) untuk forward search and pi_r(v)=-pi_f(v) untuk backward search. kita disini pakai beta=h_f(s) (lihat landmark.go).
[5] dan [7]  membuktikan bahwa bidirectional A* dengan fungsi potensial p_t dan p_s diatas ekuivalen dengan menjalankan algoritma bidirectional dijkstra dengan bobot edge l_p(v,w)=l(v,w)+p_t(w)-p_t(v)=l(v,w)-p_s(w)+p_s(v) >= 0
dari Lemma 25.1 (Reweighting does not change shortest paths) pada ref 8:
misal p=(v0,v1,...,vk) adalah any path dari v0 ke vk. then p is a shortest path from v0 to vk with weight function l if and only if it is a shortest path with weight function l_p

it is easy to see that fungsi heuristik bidirectional ALT (A* search, landmarks, and triangle inequality) diatas masih bersifat konsisten/feasible pada pada graf consisting of overlay graph H, cell level 1 C_s, cell level 1 C_t. kita cukup tunjukkan fungsi heuristik masih konsisten jika menggunakan  edges inside any level 1 cell,cut edges, dan shortcut edges (ez to proof).

s-t shortest path yang dihasilkan oleh bidirectional ALT pada graf consisting of overlay graph H, cell level 1 C_s, cell level 1 C_t ekuivalen dengan s-t shortest path yang dihasilkan oleh algoritma multilevel-dijkstra dengan edge weight l_p diatas.
dengan menggunakan lemma reweighting does not change shortest paths diatas, kita mendapatkan s-t shortest path yang dihasilkan oleh algoritma multilevel-dijkstra dengan edge weight l_p  ekuivalen dengan s-t shortest path
yang dihasillkan oleh algoritma multilevel dijkstra dengan edge weight l.
lihat multilevel_dijkstra_without_turn_cost.go untuk penjelasan multilevel-dijkstra.


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p maximum number of nodes, edges within any level-1 cell
let \hat{m_p} denote the maximum number of shortcut edges (and cut edges) within any cell
let k, n_o denote the number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + n_p + m_p + k * \hat{m_p}) * log (n_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all vertices in lowest level (level-1) cell that containing s or t and all overlay vertices in all cell other than level 1 cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, di C_s/C_t kita masih relax all edges inside C_s/C_t yang mana at most m_p, ketika di overlay graph H, kita relax shortcut edges dan cut edges yang mana at most k * \hat{m_p}
extract-min at most O(n_p+n_o) operations, yang kita insert di pq adlaah vertices inside C_s/C_t yang mana at most n_p dan overlay vertices in overlay graph H yang mana at most n_o.

inspired by Customizable Route Planning (CRP) [1] query phase code implementation by michael wegner: https://github.com/michaelwegner/CRP/blob/master/algorithm/CRPQuery.cpp
todo: di table 2 https://link.springer.com/chapter/10.1007/978-3-319-20086-6_23 atau https://arxiv.org/pdf/1411.0257 , avg runtime CRP+ALT (A* search, landmarks, and triangle inequality) atau uniSALT-p2p di tabelnya (bukan yg versi bidirectional ALT)
punya runtime 0.5ms/op jauh lebih cepet dibandingin bidirectional ALT atau biSALT-p2p di tabelnya yang cuma 0.9ms/op.. coba implement CRP+ALT (unidirectional ALT) dan coba benchmark...
*/

func (bs *CRPALTQuery[W]) ShortestPathSearch(s, t da.Index) (W, []da.Index, bool) {

	defer bs.Done()
	now := time.Now()

	if s == t {
		return 0, EmptyIndexSet, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	bs.shortestCost = 2 * util.Infinity[W]()
	bs.activeLandmarks = bs.engine.lm.SelectBestQueryLandmarks(s, t)
	bs.pirt = bs.lm.ComputeTighestLowerBound(s, t, bs.activeLandmarks) // consistent lower bound dari dist(s, t)

	sOffset, sSize := bs.engine.graph.GetVertexCellOffset(s)
	tOffset, tSize := bs.engine.graph.GetVertexCellOffset(t)
	maxV := bs.engine.graph.GetMaxVerticesInCell()

	nv := da.Index(bs.engine.numberOfVertices)
	bs.fpq.InitCRPQuery(sOffset, sSize, tOffset, tSize, maxV, nv)
	bs.bpq.InitCRPQuery(sOffset, sSize, tOffset, tSize, maxV, nv)

	svd := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID, false))
	tvd := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID, false))
	sQueryKey := da.NewQKey(s, 0, false)
	tQueryKey := da.NewQKey(t, 0, false)
	bs.fpq.Insert(s, 0, svd, sQueryKey)
	bs.bpq.Insert(t, 0, tvd, tQueryKey)

	for bs.fpq.Size() > 0 && bs.bpq.Size() > 0 {
		minForward := bs.fpq.GetMinrank()
		minBackward := bs.bpq.GetMinrank()
		if util.Ge(minForward+minBackward, W(float64(bs.shortestCost))) {
			break
		}

		queryKey := bs.fpq.ExtractMin()
		uItem := queryKey.GetItem()

		if !uItem.IsOverlay() {
			bs.fpq.Explore(uItem.GetNode())
			bs.graphSearch(uItem, s, t, true, bs.fpq, bs.bpq)
		} else {
			bs.fpq.Explore(uItem.GetNode())
			bs.overlayGraphSearch(uItem, s, t, true, bs.fpq, bs.bpq)
			bs.numExploredOverlayVertices++
		}

		queryKey = bs.bpq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.bpq.Explore(uItem.GetNode())
			bs.graphSearch(uItem, s, t, false, bs.bpq, bs.fpq)
		} else {
			bs.bpq.Explore(uItem.GetNode())
			bs.overlayGraphSearch(uItem, s, t, false, bs.bpq, bs.fpq)
			bs.numExploredOverlayVertices++
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
graphSearch. forward search & backward search dari bidirectional ALT (A* Search, landmarks, and triangle inequality)  in sel c1(s) atau c1(t).
*/
func (bs *CRPALTQuery[W]) graphSearch(uKey da.QueryKey, s, t da.Index, forw bool, pq, rpq *da.QueryHeap[da.QueryKey, W]) {
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
				hv := bs.engine.lm.ComputeTighestConsistentLowerBound(v, s, t, bs.activeLandmarks, bs.pirt, forw)
				priority := newVCost + hv
				// newVCost is better, update the vertex v cost in priority queue
				vPar := da.NewParentVertex(u, false)

				if vLabelled {
					// is key already in the priority queue, decrease its key
					pq.DecreaseKey(v, priority, newVCost, vPar)
				} else if !vLabelled {
					vData := da.NewVData(newVCost,
						vPar)
					queryKey := da.NewQKey(v, 0, false)
					// is key not in the priority queue, insert it
					pq.Insert(v, priority, vData, queryKey)
				}
			}

			bs.meet(v, pq, rpq, false)
		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			vOv := bs.engine.graph.GetOverlayVertex(v, eId, forw)
			vOv = bs.engine.offsetOverlay(vOv)
			oldVCost := pq.GetCost(vOv)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				hv := bs.engine.lm.ComputeTighestConsistentLowerBound(v, s, t, bs.activeLandmarks, bs.pirt, forw)
				priority := newVCost + hv
				vOvPar := da.NewParentVertex(u, false)
				if !vLabelled {
					vData := da.NewVData(newVCost,
						vOvPar)

					queryKey := da.NewQKey(vOv, vQueryLevel, true)
					pq.Insert(vOv, priority, vData, queryKey)
				} else {
					pq.DecreaseKey(vOv, priority, newVCost, vOvPar)
				}
			}

			bs.meet(vOv, pq, rpq, true)
		}
	}
}

/*
overlayGraphSearch. forward search & backward search dari bidirectional ALT (A* Search, Landmarks, and Triangle Inequality)  in overlay graph H.

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
func (bs *CRPALTQuery[W]) overlayGraphSearch(uKey da.QueryKey, s, t da.Index, forw bool, pq, rpq *da.QueryHeap[da.QueryKey, W]) {
	// search on overlay graph

	uOv := uKey.GetNode() // overlay vertex id
	uQueryLevel := int(uKey.GetQueryLevel())
	u := bs.engine.adjustoffsetOverlay(uOv)
	uCost := pq.GetCost(uOv)

	ovOffset, wOffset, eNum, exitNum := bs.engine.overlayGraph.NumOfShortcuts(u, uQueryLevel, forw)

	for i := da.Index(0); i < eNum; i++ {
		v, ewOffset := bs.engine.GetOV(ovOffset, wOffset, exitNum, i, forw)
		shortcutWeight := bs.engine.metrics.GetShortcutWeight(ewOffset)
		newVCost := uCost + shortcutWeight
		vOv := bs.engine.offsetOverlay(v)
		// relax edge
		oldVCost := pq.GetCost(vOv)
		if util.Lt(newVCost, oldVCost) {
			vPar := da.NewParentVertex(uOv, true)
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
					// ALT (A* Search, landmarks, and triangle inequality) lowerbound/heuristic function
					hw := bs.engine.lm.ComputeTighestConsistentLowerBound(w, s, t, bs.activeLandmarks, bs.pirt, forw)
					priority := newWCost + hw

					wPar := da.NewParentVertex(vOv, true)

					if wLabelled {
						pq.DecreaseKey(w, priority, newWCost, wPar)
					} else {
						vData := da.NewVData(newWCost, wPar)
						queryKey := da.NewQKey(w, 0, false)
						pq.Insert(w, priority, vData, queryKey)
					}
				}

				bs.meet(w, pq, rpq, false)
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				wOv = bs.engine.offsetOverlay(wOv)
				oldwOvCost := pq.GetCost(wOv)
				wLabelled := util.Lt(oldwOvCost, util.Infinity[W]())
				if util.Lt(newWCost, oldwOvCost) {
					// ALT (A* Search, landmarks, and triangle inequality) lowerbound/heuristic function
					hw := bs.engine.lm.ComputeTighestConsistentLowerBound(w, s, t, bs.activeLandmarks, bs.pirt, forw)
					priority := newWCost + hw

					wOvPar := da.NewParentVertex(vOv, true)
					if !wLabelled {
						vData := da.NewVData(newWCost, wOvPar)
						queryKey := da.NewQKey(wOv, wQueryLevel, true)
						pq.Insert(wOv, priority, vData, queryKey)
					} else {
						pq.DecreaseKey(wOv, priority, newWCost, wOvPar)
					}
				}

				bs.meet(wOv, pq, rpq, true)
			}
		}

		bs.meet(vOv, pq, rpq, true)
	}
}

func (bs *CRPALTQuery[W]) meet(v da.Index, pq, rpq *da.QueryHeap[da.QueryKey, W], overlay bool) {
	if rpq.IsExplored(v) {
		// if vertex v explored by other search, check whether we can improve the shortestCost estimate
		newEstSPCost := pq.GetCost(v) + rpq.GetCost(v)
		if util.Lt(newEstSPCost, bs.shortestCost) {
			bs.shortestCost = newEstSPCost
			var mid da.ParentVertex
			if overlay {
				mid = da.NewParentVertex(v, true)
			} else {
				mid = da.NewParentVertex(v, false)
			}
			bs.mid = mid
		}
	}
}

func (bs *CRPALTQuery[W]) GetWeight(eId da.Index, out bool) W {
	if out {
		return bs.metrics.GetWeight(eId)
	}
	reId := bs.engine.graph.GetOutId(eId)
	return bs.metrics.GetWeight(reId)
}

func (bs *CRPALTQuery[W]) Preallocate() {
	bs.fpq = bs.engine.fHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
	bs.bpq = bs.engine.bHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
}

func (bs *CRPALTQuery[W]) Done() {

	bs.fpq.Clear()
	bs.bpq.Clear()
	bs.engine.fHeapPool.Put(bs.fpq)
	bs.engine.bHeapPool.Put(bs.bpq)
}

func (bs *CRPALTQuery[W]) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(bs.numExploredVertices)
	return efficiency, bs.numExploredVertices, bs.runtime, bs.pathUnpackingRuntime
}

func (bs *CRPALTQuery[W]) GetActiveLandmarks() []da.Index {
	return bs.activeLandmarks
}

func (bs *CRPALTQuery[W]) GetTCellNumber() da.Pv {
	return bs.tCellNumber
}

func (bs *CRPALTQuery[W]) SetReroute() {
	bs.reroute = true
}

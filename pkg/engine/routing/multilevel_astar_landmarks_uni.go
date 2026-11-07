package routing

import (
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// CRPUniALTQuery represents Combination of Query Phase of Customizable Route Planning (CRP) and ALT (A*, Landmark, Triangle Inequality) Algorithm.
type CRPUniALTQuery[W util.Numeric] struct {
	engine  *CRPRoutingEngine[W]
	metrics *met.Metric[W]
	lm      *landmark.Landmark[W]
	pq      *da.QueryHeap[da.QueryKey, W]

	activeLandmarks [2]da.Index

	inf         W
	sCellNumber da.Pv
	tCellNumber da.Pv

	reroute bool

	numExploredVertices        int
	numExploredOverlayVertices int
	runtime                    int64
	pathUnpackingRuntime       int64
}

// NewCRPUniALTQuery creates and returns crpUnialtquery.
func NewCRPUniALTQuery[W util.Numeric](
	engine *CRPRoutingEngine[W],
) *CRPUniALTQuery[W] {
	crpQuery := &CRPUniALTQuery[W]{
		engine: engine,

		inf:                        util.Infinity[W](),
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


ini adalah implementasi dari fase query dari Customizable Route Planning (CRP) [1,4] + ALT (A* search, landmarks, and triangle inequality) [5], similar to the one in [10]
intinya cuma  ALT  (A* search, landmarks, and triangle inequality)  pada graf yang consisiting of overlay graph H, cell C_s, cell C_t. C_s adalah cell level 1 yang mengandung vertex s hasil multilevel partition (lihat package partitioner).
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

di implementasi ini, kita apply ALT (A* search, landmarks, and triangle inequality) [5]  pada graf consisting of overlay graph H, cell C_s, cell C_t yang mana s-t shortest path yang dihasilkan
ekuivalen dengan s-t shortest path pada graf G.


di implementasi multilevel-alt ini, kita menggunakan  ALT (A* search, landmarks, and triangle inequality) [5]
 A*, landmarks, and triangle inequality (ALT) [5] adalah algoritma  A* yang fungsi heuristik/potential nya memanfaatkan precomputed landmark shortest path distances (see ref [5] for the details)
fungsi heuristik/potential yang digunakan  ALT memiliki sifat konsisten/feasible
potential function adalah fungsi dari vertices ke bilangan real, fungsi potensial \pi_f(v) memberikan estimate sp distance dari v ke t
diberikan fungsi potensial \pi, kita mendefinisikan reduced cost dari sebuah edge dengan l_{\pi}(v,w)=l(v,w)-\pi(v)+\pi(w)
fungsi potensial \pi dikakan konsisten atau feasible jika l_{\pi} >= 0 untuk semua edges
dari Lemma 25.1 (Reweighting does not change shortest paths) pada ref 8:
misal p=(v0,v1,...,vk) adalah any path dari v0 ke vk. then p is a shortest path from v0 to vk with weight function l if and only if it is a shortest path with weight function l_{\pi}


it is easy to see that fungsi heuristik  ALT (A* search, landmarks, and triangle inequality) diatas masih bersifat konsisten/feasible pada pada graf consisting of overlay graph H, cell level 1 C_s, cell level 1 C_t. kita cukup tunjukkan fungsi heuristik masih konsisten jika menggunakan  edges inside any level 1 cell,cut edges, dan shortcut edges (ez to proof).

s-t shortest path yang dihasilkan oleh  ALT pada graf consisting of overlay graph H, cell level 1 C_s, cell level 1 C_t ekuivalen dengan s-t shortest path yang dihasilkan oleh algoritma multilevel-dijkstra dengan edge weight l_{\pi} diatas.
dengan menggunakan lemma reweighting does not change shortest paths diatas, kita mendapatkan s-t shortest path yang dihasilkan oleh algoritma multilevel-dijkstra dengan edge weight l_{\pi}  ekuivalen dengan s-t shortest path
yang dihasillkan oleh algoritma multilevel dijkstra dengan edge weight l.


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
DONE :), bener lebih cepet... di dimacs Eastern USA (eval/crp_alt/dimacs_9_runtime/main.go) sekitar 0.73 ms pake implementasi ini
*/

// ShortestPathSearch compute point-to-point fastest path.
func (bs *CRPUniALTQuery[W]) ShortestPathSearch(s, t da.Index) (W, []da.Index, bool) {

	defer bs.Done()

	if s == t {
		return 0, EmptyIndexSet, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	bs.activeLandmarks = bs.engine.lm.SelectBestQueryLandmarks(s, t)

	sOffset, sSize := bs.engine.graph.GetVertexCellOffset(s)
	tOffset, tSize := bs.engine.graph.GetVertexCellOffset(t)
	maxV := bs.engine.graph.GetMaxVerticesInCell()

	nv := da.Index(bs.engine.numberOfVertices)
	bs.pq.InitCRPQuery(sOffset, sSize, tOffset, tSize, maxV, nv)

	sQk := da.NewQKey(s, 0, false)
	svd := da.NewVData(W(0), sQk, da.InvalidVertexId)
	bs.pq.Insert(s, 0, svd)

	for !bs.pq.IsEmpty() {
		uk, uCost := bs.pq.ExtractMin()
		u := uk.GetNode()
		if !uk.IsOverlay() {
			if u == t {
				break
			}
			bs.graphSearch(u, uCost, t, bs.pq)
		} else {
			bs.overlayGraphSearch(u, uk.QueryLevel(), uCost, t, bs.pq)
			bs.numExploredOverlayVertices++
		}

		bs.numExploredVertices++
	}

	shortestCost := bs.pq.Cost(t)
	if util.Ge(shortestCost, bs.inf) {
		return bs.inf, EmptyIndexSet, false
	}

	packedPath := bs.engine.RetrievePackedPathUni(t,
		bs.pq)
	unpacker := newPathUnpackerALT(bs.engine, bs.metrics, bs.lm)
	vPath := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)
	bs.pathUnpackingRuntime = unpacker.runtime
	return shortestCost, vPath, true
}

/*
graphSearch. forward search & backward search dari bidirectional ALT (A* Search, landmarks, and triangle inequality)  in sel c1(s) atau c1(t).
*/
func (bs *CRPUniALTQuery[W]) graphSearch(u da.Index, uCost W, t da.Index, pq *da.QueryHeap[da.QueryKey, W]) {
	offset, eNum := bs.engine.graph.NumOutEdges(u)

	for eId := offset; eId < offset+eNum; eId++ {
		v := bs.engine.graph.GetHead(eId)
		eWeight := bs.metrics.GetWeight(eId)
		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.QueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(v))

		// get cost to reach v through u
		newVCost := uCost + eWeight
		if vQueryLevel == 0 {
			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in dijkstra
			// relax edge
			oldVCost := pq.Cost(v)
			if util.Lt(newVCost, oldVCost) {
				hv := bs.engine.lm.ComputeTighestLowerBound(v, t, bs.activeLandmarks)
				priority := newVCost + hv
				// newVCost is better, update the vertex v cost in priority queue

				vLabelled := util.Lt(oldVCost, bs.inf)
				if vLabelled {
					// is key already in the priority queue, decrease its key
					pq.DecreaseKey(v, priority, newVCost, u)
				} else {
					vQk := da.NewQKey(v, 0, false)
					vData := da.NewVData(newVCost, vQk, u)
					// is key not in the priority queue, insert it
					pq.Insert(v, priority, vData)
				}
			}
		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			vOv := bs.engine.graph.GetExitOverlayVertex(v, eId)
			vOv = bs.engine.offsetOverlay(vOv)
			oldVCost := pq.Cost(vOv)
			if util.Lt(newVCost, oldVCost) {
				hv := bs.engine.lm.ComputeTighestLowerBound(v, t, bs.activeLandmarks)
				priority := newVCost + hv

				vLabelled := util.Lt(oldVCost, bs.inf)
				if !vLabelled {
					vQk := da.NewQKey(vOv, vQueryLevel, true)
					vData := da.NewVData(newVCost, vQk, u)
					pq.Insert(vOv, priority, vData)
				} else {
					pq.DecreaseKey(vOv, priority, newVCost, u)
				}
			}
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
func (bs *CRPUniALTQuery[W]) overlayGraphSearch(uOv da.Index, uQueryLevel int, uCost W, t da.Index, pq *da.QueryHeap[da.QueryKey, W]) {
	// search on overlay graph

	u := bs.engine.adjustoffsetOverlay(uOv)
	ovOffset, wOffset, eNum := bs.engine.overlayGraph.NumOfOutShortcuts(u, uQueryLevel)
	for i := range eNum {
		v, seId := bs.GetOV(ovOffset, wOffset, i)
		eWeight := bs.engine.metrics.GetShortcutWeight(seId)
		newVCost := uCost + eWeight
		vOv := bs.engine.offsetOverlay(v)
		// relax edge
		oldVCost := pq.Cost(vOv)
		if util.Lt(newVCost, oldVCost) {
			vQk := da.NewQKey(vOv, uint8(uQueryLevel), true)
			vData := da.NewVData(newVCost, vQk, uOv)
			pq.Set(vOv, vData)

			// traverse cut edge (v,w), immediately process w
			wOv, vCutEdge := bs.engine.overlayGraph.GetVertexNeighbor(v)
			eWeight := bs.metrics.GetWeight(vCutEdge)
			w, wCellNumber := bs.engine.overlayGraph.GetVertexData(wOv)
			wQueryLevel := bs.engine.overlayGraph.QueryLevel(bs.sCellNumber, bs.tCellNumber, wCellNumber)
			newWCost := newVCost + eWeight

			if wQueryLevel == 0 {
				// w is in the same cell as s or t
				oldWCost := pq.Cost(w)

				if util.Lt(newWCost, oldWCost) {
					// ALT (A* Search, landmarks, and triangle inequality) lowerbound/heuristic function
					hw := bs.engine.lm.ComputeTighestLowerBound(w, t, bs.activeLandmarks)
					priority := newWCost + hw

					wLabelled := util.Lt(oldWCost, bs.inf)
					if wLabelled {
						pq.DecreaseKey(w, priority, newWCost, vOv)
					} else {
						wQk := da.NewQKey(w, 0, false)
						wData := da.NewVData(newWCost, wQk, vOv)
						pq.Insert(w, priority, wData)
					}
				}
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				wOv = bs.engine.offsetOverlay(wOv)
				oldwOvCost := pq.Cost(wOv)
				if util.Lt(newWCost, oldwOvCost) {
					// ALT (A* Search, landmarks, and triangle inequality) lowerbound/heuristic function
					hw := bs.engine.lm.ComputeTighestLowerBound(w, t, bs.activeLandmarks)
					priority := newWCost + hw

					wLabelled := util.Lt(oldwOvCost, bs.inf)
					if !wLabelled {
						wQk := da.NewQKey(wOv, wQueryLevel, true)
						wData := da.NewVData(newWCost, wQk, vOv)
						pq.Insert(wOv, priority, wData)
					} else {
						pq.DecreaseKey(wOv, priority, newWCost, vOv)
					}
				}
			}
		}
	}
}

// Preallocate handles preallocate priority queue.
func (bs *CRPUniALTQuery[W]) Preallocate() {
	bs.pq = bs.engine.uniHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
}

// GetOV returns overlay vertex at specified id.
func (bs *CRPUniALTQuery[W]) GetOV(ovOffset, wOffset, i da.Index) (da.Index, da.Index) {
	v := bs.engine.overlayGraph.GetCellExitVertex(ovOffset + i)
	return v, wOffset + i
}

// Done handles put back priority queues to sync.Pool.
func (bs *CRPUniALTQuery[W]) Done() {
	bs.pq.Clear()
	bs.engine.uniHeapPool.Put(bs.pq)
}

// GetStats returns stats of the query.
func (bs *CRPUniALTQuery[W]) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(bs.numExploredVertices)
	return efficiency, bs.numExploredVertices, bs.runtime, bs.pathUnpackingRuntime
}

// SetReroute sets reroute to true.
func (bs *CRPUniALTQuery[W]) SetReroute() {
	bs.reroute = true
}

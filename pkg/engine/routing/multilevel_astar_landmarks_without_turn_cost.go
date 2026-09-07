package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// penjelasan algoritma kueri (tanpa turn cost) dari Customizable Route Planning + ALT ada di section 3.5 & 3.6:  https://drive.google.com/file/d/1Ek7xLIsl5Kv-CSR6RdlRNYuA5iFIJaDl/view
// pdf password: <my-github-username>-<my-birth-year>-<my gdrive email without @gmail.com>

type CRPALTQuery[W util.RoutingNumber] struct {
	engine     *CRPRoutingEngine[W]
	shortestTT W
	forwMid    da.VertexEdgePair
	backwMid   da.VertexEdgePair

	forwPq  *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W]
	backwPq *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W]

	activeLandmarks []da.Index

	sCellNumber da.Pv
	tCellNumber da.Pv

	numScannedVertices        int
	numScannedOverlayVertices int
	runtime                   int64
	pathUnpackingRuntime      int64
}

func NewCRPALTQuery[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *CRPALTQuery[W] {
	crpQuery := &CRPALTQuery[W]{
		engine: engine,

		forwMid:  da.NewVertexEdgePair(0, 0, false),
		backwMid: da.NewVertexEdgePair(0, 0, true),

		numScannedVertices:        0,
		numScannedOverlayVertices: 0,
		runtime:                   0,
		pathUnpackingRuntime:      0,
	}

	crpQuery.Preallocate()
	return crpQuery
}

// Reset clears per-query state on a pooled CRPALTQuery so it can
// be reused.
func (bs *CRPALTQuery[W]) Reset() {

	bs.shortestTT = 2 * util.Infinity[W]()

	bs.forwMid = da.NewVertexEdgePair(0, 0, false)
	bs.backwMid = da.NewVertexEdgePair(0, 0, true)

	bs.sCellNumber = 0
	bs.tCellNumber = 0

	bs.numScannedVertices = 0
	bs.numScannedOverlayVertices = 0
	bs.runtime = 0
	bs.pathUnpackingRuntime = 0
}

/*
implementasi dari:
1. query phase (without turn costs): Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.
2. Sungwon Jung and S. Pramanik, "An efficient path computation model for hierarchically structured topographical road maps," in IEEE Transactions on Knowledge and Data Engineering, vol. 14, no. 5, pp. 1029-1046, Sept.-Oct. 2002, doi: 10.1109/TKDE.2002.1033772.
keywords: {Computational modeling;Roads;Navigation;Computational efficiency;Performance analysis;Shortest path problem;Concurrent computing;Cost function;Automobiles;Space exploration},
3. Haeupler, B. et al. (2025) “Bidirectional Dijkstra's Algorithm is Instance-Optimal,” in 2025 Symposium on Simplicity in Algorithms (SOSA). Society for Industrial and Applied Mathematics (Proceedings), pp. 202–215. Available at: https://doi.org/10.1137/1.9781611978315.16.
4. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
5. ALT query phase: Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
6. https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf
7. bidirectional A*: Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.
8. Cormen, T.H. et al. (2009) Introduction to Algorithms. 3th ed. Cambridge, MA, USA: MIT Press


ini adalah implementasi dari fase query dari Customizable Route Planning (CRP) [1] + Bidirectional ALT (A* search, landmarks, and triangle inequality) [5]  tanpa incorporate turn costs.
intinya cuma bidirectional ALT  (A* search, landmarks, and triangle inequality)  pada graf yang consisiting of overlay graph H, cell C_s, cell C_t. C_s adalah cell level 1 yang mengandung vertex s hasil multilevel partition (lihat package partitioner).
Setiap cell C_v memiliki vertices (all inside cell C_v) dan edges (semua endpoints nya inside C_v), vertices dan edges dari cell C_v adalah subset dari vertices dan edges dari graf G.
overlay graph H adalah graf yang mengandung all boundary/overlay vertices, all cut/boundary edges, all shortcut edges di setiap cells hasil multilevel partitioning.
boundary vertices adalah vertices yang punya setidaknya satu edge yang kedua endpoint nya (tail dan head) di cell yang berbeda, edge yang kedua endpointnya in different cell ini disebut cut/boundary edge.
entry boundary/overlay vertex adalah boundary/overlay vertex yang dia jadi head dari cut edge. exit boundary/overlay vertex adalah boundary/overlay vertex yang dia jadi tail dari cut edge.

shortcut dari setiap cell di overlay graph adalah shortest path dari entry boundary/overlay vertex ke exit boundary/overlay vertex yang dicompute dengan hanya menggunakan vertices dan edges inside that cell.

Customizable Route Planning (CRP) adalah extensi dari algoritma HiTi [2] yang diterapkan pada road network graph.
correctness dari algoritma ini dapat dilihat pada proof dari theorem 4.4 ref [2]. inti dari theorem 4.4 adalah shortest path dari simpul s ke simpul t pada graf yang terdiri dari overlay graph H, cell C_s, cell C_t ekuivalen
dengan s-t shortest path pada graf G.
untuk any s-t shortest path, kita bisa decompose edges penyusun s-t shortest path dengan edges inside cell C_s, edges inside C_t, cut edges in any cells, atau edges inside any cell (selain C_s dan C_t).
tapi karena di fase kustomisasi CRP [1] dan HiTi [2], kita compute shortcuts di setiap cell yang mana adalah shortest path dari entry boundary vertex ke exit boundary vertex dengan hanya menggunakan vertices and edges inside that cell.
bagian "edges inside any cell (selain C_s dan C_t)" bisa kita ganti dengan shortcuts di overlay graph H yang udah kita precompute di fase kustomisasi.
proof of correctness bidirectional dijkstra bisa dilihat di proof of correctness Algorithm 2 di ref [3]

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
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + n_p + m_p + k * \hat{m_p}) * log (n_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all vertices in lowest level cell that containing s or t and all overlay vertices in all cell other than level 1 cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, di C_s/C_t kita masih relax all edges inside C_s/C_t yang mana at most m_p, ketika di overlay graph H, kita relax shortcut edges yang mana at most k * \hat{m_p}
extract-min at most O(n_p+n_o) operations, yang kita insert di pq adlaah vertices inside C_s/C_t yang mana at most n_p dan overlay vertices in overlay graph H yang mana at most n_o.


versi query phase dari CRP + bidirectional ALT yang support turn costs & turn restrictions [4] dapat dilihat pada multilevel_astar_landmarks.go, yang mana implementasi dari query phase dari CRP yang menggunakan compact graph representation/turn tables yang mana simulates arc-based expanded graph representation, diadaptasi dari implementasi CRP: https://github.com/michaelwegner/CRP, jauh lebih ribet dari  implemetasi ini....
implementasi query phase dari CRP yang support turn costs & turn restrictions yang jauh lebih mudah dipahami dapat dilihat di  https://github.com/Project-OSRM/osrm-backend/blob/master/include/engine/routing_algorithms/routing_base_mld.hpp   yang mana osrm pakai arc-based expanded graph : https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation

*/

func (bs *CRPALTQuery[W]) ShortestPathSearch(s, t da.Index) (W, []da.Index, bool) {

	defer bs.Done()
	now := time.Now()

	if s == t {
		return 0, EmptyIndexSet, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	bs.shortestTT = 2 * util.Infinity[W]()
	bs.activeLandmarks = bs.engine.lm.SelectBestQueryLandmarks(s, t)

	sVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	tVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, true))
	sQueryKey := da.NewCRPQueryKeyNoTurnCost(s, 0, false)
	tQueryKey := da.NewCRPQueryKeyNoTurnCost(t, 0, false)
	bs.forwPq.Insert(s, 0, sVertexInfo, sQueryKey)
	bs.backwPq.Insert(t, 0, tVertexInfo, tQueryKey)

	for bs.forwPq.Size() > 0 && bs.backwPq.Size() > 0 {
		minForward := bs.forwPq.GetMinrank()
		minBackward := bs.backwPq.GetMinrank()
		if util.Ge(minForward+minBackward, W(float64(bs.shortestTT))) {
			break
		}

		queryKey := bs.forwPq.ExtractMin()
		uItem := queryKey.GetItem()

		if !uItem.IsOverlay() {
			bs.forwPq.Explore(uItem.GetNode())
			bs.forwardGraphSearch(uItem, s, t)
		} else {
			bs.forwPq.Explore(uItem.GetNode())
			bs.forwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		queryKey = bs.backwPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.backwPq.Explore(uItem.GetNode())

			bs.backwardGraphSearch(uItem, s, t)
		} else {
			bs.backwPq.Explore(uItem.GetNode())

			bs.backwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		bs.numScannedVertices += 2
	}

	if bs.shortestTT == 2*util.Infinity[W]() {
		return util.Infinity[W](), EmptyIndexSet, false
	}

	packedPath := bs.engine.RetrievePackedPathNoTurnCost(bs.forwMid, bs.backwMid,
		bs.forwPq, bs.backwPq, bs.sCellNumber, s, t)

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	unpacker := NewPathUnpackerALTNoTurnCost(bs.engine)
	defer unpacker.DonePooled()
	edgeIdPath := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)
	bs.pathUnpackingRuntime = unpacker.runtime

	return bs.shortestTT, edgeIdPath, true
}

/*
graphSearch. forward search dari bidirectional ALT di sel c1(s) atau c1(t).
*/
func (bs *CRPALTQuery[W]) forwardGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {

	uId := uItem.GetNode()
	uTT := bs.forwPq.GetPriority(uId)

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOfNoTurnCost(uId, func(eId, head, entryPoint da.Index) {
		vId := head

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		eWeight := bs.engine.getWeight(eId, true)

		// get cost to reach v through u
		newTT := uTT + eWeight

		if util.Ge(newTT, util.Infinity[W]()) {
			return
		}

		pfv, _ := bs.engine.lm.FindTighestConsistentLowerBound(vId, source, target, bs.activeLandmarks)
		priority := newTT + pfv

		if vQueryLevel == 0 {

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in dijkstra

			// relax edge
			oldVIdTT := bs.forwPq.GetPriority(vId)
			vLabelled := util.Lt(oldVIdTT, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newTT, oldVIdTT)) {
				if vLabelled {
					// newTT is bsCellNumberetter, update the forwardInfo
					// is key already in the priority queue, decrease its key

					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)
					bs.forwPq.DecreaseKey(vId, priority, newTT, newPar)
				} else if !vLabelled {

					vertexInfo := da.NewVertexInfo(newTT,
						da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))
					queryKey := da.NewCRPQueryKeyNoTurnCost(vId, 0, false)
					// is key not in the priority queue, insert it
					bs.forwPq.Insert(vId, priority, vertexInfo, queryKey)
				}
			}

			exploredByBackwSearch := bs.backwPq.IsExplored(vId)
			vIdForwTT := bs.forwPq.GetPriority(vId)
			vIdBackwTT := bs.backwPq.GetPriority(vId)

			newPathTT := vIdForwTT + vIdBackwTT
			if exploredByBackwSearch && util.Lt(newPathTT, bs.shortestTT) {
				bs.shortestTT = newPathTT
				bs.forwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, false)
				bs.backwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, true)
			}
		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := bs.engine.graph.GetOverlayVertex(vId, entryPoint, false)
			ovVId := bs.engine.offsetOverlayNoTurnCost(v)
			oldOvVIdTT := bs.forwPq.GetPriority(ovVId)
			vLabelled := util.Lt(oldOvVIdTT, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newTT, oldOvVIdTT)) {
				newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)

				if !vLabelled {
					vertexInfo := da.NewVertexInfo(newTT,
						newPar)

					queryKey := da.NewCRPQueryKeyNoTurnCost(ovVId, vQueryLevel, true)
					bs.forwPq.Insert(ovVId, priority, vertexInfo, queryKey)
				} else {
					bs.forwPq.DecreaseKey(ovVId, priority, newTT, newPar)
				}
			}

			exploredByBackwSearch := bs.backwPq.IsExplored(ovVId)
			// if v explored by backward search, check whether we can improve the shortestPath
			newEstSpCost := bs.forwPq.GetPriority(ovVId) + bs.backwPq.GetPriority(ovVId)
			if exploredByBackwSearch && util.Lt(newEstSpCost, bs.shortestTT) {
				bs.shortestTT = newEstSpCost

				mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
				mid.SetIsOverlayVertex()
				bs.forwMid = mid
				bs.backwMid = mid
			}
		}
	})
}

func (bs *CRPALTQuery[W]) backwardGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search backward on graph level 1

	uId := uItem.GetNode()

	uTT := bs.backwPq.GetPriority(uId)

	bs.engine.graph.ForInEdgesOfNoTurnCost(uId, func(eId, tail, exitPoint da.Index) {
		vId := tail

		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		eWeight := bs.engine.getWeight(eId, false)

		newTT := uTT + eWeight

		if util.Ge(newTT, util.Infinity[W]()) {
			return
		}

		// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
		_, prv := bs.engine.lm.FindTighestConsistentLowerBound(vId, source, target, bs.activeLandmarks)
		priority := newTT + prv

		if vQueryLevel == 0 {

			// relax edge
			oldVIdTT := bs.backwPq.GetPriority(vId)
			vLabelled := util.Lt(oldVIdTT, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newTT, oldVIdTT)) {

				if vLabelled {
					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, true)
					bs.backwPq.DecreaseKey(vId, priority, newTT, newPar)
				} else {
					vertexInfo := da.NewVertexInfo(newTT,
						da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))
					queryKey := da.NewCRPQueryKeyNoTurnCost(vId, 0, false)
					bs.backwPq.Insert(vId, priority, vertexInfo, queryKey)
				}
			}

			exploredByForwSearch := bs.forwPq.IsExplored(vId)

			vIdBackwTT := bs.backwPq.GetPriority(vId)
			vIdForwTT := bs.forwPq.GetPriority(vId)
			newPathTT := vIdForwTT +
				vIdBackwTT
			if exploredByForwSearch && util.Lt(newPathTT, bs.shortestTT) {

				bs.shortestTT = newPathTT
				bs.forwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, false)
				bs.backwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, true)
			}
		} else {
			// v is in another cell on higher level
			// Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			v, _ := bs.engine.graph.GetOverlayVertex(vId, exitPoint, true)
			ovVId := bs.engine.offsetOverlayNoTurnCost(v)
			oldOvVIdTT := bs.backwPq.GetPriority(ovVId)
			vLabelled := util.Lt(oldOvVIdTT, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newTT, oldOvVIdTT)) {
				newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, true)

				if !vLabelled {
					vVertexInfo := da.NewVertexInfo(newTT,
						newPar)
					queryKey := da.NewCRPQueryKeyNoTurnCost(ovVId, vQueryLevel, true)
					bs.backwPq.Insert(ovVId, priority, vVertexInfo, queryKey)
				} else {

					bs.backwPq.DecreaseKey(ovVId, priority, newTT, newPar)
				}
			}

			exploredByForwSearch := bs.forwPq.IsExplored(ovVId)
			newEstSpCost := bs.forwPq.GetPriority(ovVId) + bs.backwPq.GetPriority(ovVId)
			if exploredByForwSearch && util.Lt(newEstSpCost, bs.shortestTT) {
				bs.shortestTT = newEstSpCost

				mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
				mid.SetIsOverlayVertex()
				bs.forwMid = mid
				bs.backwMid = mid
			}
		}
	})
}

func (bs *CRPALTQuery[W]) forwardOverlayGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search on overlay graph

	uId := uItem.GetNode() // overlay vertex id

	uQueryLevel := int(uItem.GetQueryLevel())

	adjUId := bs.engine.adjustOffsetOverlayNoTurnCost(uId)

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	bs.engine.overlayGraph.ForOutNeighborsOf(adjUId, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTT := bs.forwPq.GetPriority(uId) + shortcutWeight

		if util.Ge(newTT, util.Infinity[W]()) {
			return
		}
		ovVId := bs.engine.offsetOverlayNoTurnCost(v)

		// traverse edge to next cell
		vOriEdgeId := vVertex.GetCutEdge()

		eWeight := bs.engine.getWeight(vOriEdgeId, true)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOrigVId()

		// relax edge
		oldOvVIdTT := bs.forwPq.GetPriority(ovVId)
		vLabelled := util.Lt(oldOvVIdTT, util.Infinity[W]())
		if !vLabelled || (vLabelled && util.Lt(newTT, oldOvVIdTT)) {
			vPar := da.NewVertexEdgePair(adjUId, da.INVALID_EDGE_ID, false)
			vPar.SetIsOverlayVertex()
			bs.forwPq.Set(ovVId, da.NewVertexInfo(newTT,
				vPar), da.NewCRPQueryKeyNoTurnCost(ovVId, uint8(uQueryLevel), true))

			bs.forwPq.Explore(ovVId)

			newTT = bs.forwPq.GetPriority(ovVId) + eWeight

			if util.Ge(newTT, util.Infinity[W]()) {
				return
			}

			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			pfw, _ := bs.engine.lm.FindTighestConsistentLowerBound(wId, source, target, bs.activeLandmarks)
			priority := newTT + pfw

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				oldWIdTT := bs.forwPq.GetPriority(wId)
				wLabelled := util.Lt(oldWIdTT, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newTT, oldWIdTT)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
					newPar.SetIsOverlayVertex()

					if wLabelled {
						bs.forwPq.DecreaseKey(wId, priority, newTT, newPar)
					} else {
						vertexInfo := da.NewVertexInfo(newTT, newPar)
						queryKey := da.NewCRPQueryKeyNoTurnCost(wId, 0, false)
						bs.forwPq.Insert(wId, priority, vertexInfo, queryKey)
					}
				}

				exploredByBackwSearch := bs.backwPq.IsExplored(wId)

				wIdForwardTT := bs.forwPq.GetPriority(wId)
				wIdBackwardTT := bs.backwPq.GetPriority(wId)

				newPathTT := wIdForwardTT +
					wIdBackwardTT
				if exploredByBackwSearch && util.Lt(newPathTT, bs.shortestTT) {

					bs.shortestTT = newPathTT

					bs.forwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, false)
					bs.backwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, true)
				}
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				ovWId := bs.engine.offsetOverlayNoTurnCost(w)
				oldOverlayWIdTT := bs.forwPq.GetPriority(ovWId)
				wLabelled := util.Lt(oldOverlayWIdTT, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newTT, oldOverlayWIdTT)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
					newPar.SetIsOverlayVertex()

					if !wLabelled {
						vertexInfo := da.NewVertexInfo(newTT, newPar)
						queryKey := da.NewCRPQueryKeyNoTurnCost(ovWId, wQueryLevel, true)
						bs.forwPq.Insert(ovWId, priority, vertexInfo, queryKey)
					} else {
						bs.forwPq.DecreaseKey(ovWId, priority, newTT, newPar)
					}
				}

				exploredByBackwSearch := bs.backwPq.IsExplored(ovWId)
				newEstSpCost := bs.forwPq.GetPriority(ovWId) + bs.backwPq.GetPriority(ovWId)
				if exploredByBackwSearch && util.Lt(newEstSpCost, bs.shortestTT) {
					// if overlay vertex w explored by backward search, check whether we can improve the shortestPath
					bs.shortestTT = newEstSpCost

					mid := da.NewVertexEdgePair(w, da.INVALID_EDGE_ID, false)
					mid.SetIsOverlayVertex()
					bs.forwMid = mid
					bs.backwMid = mid
				}
			}
		}

		exploredByBackwSearch := bs.backwPq.IsExplored(ovVId)
		newEstSpCost := bs.forwPq.GetPriority(ovVId) + bs.backwPq.GetPriority(ovVId)
		if exploredByBackwSearch && util.Lt(newEstSpCost, bs.shortestTT) {

			bs.shortestTT = newEstSpCost
			mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
			mid.SetIsOverlayVertex()
			bs.forwMid = mid
			bs.backwMid = mid
		}
	})
}

func (bs *CRPALTQuery[W]) backwardOverlayGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search backward on overlay graph

	uId := uItem.GetNode()

	uQueryLevel := uItem.GetQueryLevel()
	adjUId := bs.engine.adjustOffsetOverlayNoTurnCost(uId)

	bs.engine.overlayGraph.ForInNeighborsOf(adjUId, int(uQueryLevel), func(v da.Index, wOffset da.Index) {
		shortcutWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTT := bs.backwPq.GetPriority(uId) + shortcutWeight

		if util.Ge(newTT, util.Infinity[W]()) {
			return
		}

		ovVId := bs.engine.offsetOverlayNoTurnCost(v)
		// traverse edge to next cell
		vOriEdgeId := vVertex.GetCutEdge()

		inEdgeWeight := bs.engine.getWeight(vOriEdgeId, false)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOrigVId()

		// relax edge
		oldOvVIdTT := bs.backwPq.GetPriority(ovVId)
		vLabelled := util.Lt(oldOvVIdTT, util.Infinity[W]())
		if !vLabelled || (vLabelled && util.Lt(newTT, oldOvVIdTT)) {
			newPar := da.NewVertexEdgePair(adjUId, da.INVALID_EDGE_ID, true)
			newPar.SetIsOverlayVertex()

			bs.backwPq.Set(ovVId, da.NewVertexInfo(newTT, newPar), da.NewCRPQueryKeyNoTurnCost(ovVId,
				uint8(uQueryLevel), true))

			bs.backwPq.Explore(ovVId)

			newTT = bs.backwPq.GetPriority(ovVId) + inEdgeWeight

			if util.Ge(newTT, util.Infinity[W]()) {
				return
			}

			// ALT (A*, landmarks, and triangle inequality) lowerbound/heuristic function
			_, prw := bs.engine.lm.FindTighestConsistentLowerBound(wId, source, target, bs.activeLandmarks)
			priority := newTT + prw

			if wQueryLevel == 0 {

				// relax edge
				oldWIdTT := bs.backwPq.GetPriority(wId)
				wLabelled := util.Lt(oldWIdTT, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newTT, oldWIdTT)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, true)
					newPar.SetIsOverlayVertex()

					if wLabelled {
						bs.backwPq.DecreaseKey(wId, priority, newTT, newPar)
					} else {
						queryKey := da.NewCRPQueryKeyNoTurnCost(wId, 0, false)
						vertexInfo := da.NewVertexInfo(newTT, newPar)
						bs.backwPq.Insert(wId, priority, vertexInfo, queryKey)
					}
				}

				wIdBackwardTT := bs.backwPq.GetPriority(wId)
				exploredByForwSearch := bs.forwPq.IsExplored(wId)
				wIdForwardTT := bs.forwPq.GetPriority(wId)

				newPathTT := wIdForwardTT +
					wIdBackwardTT
				if exploredByForwSearch && util.Lt(newPathTT, bs.shortestTT) {

					bs.shortestTT = newPathTT
					bs.forwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, false)
					bs.backwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, true)
				}
			} else {
				ovWId := bs.engine.offsetOverlayNoTurnCost(w)
				oldOverlayWIdTT := bs.backwPq.GetPriority(ovWId)
				wLabelled := util.Lt(oldOverlayWIdTT, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newTT, oldOverlayWIdTT)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, true)
					newPar.SetIsOverlayVertex()
					if !wLabelled {
						queryKey := da.NewCRPQueryKeyNoTurnCost(ovWId, wQueryLevel, true)
						vertexInfo := da.NewVertexInfo(newTT, newPar)
						bs.backwPq.Insert(ovWId, priority, vertexInfo, queryKey)
					} else {
						bs.backwPq.DecreaseKey(ovWId, priority, newTT, newPar)
					}
				}

				exploredByForwSearch := bs.forwPq.IsExplored(ovWId)
				newEstSpCost := bs.forwPq.GetPriority(ovWId) + bs.backwPq.GetPriority(ovWId)
				if exploredByForwSearch && util.Lt(newEstSpCost, bs.shortestTT) {
					bs.shortestTT = newEstSpCost

					mid := da.NewVertexEdgePair(w, da.INVALID_EDGE_ID, false)
					mid.SetIsOverlayVertex()
					bs.forwMid = mid
					bs.backwMid = mid
				}
			}
		}

		exploredByForwSearch := bs.forwPq.IsExplored(ovVId)
		newEstSpCost := bs.backwPq.GetPriority(ovVId) + bs.forwPq.GetPriority(ovVId)
		if exploredByForwSearch && util.Lt(newEstSpCost, bs.shortestTT) {
			bs.shortestTT = newEstSpCost
			mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
			mid.SetIsOverlayVertex()
			bs.forwMid = mid
			bs.backwMid = mid
		}
	})
}

func (bs *CRPALTQuery[W]) Preallocate() {
	bs.forwPq = bs.engine.fHeapNoTurnCostPool.Get().(*da.QueryHeap[da.CRPQueryKeyNoTurnCost, W])
	bs.backwPq = bs.engine.bHeapNoTurnCostPool.Get().(*da.QueryHeap[da.CRPQueryKeyNoTurnCost, W])
}

func (bs *CRPALTQuery[W]) Done() {

	bs.forwPq.Clear()
	bs.backwPq.Clear()
	bs.engine.fHeapNoTurnCostPool.Put(bs.forwPq)
	bs.engine.bHeapNoTurnCostPool.Put(bs.backwPq)
}

func (bs *CRPALTQuery[W]) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(bs.numScannedVertices)
	return efficiency, bs.numScannedVertices, bs.runtime, bs.pathUnpackingRuntime
}

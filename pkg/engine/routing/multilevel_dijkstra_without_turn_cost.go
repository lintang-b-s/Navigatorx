package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// penjelasan algoritma kueri (tanpa turn cost) dari Customizable Route Planning ada di section 3.5:  https://drive.google.com/file/d/1Ek7xLIsl5Kv-CSR6RdlRNYuA5iFIJaDl/view
// pdf password: <my-github-username>-<my-birth-year>-<my gdrive email without @gmail.com>

type CRPQuery[W util.RoutingNumber] struct {
	engine       *CRPRoutingEngine[W]
	shortestCost W
	forwMid      da.VertexEdgePair
	backwMid     da.VertexEdgePair

	forwPq  *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W]
	backwPq *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W]

	sCellNumber da.Pv
	tCellNumber da.Pv

	numExploredVertices        int
	numExploredOverlayVertices int
	runtime                    int64
	pathUnpackingRuntime       int64
}

func NewCRPQuery[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *CRPQuery[W] {
	crpQuery := &CRPQuery[W]{
		engine: engine,

		forwMid:  da.NewVertexEdgePair(0, 0, false),
		backwMid: da.NewVertexEdgePair(0, 0, true),

		numExploredVertices:        0,
		numExploredOverlayVertices: 0,
		runtime:                    0,
		pathUnpackingRuntime:       0,
	}

	crpQuery.Preallocate()
	return crpQuery
}

// Reset clears per-query state on a pooled CRPQuery so it can
// be reused.
func (bs *CRPQuery[W]) Reset() {

	bs.shortestCost = 2 * util.Infinity[W]()

	bs.forwMid = da.NewVertexEdgePair(0, 0, false)
	bs.backwMid = da.NewVertexEdgePair(0, 0, true)

	bs.sCellNumber = 0
	bs.tCellNumber = 0

	bs.numExploredVertices = 0
	bs.numExploredOverlayVertices = 0
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

ini adalah implementasi dari fase query dari Customizable Route Planning (CRP) [1] / multilevel-dijkstra tanpa incorporate turn costs.
intinya cuma bidirectional dijkstra pada graf yang consisiting of overlay graph H, cell C_s, cell C_t. C_s adalah cell level 1 yang mengandung vertex s hasil multilevel partition (lihat package partitioner).
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

proof of correctness bidirectional dijkstra bisa dilihat di proof of correctness Algorithm 2 di ref [3]. di implementasi ini, kita apply bidirectional dijkstra pada graf consisting of overlay graph H, cell C_s, cell C_t yang mana s-t shortest path yang dihasilkan
ekuivalen dengan s-t shortest path pada graf G.


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + n_p + m_p + k * \hat{m_p}) * log (n_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all vertices in lowest level cell that containing s or t and all overlay vertices in all cell other than level 1 cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, di C_s/C_t kita masih relax all edges inside C_s/C_t yang mana at most m_p, ketika di overlay graph H, kita relax shortcut edges yang mana at most k * \hat{m_p}
extract-min at most O(n_p+n_o) operations, yang kita insert di pq adlaah vertices inside C_s/C_t yang mana at most n_p dan overlay vertices in overlay graph H yang mana at most n_o.


versi query phase dari CRP yang support turn costs & turn restrictions [4] dapat dilihat pada multilevel_dijkstra.go, yang mana implementasi dari query phase dari CRP yang menggunakan compact graph representation/turn tables yang mana simulates arc-based expanded graph representation, diadaptasi dari implementasi CRP: https://github.com/michaelwegner/CRP, jauh lebih ribet dari  implemetasi ini....
implementasi query phase dari CRP yang support turn costs & turn restrictions yang jauh lebih mudah dipahami dapat dilihat di  https://github.com/Project-OSRM/osrm-backend/blob/master/include/engine/routing_algorithms/routing_base_mld.hpp   yang mana osrm pakai arc-based expanded graph : https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation

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

	sVertexData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	tVertexData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, true))
	sQueryKey := da.NewCRPQueryKeyNoTurnCost(s, 0, false)
	tQueryKey := da.NewCRPQueryKeyNoTurnCost(t, 0, false)
	bs.forwPq.Insert(s, 0, sVertexData, sQueryKey)
	bs.backwPq.Insert(t, 0, tVertexData, tQueryKey)

	for bs.forwPq.Size() > 0 && bs.backwPq.Size() > 0 {
		minForward := bs.forwPq.GetMinrank()
		minBackward := bs.backwPq.GetMinrank()
		if util.Ge(minForward+minBackward, W(float64(bs.shortestCost))) {
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
			bs.numExploredOverlayVertices++
		}

		queryKey = bs.backwPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.backwPq.Explore(uItem.GetNode())

			bs.backwardGraphSearch(uItem, s, t)
		} else {
			bs.backwPq.Explore(uItem.GetNode())

			bs.backwardOverlayGraphSearch(uItem, s, t)
			bs.numExploredOverlayVertices++
		}

		bs.numExploredVertices += 2
	}

	if bs.shortestCost == 2*util.Infinity[W]() {
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

	return bs.shortestCost, edgeIdPath, true
}

/*
graphSearch. forward search dari bidirectional ALT di sel c1(s) atau c1(t).
*/
func (bs *CRPQuery[W]) forwardGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {

	uId := uItem.GetNode()
	uCost := bs.forwPq.GetCost(uId)

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOfNoTurnCost(uId, func(eId, head, entryPoint da.Index) {
		vId := head

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		eWeight := bs.engine.getWeight(eId, true)

		// get cost to reach v through u
		newVCost := uCost + eWeight

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}

		if vQueryLevel == 0 {

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in dijkstra

			// relax edge
			oldVCost := bs.forwPq.GetCost(vId)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldVCost)) {
				if vLabelled {
					// newVCost is bsCellNumberetter, update the forwardData
					// is key already in the priority queue, decrease its key

					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)
					bs.forwPq.DecreaseKey(vId, newVCost, newVCost, newPar)
				} else if !vLabelled {

					vData := da.NewVertexData(newVCost,
						da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))
					queryKey := da.NewCRPQueryKeyNoTurnCost(vId, 0, false)
					// is key not in the priority queue, insert it
					bs.forwPq.Insert(vId, newVCost, vData, queryKey)
				}
			}

			exploredByBackwSearch := bs.backwPq.IsExplored(vId)
			vIdForwCost := bs.forwPq.GetCost(vId)
			vIdBackwCost := bs.backwPq.GetCost(vId)

			newPathCost := vIdForwCost + vIdBackwCost
			if exploredByBackwSearch && util.Lt(newPathCost, bs.shortestCost) {
				bs.shortestCost = newPathCost
				bs.forwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, false)
				bs.backwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, true)
			}
		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := bs.engine.graph.GetOverlayVertex(vId, entryPoint, false)
			ovVId := bs.engine.offsetOverlayNoTurnCost(v)
			oldOverlayVIdCost := bs.forwPq.GetCost(ovVId)
			vLabelled := util.Lt(oldOverlayVIdCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldOverlayVIdCost)) {
				newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)

				if !vLabelled {
					vData := da.NewVertexData(newVCost,
						newPar)

					queryKey := da.NewCRPQueryKeyNoTurnCost(ovVId, vQueryLevel, true)
					bs.forwPq.Insert(ovVId, newVCost, vData, queryKey)
				} else {
					bs.forwPq.DecreaseKey(ovVId, newVCost, newVCost, newPar)
				}
			}

			exploredByBackwSearch := bs.backwPq.IsExplored(ovVId)
			// if v explored by backward search, check whether we can improve the shortestPath
			newEstSpCost := bs.forwPq.GetCost(ovVId) + bs.backwPq.GetCost(ovVId)
			if exploredByBackwSearch && util.Lt(newEstSpCost, bs.shortestCost) {
				bs.shortestCost = newEstSpCost

				mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
				mid.SetIsOverlayVertex()
				bs.forwMid = mid
				bs.backwMid = mid
			}
		}
	})
}

func (bs *CRPQuery[W]) backwardGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search backward on graph level 1

	uId := uItem.GetNode()

	uCost := bs.backwPq.GetCost(uId)

	bs.engine.graph.ForInEdgesOfNoTurnCost(uId, func(eId, tail, exitPoint da.Index) {
		vId := tail

		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		eWeight := bs.engine.getWeight(eId, false)

		newVCost := uCost + eWeight

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}

		if vQueryLevel == 0 {

			// relax edge
			oldVCost := bs.backwPq.GetCost(vId)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldVCost)) {

				if vLabelled {
					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, true)
					bs.backwPq.DecreaseKey(vId, newVCost, newVCost, newPar)
				} else {
					vData := da.NewVertexData(newVCost,
						da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))
					queryKey := da.NewCRPQueryKeyNoTurnCost(vId, 0, false)
					bs.backwPq.Insert(vId, newVCost, vData, queryKey)
				}
			}

			exploredByForwSearch := bs.forwPq.IsExplored(vId)

			vIdBackwCost := bs.backwPq.GetCost(vId)
			vIdForwCost := bs.forwPq.GetCost(vId)
			newPathCost := vIdForwCost +
				vIdBackwCost
			if exploredByForwSearch && util.Lt(newPathCost, bs.shortestCost) {

				bs.shortestCost = newPathCost

				bs.forwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, false)
				bs.backwMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, true)
			}
		} else {
			// v is in another cell on higher level
			// Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			v, _ := bs.engine.graph.GetOverlayVertex(vId, exitPoint, true)
			ovVId := bs.engine.offsetOverlayNoTurnCost(v)
			oldOverlayVIdCost := bs.backwPq.GetCost(ovVId)
			vLabelled := util.Lt(oldOverlayVIdCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldOverlayVIdCost)) {
				newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, true)

				if !vLabelled {
					vVertexData := da.NewVertexData(newVCost,
						newPar)
					queryKey := da.NewCRPQueryKeyNoTurnCost(ovVId, vQueryLevel, true)
					bs.backwPq.Insert(ovVId, newVCost, vVertexData, queryKey)
				} else {

					bs.backwPq.DecreaseKey(ovVId, newVCost, newVCost, newPar)
				}
			}

			exploredByForwSearch := bs.forwPq.IsExplored(ovVId)
			newEstSpCost := bs.forwPq.GetCost(ovVId) + bs.backwPq.GetCost(ovVId)
			if exploredByForwSearch && util.Lt(newEstSpCost, bs.shortestCost) {
				bs.shortestCost = newEstSpCost

				mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
				mid.SetIsOverlayVertex()
				bs.forwMid = mid
				bs.backwMid = mid
			}
		}
	})
}

func (bs *CRPQuery[W]) forwardOverlayGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search on overlay graph

	uId := uItem.GetNode() // overlay vertex id

	uQueryLevel := int(uItem.GetQueryLevel())

	adjUId := bs.engine.adjustOffsetOverlayNoTurnCost(uId)

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	bs.engine.overlayGraph.ForOutNeighborsOf(adjUId, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newVCost := bs.forwPq.GetCost(uId) + shortcutOutWeight

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}
		ovVId := bs.engine.offsetOverlayNoTurnCost(v)

		// traverse edge to next cell
		vCutEdgeId := vVertex.GetCutEdge()

		eWeight := bs.engine.getWeight(vCutEdgeId, true)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOrigVId()

		// relax edge
		oldOverlayVIdCost := bs.forwPq.GetCost(ovVId)
		vLabelled := util.Lt(oldOverlayVIdCost, util.Infinity[W]())
		if !vLabelled || (vLabelled && util.Lt(newVCost, oldOverlayVIdCost)) {
			vPar := da.NewVertexEdgePair(adjUId, da.INVALID_EDGE_ID, false)
			vPar.SetIsOverlayVertex()
			bs.forwPq.Set(ovVId, da.NewVertexData(newVCost,
				vPar), da.NewCRPQueryKeyNoTurnCost(ovVId, uint8(uQueryLevel), true))

			bs.forwPq.Explore(ovVId)

			newVCost = bs.forwPq.GetCost(ovVId) + eWeight

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				oldWIdCost := bs.forwPq.GetCost(wId)
				wLabelled := util.Lt(oldWIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldWIdCost)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
					newPar.SetIsOverlayVertex()

					if wLabelled {
						bs.forwPq.DecreaseKey(wId, newVCost, newVCost, newPar)
					} else {
						vData := da.NewVertexData(newVCost, newPar)
						queryKey := da.NewCRPQueryKeyNoTurnCost(wId, 0, false)
						bs.forwPq.Insert(wId, newVCost, vData, queryKey)
					}
				}

				exploredByBackwSearch := bs.backwPq.IsExplored(wId)

				wIdForwardCost := bs.forwPq.GetCost(wId)
				wIdBackwardCost := bs.backwPq.GetCost(wId)

				newPathCost := wIdForwardCost +
					wIdBackwardCost
				if exploredByBackwSearch && util.Lt(newPathCost, bs.shortestCost) {

					bs.shortestCost = newPathCost

					bs.forwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, false)
					bs.backwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, true)
				}
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				ovWId := bs.engine.offsetOverlayNoTurnCost(w)
				oldOverlayWIdCost := bs.forwPq.GetCost(ovWId)
				wLabelled := util.Lt(oldOverlayWIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldOverlayWIdCost)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
					newPar.SetIsOverlayVertex()

					if !wLabelled {
						vData := da.NewVertexData(newVCost, newPar)
						queryKey := da.NewCRPQueryKeyNoTurnCost(ovWId, wQueryLevel, true)
						bs.forwPq.Insert(ovWId, newVCost, vData, queryKey)
					} else {
						bs.forwPq.DecreaseKey(ovWId, newVCost, newVCost, newPar)
					}
				}

				exploredByBackwSearch := bs.backwPq.IsExplored(ovWId)
				newEstSpCost := bs.forwPq.GetCost(ovWId) + bs.backwPq.GetCost(ovWId)
				if exploredByBackwSearch && util.Lt(newEstSpCost, bs.shortestCost) {
					// if overlay vertex w explored by backward search, check whether we can improve the shortestPath
					bs.shortestCost = newEstSpCost

					mid := da.NewVertexEdgePair(w, da.INVALID_EDGE_ID, false)
					mid.SetIsOverlayVertex()
					bs.forwMid = mid
					bs.backwMid = mid
				}
			}
		}

		exploredByBackwSearch := bs.backwPq.IsExplored(ovVId)
		newEstSpCost := bs.forwPq.GetCost(ovVId) + bs.backwPq.GetCost(ovVId)
		if exploredByBackwSearch && util.Lt(newEstSpCost, bs.shortestCost) {

			bs.shortestCost = newEstSpCost
			mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
			mid.SetIsOverlayVertex()
			bs.forwMid = mid
			bs.backwMid = mid
		}
	})
}

func (bs *CRPQuery[W]) backwardOverlayGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search backward on overlay graph

	uId := uItem.GetNode()

	uQueryLevel := uItem.GetQueryLevel()
	adjUId := bs.engine.adjustOffsetOverlayNoTurnCost(uId)

	bs.engine.overlayGraph.ForInNeighborsOf(adjUId, int(uQueryLevel), func(v da.Index, wOffset da.Index) {
		shortcutInWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newVCost := bs.backwPq.GetCost(uId) + shortcutInWeight

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}

		ovVId := bs.engine.offsetOverlayNoTurnCost(v)
		// traverse edge to next cell
		vCutEdgeId := vVertex.GetCutEdge()

		inEdgeWeight := bs.engine.getWeight(vCutEdgeId, false)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOrigVId()

		// relax edge
		oldOverlayVIdCost := bs.backwPq.GetCost(ovVId)
		vLabelled := util.Lt(oldOverlayVIdCost, util.Infinity[W]())
		if !vLabelled || (vLabelled && util.Lt(newVCost, oldOverlayVIdCost)) {
			newPar := da.NewVertexEdgePair(adjUId, da.INVALID_EDGE_ID, true)
			newPar.SetIsOverlayVertex()

			bs.backwPq.Set(ovVId, da.NewVertexData(newVCost, newPar), da.NewCRPQueryKeyNoTurnCost(ovVId,
				uint8(uQueryLevel), true))

			bs.backwPq.Explore(ovVId)

			newVCost = bs.backwPq.GetCost(ovVId) + inEdgeWeight

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			if wQueryLevel == 0 {

				// relax edge
				oldWIdCost := bs.backwPq.GetCost(wId)
				wLabelled := util.Lt(oldWIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldWIdCost)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, true)
					newPar.SetIsOverlayVertex()

					if wLabelled {
						bs.backwPq.DecreaseKey(wId, newVCost, newVCost, newPar)
					} else {
						queryKey := da.NewCRPQueryKeyNoTurnCost(wId, 0, false)
						vData := da.NewVertexData(newVCost, newPar)
						bs.backwPq.Insert(wId, newVCost, vData, queryKey)
					}
				}

				wIdBackwardCost := bs.backwPq.GetCost(wId)
				exploredByForwSearch := bs.forwPq.IsExplored(wId)
				wIdForwardCost := bs.forwPq.GetCost(wId)

				newPathCost := wIdForwardCost +
					wIdBackwardCost
				if exploredByForwSearch && util.Lt(newPathCost, bs.shortestCost) {

					bs.shortestCost = newPathCost
					bs.forwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, false)
					bs.backwMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, true)
				}
			} else {
				ovWId := bs.engine.offsetOverlayNoTurnCost(w)
				oldOverlayWIdCost := bs.backwPq.GetCost(ovWId)
				wLabelled := util.Lt(oldOverlayWIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldOverlayWIdCost)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, true)
					newPar.SetIsOverlayVertex()
					if !wLabelled {
						queryKey := da.NewCRPQueryKeyNoTurnCost(ovWId, wQueryLevel, true)
						vData := da.NewVertexData(newVCost, newPar)
						bs.backwPq.Insert(ovWId, newVCost, vData, queryKey)
					} else {
						bs.backwPq.DecreaseKey(ovWId, newVCost, newVCost, newPar)
					}
				}

				exploredByForwSearch := bs.forwPq.IsExplored(ovWId)
				newEstSpCost := bs.forwPq.GetCost(ovWId) + bs.backwPq.GetCost(ovWId)
				if exploredByForwSearch && util.Lt(newEstSpCost, bs.shortestCost) {
					bs.shortestCost = newEstSpCost

					mid := da.NewVertexEdgePair(w, da.INVALID_EDGE_ID, false)
					mid.SetIsOverlayVertex()
					bs.forwMid = mid
					bs.backwMid = mid
				}
			}
		}

		exploredByForwSearch := bs.forwPq.IsExplored(ovVId)
		newEstSpCost := bs.backwPq.GetCost(ovVId) + bs.forwPq.GetCost(ovVId)
		if exploredByForwSearch && util.Lt(newEstSpCost, bs.shortestCost) {
			bs.shortestCost = newEstSpCost
			mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
			mid.SetIsOverlayVertex()
			bs.forwMid = mid
			bs.backwMid = mid
		}
	})
}

func (bs *CRPQuery[W]) Preallocate() {
	bs.forwPq = bs.engine.fHeapNoTurnCostPool.Get().(*da.QueryHeap[da.CRPQueryKeyNoTurnCost, W])
	bs.backwPq = bs.engine.bHeapNoTurnCostPool.Get().(*da.QueryHeap[da.CRPQueryKeyNoTurnCost, W])
}

func (bs *CRPQuery[W]) Done() {

	bs.forwPq.Clear()
	bs.backwPq.Clear()
	bs.engine.fHeapNoTurnCostPool.Put(bs.forwPq)
	bs.engine.bHeapNoTurnCostPool.Put(bs.backwPq)
}

func (bs *CRPQuery[W]) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(bs.numExploredVertices)
	return efficiency, bs.numExploredVertices, bs.runtime, bs.pathUnpackingRuntime
}

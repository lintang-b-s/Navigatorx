package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPBidirectionalSearchWithoutTurnCost[W util.RoutingNumber] struct {
	engine             *CRPRoutingEngine[W]
	shortestTravelTime W
	forwardMid         da.VertexEdgePair
	backwardMid        da.VertexEdgePair

	forwardPq  *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W]
	backwardPq *da.QueryHeap[da.CRPQueryKeyNoTurnCost, W]

	sCellNumber da.Pv
	tCellNumber da.Pv

	numScannedVertices        int
	numScannedOverlayVertices int
	runtime                   int64
	pathUnpackingRuntime      int64
}

func NewCRPBidirectionalSearchWithoutTurnCost[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *CRPBidirectionalSearchWithoutTurnCost[W] {
	crpQuery := &CRPBidirectionalSearchWithoutTurnCost[W]{
		engine: engine,

		forwardMid:  da.NewVertexEdgePair(0, 0, false),
		backwardMid: da.NewVertexEdgePair(0, 0, true),

		numScannedVertices:        0,
		numScannedOverlayVertices: 0,
		runtime:                   0,
		pathUnpackingRuntime:      0,
	}

	crpQuery.Preallocate()
	return crpQuery
}

// Reset clears per-query state on a pooled CRPBidirectionalSearchWithoutTurnCost so it can
// be reused.
func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) Reset() {

	bs.shortestTravelTime = 2 * util.Infinity[W]()

	bs.forwardMid = da.NewVertexEdgePair(0, 0, false)
	bs.backwardMid = da.NewVertexEdgePair(0, 0, true)

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

ini adalah implementasi dari fase query dari Customizable Route Planning (CRP) [1] / multilevel-dijkstra tanpa incorporate turn costs.
intinya cuma bidirectional dijkstra pada graf yang consisiting of overlay graph H, cell C_s, cell C_t. C_s adalah cell level 1 yang mengandung vertex s hasil multilevel partition (lihat package partitioner).
Setiap cell C_v memiliki vertices (all inside cell C_v) dan edges (semua endpoints nya inside C_v), vertices dan edges dari cell C_v adalah subset dari vertices dan edges dari graf G.
overlay graph H adalah graf yang mengandung all boundary vertices, all cut/boundary edges, all shortcut edges di setiap cells hasil multilevel partitioning.
boundary vertices adalah vertices yang punya setidaknya satu edge yang ekdua endpoint nya (tail dan head) di cell yang berbeda, edge yang kedua endpointnya in different cell ini disebut cut edge.
entry boundary vertex adalah boundary vertex yang dia jadi head dari cut edge. exit boundary vertex adalah boundary vertex yang dia jadi tail dari cut edge.

shortcut dari setiap cell di overlay graph adalah shortest path dari entry boundary vertex ke exit boundary vertex yang dicompute dengan hanya menggunakan vertices dan edges inside that cell.

Customizable Route Planning (CRP) adalah extensi dari algoritma HiTi [2] yang diterapkan pada road network graph.
correctness dari algoritma ini dapat dilihat pada proof dari theorem 4.4 ref [2]. inti dari theorem 4.4 adalah shortest path dari simpul s ke simpul t pada graf yang terdiri dari overlay graph H, cell C_s, cell C_t ekuivalen
dengan s-t shortest path pada graf G.
untuk any s-t shortest path, kita bisa decompose edges penyusun s-t shortest path dengan edges inside cell C_s, edges inside C_t, cut edges in any cells, edges inside any cell (selain C_s dan C_t).
tapi karena di fase kustomisasi CRP [1] dan HiTi [2], kita compute shortcuts di setiap cell yang mana adalah shortest path dari entry boundary vertex ke exit boundary vertex dengan hanya menggunakan vertices and edges inside that cell.
bagian "edges inside any cell (selain C_s dan C_t)" bisa kita ganti dengan shortcuts di overlay graph H yang udah kita precompute di fase kustomisasi.

proof of correctness bidirectional dijkstra bisa dilihat di proof of correctness Algorithm 2 di ref [3]. di implementasi ini, kita apply bidirectional dijkstra pada graf consisting of overlay graph H, cell C_s, cell C_t yang mana s-t shortest path yang dihasilkan
ekuivalen dengan s-t shortest path dengan menggunakan graf G.


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + n_p + m_p + k * \hat{m_p}) * log (n_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all vertices in lowest level cell that containing s or t and all overlay vertices in all cell other than level 1 cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, di C_s/C_t kita masih relax all edges inside C_s/C_t yang mana at most m_p, ketika di overlay graph H, kita relax shortcut edges yang mana at most k * \hat{m_p}
extract-min at most O(n_p+n_o) operations, yang kita insert di pq adlaah vertices inside C_s/C_t yang mana at most n_p dan overlay vertices in overlay graph H yang mana at most n_o.


versi query phase dari CRP yang support turn costs & turn restrictions [4] dapat dilihat pada multilevel_dijkstra.go, yang mana implementasi dari query phase dari CRP yang menggunakan compact graph representation/turn tables yang mana simulates arc-based expanded graph representation, diadaptasi dari implementasi CRP: https://github.com/michaelwegner/CRP, jauh lebih ribet dari  implemetasi ini....
implementasi query phase dari CRP yang support turn costs & turn restrictions yang jauh lebih mudah dipahami dapat dilihat di  https://github.com/Project-OSRM/osrm-backend/blob/master/include/engine/routing_algorithms/routing_base_mld.hpp   yang mana osrm pakai arc-based expanded graph : https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation

*/

func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) ShortestPathSearch(s, t da.Index) (W, []da.Index, bool) {

	defer bs.Done()
	now := time.Now()

	if s == t {
		return 0, EmptyIndexSet, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	bs.shortestTravelTime = 2 * util.Infinity[W]()

	sVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, false))
	tVertexInfo := da.NewVertexInfo(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, da.INVALID_EDGE_ID, true))
	sQueryKey := da.NewCRPQueryKeyNoTurnCost(s, 0, false)
	tQueryKey := da.NewCRPQueryKeyNoTurnCost(t, 0, false)
	bs.forwardPq.Insert(s, 0, sVertexInfo, sQueryKey)
	bs.backwardPq.Insert(t, 0, tVertexInfo, tQueryKey)

	for bs.forwardPq.Size() > 0 && bs.backwardPq.Size() > 0 {
		minForward := bs.forwardPq.GetMinrank()
		minBackward := bs.backwardPq.GetMinrank()
		if util.Ge(minForward+minBackward, W(float64(bs.shortestTravelTime))) {
			break
		}

		queryKey := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()

		if !uItem.IsOverlay() {
			bs.forwardPq.Scan(uItem.GetNode())

			bs.forwardGraphSearch(uItem, s, t)
		} else {
			bs.forwardPq.Scan(uItem.GetNode())

			bs.forwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		queryKey = bs.backwardPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.backwardPq.Scan(uItem.GetNode())

			bs.backwardGraphSearch(uItem, s, t)
		} else {
			bs.backwardPq.Scan(uItem.GetNode())

			bs.backwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		bs.numScannedVertices += 2
	}

	if bs.shortestTravelTime == 2*util.Infinity[W]() {
		return util.Infinity[W](), EmptyIndexSet, false
	}

	packedPath := bs.engine.RetrievePackedPathNoTurnCost(bs.forwardMid, bs.backwardMid,
		bs.forwardPq, bs.backwardPq, bs.sCellNumber, s, t)

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	unpacker := NewPathUnpackerALTNoTurnCost(bs.engine)
	defer unpacker.DonePooled()
	edgeIdPath := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)

	return bs.shortestTravelTime, edgeIdPath, true
}

/*
graphSearch. bidirectional dijkstra search on graph level 1.
*/
func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) forwardGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {

	uId := uItem.GetNode()
	uTravelTime := bs.forwardPq.GetPriority(uId)

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOfNoTurnCost(uId, func(eId, head, entryPoint da.Index) {
		vId := head

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.getWeight(eId, true)

		// get cost to reach v through u
		newTravelTime := uTravelTime + edgeWeight

		if util.Ge(newTravelTime, util.Infinity[W]()) {
			return
		}

		if vQueryLevel == 0 {

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in dijkstra

			// relax edge
			oldVIdTravelTime := bs.forwardPq.GetPriority(vId)
			vAlreadyLabelled := util.Lt(oldVIdTravelTime, util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, oldVIdTravelTime)) {
				if vAlreadyLabelled {
					// newTravelTime is bsCellNumberetter, update the forwardInfo
					// is key already in the priority queue, decrease its key

					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)
					bs.forwardPq.DecreaseKey(vId, newTravelTime, newTravelTime, newPar)
				} else if !vAlreadyLabelled {

					vertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))
					queryKey := da.NewCRPQueryKeyNoTurnCost(vId, 0, false)
					// is key not in the priority queue, insert it
					bs.forwardPq.Insert(vId, newTravelTime, vertexInfo, queryKey)
				}
			}

			scannedByBackwardSearch := bs.backwardPq.IsScanned(vId)
			vIdForwardTravelTime := bs.forwardPq.GetPriority(vId)
			vIdBackwardTravelTime := bs.backwardPq.GetPriority(vId)

			newPathTravelTime := vIdForwardTravelTime + vIdBackwardTravelTime
			if scannedByBackwardSearch && util.Lt(newPathTravelTime, bs.shortestTravelTime) {
				bs.shortestTravelTime = newPathTravelTime
				bs.forwardMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, true)
			}
		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := bs.engine.graph.GetOverlayVertex(vId, entryPoint, false)
			overlayVId := bs.engine.offsetOverlayNoTurnCost(v)
			oldOverlayVIdTravelTime := bs.forwardPq.GetPriority(overlayVId)
			vAlreadyLabelled := util.Lt(oldOverlayVIdTravelTime, util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, oldOverlayVIdTravelTime)) {
				newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false)

				if !vAlreadyLabelled {
					vertexInfo := da.NewVertexInfo(newTravelTime,
						newPar)

					queryKey := da.NewCRPQueryKeyNoTurnCost(overlayVId, vQueryLevel, true)
					bs.forwardPq.Insert(overlayVId, newTravelTime, vertexInfo, queryKey)
				} else {
					bs.forwardPq.DecreaseKey(overlayVId, newTravelTime, newTravelTime, newPar)
				}
			}

			scannedByBackwardSearch := bs.backwardPq.IsScanned(overlayVId)
			// if v scanned by backward search, check whether we can improve the shortestPath
			newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)
			if scannedByBackwardSearch && util.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
				bs.shortestTravelTime = newEstimateShortestPathCost

				mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
				mid.SetIsOverlayVertex()
				bs.forwardMid = mid
				bs.backwardMid = mid

			}
		}
	})
}

func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) backwardGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search backward on graph level 1

	uId := uItem.GetNode()

	uTravelTime := bs.backwardPq.GetPriority(uId)

	bs.engine.graph.ForInEdgesOfNoTurnCost(uId, func(eId, tail, exitPoint da.Index) {
		vId := tail

		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.getWeight(eId, false)

		newTravelTime := uTravelTime + edgeWeight

		if util.Ge(newTravelTime, util.Infinity[W]()) {
			return
		}

		if vQueryLevel == 0 {

			// relax edge
			oldVIdTravelTime := bs.backwardPq.GetPriority(vId)
			vAlreadyLabelled := util.Lt(oldVIdTravelTime, util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, oldVIdTravelTime)) {

				if vAlreadyLabelled {
					newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, true)
					bs.backwardPq.DecreaseKey(vId, newTravelTime, newTravelTime, newPar)
				} else {
					vertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, false))
					queryKey := da.NewCRPQueryKeyNoTurnCost(vId, 0, false)
					bs.backwardPq.Insert(vId, newTravelTime, vertexInfo, queryKey)
				}
			}

			scannedByForwardSearch := bs.forwardPq.IsScanned(vId)

			vIdBackwardTravelTime := bs.backwardPq.GetPriority(vId)
			vIdForwardTravelTime := bs.forwardPq.GetPriority(vId)
			newPathTravelTime := vIdForwardTravelTime +
				vIdBackwardTravelTime
			if scannedByForwardSearch && util.Lt(newPathTravelTime, bs.shortestTravelTime) {

				bs.shortestTravelTime = newPathTravelTime

				bs.forwardMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, da.INVALID_EDGE_ID, true)
			}
		} else {
			// v is in another cell on higher level
			// Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			v, _ := bs.engine.graph.GetOverlayVertex(vId, exitPoint, true)
			overlayVId := bs.engine.offsetOverlayNoTurnCost(v)
			oldOverlayVIdTravelTime := bs.backwardPq.GetPriority(overlayVId)
			vAlreadyLabelled := util.Lt(oldOverlayVIdTravelTime, util.Infinity[W]())
			if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, oldOverlayVIdTravelTime)) {
				newPar := da.NewVertexEdgePair(uId, da.INVALID_EDGE_ID, true)

				if !vAlreadyLabelled {
					vVertexInfo := da.NewVertexInfo(newTravelTime,
						newPar)
					queryKey := da.NewCRPQueryKeyNoTurnCost(overlayVId, vQueryLevel, true)
					bs.backwardPq.Insert(overlayVId, newTravelTime, vVertexInfo, queryKey)
				} else {

					bs.backwardPq.DecreaseKey(overlayVId, newTravelTime, newTravelTime, newPar)
				}
			}

			scannedByForwardSearch := bs.forwardPq.IsScanned(overlayVId)
			newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)
			if scannedByForwardSearch && util.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
				bs.shortestTravelTime = newEstimateShortestPathCost

				mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
				mid.SetIsOverlayVertex()
				bs.forwardMid = mid
				bs.backwardMid = mid

			}
		}
	})
}

func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) forwardOverlayGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search on overlay graph

	uId := uItem.GetNode() // overlay vertex id

	uQueryLevel := int(uItem.GetQueryLevel())

	adjUId := bs.engine.adjustOffsetOverlayNoTurnCost(uId)

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	bs.engine.overlayGraph.ForOutNeighborsOf(adjUId, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.forwardPq.GetPriority(uId) + shortcutOutEdgeWeight

		if util.Ge(newTravelTime, util.Infinity[W]()) {
			return
		}
		overlayVId := bs.engine.offsetOverlayNoTurnCost(v)

		// traverse edge to next cell
		vOriEdgeId := vVertex.GetOriginalEdge()

		edgeWeight := bs.engine.getWeight(vOriEdgeId, true)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOriginalVertex()

		// relax edge
		oldOverlayVIdTravelTime := bs.forwardPq.GetPriority(overlayVId)
		vAlreadyLabelled := util.Lt(oldOverlayVIdTravelTime, util.Infinity[W]())
		if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, oldOverlayVIdTravelTime)) {
			vPar := da.NewVertexEdgePair(adjUId, da.INVALID_EDGE_ID, false)
			vPar.SetIsOverlayVertex()
			bs.forwardPq.Set(overlayVId, da.NewVertexInfo(newTravelTime,
				vPar), da.NewCRPQueryKeyNoTurnCost(overlayVId, uint8(uQueryLevel), true))

			bs.forwardPq.Scan(overlayVId)

			newTravelTime = bs.forwardPq.GetPriority(overlayVId) + edgeWeight

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				oldWIdTravelTime := bs.forwardPq.GetPriority(wId)
				wAlreadyLabelled := util.Lt(oldWIdTravelTime, util.Infinity[W]())
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, oldWIdTravelTime)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
					newPar.SetIsOverlayVertex()

					if wAlreadyLabelled {
						bs.forwardPq.DecreaseKey(wId, newTravelTime, newTravelTime, newPar)
					} else {
						vertexInfo := da.NewVertexInfo(newTravelTime, newPar)
						queryKey := da.NewCRPQueryKeyNoTurnCost(wId, 0, false)
						bs.forwardPq.Insert(wId, newTravelTime, vertexInfo, queryKey)
					}
				}

				scannedByBackwardSearch := bs.backwardPq.IsScanned(wId)

				wIdForwardTravelTime := bs.forwardPq.GetPriority(wId)
				wIdBackwardTravelTime := bs.backwardPq.GetPriority(wId)

				newPathTravelTime := wIdForwardTravelTime +
					wIdBackwardTravelTime
				if scannedByBackwardSearch && util.Lt(newPathTravelTime, bs.shortestTravelTime) {

					bs.shortestTravelTime = newPathTravelTime

					bs.forwardMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, false)
					bs.backwardMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, true)
				}
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				overlayWId := bs.engine.offsetOverlayNoTurnCost(w)
				oldOverlayWIdTravelTime := bs.forwardPq.GetPriority(overlayWId)
				wAlreadyLabelled := util.Lt(oldOverlayWIdTravelTime, util.Infinity[W]())
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, oldOverlayWIdTravelTime)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
					newPar.SetIsOverlayVertex()

					if !wAlreadyLabelled {
						vertexInfo := da.NewVertexInfo(newTravelTime, newPar)
						queryKey := da.NewCRPQueryKeyNoTurnCost(overlayWId, wQueryLevel, true)
						bs.forwardPq.Insert(overlayWId, newTravelTime, vertexInfo, queryKey)
					} else {
						bs.forwardPq.DecreaseKey(overlayWId, newTravelTime, newTravelTime, newPar)
					}
				}

				scannedByBackwardSearch := bs.backwardPq.IsScanned(overlayWId)
				newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayWId) + bs.backwardPq.GetPriority(overlayWId)
				if scannedByBackwardSearch && util.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
					// if overlay vertex w scanned by backward search, check whether we can improve the shortestPath
					bs.shortestTravelTime = newEstimateShortestPathCost

					mid := da.NewVertexEdgePair(w, da.INVALID_EDGE_ID, false)
					mid.SetIsOverlayVertex()
					bs.forwardMid = mid
					bs.backwardMid = mid
				}
			}
		}

		scannedByBackwardSearch := bs.backwardPq.IsScanned(overlayVId)
		newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)
		if scannedByBackwardSearch && util.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {

			bs.shortestTravelTime = newEstimateShortestPathCost

			mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
			mid.SetIsOverlayVertex()
			bs.forwardMid = mid
			bs.backwardMid = mid

		}
	})
}

func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) backwardOverlayGraphSearch(uItem da.CRPQueryKeyNoTurnCost, source, target da.Index) {
	// search backward on overlay graph

	uId := uItem.GetNode()

	uQueryLevel := uItem.GetQueryLevel()
	adjUId := bs.engine.adjustOffsetOverlayNoTurnCost(uId)

	bs.engine.overlayGraph.ForInNeighborsOf(adjUId, int(uQueryLevel), func(v da.Index, wOffset da.Index) {
		shortcutInEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.backwardPq.GetPriority(uId) + shortcutInEdgeWeight

		if util.Ge(newTravelTime, util.Infinity[W]()) {
			return
		}

		overlayVId := bs.engine.offsetOverlayNoTurnCost(v)
		// traverse edge to next cell
		vOriEdgeId := vVertex.GetOriginalEdge()

		inEdgeWeight := bs.engine.getWeight(vOriEdgeId, false)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOriginalVertex()

		// relax edge
		oldOverlayVIdTravelTime := bs.backwardPq.GetPriority(overlayVId)
		vAlreadyLabelled := util.Lt(oldOverlayVIdTravelTime, util.Infinity[W]())
		if !vAlreadyLabelled || (vAlreadyLabelled && util.Lt(newTravelTime, oldOverlayVIdTravelTime)) {
			newPar := da.NewVertexEdgePair(adjUId, da.INVALID_EDGE_ID, true)
			newPar.SetIsOverlayVertex()

			bs.backwardPq.Set(overlayVId, da.NewVertexInfo(newTravelTime, newPar), da.NewCRPQueryKeyNoTurnCost(overlayVId,
				uint8(uQueryLevel), true))

			bs.backwardPq.Scan(overlayVId)

			newTravelTime = bs.backwardPq.GetPriority(overlayVId) + inEdgeWeight

			if util.Ge(newTravelTime, util.Infinity[W]()) {
				return
			}

			if wQueryLevel == 0 {

				// relax edge
				oldWIdTravelTime := bs.backwardPq.GetPriority(wId)
				wAlreadyLabelled := util.Lt(oldWIdTravelTime, util.Infinity[W]())
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, oldWIdTravelTime)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, true)
					newPar.SetIsOverlayVertex()

					if wAlreadyLabelled {
						bs.backwardPq.DecreaseKey(wId, newTravelTime, newTravelTime, newPar)
					} else {
						queryKey := da.NewCRPQueryKeyNoTurnCost(wId, 0, false)
						vertexInfo := da.NewVertexInfo(newTravelTime, newPar)
						bs.backwardPq.Insert(wId, newTravelTime, vertexInfo, queryKey)
					}
				}

				wIdBackwardTravelTime := bs.backwardPq.GetPriority(wId)
				scannedByForwardSearch := bs.forwardPq.IsScanned(wId)
				wIdForwardTravelTime := bs.forwardPq.GetPriority(wId)

				newPathTravelTime := wIdForwardTravelTime +
					wIdBackwardTravelTime
				if scannedByForwardSearch && util.Lt(newPathTravelTime, bs.shortestTravelTime) {

					bs.shortestTravelTime = newPathTravelTime
					bs.forwardMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, false)
					bs.backwardMid = da.NewVertexEdgePair(wId, da.INVALID_EDGE_ID, true)
				}
			} else {
				overlayWId := bs.engine.offsetOverlayNoTurnCost(w)
				oldOverlayWIdTravelTime := bs.backwardPq.GetPriority(overlayWId)
				wAlreadyLabelled := util.Lt(oldOverlayWIdTravelTime, util.Infinity[W]())
				if !wAlreadyLabelled || (wAlreadyLabelled && util.Lt(newTravelTime, oldOverlayWIdTravelTime)) {
					newPar := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, true)
					newPar.SetIsOverlayVertex()

					if !wAlreadyLabelled {
						queryKey := da.NewCRPQueryKeyNoTurnCost(overlayWId, wQueryLevel, true)
						vertexInfo := da.NewVertexInfo(newTravelTime, newPar)
						bs.backwardPq.Insert(overlayWId, newTravelTime, vertexInfo, queryKey)
					} else {

						bs.backwardPq.DecreaseKey(overlayWId, newTravelTime, newTravelTime, newPar)
					}
				}

				scannedByForwardSearch := bs.forwardPq.IsScanned(overlayWId)
				newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayWId) + bs.backwardPq.GetPriority(overlayWId)
				if scannedByForwardSearch && util.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
					bs.shortestTravelTime = newEstimateShortestPathCost

					mid := da.NewVertexEdgePair(w, da.INVALID_EDGE_ID, false)
					mid.SetIsOverlayVertex()
					bs.forwardMid = mid
					bs.backwardMid = mid

				}
			}
		}

		scannedByForwardSearch := bs.forwardPq.IsScanned(overlayVId)
		newEstimateShortestPathCost := bs.backwardPq.GetPriority(overlayVId) + bs.forwardPq.GetPriority(overlayVId)
		if scannedByForwardSearch && util.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
			bs.shortestTravelTime = newEstimateShortestPathCost
			mid := da.NewVertexEdgePair(v, da.INVALID_EDGE_ID, false)
			mid.SetIsOverlayVertex()
			bs.forwardMid = mid
			bs.backwardMid = mid
		}
	})
}

func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) Preallocate() {
	bs.forwardPq = bs.engine.fHeapNoTurnCostPool.Get().(*da.QueryHeap[da.CRPQueryKeyNoTurnCost, W])
	bs.backwardPq = bs.engine.bHeapNoTurnCostPool.Get().(*da.QueryHeap[da.CRPQueryKeyNoTurnCost, W])
}

func (bs *CRPBidirectionalSearchWithoutTurnCost[W]) Done() {

	bs.forwardPq.Clear()
	bs.backwardPq.Clear()
	bs.engine.fHeapNoTurnCostPool.Put(bs.forwardPq)
	bs.engine.bHeapNoTurnCostPool.Put(bs.backwardPq)

}

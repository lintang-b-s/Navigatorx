package routing

import (
	"time"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type CRPQuery[W util.RoutingNumber] struct {
	engine               *CRPRoutingEngine[W]
	shortestCost         W
	mid                  da.ParentVertex
	fpq                  *da.QueryHeap[da.QueryKey, W]
	bpq                  *da.QueryHeap[da.QueryKey, W]
	sCellNumber          da.Pv
	tCellNumber          da.Pv
	viaVertices          []ViaVertex
	numExploredVertices  int
	runtime              int64
	pathUnpackingRuntime int64
	ub                   float64

	maxSearchRad W

	scpSet          map[uint64]uint8
	forAlternatives bool
	reroute         bool
}

func NewCRPQuery[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *CRPQuery[W] {
	crpQuery := &CRPQuery[W]{
		engine:               engine,
		mid:                  da.NewParentVertex(0),
		viaVertices:          make([]ViaVertex, 0),
		numExploredVertices:  0,
		runtime:              0,
		pathUnpackingRuntime: 0,
		ub:                   1.0,
		maxSearchRad:         2 * util.Infinity[W](),
	}

	crpQuery.Preallocate()
	return crpQuery
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
5. Towers, M. (2020). Bidirectional Dijkstra. https://www.homepages.ucl.ac.uk/~ucahmto/math/2020/05/30/bidirectional-dijkstra.html. Diakses tanggal: 5 Agustus 2026.

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
untuk any s-t shortest path, kita bisa decompose edges penyusun s-t shortest path dengan edges inside cell C_s, edges inside C_t, cut edges in any cells, atau edges inside any other cell (selain C_s dan C_t).
tapi karena di fase kustomisasi CRP [1] dan HiTi [2], kita compute shortcut edges di setiap cell yang mana adalah shortest path dari entry boundary vertex ke exit boundary vertex dengan hanya menggunakan vertices and edges inside that cell.
dengan menggunakan sifat optimal substructure dari shortest path, bagian "edges inside any cell (selain C_s dan C_t)" bisa kita ganti dengan shortcut edges di overlay graph H yang udah kita precompute di fase kustomisasi.

proof of correctness bidirectional dijkstra bisa dilihat di proof of correctness Algorithm 2 di ref [5] dan [3]. di implementasi ini, kita apply bidirectional dijkstra pada graf consisting of overlay graph H, cell C_s, cell C_t yang mana s-t shortest path yang dihasilkan
ekuivalen dengan s-t shortest path pada graf G.


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcut edges within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + n_p + m_p + k * \hat{m_p}) * log (n_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all vertices in lowest level cell that containing s or t and all overlay vertices in all cell other than level 1 cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, di C_s/C_t kita masih relax all edges inside C_s/C_t yang mana at most m_p, ketika di overlay graph H, kita relax shortcut edges yang mana at most k * \hat{m_p}
extract-min at most O(n_p+n_o) operations, yang kita insert di pq adlaah vertices inside C_s/C_t yang mana at most n_p dan overlay vertices in overlay graph H yang mana at most n_o.


inspired by crp query code implementation by michael wegner: https://github.com/michaelwegner/CRP/blob/master/algorithm/CRPQuery.cpp

*/

func (bs *CRPQuery[W]) ShortestPathSearch(s, t da.Index) (W, []da.Index, bool) {

	now := time.Now()

	if s == t {
		return 0, EmptyIndexSet, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	bs.shortestCost = 2 * util.Infinity[W]()

	sVertexData := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID))
	tVertexData := da.NewVData(W(0), da.NewParentVertex(da.INVALID_VERTEX_ID))
	sQueryKey := da.NewQKey(s, 0, false)
	tQueryKey := da.NewQKey(t, 0, false)
	bs.fpq.Insert(s, 0, sVertexData, sQueryKey)
	bs.bpq.Insert(t, 0, tVertexData, tQueryKey)

	addViaVertex := func(uqk da.QueryKey, forward bool, overlay bool) {
		if !overlay {
			u := uqk.GetNode()
			if forward {
				if bs.bpq.IsExplored(u) {
					bs.viaVertices = append(bs.viaVertices, NewViaVertex(u, false))
				}
			} else {
				if bs.fpq.IsExplored(u) {
					bs.viaVertices = append(bs.viaVertices, NewViaVertex(u, false))
				}
			}
		} else {
			uOvId := uqk.GetNode() // offseted overlay vertex id

			if forward {
				if bs.bpq.IsExplored(uOvId) {
					bs.viaVertices = append(bs.viaVertices, NewViaVertex(uOvId, true))
				}
			} else {
				if bs.fpq.IsExplored(uOvId) {
					bs.viaVertices = append(bs.viaVertices, NewViaVertex(uOvId, true))
				}
			}
		}
	}

	for bs.fpq.Size() > 0 && bs.bpq.Size() > 0 {
		minForward := bs.fpq.GetMinrank()
		minBackward := bs.bpq.GetMinrank()

		if util.Ge(minForward+minBackward, W(float64(bs.shortestCost)*(bs.ub))) || util.Ge(minForward+minBackward, W(bs.maxSearchRad)) {
			break
		}

		queryKey := bs.fpq.ExtractMin()
		uqk := queryKey.GetItem()

		if !uqk.IsOverlay() {
			bs.fpq.Explore(uqk.GetNode())
			bs.forwardGraphSearch(uqk, s, t)
			addViaVertex(uqk, true, false)
		} else {
			bs.fpq.Explore(uqk.GetNode())
			bs.forwardOverlayGraphSearch(uqk, s, t)
			addViaVertex(uqk, true, true)
		}

		queryKey = bs.bpq.ExtractMin()
		uqk = queryKey.GetItem()
		if !uqk.IsOverlay() {
			bs.bpq.Explore(uqk.GetNode())
			bs.backwardGraphSearch(uqk, s, t)
			addViaVertex(uqk, false, false)
		} else {
			bs.bpq.Explore(uqk.GetNode())
			bs.backwardOverlayGraphSearch(uqk, s, t)
			addViaVertex(uqk, false, true)
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

	unpacker := newPathUnpackerALT(bs.engine, true)
	edgeIdPath := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)
	bs.scpSet = unpacker.scSet
	bs.pathUnpackingRuntime = unpacker.runtime

	return bs.shortestCost, edgeIdPath, true
}

/*
forwardGraphSearch. forward search dari bidirectional Dijkstra di sel c1(s) atau c1(t).
*/
func (bs *CRPQuery[W]) forwardGraphSearch(uqk da.QueryKey, source, target da.Index) {

	uId := uqk.GetNode()
	uCost := bs.fpq.GetCost(uId)

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOf(uId, func(eId, head, entryPoint da.Index) {
		vId := head

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		eWeight := bs.engine.getWeight(eId, true)

		// get cost to reach v through u
		newVCost := uCost + eWeight

		if vQueryLevel == 0 {

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in dijkstra

			// relax edge
			oldVCost := bs.fpq.GetCost(vId)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				if vLabelled {
					// newVCost is bsCellNumberetter, update the forwardData
					// is key already in the priority queue, decrease its key

					newPar := da.NewParentVertex(uId)
					bs.fpq.DecreaseKey(vId, newVCost, newVCost, newPar)
				} else if !vLabelled {

					vData := da.NewVData(newVCost,
						da.NewParentVertex(uId))
					queryKey := da.NewQKey(vId, 0, false)
					// is key not in the priority queue, insert it
					bs.fpq.Insert(vId, newVCost, vData, queryKey)
				}
			}

			exploredByBackwSearch := bs.bpq.IsExplored(vId)
			vIdForwCost := bs.fpq.GetCost(vId)
			vIdBackwCost := bs.bpq.GetCost(vId)

			newEstSPCost := vIdForwCost + vIdBackwCost
			if exploredByBackwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
				bs.shortestCost = newEstSPCost
				bs.mid = da.NewParentVertex(vId)

			}
		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := bs.engine.graph.GetOverlayVertex(vId, entryPoint, false)
			vOvId := bs.engine.offsetOverlay(v)
			oldVCost := bs.fpq.GetCost(vOvId)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				newPar := da.NewParentVertex(uId)

				if !vLabelled {
					vData := da.NewVData(newVCost,
						newPar)

					queryKey := da.NewQKey(vOvId, vQueryLevel, true)
					bs.fpq.Insert(vOvId, newVCost, vData, queryKey)
				} else {
					bs.fpq.DecreaseKey(vOvId, newVCost, newVCost, newPar)
				}
			}

			exploredByBackwSearch := bs.bpq.IsExplored(vOvId)
			// if v explored by backward search, check whether we can improve the shortestPath
			newEstSPCost := bs.fpq.GetCost(vOvId) + bs.bpq.GetCost(vOvId)
			if exploredByBackwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
				bs.shortestCost = newEstSPCost

				mid := da.NewParentVertex(vOvId)
				mid.SetIsOverlayVertex()
				bs.mid = mid

			}
		}
	})
}

/*
backwardGraphSearch. backward search dari bidirectional Dijkstra di sel c1(s) atau c1(t).
*/
func (bs *CRPQuery[W]) backwardGraphSearch(uqk da.QueryKey, source, target da.Index) {
	// search backward on graph level 1

	uId := uqk.GetNode()

	uCost := bs.bpq.GetCost(uId)

	bs.engine.graph.ForInEdgesOf(uId, func(eId, tail, exitPoint da.Index) {
		vId := tail

		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		eWeight := bs.engine.getWeight(eId, false)

		newVCost := uCost + eWeight

		if vQueryLevel == 0 {

			// relax edge
			oldVCost := bs.bpq.GetCost(vId)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {

				if vLabelled {
					newPar := da.NewParentVertex(uId)
					bs.bpq.DecreaseKey(vId, newVCost, newVCost, newPar)
				} else {
					vData := da.NewVData(newVCost,
						da.NewParentVertex(uId))
					queryKey := da.NewQKey(vId, 0, false)
					bs.bpq.Insert(vId, newVCost, vData, queryKey)
				}
			}

			exploredByForwSearch := bs.fpq.IsExplored(vId)

			vIdBackwCost := bs.bpq.GetCost(vId)
			vIdForwCost := bs.fpq.GetCost(vId)
			newEstSPCost := vIdForwCost +
				vIdBackwCost
			if exploredByForwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
				bs.shortestCost = newEstSPCost
				bs.mid = da.NewParentVertex(vId)
			}
		} else {
			// v is in another cell on higher level
			// Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			v, _ := bs.engine.graph.GetOverlayVertex(vId, exitPoint, true)
			vOvId := bs.engine.offsetOverlay(v)
			oldVCost := bs.bpq.GetCost(vOvId)
			vLabelled := util.Lt(oldVCost, util.Infinity[W]())
			if util.Lt(newVCost, oldVCost) {
				newPar := da.NewParentVertex(uId)

				if !vLabelled {
					vVertexData := da.NewVData(newVCost,
						newPar)
					queryKey := da.NewQKey(vOvId, vQueryLevel, true)
					bs.bpq.Insert(vOvId, newVCost, vVertexData, queryKey)
				} else {

					bs.bpq.DecreaseKey(vOvId, newVCost, newVCost, newPar)
				}
			}

			exploredByForwSearch := bs.fpq.IsExplored(vOvId)
			newEstSPCost := bs.fpq.GetCost(vOvId) + bs.bpq.GetCost(vOvId)
			if exploredByForwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
				bs.shortestCost = newEstSPCost
				mid := da.NewParentVertex(vOvId)
				mid.SetIsOverlayVertex()
				bs.mid = mid
			}
		}
	})
}

func (bs *CRPQuery[W]) forwardOverlayGraphSearch(uqk da.QueryKey, source, target da.Index) {
	// search on overlay graph

	uOvId := uqk.GetNode() // overlay vertex id

	uQueryLevel := int(uqk.GetQueryLevel())

	uId := bs.engine.adjustoffsetOverlay(uOvId)

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	bs.engine.overlayGraph.ForOutNeighborsOf(uId, uQueryLevel, func(v da.Index, wOffset da.Index) {
		scWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newVCost := bs.fpq.GetCost(uOvId) + scWeight

		vOvId := bs.engine.offsetOverlay(v)

		// traverse edge to next cell
		vCutEdgeId := vVertex.GetCutEdge()

		eWeight := bs.engine.getWeight(vCutEdgeId, true)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOrigVId()

		// relax edge
		oldVCost := bs.fpq.GetCost(vOvId)

		if util.Lt(newVCost, oldVCost) {
			vPar := da.NewParentVertex(uOvId)
			vPar.SetIsOverlayVertex()
			bs.fpq.Set(vOvId, da.NewVData(newVCost,
				vPar), da.NewQKey(vOvId, uint8(uQueryLevel), true))

			bs.fpq.Explore(vOvId)

			newVCost = bs.fpq.GetCost(vOvId) + eWeight

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				oldWCost := bs.fpq.GetCost(wId)
				wLabelled := util.Lt(oldWCost, util.Infinity[W]())
				if util.Lt(newVCost, oldWCost) {
					newPar := da.NewParentVertex(vOvId)
					newPar.SetIsOverlayVertex()

					if wLabelled {
						bs.fpq.DecreaseKey(wId, newVCost, newVCost, newPar)
					} else {
						vData := da.NewVData(newVCost, newPar)
						queryKey := da.NewQKey(wId, 0, false)
						bs.fpq.Insert(wId, newVCost, vData, queryKey)
					}
				}

				exploredByBackwSearch := bs.bpq.IsExplored(wId)

				wForwCost := bs.fpq.GetCost(wId)
				wBackwCost := bs.bpq.GetCost(wId)

				newEstSPCost := wForwCost + wBackwCost
				if exploredByBackwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
					bs.shortestCost = newEstSPCost
					bs.mid = da.NewParentVertex(wId)
				}
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				wOvId := bs.engine.offsetOverlay(w)
				oldOverlayWIdCost := bs.fpq.GetCost(wOvId)
				wLabelled := util.Lt(oldOverlayWIdCost, util.Infinity[W]())
				if util.Lt(newVCost, oldOverlayWIdCost) {
					newPar := da.NewParentVertex(vOvId)
					newPar.SetIsOverlayVertex()

					if !wLabelled {
						vData := da.NewVData(newVCost, newPar)
						queryKey := da.NewQKey(wOvId, wQueryLevel, true)
						bs.fpq.Insert(wOvId, newVCost, vData, queryKey)
					} else {
						bs.fpq.DecreaseKey(wOvId, newVCost, newVCost, newPar)
					}
				}

				exploredByBackwSearch := bs.bpq.IsExplored(wOvId)
				newEstSPCost := bs.fpq.GetCost(wOvId) + bs.bpq.GetCost(wOvId)
				if exploredByBackwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
					// if overlay vertex w explored by backward search, check whether we can improve the shortestPath
					bs.shortestCost = newEstSPCost
					mid := da.NewParentVertex(wOvId)
					mid.SetIsOverlayVertex()
					bs.mid = mid
				}
			}
		}

		exploredByBackwSearch := bs.bpq.IsExplored(vOvId)
		newEstSPCost := bs.fpq.GetCost(vOvId) + bs.bpq.GetCost(vOvId)
		if exploredByBackwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
			bs.shortestCost = newEstSPCost
			mid := da.NewParentVertex(vOvId)
			mid.SetIsOverlayVertex()
			bs.mid = mid
		}
	})
}

func (bs *CRPQuery[W]) backwardOverlayGraphSearch(uqk da.QueryKey, source, target da.Index) {
	// search backward on overlay graph

	uOvId := uqk.GetNode()

	uQueryLevel := uqk.GetQueryLevel()
	uId := bs.engine.adjustoffsetOverlay(uOvId)

	bs.engine.overlayGraph.ForInNeighborsOf(uId, int(uQueryLevel), func(v da.Index, wOffset da.Index) {
		scWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newVCost := bs.bpq.GetCost(uOvId) + scWeight

		vOvId := bs.engine.offsetOverlay(v)
		// traverse edge to next cell
		vCutEdgeId := vVertex.GetCutEdge()

		inEdgeWeight := bs.engine.getWeight(vCutEdgeId, false)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		wId := wVertex.GetOrigVId()

		// relax edge
		oldVCost := bs.bpq.GetCost(vOvId)
		if util.Lt(newVCost, oldVCost) {
			newPar := da.NewParentVertex(uOvId)
			newPar.SetIsOverlayVertex()

			bs.bpq.Set(vOvId, da.NewVData(newVCost, newPar), da.NewQKey(vOvId,
				uint8(uQueryLevel), true))

			bs.bpq.Explore(vOvId)

			newVCost = bs.bpq.GetCost(vOvId) + inEdgeWeight

			if wQueryLevel == 0 {

				// relax edge
				oldWCost := bs.bpq.GetCost(wId)
				wLabelled := util.Lt(oldWCost, util.Infinity[W]())
				if util.Lt(newVCost, oldWCost) {
					newPar := da.NewParentVertex(vOvId)
					newPar.SetIsOverlayVertex()

					if wLabelled {
						bs.bpq.DecreaseKey(wId, newVCost, newVCost, newPar)
					} else {
						queryKey := da.NewQKey(wId, 0, false)
						vData := da.NewVData(newVCost, newPar)
						bs.bpq.Insert(wId, newVCost, vData, queryKey)
					}
				}

				wBackwCost := bs.bpq.GetCost(wId)
				exploredByForwSearch := bs.fpq.IsExplored(wId)
				wForwCost := bs.fpq.GetCost(wId)

				newEstSPCost := wForwCost + wBackwCost
				if exploredByForwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
					bs.shortestCost = newEstSPCost
					bs.mid = da.NewParentVertex(wId)
				}
			} else {
				wOvId := bs.engine.offsetOverlay(w)
				oldOverlayWIdCost := bs.bpq.GetCost(wOvId)
				wLabelled := util.Lt(oldOverlayWIdCost, util.Infinity[W]())
				if util.Lt(newVCost, oldOverlayWIdCost) {
					newPar := da.NewParentVertex(vOvId)
					newPar.SetIsOverlayVertex()
					if !wLabelled {
						queryKey := da.NewQKey(wOvId, wQueryLevel, true)
						vData := da.NewVData(newVCost, newPar)
						bs.bpq.Insert(wOvId, newVCost, vData, queryKey)
					} else {
						bs.bpq.DecreaseKey(wOvId, newVCost, newVCost, newPar)
					}
				}

				exploredByForwSearch := bs.fpq.IsExplored(wOvId)
				newEstSPCost := bs.fpq.GetCost(wOvId) + bs.bpq.GetCost(wOvId)
				if exploredByForwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
					bs.shortestCost = newEstSPCost

					mid := da.NewParentVertex(wOvId)
					mid.SetIsOverlayVertex()
					bs.mid = mid

				}
			}
		}

		exploredByForwSearch := bs.fpq.IsExplored(vOvId)
		newEstSPCost := bs.bpq.GetCost(vOvId) + bs.fpq.GetCost(vOvId)
		if exploredByForwSearch && util.Lt(newEstSPCost, bs.shortestCost) {
			bs.shortestCost = newEstSPCost
			mid := da.NewParentVertex(vOvId)
			mid.SetIsOverlayVertex()
			bs.mid = mid
		}
	})
}

func (bs *CRPQuery[W]) Preallocate() {
	bs.fpq = bs.engine.fHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
	bs.bpq = bs.engine.bHeapPool.Get().(*da.QueryHeap[da.QueryKey, W])
}

func (bs *CRPQuery[W]) Done() {
	if !bs.forAlternatives {
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

func (bs *CRPQuery[W]) SetMaxSearchRadiusSecs(maxSearchRadiusSecs float64) {
	weight := util.WeightFromSeconds[W](maxSearchRadiusSecs)
	bs.maxSearchRad = weight
}

func (bs *CRPQuery[W]) SetUpperBound(ub float64) {
	bs.ub = ub
}

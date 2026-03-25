package routing

import (
	"math"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
)

type CRPBidirectionalSearch struct {
	engine             *CRPRoutingEngine
	shortestTravelTime float64
	forwardMid         da.VertexEdgePair
	backwardMid        da.VertexEdgePair

	forwardPq     *da.QueryHeap[da.CRPQueryKey]
	backwardPq    *da.QueryHeap[da.CRPQueryKey]
	stallingEntry []float64
	stallingExit  []float64

	fScanned []bool
	bScanned []bool

	sCellNumber da.Pv
	tCellNumber da.Pv

	sForwardId  da.Index
	tBackwardId da.Index

	upperBound float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)

	numScannedVertices        int
	numScannedOverlayVertices int
	runtime                   int64
	pathUnpackingRuntime      int64
	lastpqSum                 float64
	forAlternativeRoutes      bool

	shortcutPathSet map[uint64]uint8
}

func NewCRPBidirectionalSearch(engine *CRPRoutingEngine, upperBound float64) *CRPBidirectionalSearch {
	crpQuery := &CRPBidirectionalSearch{
		engine: engine,

		forwardMid:    da.NewVertexEdgePair(0, 0, false),
		backwardMid:   da.NewVertexEdgePair(0, 0, true),
		upperBound:    upperBound,
		stallingEntry: make([]float64, 0),
		stallingExit:  make([]float64, 0),
		fScanned:      make([]bool, 0),
		bScanned:      make([]bool, 0),

		numScannedVertices:        0,
		lastpqSum:                 0,
		numScannedOverlayVertices: 0,
		shortcutPathSet:           make(map[uint64]uint8),
	}
	crpQuery.Preallocate()
	return crpQuery
}

/*
implementation of:
1. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
2. query phase: Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any partition
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
extract-min at most O(m_p+n_o) operations



ingat bahwa pada graf standard (tanpa incorporate turn costs), kita menjalankan dijkstra dengan repeatly memilih vertex u  dengan minimum shortest path estimate dari priority queue, add ke  set of scanned vertices S
dan relax semua out edges dari vertex u

karena kita incorporate turn costs, kita menggunakan turn-aware dijkstra (see ref[1]) pada forwardGraphSearch dan backwardGraphSearch :
di implementasi ini kita pakai edgeId sebagai item dari priority queue node
di turn-aware dijkstra yang dijelaskan ref [1], kita maintain triples (v,i,d) pada priority queue
v adalah vertex id, i adalah entry point (in edge yang head nya = v) pada v, dan d adalah shortest-path estimate dari s ke v melalui entry point i.
pendekatan seperti ini lebih lambat jika dibanding dengan hanya menggunakan vertex v sebagai item di priority queue node karena size dari pq tergantung dari jumlah edges yang di scan
dan biasanya di graph road network openstreetmap jumlah edges jauh lebih banyak dibanding jumlah vertices
let dist'(s,⋅) adalah shortest path estimate dari simpul s ke any (entry point, vertex). demikian juga untuk t ke any (exit point, vertex)
di implementasi ini, kita entry ke simpul s dengan menggunakan dummy edge (s,s) dengan turn cost 0 ke exit point manapun.
di awal kita set dist'(s,(dummyEntry point, s))=0 dan dist'(t, (dummyExit point, t))=0


untuk mengurangi slowdown dari pendekatan turn-aware dijkstra, kita menerapkan teknik stalling yang dijelaskan pada ref [1]
inti dari teknik stalling adalah:
misal kita punya vertex u dengan entry point i1, i2 dan exit point j1, j2. misal outDegree dari u adalah 2
contoh turn cost dari case ini adalah ketika kita keluar dari vertex u melalui entry point i1 ke j2
disini kita simpan turn cost dari i1->u->i2 di graph.turnTables[u.turnTablePtr + 0*2+1]

state dari setiap item (bisa berupa (entry/exit point, vertex) atau overlay vertex) dibagi menjadi tiga:
unreachable, labelled, dan scanned. pada awal algoritma pencarian (sebelum scan s), semua item memiliki state unreacahble
sesudah relaksasi edge (v,w) dengan entry point i item (i,w) memiliki state labelled
setiap item yang sudah di extractMin prioirty queue memiliki state scanned.

kita represent turn cost dari entry i1 ke exit i2 melalui u dengna T_u[i1,i2]
misal kita udah scan (i1,u,d_{i1}) sebelumnya
(i2, u, d_{i2}) bisa lebih baik dari (i1,u,d_{i1}) iff (lebih baik maksudnya shortest path estimate dari s ke u dengan turn costs lebih baik melalui entry point i2 dibanding i1):
terdapat k in {j1, j2}, dist'(s,(i2, u)) + T_u[i2,k] <= dist'(s, (i1, u)) + T_u[i1, k]

dengan ini, kita tahu (i2, u, d_{i2}) tidak lebih baik dari (i1,u,d_{i1}) (atau  (i2, u, d_{i2}) bisa kita prune) iff:
untuk semua k in {j1, j2}, dist'(s,(i2, u)) + T_u[i2,k] > dist'(s, (i1, u)) + T_u[i1, k]
atau
dist'(s,(i2, u)) > dist'(s, (i1, u)) + max_k { T_u[i1, k] -  T_u[i2,k]}

setiap kali kita scan entry point i dari vertex v with distance dist'(s,(i,v))
kita set b_v (bs.stallingEntry di implementasi ini, tapi langsung pakai edgeId instead of (entryPoint, v)) setiap entry point k dari v, dengan
b_v[k] = min{ b_v[k], dist'(s,(i,v)) + max_j { T_v[i, j] -  T_v[k,j]} }, inisialisasi awal dari b_v[⋅] adalah infinity utk semua vertices v
setelah scan (i,v, dist'(s,(i,v))), kita relaksasi semua out edges dari v
misal salah satu edge nya adalah (v,w) dengan entry point wi1
kita gak insert (wi1, w, dist'(s,(wi1,w))) ke heap jika dist'(s,(wi1,w)) > b_w[wi1]
max_j { T_v[i, j] -  T_v[k,j]}  kita precompute untuk setiap pasang (i,k) di metric.go
yang kita implementasikan di forwardGraphSearch (dan backwardGraph search, tapi untuk backward graph search kita pakai turn cost dari exit ke entry)



setiap iterasi dari algoritma CRP query kita ambi minimum-distance item dari pq,
item bisa berupa (i,u) dengan i adalah entry/exit point ke vertex u atau overlay vertex u.
jika item berupa (i,u), kita menjalankan turn aware bidirectional dijkstra
else kita menjalankan bidirectional overlay graph search

di bidirectional overlay graph search, kita melakukan relaksasi shortcut edges dari u yang cost (sudah include turn costs) nya sudah kita precompute
di fase kustomisasi Customizable Route planning (CRP) [1] di customizer.go.
level transition (dari base ke overlay atau sebaliknya) terjadi ketika:
u dan v memiliki query level yang berbeda.
query level dari vertex v adalah: highest level s.t. vertex v is not at the same cell as s or t
kalau transition ke level > 1, kita add overlay vertex v ke priority queue
kalau transition ke level 0. kita add (j,v) ( dengan j adalah entry point ke v dari u) ke priority queue.

path dari overlay graph search akan berpola:
exitVertex_0->entryVertex_1->exitVertex_1->entryVertex_2->exitVertex_2->....->entryVertex_n->exitVertex_n->entryVertex_{n+1}
exitVertex_0 adalah overlay vertex yang masih satu sel dengan sel dari vertex s pada level 1.
entryVertex_{n+1} adalah overlay vertex yang masih satu sel dengan sel dari vertex t pada level 1.

entryVertex adalah overlay vertex yang memiliki setidaknya satu 1 in edge yang tail vertex dari edge berada di sel (di suatu level) yang berbeda dengan sel (di suatu level) dari entryVertex
in/out edge (u,v) -> tail = u, head = v, out edge arahnya dari u ke v, in edge arah nya dari v ke u dengan bobot kedua edge sama.

exitVertex adalah overlay vertex yang memiliki setidaknya satu 1 out edge yang head vertex dari edge berada di sel (di suatu level) yang berbeda dengan sel (di suatu level) dari exitVertex

setiap shortcut edges yang dikunjungi forwardOverlayGraphSearch(), adalah (entryVertex, exitVertex)
di implementasi ini melakukan optimasi pada overlay graph search yang dilakukan pada ref[1]:
setiap kali relax shortcut edge (u,v) (misal di forwardOverlayGraphSearch()), kita langsung scan v dan relax cut edge (v,w) dari overlay vertex v.
cut edge adalah edge yang tail dan head nya berada di sel yang berbeda (di suatu level).
setiap vertex u yang memiliki cut edges > 1 akan dibuatkan overlay vertices untuk masing masing cut edges.

karena kita appply bidirectional dijkstra (ke base graph dan overlay graph) di implementasi ini, kita melakukan hal yang mirip seperti di ref[1] dan ref[6]:
untuk base graph:
(misal untuk forwardGraphSearch) setiap kali kita scan item u (u bisa berupa pasangan (entry,vertex) atau overlay vertex) dan relax edge (u,v) (v adalah vertex) dengan entry point i, kita cek semua possible turns pada vertex v
kita cek semua exit point dari v dan cek apakah salah satu (exit point, v) sudah di scan di backward search
kalau sudah discan  -> kita bisa update \mu (shortest st-path estimate)
\mu diupdate kalau sum dari shortest path estimate dari s ke v melalui entry point i + sp estimate dari t ke v (melalui exit point yang discan backward search) kurang dari \mu

untuk overlay graph:
(misal untuk forwardOverlayGraphSearch) setiap kali kita scan item u (u bisa berupa pasangan (entry,vertex) atau overlay vertex) dan relax edge (u,v) (v adalah overlay vertex)
kita cek apakah overlay vertex v udah di scan oleh backward search
kalau udha di scan -> kita bisa update \mu (shortest st-path estimate)
\mu diupdate kalau sum dari shortest path estimate dari s ke v + sp estimate dari t ke v kurang dari \mu.

search terminates ketika sum dari minimum keys of both priority queues exceeds \mu. (proof of correctness dari kriteria pemberhentian ini dapat dilihat pada ref[6])

di implementasi multilevel-dijkstra ini, kita menggunakan bidirectional dijkstra with turn costs
*/

func (bs *CRPBidirectionalSearch) ShortestPathSearch(asId, atId da.Index) (float64, float64, []da.Coordinate,
	[]da.OutEdge, bool) {

	defer bs.Done()
	now := time.Now()

	s := bs.engine.graph.GetOutEdge(asId).GetHead()
	t := bs.engine.graph.GetInEdge(atId).GetTail()

	if s == t {
		return 0, 0, []da.Coordinate{}, []da.OutEdge{}, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	sForwardId := bs.engine.graph.GetEntryOffset(s) + da.Index(bs.engine.graph.GetOutEdge(asId).GetEntryPoint())
	tBackwardId := bs.engine.graph.GetExitOffset(t) + da.Index(bs.engine.graph.GetInEdge(atId).GetExitPoint())

	sForwardId = bs.engine.offsetForward(s, sForwardId, bs.engine.graph.GetCellNumber(s), bs.sCellNumber)
	tBackwardId = bs.engine.offsetBackward(t, tBackwardId, bs.engine.graph.GetCellNumber(t), bs.sCellNumber)

	bs.sForwardId = sForwardId
	bs.tBackwardId = tBackwardId

	bs.shortestTravelTime = 2 * pkg.INF_WEIGHT

	sVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, sForwardId, false))
	tVertexInfo := da.NewVertexInfo(0, da.NewVertexEdgePair(da.INVALID_VERTEX_ID, tBackwardId, true))
	sQueryKey := da.NewCRPQueryKey(s, sForwardId, false)
	tQueryKey := da.NewCRPQueryKey(t, tBackwardId, false)
	bs.forwardPq.Insert(sForwardId, 0, sVertexInfo, sQueryKey)
	bs.backwardPq.Insert(tBackwardId, 0, tVertexInfo, tQueryKey)

	close := func(id da.Index, queryHeap *da.QueryHeap[da.CRPQueryKey]) {
		// scan item (can be edgeId or overlay vertex id)
		queryHeap.Scan(id)
	}

	for bs.forwardPq.Size() > 0 && bs.backwardPq.Size() > 0 {
		minForward := bs.forwardPq.GetMinrank()
		minBackward := bs.backwardPq.GetMinrank()
		if da.Ge(minForward+minBackward, (bs.shortestTravelTime)*(bs.upperBound)) {
			bs.lastpqSum = minForward + minBackward
			break
		}

		queryKey := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()

		if !uItem.IsOverlay() {
			close(uItem.GetEntryExitPoint(), bs.forwardPq)
			bs.forwardGraphSearch(uItem, s, t)
		} else {
			close(bs.engine.offsetOverlay(uItem.GetNode()), bs.forwardPq)
			bs.forwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		queryKey = bs.backwardPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			close(uItem.GetEntryExitPoint(), bs.backwardPq)
			bs.backwardGraphSearch(uItem, s, t)
		} else {
			close(bs.engine.offsetOverlay(uItem.GetNode()), bs.backwardPq)
			bs.backwardOverlayGraphSearch(uItem, s, t)
			bs.numScannedOverlayVertices++
		}

		bs.numScannedVertices += 2
	}

	if da.Eq(bs.shortestTravelTime, 2*pkg.INF_WEIGHT) {
		return pkg.INF_WEIGHT, 2 * pkg.INF_WEIGHT, []da.Coordinate{}, []da.OutEdge{}, false
	}

	packedPath := bs.engine.RetrievePackedPath(bs.forwardMid, bs.backwardMid,
		bs.forwardPq, bs.backwardPq, bs.sForwardId, bs.tBackwardId, bs.sCellNumber)

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	unpacker := NewPathUnpacker(bs.engine, bs.engine.metrics, bs.engine.puCache, true, false)
	edgeIdPath, shortcutPathSet := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)
	bs.shortcutPathSet = shortcutPathSet
	bs.pathUnpackingRuntime = unpacker.GetStats()

	finalEdgePath, finalPath, totalDistance := bs.engine.GetEdgePath(edgeIdPath)

	return bs.shortestTravelTime, totalDistance, finalPath, finalEdgePath, true
}

/*
graphSearch. turn-aware bidirectional dijkstra search on graph level 1.
*/
func (bs *CRPBidirectionalSearch) forwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {

	uId := uItem.GetNode()
	uEntryId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	uEntryPoint := bs.engine.getEntryPoint(uId, uEntryId, bs.engine.graph.GetEntryOffset(uId))

	// stalling
	uInDeg := bs.engine.graph.GetInDegree(uId)
	otherUEntryId := bs.engine.offsetForward(uId, bs.engine.graph.GetEntryOffset(uId), bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	uEntryIdTravelTime := bs.forwardPq.GetPriority(uEntryId)
	for j := da.Index(0); j < uInDeg; j++ {

		stallingOffset := uInDeg*uEntryPoint + j
		bui := math.Max(0, uEntryIdTravelTime+
			bs.engine.metrics.GetEntryStallingTableCost(uId, stallingOffset))

		if val := bs.stallingEntry[otherUEntryId]; da.Eq(val, pkg.INF_WEIGHT) {
			bs.stallingEntry[otherUEntryId] = bui
		} else {
			bs.stallingEntry[otherUEntryId] = math.Min(bs.stallingEntry[otherUEntryId], bui)
		}
		otherUEntryId++
	}

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOf(uId, uEntryPoint, func(outArc *da.OutEdge, exitPoint da.Index, turnType pkg.TurnType) {
		vId := outArc.GetHead()

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.metrics.GetWeight(outArc)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)
		if uId == source {
			turnCost = 0
		}
		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newTravelTime := uEntryIdTravelTime + edgeWeight + turnCost

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}
		vEntryId := bs.engine.graph.GetEntryOffset(vId) + da.Index(outArc.GetEntryPoint())
		if vQueryLevel == 0 {

			vEntryId = bs.engine.offsetForward(vId, vEntryId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			// relax edge
			oldVEntryIdTravelTime := bs.forwardPq.GetPriority(vEntryId)
			vAlreadyLabelled := da.Lt(oldVEntryIdTravelTime, pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldVEntryIdTravelTime)) {
				if bvi := bs.stallingEntry[vEntryId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
					// stalled
					return
				}

				if vAlreadyLabelled {
					// newTravelTime is better, update the forwardInfo
					// is key already in the priority queue, decrease its key

					newPar := da.NewVertexEdgePair(uId, uEntryId, false)
					bs.forwardPq.DecreaseKey(vEntryId, newTravelTime, newTravelTime, newPar)
				} else if !vAlreadyLabelled {

					vertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(uId, uEntryId, false))
					queryKey := da.NewCRPQueryKey(vId, vEntryId, false)
					// is key not in the priority queue, insert it
					bs.forwardPq.Insert(vEntryId, newTravelTime, vertexInfo, queryKey)
				}
			}

			// check wether we already Labelled an exit point of vId

			exitOffset := bs.engine.graph.GetExitOffset(vId)

			exitOffset = bs.engine.offsetBackward(vId, exitOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vExitId := exitOffset

			newVEntryIdTravelTime := bs.forwardPq.GetPriority(vEntryId)
			// traverse outEdges of v
			bs.engine.graph.ForOutEdgesOf(vId, da.Index(outArc.GetEntryPoint()), func(e2 *da.OutEdge,
				exitPoint da.Index, turnType2 pkg.TurnType) {

				//  check if forward and backward search already scanned  exit point of v. if so, check whether we can improve the shortest path
				scannedByBackwardSearch := bs.backwardPq.IsScanned(vExitId)
				vExitIdTravelTime := bs.backwardPq.GetPriority(vExitId)
				if scannedByBackwardSearch && da.Lt(newVEntryIdTravelTime+bs.engine.metrics.GetTurnCost(turnType2)+
					vExitIdTravelTime, bs.shortestTravelTime) {

					bs.shortestTravelTime = newVEntryIdTravelTime + bs.engine.metrics.GetTurnCost(turnType2) +
						vExitIdTravelTime

					bs.forwardMid = da.NewVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = da.NewVertexEdgePair(vId, vExitId, true)
				}
				vExitId++
			})

		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := bs.engine.graph.GetOverlayVertex(vId, outArc.GetEntryPoint(), false)
			overlayVId := bs.engine.offsetOverlay(v)
			oldOverlayVIdTravelTime := bs.forwardPq.GetPriority(overlayVId)
			vAlreadyLabelled := da.Lt(oldOverlayVIdTravelTime, pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldOverlayVIdTravelTime)) {

				if !vAlreadyLabelled {

					vertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(uId, uEntryId, false))
					vertexInfo.SetFirstOverlayEntryExitId(vEntryId)

					queryKey := da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)
					bs.forwardPq.Insert(overlayVId, newTravelTime, vertexInfo, queryKey)
				} else {

					bs.forwardPq.SetFirstOverlayEntryExitId(overlayVId, vEntryId)
					newPar := da.NewVertexEdgePair(uId, uEntryId, false)
					bs.forwardPq.DecreaseKey(overlayVId, newTravelTime, newTravelTime, newPar)
				}
			}

			scannedByBackwardSearch := bs.backwardPq.IsScanned(overlayVId)
			// if overlay vertex v scanned by backward search, check whether we can improve the shortestPath
			newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)
			if scannedByBackwardSearch && da.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
				bs.shortestTravelTime = newEstimateShortestPathCost

				bs.forwardMid = da.NewVertexEdgePair(vId, overlayVId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, overlayVId, true)

				vOverlay := bs.engine.overlayGraph.GetVertex(v)
				vInEdge := bs.engine.graph.GetInEdge(vOverlay.GetOriginalEdge())

				vExitId := bs.engine.graph.GetExitOffset(vInEdge.GetTail()) + da.Index(vInEdge.GetExitPoint())
				vEntryId := bs.engine.graph.GetEntryOffset(vId) + bs.engine.graph.GetEntryOrder(vId, vInEdge.GetEdgeId())

				vExitId = bs.engine.offsetBackward(uId, vExitId, vOverlay.GetCellNumber(), bs.sCellNumber)
				vEntryId = bs.engine.offsetForward(vId, vEntryId, vOverlay.GetCellNumber(), bs.sCellNumber)

			}
		}
	})
}

func (bs *CRPBidirectionalSearch) backwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search backward on graph level 1
	// basically same as forward search, but using inEdges and exitPoint instead of outEdges and entryPoint

	uId := uItem.GetNode()
	uExitId := uItem.GetEntryExitPoint() // index of outEdge that have endpoint from vertex uId

	uExitPoint := bs.engine.getExitPoint(uId, uExitId, bs.engine.graph.GetExitOffset(uId))

	// stalling
	uOutDeg := bs.engine.graph.GetOutDegree(uId)
	otherUExitId := bs.engine.offsetBackward(uId, bs.engine.graph.GetExitOffset(uId),
		bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	uExitIdTravelTime := bs.backwardPq.GetPriority(uExitId)
	for j := da.Index(0); j < uOutDeg; j++ {

		stallingOffset := uOutDeg*uExitPoint + j
		bui := math.Max(0, uExitIdTravelTime+
			bs.engine.metrics.GetExitStallingTableCost(uId, stallingOffset))

		if val := bs.stallingExit[otherUExitId]; da.Eq(val, pkg.INF_WEIGHT) {
			bs.stallingExit[otherUExitId] = bui
		} else {
			bs.stallingExit[otherUExitId] = math.Min(bs.stallingExit[otherUExitId], bui)
		}
		otherUExitId++
	}

	bs.engine.graph.ForInEdgesOf(uId, uExitPoint, func(inArc *da.InEdge, entryPoint da.Index, turnType pkg.TurnType) {
		vId := inArc.GetTail()

		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.metrics.GetWeight(inArc)

		turnCost := bs.engine.metrics.GetTurnCost(turnType)

		if uId == target {
			turnCost = 0
		}

		newTravelTime := uExitIdTravelTime + edgeWeight + turnCost
		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		vExitId := bs.engine.graph.GetExitOffset(vId) + da.Index(inArc.GetExitPoint())

		if vQueryLevel == 0 {

			vExitId = bs.engine.offsetBackward(vId, vExitId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// relax edge
			oldVExitIdTravelTime := bs.backwardPq.GetPriority(vExitId)
			vAlreadyLabelled := da.Lt(oldVExitIdTravelTime, pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldVExitIdTravelTime)) {

				if bvi := bs.stallingExit[vExitId]; da.Lt(bvi, pkg.INF_WEIGHT) && da.Gt(newTravelTime, bvi) {
					// stalled
					return
				}

				if vAlreadyLabelled {
					newPar := da.NewVertexEdgePair(uId, uExitId, true)
					bs.backwardPq.DecreaseKey(vExitId, newTravelTime, newTravelTime, newPar)
				} else {
					vertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(uId, uExitId, false))
					queryKey := da.NewCRPQueryKey(vId, vExitId, false)
					bs.backwardPq.Insert(vExitId, newTravelTime, vertexInfo, queryKey)
				}
			}

			entryOffset := bs.engine.graph.GetEntryOffset(vId)

			entryOffset = bs.engine.offsetForward(vId, entryOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vEntryId := entryOffset

			newVExitIdTravelTime := bs.backwardPq.GetPriority(vExitId)
			bs.engine.graph.ForInEdgesOf(vId, da.Index(inArc.GetExitPoint()), func(inArc2 *da.InEdge,
				entryPoint2 da.Index, turnType2 pkg.TurnType) {
				scannedByForwardSearch := bs.forwardPq.IsScanned(vEntryId)
				vEntryIdTravelTime := bs.forwardPq.GetPriority(vEntryId)
				if scannedByForwardSearch && da.Lt(vEntryIdTravelTime+bs.engine.metrics.GetTurnCost(turnType2)+
					newVExitIdTravelTime, bs.shortestTravelTime) {

					bs.shortestTravelTime = vEntryIdTravelTime + bs.engine.metrics.GetTurnCost(turnType2) +
						newVExitIdTravelTime

					bs.forwardMid = da.NewVertexEdgePair(vId, vEntryId, false)
					bs.backwardMid = da.NewVertexEdgePair(vId, vExitId, true)

				}
				vEntryId++
			})

		} else {
			// v is in another cell on higher level
			v, _ := bs.engine.graph.GetOverlayVertex(vId, inArc.GetExitPoint(), true)
			overlayVId := bs.engine.offsetOverlay(v)
			oldOverlayVIdTravelTime := bs.backwardPq.GetPriority(overlayVId)
			vAlreadyLabelled := da.Lt(oldOverlayVIdTravelTime, pkg.INF_WEIGHT)
			if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldOverlayVIdTravelTime)) {

				if !vAlreadyLabelled {

					vVertexInfo := da.NewVertexInfo(newTravelTime,
						da.NewVertexEdgePair(uId, uExitId, true))
					vVertexInfo.SetFirstOverlayEntryExitId(vExitId)
					queryKey := da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)
					bs.backwardPq.Insert(overlayVId, newTravelTime, vVertexInfo, queryKey)
				} else {
					bs.backwardPq.SetFirstOverlayEntryExitId(overlayVId, vExitId)
					newPar := da.NewVertexEdgePair(uId, uExitId, true)
					bs.backwardPq.DecreaseKey(overlayVId, newTravelTime, newTravelTime, newPar)
				}
			}

			scannedByForwardSearch := bs.forwardPq.IsScanned(overlayVId)
			newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)
			if scannedByForwardSearch && da.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
				bs.shortestTravelTime = newEstimateShortestPathCost

				bs.forwardMid = da.NewVertexEdgePair(vId, overlayVId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, overlayVId, true)

				vOverlay := bs.engine.overlayGraph.GetVertex(v)
				vOutEdge := bs.engine.graph.GetOutEdge(vOverlay.GetOriginalEdge())

				vEntryId := bs.engine.graph.GetEntryOffset(vOutEdge.GetHead()) + da.Index(vOutEdge.GetEntryPoint())
				vExitId := bs.engine.graph.GetExitOffset(vId) + bs.engine.graph.GetExitOrder(vId, vOutEdge.GetEdgeId())

				vExitId = bs.engine.offsetBackward(vId, vExitId, vOverlay.GetCellNumber(), bs.sCellNumber)
				vEntryId = bs.engine.offsetForward(uId, vEntryId, vOverlay.GetCellNumber(), bs.sCellNumber)

			}
		}
	})
}

/*
search on overlay graph
*/
func (bs *CRPBidirectionalSearch) forwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search on overlay graph

	u := uItem.GetNode()              // overlay vertex id
	uId := bs.engine.offsetOverlay(u) // offset overlay vertex id
	uVertex := bs.engine.overlayGraph.GetVertex(u)
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertices v that has shortcut edge u->v in level l within the same cell as u.
	bs.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutOutEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.forwardPq.GetPriority(uId) + shortcutOutEdgeWeight
		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}
		overlayVId := bs.engine.offsetOverlay(v)

		// traverse edge to next cell
		vOriEdgeId := vVertex.GetOriginalEdge()
		outEdge := bs.engine.graph.GetOutEdge(vOriEdgeId)
		edgeWeight := bs.engine.metrics.GetWeight(outEdge)

		// w is in the next cell from v cell

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		originalWId := wVertex.GetOriginalVertex()

		// relax edge
		oldOverlayVIdTravelTime := bs.forwardPq.GetPriority(overlayVId)
		vAlreadyLabelled := da.Lt(oldOverlayVIdTravelTime, pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldOverlayVIdTravelTime)) {
			bs.forwardPq.Set(overlayVId, da.NewVertexInfo(newTravelTime,
				da.NewVertexEdgePair(uVertex.GetOriginalVertex(), uId, false)), da.NewCRPQueryKey(da.INVALID_VERTEX_ID,
				da.INVALID_EDGE_ID, true))

			// karena kita langsung scan v & traverse to its neighbor (exit vertex dari suatu cell), kita harus tandain kalau v udah di scan
			bs.forwardPq.Scan(overlayVId)

			newTravelTime = bs.forwardPq.GetPriority(overlayVId) + edgeWeight
			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				wEntryId := bs.engine.graph.GetEntryOffset(originalWId) + da.Index(outEdge.GetEntryPoint())

				wEntryId = bs.engine.offsetForward(originalWId, wEntryId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert entryPoint of w to forwardPq
				oldWEntryIdTravelTime := bs.forwardPq.GetPriority(wEntryId)
				wAlreadyLabelled := da.Lt(oldWEntryIdTravelTime, pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, oldWEntryIdTravelTime)) {
					if wAlreadyLabelled {

						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
						bs.forwardPq.DecreaseKey(wEntryId, newTravelTime, newTravelTime, newPar)
					} else {

						vertexInfo := da.NewVertexInfo(newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))
						queryKey := da.NewCRPQueryKey(originalWId, wEntryId, false)
						bs.forwardPq.Insert(wEntryId, newTravelTime, vertexInfo, queryKey)
					}
				}

				exitOffset := bs.engine.graph.GetExitOffset(originalWId)

				exitOffset = bs.engine.offsetBackward(originalWId, exitOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				newWEntryIdTravelTime := bs.forwardPq.GetPriority(wEntryId)
				wExitId := exitOffset
				bs.engine.graph.ForOutEdgesOf(originalWId, da.Index(outEdge.GetEntryPoint()), func(e *da.OutEdge, exitPoint da.Index, turn pkg.TurnType) {
					//  check if forward and backward search already scanned exit point of w. if so, check whether we can improve the shortest path
					scannedByBackwardSearch := bs.backwardPq.IsScanned(wExitId)
					wExitIdTravelTime := bs.backwardPq.GetPriority(wExitId)
					if scannedByBackwardSearch && da.Lt(newWEntryIdTravelTime+bs.engine.metrics.GetTurnCost(turn)+
						wExitIdTravelTime, bs.shortestTravelTime) {

						bs.shortestTravelTime = newWEntryIdTravelTime + bs.engine.metrics.GetTurnCost(turn) +
							wExitIdTravelTime

						bs.forwardMid = da.NewVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = da.NewVertexEdgePair(originalWId, wExitId, true)

					}
					wExitId++
				})
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardOverlayPq, because we need to traverse & relax shortcut edges in overlay graph
				overlayWId := bs.engine.offsetOverlay(w)
				oldOverlayWIdTravelTime := bs.forwardPq.GetPriority(overlayWId)
				wAlreadyLabelled := da.Lt(oldOverlayWIdTravelTime, pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, oldOverlayWIdTravelTime)) {

					if !wAlreadyLabelled {

						vertexInfo := da.NewVertexInfo(newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false))
						queryKey := da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)
						bs.forwardPq.Insert(overlayWId, newTravelTime, vertexInfo, queryKey)
					} else {

						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
						bs.forwardPq.DecreaseKey(overlayWId, newTravelTime, newTravelTime, newPar)
					}
				}

				scannedByBackwardSearch := bs.backwardPq.IsScanned(overlayWId)
				newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayWId) + bs.backwardPq.GetPriority(overlayWId)
				if scannedByBackwardSearch && da.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
					// if overlay vertex w Labelled by backward search, check whether we can improve the shortestPath
					bs.shortestTravelTime = newEstimateShortestPathCost

					bs.forwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, false)
					bs.backwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, true)

					wInEdge := bs.engine.graph.GetInEdge(wVertex.GetOriginalEdge())
					wExitId := bs.engine.graph.GetExitOffset(wInEdge.GetTail()) + da.Index(wInEdge.GetExitPoint())
					wEntryId := bs.engine.graph.GetEntryOffset(originalWId) + bs.engine.graph.GetEntryOrder(originalWId, wInEdge.GetEdgeId())

					wExitId = bs.engine.offsetBackward(vVertex.GetOriginalVertex(), wExitId, wVertex.GetCellNumber(), bs.sCellNumber)
					wEntryId = bs.engine.offsetForward(originalWId, wEntryId, wVertex.GetCellNumber(), bs.sCellNumber)

				}
			}
		}

		scannedByBackwardSearch := bs.backwardPq.IsScanned(overlayVId)
		newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayVId) + bs.backwardPq.GetPriority(overlayVId)
		if scannedByBackwardSearch && da.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {

			bs.shortestTravelTime = newEstimateShortestPathCost

			bs.forwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
			bs.backwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

			originalVId := vVertex.GetOriginalVertex()
			vOutEdge := bs.engine.graph.GetOutEdge(vVertex.GetOriginalEdge())
			vEntryId := bs.engine.graph.GetEntryOffset(vOutEdge.GetHead()) + da.Index(vOutEdge.GetEntryPoint())
			vExitId := bs.engine.graph.GetExitOffset(originalVId) + bs.engine.graph.GetExitOrder(originalVId, vOutEdge.GetEdgeId())

			vExitId = bs.engine.offsetBackward(originalVId, vExitId, vVertex.GetCellNumber(), bs.sCellNumber)
			vEntryId = bs.engine.offsetForward(originalWId, vEntryId, vVertex.GetCellNumber(), bs.sCellNumber)

		}
	})
}

func (bs *CRPBidirectionalSearch) backwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search backward on overlay graph
	// basically same as forward search on overlayGraph, but using inEdges and exitPoint instead of outEdges and entryPoint

	u := uItem.GetNode()

	uId := bs.engine.offsetOverlay(u) // offset overlay id
	uVertex := bs.engine.overlayGraph.GetVertex(u)

	uQueryLevel := uItem.GetEntryExitPoint()

	bs.engine.overlayGraph.ForInNeighborsOf(u, int(uQueryLevel), func(v da.Index,
		wOffset da.Index) {

		shortcutInEdgeWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newTravelTime := bs.backwardPq.GetPriority(uId) + shortcutInEdgeWeight

		if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
			return
		}

		overlayVId := bs.engine.offsetOverlay(v)
		// traverse edge to next cell
		vOriEdgeId := vVertex.GetOriginalEdge()
		inEdge := bs.engine.graph.GetInEdge(vOriEdgeId)

		inEdgeWeight := bs.engine.metrics.GetWeight(inEdge)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		originalWId := wVertex.GetOriginalVertex()

		// relax edge
		oldOverlayVIdTravelTime := bs.backwardPq.GetPriority(overlayVId)
		vAlreadyLabelled := da.Lt(oldOverlayVIdTravelTime, pkg.INF_WEIGHT)
		if !vAlreadyLabelled || (vAlreadyLabelled && da.Lt(newTravelTime, oldOverlayVIdTravelTime)) {

			bs.backwardPq.Set(overlayVId, da.NewVertexInfo(newTravelTime,
				da.NewVertexEdgePair(uVertex.GetOriginalVertex(), uId, true)), da.NewCRPQueryKey(da.INVALID_VERTEX_ID,
				da.INVALID_EDGE_ID, true))

			bs.backwardPq.Scan(overlayVId)

			newTravelTime = bs.backwardPq.GetPriority(overlayVId) + inEdgeWeight

			if da.Ge(newTravelTime, pkg.INF_WEIGHT) {
				return
			}

			if wQueryLevel == 0 {

				wExitId := bs.engine.graph.GetExitOffset(originalWId) + da.Index(inEdge.GetExitPoint())

				wExitId = bs.engine.offsetBackward(originalWId, wExitId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax edge
				oldWExitIdTravelTime := bs.backwardPq.GetPriority(wExitId)
				wAlreadyLabelled := da.Lt(oldWExitIdTravelTime, pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, oldWExitIdTravelTime)) {
					if wAlreadyLabelled {
						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)
						bs.backwardPq.DecreaseKey(wExitId, newTravelTime, newTravelTime, newPar)
					} else {
						queryKey := da.NewCRPQueryKey(originalWId, wExitId, false)

						vertexInfo := da.NewVertexInfo(newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))
						bs.backwardPq.Insert(wExitId, newTravelTime, vertexInfo, queryKey)
					}
				}

				// check whether we already scanned an entry point in forward search
				entryOffset := bs.engine.graph.GetEntryOffset(originalWId)

				entryOffset = bs.engine.offsetForward(originalWId, entryOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				wEntryId := entryOffset

				newWExitIdTravelTime := bs.backwardPq.GetPriority(wExitId)
				bs.engine.graph.ForInEdgesOf(originalWId, da.Index(inEdge.GetExitPoint()), func(e *da.InEdge,
					entryPoint da.Index, turn pkg.TurnType) {
					scannedByForwardSearch := bs.forwardPq.IsScanned(wEntryId)
					wEntryIdTravelTime := bs.forwardPq.GetPriority(wEntryId)
					if scannedByForwardSearch && da.Lt(wEntryIdTravelTime+bs.engine.metrics.GetTurnCost(turn)+
						newWExitIdTravelTime, bs.shortestTravelTime) {

						bs.shortestTravelTime = wEntryIdTravelTime + bs.engine.metrics.GetTurnCost(turn) +
							newWExitIdTravelTime

						bs.forwardMid = da.NewVertexEdgePair(originalWId, wEntryId, false)
						bs.backwardMid = da.NewVertexEdgePair(originalWId, wExitId, true)

					}
					wEntryId++
				})
			} else {
				overlayWId := bs.engine.offsetOverlay(w)
				oldOverlayWIdTravelTime := bs.backwardPq.GetPriority(overlayWId)
				wAlreadyLabelled := da.Lt(oldOverlayWIdTravelTime, pkg.INF_WEIGHT)
				if !wAlreadyLabelled || (wAlreadyLabelled && da.Lt(newTravelTime, oldOverlayWIdTravelTime)) {

					if !wAlreadyLabelled {
						queryKey := da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)

						vertexInfo := da.NewVertexInfo(newTravelTime,
							da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true))
						bs.backwardPq.Insert(overlayWId, newTravelTime, vertexInfo, queryKey)
					} else {
						newPar := da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)
						bs.backwardPq.DecreaseKey(overlayWId, newTravelTime, newTravelTime, newPar)
					}
				}

				scannedByForwardSearch := bs.forwardPq.IsScanned(overlayWId)
				newEstimateShortestPathCost := bs.forwardPq.GetPriority(overlayWId) + bs.backwardPq.GetPriority(overlayWId)
				if scannedByForwardSearch && da.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {
					bs.shortestTravelTime = newEstimateShortestPathCost

					bs.forwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, false)
					bs.backwardMid = da.NewVertexEdgePair(wVertex.GetOriginalVertex(), overlayWId, true)

					wOutEdge := bs.engine.graph.GetOutEdge(wVertex.GetOriginalEdge())
					wEntryId := bs.engine.graph.GetEntryOffset(wOutEdge.GetHead()) + da.Index(wOutEdge.GetEntryPoint())
					wExitId := bs.engine.graph.GetExitOffset(originalWId) + bs.engine.graph.GetExitOrder(originalWId, wOutEdge.GetEdgeId())

					wExitId = bs.engine.offsetBackward(originalWId, wExitId, wVertex.GetCellNumber(), bs.sCellNumber)
					wEntryId = bs.engine.offsetForward(vVertex.GetOriginalVertex(), wEntryId, wVertex.GetCellNumber(), bs.sCellNumber)

				}
			}
		}

		scannedByForwardSearch := bs.forwardPq.IsScanned(overlayVId)
		newEstimateShortestPathCost := bs.backwardPq.GetPriority(overlayVId) + bs.forwardPq.GetPriority(overlayVId)
		if scannedByForwardSearch && da.Lt(newEstimateShortestPathCost, bs.shortestTravelTime) {

			bs.shortestTravelTime = newEstimateShortestPathCost

			bs.forwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, false)
			bs.backwardMid = da.NewVertexEdgePair(vVertex.GetOriginalVertex(), overlayVId, true)

			originalVId := vVertex.GetOriginalVertex()
			vInEdge := bs.engine.graph.GetInEdge(vVertex.GetOriginalEdge())
			vExitId := bs.engine.graph.GetExitOffset(vInEdge.GetTail()) + da.Index(vInEdge.GetExitPoint())
			vEntryId := bs.engine.graph.GetEntryOffset(originalVId) + bs.engine.graph.GetEntryOrder(originalVId, vInEdge.GetEdgeId())

			vExitId = bs.engine.offsetBackward(originalWId, vExitId, vVertex.GetCellNumber(), bs.sCellNumber)
			vEntryId = bs.engine.offsetForward(originalVId, vEntryId, vVertex.GetCellNumber(), bs.sCellNumber)

		}
	})
}

func (bs *CRPBidirectionalSearch) Preallocate() {
	maxEdgesInCell := bs.engine.graph.GetMaxEdgesInCell()

	bs.stallingEntry = make([]float64, maxEdgesInCell*2)
	bs.stallingExit = make([]float64, maxEdgesInCell*2)
	initInfWeight(bs.stallingEntry)
	initInfWeight(bs.stallingExit)

	bs.forwardPq = bs.engine.fHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	bs.backwardPq = bs.engine.bHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey])
	bs.forwardPq.Clear()
	bs.backwardPq.Clear()
}

func (bs *CRPBidirectionalSearch) Done() {

	if bs.forAlternativeRoutes {
		return
	}

	bs.engine.fHeapPool.Put(bs.forwardPq)
	bs.engine.bHeapPool.Put(bs.backwardPq)

}

func (bs *CRPBidirectionalSearch) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(bs.numScannedVertices)
	return efficiency, bs.numScannedVertices, bs.runtime, bs.pathUnpackingRuntime
}

func (bs *CRPBidirectionalSearch) GetLastPQSum() float64 {
	return bs.lastpqSum
}

func (bs *CRPBidirectionalSearch) SetForAlternativeRoutes(yes bool) {
	bs.forAlternativeRoutes = yes
}

func (bs *CRPBidirectionalSearch) getShortcutPathSet() map[uint64]uint8 {
	return bs.shortcutPathSet
}

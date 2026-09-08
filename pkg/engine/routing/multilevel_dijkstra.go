package routing

import (
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// todo: yg query pakai turn cost ganti pakai pendekatan OSRM aja https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation  atau
// atau edge-based model (sama aja) disini: https://drops.dagstuhl.de/storage/01oasics/oasics-vol085-atmos2020/OASIcs.ATMOS.2020.9/OASIcs.ATMOS.2020.9.pdf
// ntar query with turn cost bisa pakai kode multilevel_dijkstra_without_turn_cost.go kalau pakai edge-based model
// this compact model buat support query with turn cost (& turn restrictions) ribet bgt gokil
// biar gak sama kaya paten ini juga, CRP yg dijelasin disini pakai compact representation: https://patents.google.com/patent/US20130231862A1/en

// penjelasan algoritma kueri (dengan turn cost) dari Customizable Route Planning ada di section 3.5:  https://drive.google.com/file/d/16X4_D82-dBz5CEKTLBWPb8DtG52eVMl5/view
// pdf password: <my-github-username>-<my-birth-year>-<my gdrive email without @gmail.com>
// without turn costs & turn restrictions: multilevel_dijkstra_without_turn_cost.go

type CRPQueryTurnCost[W util.RoutingNumber] struct {
	engine       *CRPRoutingEngine[W]
	shortestCost W
	forwardMid   da.VertexEdgePair
	backwardMid  da.VertexEdgePair

	forwardPq  *da.QueryHeap[da.CRPQueryKey, W]
	backwardPq *da.QueryHeap[da.CRPQueryKey, W]
	stallingEn []W
	stallingEx []W

	fExplored []bool
	bExplored []bool

	sCellNumber da.Pv
	tCellNumber da.Pv

	inSId  da.Index
	outTId da.Index

	upperBound      float64 // upperbound for finding alternative routes (see page 15 Customizable Route Planning in Road Networks by Delling et al.)
	maxSearchRadius W

	numExploredVertices        int
	numExploredOverlayVertices int
	runtime                    int64
	pathUnpackingRuntime       int64
	lastpqSum                  float64

	forAlternativeRoutes bool
	reroute              bool

	shortcutPathSet map[uint64]uint8
	viaVertices     []da.ViaVertex
}

func NewCRPQueryTurnCost[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
	upperBound float64,
) *CRPQueryTurnCost[W] {
	if engine == nil {
		return newCRPQueryTurnCostAlloc[W](nil)
	}
	crpQuery := engine.bidirSearchPool.Get().(*CRPQueryTurnCost[W])
	crpQuery.Reset(upperBound)

	crpQuery.Preallocate()
	return crpQuery
}

func newCRPQueryTurnCostAlloc[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *CRPQueryTurnCost[W] {
	maxEdgesInCell := engine.graph.GetMaxEdgesInCell()

	return &CRPQueryTurnCost[W]{
		engine: engine,

		forwardMid:  da.NewVertexEdgePair(0, 0, false),
		backwardMid: da.NewVertexEdgePair(0, 0, true),

		stallingEn:                 make([]W, maxEdgesInCell*2),
		stallingEx:                 make([]W, maxEdgesInCell*2),
		fExplored:                  make([]bool, 0),
		bExplored:                  make([]bool, 0),
		shortcutPathSet:            make(map[uint64]uint8),
		viaVertices:                make([]da.ViaVertex, 0, VIA_VERTICES_INITIAL_CAPACITY),
		numExploredVertices:        0,
		numExploredOverlayVertices: 0,
		runtime:                    0,
		pathUnpackingRuntime:       0,
		lastpqSum:                  0,
		maxSearchRadius:            2 * util.Infinity[W](),
	}
}

// Reset clears per-query state on a pooled CRPQueryTurnCost so it can
// be reused.
func (bs *CRPQueryTurnCost[W]) Reset(upperBound float64) {
	bs.upperBound = upperBound
	bs.shortestCost = 2 * util.Infinity[W]()

	bs.forwardMid = da.NewVertexEdgePair(0, 0, false)
	bs.backwardMid = da.NewVertexEdgePair(0, 0, true)

	bs.sCellNumber = 0
	bs.tCellNumber = 0
	bs.inSId = 0
	bs.outTId = 0

	bs.numExploredVertices = 0
	bs.numExploredOverlayVertices = 0
	bs.runtime = 0
	bs.pathUnpackingRuntime = 0
	bs.lastpqSum = 0

	bs.forAlternativeRoutes = false
	bs.reroute = false

	// keep capacity of the via-vertex slice and the shortcut-set map so
	// the next query can reuse the already-paid-for allocation.
	bs.viaVertices = bs.viaVertices[:0]
	if bs.shortcutPathSet != nil {
		for k := range bs.shortcutPathSet {
			delete(bs.shortcutPathSet, k)
		}
	} else {
		bs.shortcutPathSet = make(map[uint64]uint8)
	}

	bs.maxSearchRadius = 2 * util.Infinity[W]()
}

/*
implementation of:
1. query phase:  Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.
2. query phase: Delling, D. et al. (2011) “Customizable Route Planning,” in P.M. Pardalos and S. Rebennack (eds.) Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 376–387. Available at: https://doi.org/10.1007/978-3-642-20662-7_32.
3. ALT query phase: Goldberg, A.V. and Harrelson, lm. (2005) ‘Computing the shortest path: A* search meets graph theory’, in Proceedings of the Sixteenth Annual ACM-SIAM Symposium on Discrete Algorithms. USA: Society for Industrial and Applied Mathematics (SODA ’05), pp. 156–165.
4. bidirectional A*: Ikeda, T. et al. (1994) ‘A fast algorithm for finding better routes by AI search techniques’, in Proceedings of VNIS’94 - 1994 Vehicle Navigation and Information Systems Conference, pp. 291–296. Available at: https://doi.org/10.1109/VNIS.1994.396824.
5. consistent heuristic for A* & optimality of A*: Hart, P.E., Nilsson, N.J. and Raphael, B. (1968) “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics, 4(2), pp. 100–107. Available at: https://doi.org/10.1109/TSSC.1968.300136.
6. Haeupler, B. et al. (2025) “Bidirectional Dijkstra's Algorithm is Instance-Optimal,” in 2025 Symposium on Simplicity in Algorithms (SOSA). Society for Industrial and Applied Mathematics (Proceedings), pp. 202–215. Available at: https://doi.org/10.1137/1.9781611978315.16.
7. Cormen, T.H. et al. (2009) Introduction to Algorithms. 3th ed. Cambridge, MA, USA: MIT Press
8. https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf


time complexity (ref: https://www.vldb.org/pvldb/vol18/p3326-farhan.pdf):
let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any cell
let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than level 1 cell that containing s or t
decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
extract-min at most O(m_p+n_o) operations



ingat bahwa pada graf standard (tanpa incorporate turn costs), kita menjalankan dijkstra dengan repeatly memilih vertex u  dengan minimum shortest path estimate dari priority queue, add ke  set of explored vertices S
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
unreachable, labelled, dan explored. pada awal algoritma pencarian (sebelum scan s), semua item memiliki state unreacahble
sesudah relaksasi edge (v,w) dengan entry point i item (i,w) memiliki state labelled
setiap item yang sudah di extractMin prioirty queue memiliki state explored.

kita represent turn cost dari entry i1 ke exit i2 melalui u dengna T_u[i1,i2]
misal kita udah scan (i1,u,d_{i1}) sebelumnya
(i2, u, d_{i2}) bisa lebih baik dari (i1,u,d_{i1}) iff (lebih baik maksudnya shortest path estimate dari s ke u dengan turn costs lebih baik melalui entry point i2 dibanding i1):
terdapat k in {j1, j2}, dist'(s,(i2, u)) + T_u[i2,k] <= dist'(s, (i1, u)) + T_u[i1, k]

dengan ini, kita tahu (i2, u, d_{i2}) tidak lebih baik dari (i1,u,d_{i1}) (atau  (i2, u, d_{i2}) bisa kita prune) iff:
untuk semua k in {j1, j2}, dist'(s,(i2, u)) + T_u[i2,k] > dist'(s, (i1, u)) + T_u[i1, k]
atau
dist'(s,(i2, u)) > dist'(s, (i1, u)) + max_k { T_u[i1, k] -  T_u[i2,k]}

setiap kali kita scan entry point i dari vertex v with distance dist'(s,(i,v))
kita set b_v (bs.stallingEn di implementasi ini, tapi langsung pakai edgeId instead of (enPoint, v)) setiap entry point k dari v, dengan
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
\mu diupdate kalau sum dari shortest path estimate dari s ke v melalui entry point i + sp estimate dari v ke t (melalui exit point yang discan backward search) kurang dari \mu

untuk overlay graph:
(misal untuk forwardOverlayGraphSearch) setiap kali kita scan item u (u bisa berupa pasangan (entry,vertex) atau overlay vertex) dan relax edge (u,v) (v adalah overlay vertex)
kita cek apakah overlay vertex v udah di scan oleh backward search
kalau udha di scan -> kita bisa update \mu (shortest st-path estimate)
\mu diupdate kalau sum dari shortest path estimate dari s ke v + sp estimate dari v ke t kurang dari \mu.

search terminates ketika sum dari minimum keys of both priority queues exceeds \mu. (proof of correctness dari kriteria pemberhentian ini dapat dilihat pada ref[6])

di implementasi multilevel-dijkstra ini, kita menggunakan bidirectional dijkstra with turn costs


ini adalah implementasi dari fase query dari Customizable Route Planning (CRP) [1] / multilevel-dijkstra incorporate turn costs.
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

proof of correctness bidirectional dijkstra bisa dilihat di proof of correctness Algorithm 2 di ref [3]. di implementasi ini, kita apply bidirectional turn-aware implementation of dijkstra pada graf consisting of overlay graph H, cell C_s, cell C_t yang mana s-t shortest path yang dihasilkan
ekuivalen dengan s-t shortest path (with turn costs) pada graf G.



kode ini terinspirasi oleh kode implementasi Customizable Route Planning (CRP) yang dibuat oleh Michael Wegner: https://github.com/michaelwegner/CRP


return:
travelTime
distance
coordinates []da.Coordinate{}
edgeIds
found shortest path or not


karena di multilevel-dijkstra/multilevel-alt kita query dari origin phantom node ke destination phantom node (lihat phantom_node.go):

q -originPhantomEdge->s->....................... -t-destPhantomEdge->z

originPhantomEdge:						destPhantomEdge:
q------originPhantomNode----->s    t------destPhantomNode----->z

shortest path di multilevel-dijkstra/multilevel alt adalah shortest path dari s ke t + turn cost dari originPhantomEdge ke outEdge dari s +
turn cost dari inEdge dari t ke destPhantomEdge ..
kita udah include turn cost dari originPhantomEdge ke other outEdge dari s di awal forward search dan cost dari inEdge dari t ke destPhantomEdge di awal backward search
sebenarnya setelah multilevel-dijkstra selesai (di routing.go), kita tambahin sp cost nya dengan travelTime(originPhantomNode, s) + travelTime(t, destPhantomNode)

inti dari multilevel-dijkstra [1]:
- ketika kita scan vertex v (extracted from pq) di forward search, by proof of correctness dari alg dijkstra [7] ( without turn cost) -> est cost dari s to v udah equal to shortest path cost
- ketika kita scan vertex v (extracted from pq) di backward search (pakai reversed edges), by proof of correctness dari alg dijkstra [7] ( without turn cost) -> est cost dari v to t udah equal to shortest path cost, kenapa??
karena di backward search kita pakai reversed edges: semua edges (v,u) dengan (u,v)\in E, l(v,u)=l(u,v)  (see Single-destination shortest-paths problem in ref[7])
- setelah forward search keluar dari cell level 1 dari s (sebelum masuk ke cell level 1 nya t), kita relax only shortcut edges di cell level >= 1 selain sel nya s atau t
- setelah backward search keluar dari cell level 1 dari t (sebelum masuk ke cell level 1 nya s), kita relax only shortcut edges di cell level >= 1 selain sel nya s atau t
- saat forward search masuk ke cell level 1 dari t, kita relax original edges (yang contained in cell level 1 dari t) dari graph nya.
- saat backward search masuk ke cell level 1 dari s, kita relax reversed edges (yang contained in cell level 1 dari s) dari graph nya.
- karena bidirectional search, kita pakai kriteria pemberhentian dari algoritma 2 [6] dan update esimated sp cost dari s ke t (\mu) setiap kali relax edge(u,v) yang head nya (v) udah di scan (extracted from pq) oleh another search seperti pada ref [6]: https://kam.mff.cuni.cz/~spring/media/papers/5/bidirectional_dijkstra.pdf
- karena pakai turn cost pakai trik dijskstra with turn cost on compact graph yang dijelaskan pada section 4.2 ref[1]: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
- di graph_builder.go kita tambahkan dummy Outedge/InEdge (dengan turn cost ke other edge dari headnya/tailnya sama dengan 0) pada vertex yang outDegree/inDegree nya 0 agar saat kita tetap bisa compute shortest path dari s (yang inDegre nya 0) ke t (yang outDegree nya 0) (tested on tests/shortestpath dan tests/shortestpath_crp_alt)
misal:
 s -> v - > t
inDegre(s)=0
outDegree(t)=0
kita tambahin dummy inEdge ke s dan dummy OUtEdge ke t
q -> s -> v -> t -> z

turnCost(q->s->v) = 0
turnCost(v->t->z) = 0


untuk referensi lain implementasi routing with turn cost di road network dapat dilihat di:
1. https://dl.acm.org/doi/10.5555/2008623.2008634
2. https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf
3. multilevel-dijkstranya OSRM: https://github.com/Project-OSRM/osrm-backend/blob/master/include/engine/routing_algorithms/routing_base_mld.hpp    also see osrm graph representation: https://github.com/Project-OSRM/osrm-backend/wiki/Graph-representation
4. customizable contraction hierarchies with turn costs: https://drops.dagstuhl.de/storage/01oasics/oasics-vol085-atmos2020/OASIcs.ATMOS.2020.9/OASIcs.ATMOS.2020.9.pdf   atau  https://i11www.iti.kit.edu/_media/teaching/theses/ba-zuendorf-19.pdf

*/

func (bs *CRPQueryTurnCost[W]) ShortestPathSearch(sp, tp da.PhantomNode) (W, []da.Index, bool) {

	defer bs.Done()
	now := time.Now()

	asId := sp.GetOutEdgeId()
	atId := tp.GetInEdgeId()

	// asId: Id of outEdge u->s  (head dari outEdge = s, tail dari outEdge = u )
	// atId: Id of inEdge  t->v  (head dari inEdge = v, tail dari inEdge = t )
	asOutEdge := bs.engine.graph.GetOutEdge(asId)
	s := asOutEdge.GetHead()
	atInEdge := bs.engine.graph.GetInEdge(atId)
	t := atInEdge.GetTail()

	if s == t {
		return 0, EmptyIndexSet, true
	}

	bs.sCellNumber = bs.engine.graph.GetCellNumber(s)
	bs.tCellNumber = bs.engine.graph.GetCellNumber(t)

	inSId := bs.engine.graph.GetInEdgeId(s, da.Index(asOutEdge.GetEntryPoint()))
	outTId := bs.engine.graph.GetExitOffset(t) + da.Index(atInEdge.GetExitPoint())

	inSId = bs.engine.offsetForward(s, inSId, bs.engine.graph.GetCellNumber(s), bs.sCellNumber)
	outTId = bs.engine.offsetBackward(t, outTId, bs.engine.graph.GetCellNumber(t), bs.sCellNumber)

	bs.inSId = inSId
	bs.outTId = outTId

	bs.shortestCost = 2 * util.Infinity[W]()

	sVertexData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, inSId, false))
	tVertexData := da.NewVertexData(W(0), da.NewVertexEdgePair(da.INVALID_VERTEX_ID, outTId, true))
	sQueryKey := da.NewCRPQueryKey(s, inSId, false)
	tQueryKey := da.NewCRPQueryKey(t, outTId, false)
	bs.forwardPq.Insert(inSId, 0, sVertexData, sQueryKey)
	bs.backwardPq.Insert(outTId, 0, tVertexData, tQueryKey)

	addViaVertex := func(uItem da.CRPQueryKey, forward bool, overlay bool) {
		if !overlay && forward {
			uId := uItem.GetNode()
			uInId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

			uEnPoint := bs.engine.getEntryPoint(uId, uInId, bs.engine.graph.GetEntryOffset(uId))
			outOffset := bs.engine.graph.GetExitOffset(uId)

			outOffset = bs.engine.offsetBackward(uId, outOffset, bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

			uOutId := outOffset
			bs.engine.graph.ForOutEdgesOf(uId, uEnPoint, func(_, _ da.Index, _, _, turnTableId da.Index, _ pkg.TurnType,
				_ pkg.OsmHighwayType) {

				exploredByBackwardSearch := bs.backwardPq.IsExplored(uOutId)
				if exploredByBackwardSearch {
					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(uId, uInId, uOutId, uId, false))
				}

				uOutId++
			})
		} else if !overlay && !forward {
			uId := uItem.GetNode()
			uOutId := uItem.GetEntryExitPoint() // index of outEdge that have endpoint from vertex uId

			uExitPoint := bs.engine.getExitPoint(uId, uOutId, bs.engine.graph.GetExitOffset(uId))

			inOffset := bs.engine.graph.GetEntryOffset(uId)

			inOffset = bs.engine.offsetForward(uId, inOffset, bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

			uInId := inOffset

			bs.engine.graph.ForInEdgesOf(uId, uExitPoint, func(_, _ da.Index, _, _, turnTableId da.Index,
				_ pkg.TurnType, _ pkg.OsmHighwayType) {
				exploredByForwardSearch := bs.forwardPq.IsExplored(uInId)
				if exploredByForwardSearch {
					bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(uId, uInId, uOutId, uId, false))
				}
				uInId++
			})
		} else {
			u := uItem.GetNode()                // overlay vertex id
			ovUId := bs.engine.offsetOverlay(u) // offset overlay vertex id
			exploredByBackwardSearch := bs.backwardPq.IsExplored(ovUId)
			exploredByForwardSearch := bs.forwardPq.IsExplored(ovUId)
			uOvVertex := bs.engine.overlayGraph.GetVertex(u)
			oriUId := uOvVertex.GetOrigVId()
			if exploredByBackwardSearch && exploredByForwardSearch {
				bs.viaVertices = append(bs.viaVertices, da.NewViaVertex(ovUId, 0, 0, oriUId, true))
			}
		}
	}

	for bs.forwardPq.Size() > 0 && bs.backwardPq.Size() > 0 {
		minForward := bs.forwardPq.GetMinrank()
		minBackward := bs.backwardPq.GetMinrank()
		if util.Ge(minForward+minBackward, W(float64(bs.shortestCost)*(bs.upperBound))) || util.Ge(minForward+minBackward, W(bs.maxSearchRadius)) {

			bs.lastpqSum = bs.engine.GetCostFunction().WeightToSeconds(minForward + minBackward)
			break
		}

		queryKey := bs.forwardPq.ExtractMin()
		uItem := queryKey.GetItem()

		if !uItem.IsOverlay() {
			bs.forwardPq.Explore(uItem.GetEntryExitPoint())
			addViaVertex(uItem, true, false)
			bs.forwardGraphSearch(uItem, s, t)
		} else {
			bs.forwardPq.Explore(bs.engine.offsetOverlay(uItem.GetNode()))
			addViaVertex(uItem, true, true)
			bs.forwardOverlayGraphSearch(uItem, s, t)
			bs.numExploredOverlayVertices++
		}

		queryKey = bs.backwardPq.ExtractMin()
		uItem = queryKey.GetItem()
		if !uItem.IsOverlay() {
			bs.backwardPq.Explore(uItem.GetEntryExitPoint())
			addViaVertex(uItem, false, false)
			bs.backwardGraphSearch(uItem, s, t)
		} else {
			bs.backwardPq.Explore(bs.engine.offsetOverlay(uItem.GetNode()))
			addViaVertex(uItem, false, true)
			bs.backwardOverlayGraphSearch(uItem, s, t)
			bs.numExploredOverlayVertices++
		}

		bs.numExploredVertices += 2
	}

	if bs.shortestCost == 2*util.Infinity[W]() {
		return util.Infinity[W](), EmptyIndexSet, false
	}

	packedPath := bs.engine.RetrievePackedPath(bs.forwardMid, bs.backwardMid,
		bs.forwardPq, bs.backwardPq, bs.inSId, bs.outTId, bs.sCellNumber, s, t)

	dur := time.Since(now).Milliseconds()
	bs.runtime = dur

	unpacker := NewPathUnpackerALT(bs.engine)
	defer unpacker.DonePooled()
	edgeIdPath, shortcutPathSet := unpacker.unpackPath(packedPath, bs.sCellNumber, bs.tCellNumber)
	bs.shortcutPathSet = shortcutPathSet
	bs.pathUnpackingRuntime = unpacker.GetStats()

	return bs.shortestCost, edgeIdPath, true
}

/*
graphSearch. turn-aware bidirectional dijkstra search on graph level 1.
*/
func (bs *CRPQueryTurnCost[W]) forwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {

	uId := uItem.GetNode()
	uInId := uItem.GetEntryExitPoint() // index of inedge that point to vertex uId

	uEnPoint := bs.engine.getEntryPoint(uId, uInId, bs.engine.graph.GetEntryOffset(uId))

	// stalling
	uInDeg := bs.engine.graph.GetInDegree(uId)
	otherUInId := bs.engine.offsetForward(uId, bs.engine.graph.GetEntryOffset(uId), bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	uInIdCost := bs.forwardPq.GetCost(uInId)
	for j := da.Index(0); j < uInDeg; j++ {

		stallingOffset := uInDeg*uEnPoint + j
		bui := max(uInIdCost+
			bs.engine.metrics.GetEntryStallingTableCost(uId, stallingOffset), 0)

		if val := bs.stallingEn[otherUInId]; util.Eq(val, util.Infinity[W]()) {
			bs.stallingEn[otherUInId] = bui
		} else {
			bs.stallingEn[otherUInId] = min(bs.stallingEn[otherUInId], bui)
		}
		otherUInId++
	}

	// traverse outEdges of u
	bs.engine.graph.ForOutEdgesOf(uId, uEnPoint, func(eId, head da.Index, exPoint, enPoint, turnTableId da.Index, turnType pkg.TurnType,
		hwType pkg.OsmHighwayType) {
		vId := head

		// get query level of v l_st(v)
		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.getWeight(eId, true)

		if bs.reroute && turnType == pkg.U_TURN {
			return
		}

		turnCost := bs.engine.metrics.GetTurnCost(turnTableId)

		// get cost to reach v through u + turn cost from inEdge to outEdge of u
		newVCost := uInIdCost + edgeWeight + turnCost

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}
		vInId := bs.engine.graph.GetInEdgeId(vId, da.Index(enPoint))
		if vQueryLevel == 0 {

			vInId = bs.engine.offsetForward(vId, vInId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// if query level of v is 0, then v is in the same cell as s or t in the lowest level
			// then, we just do edge relaxation as usual in turn-aware dijkstra

			// relax edge
			oldVInIdCost := bs.forwardPq.GetCost(vInId)
			vLabelled := util.Lt(oldVInIdCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldVInIdCost)) {
				if bvi := bs.stallingEn[vInId]; util.Lt(bvi, util.Infinity[W]()) && util.Gt(newVCost, bvi) {
					// stalled, newTraveltime= dist'(s,(vInId, v))
					// dist'(s,(vInId, v)) > dist'(s, (, v)) + max_k { T_u[, k] -  T_u[vInId,k]}
					return
				}

				if vLabelled {
					// newVCost is better, update the forwardData
					// is key already in the priority queue, decrease its key

					newPar := da.NewVertexEdgePair(uId, uInId, false)
					bs.forwardPq.DecreaseKey(vInId, newVCost, newVCost, newPar)
				} else if !vLabelled {

					vData := da.NewVertexData(newVCost,
						da.NewVertexEdgePair(uId, uInId, false))
					queryKey := da.NewCRPQueryKey(vId, vInId, false)
					// is key not in the priority queue, insert it
					bs.forwardPq.Insert(vInId, newVCost, vData, queryKey)
				}
			}

			outOffset := bs.engine.graph.GetExitOffset(vId)

			outOffset = bs.engine.offsetBackward(vId, outOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vOutId := outOffset

			newVInIdCost := bs.forwardPq.GetCost(vInId)
			// traverse outEdges of v
			bs.engine.graph.ForOutEdgesOf(vId, enPoint, func(_, _ da.Index, _, _, turnTableId2 da.Index, turnType2 pkg.TurnType,
				_ pkg.OsmHighwayType) {

				// check if forward and backward search already explored entry point and  exit point of v. if so, check whether we can improve the shortest path
				exploredByBackwardSearch := bs.backwardPq.IsExplored(vOutId)
				vOutIdCost := bs.backwardPq.GetCost(vOutId)

				midTurnCost := bs.engine.metrics.GetTurnCost(turnTableId2)

				newPathCost := newVInIdCost + midTurnCost +
					vOutIdCost
				if exploredByBackwardSearch && util.Lt(newPathCost, bs.shortestCost) {
					bs.shortestCost = newPathCost
					bs.forwardMid = da.NewVertexEdgePair(vId, vInId, false)
					bs.backwardMid = da.NewVertexEdgePair(vId, vOutId, true)
				}
				vOutId++
			})

		} else {
			// v is in another cell on higher level
			// but the item in priority queue is (v, l_st(v)), because we need to traverse & relax shortcut edges in overlay graph (see overlayGraphSearch method)
			v, _ := bs.engine.graph.GetOverlayVertex(vId, enPoint, false)
			ovVId := bs.engine.offsetOverlay(v)
			oldOvVIdCost := bs.forwardPq.GetCost(ovVId)
			vLabelled := util.Lt(oldOvVIdCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldOvVIdCost)) {

				newPar := da.NewVertexEdgePair(uId, uInId, false)
				if !vLabelled {
					queryKey := da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)
					vData := da.NewVertexData(newVCost, newPar)
					bs.forwardPq.Insert(ovVId, newVCost, vData, queryKey)
				} else {
					bs.forwardPq.DecreaseKey(ovVId, newVCost, newVCost, newPar)
				}
			}

			exploredByBackwardSearch := bs.backwardPq.IsExplored(ovVId)
			// if v explored by backward search, check whether we can improve the shortestPath
			newEstSpCost := bs.forwardPq.GetCost(ovVId) + bs.backwardPq.GetCost(ovVId)
			if exploredByBackwardSearch && util.Lt(newEstSpCost, bs.shortestCost) {
				bs.shortestCost = newEstSpCost

				bs.forwardMid = da.NewVertexEdgePair(vId, ovVId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, ovVId, true)

			}
		}
	})
}

func (bs *CRPQueryTurnCost[W]) backwardGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search backward on graph level 1
	//same as forward search, but using inEdges and exPoint instead of outEdges and enPoint

	uId := uItem.GetNode()
	uOutId := uItem.GetEntryExitPoint() // index of outEdge that have endpoint from vertex uId

	uExitPoint := bs.engine.getExitPoint(uId, uOutId, bs.engine.graph.GetExitOffset(uId))

	// stalling
	uOutDeg := bs.engine.graph.GetOutDegree(uId)
	otherUOutId := bs.engine.offsetBackward(uId, bs.engine.graph.GetExitOffset(uId),
		bs.engine.graph.GetCellNumber(uId), bs.sCellNumber)

	uOutIdCost := bs.backwardPq.GetCost(uOutId)
	for j := da.Index(0); j < uOutDeg; j++ {

		stallingOffset := uOutDeg*uExitPoint + j
		bui := max(0, uOutIdCost+
			bs.engine.metrics.GetExitStallingTableCost(uId, stallingOffset))

		if val := bs.stallingEx[otherUOutId]; util.Eq(val, util.Infinity[W]()) {
			bs.stallingEx[otherUOutId] = bui
		} else {
			bs.stallingEx[otherUOutId] = min(bs.stallingEx[otherUOutId], bui)
		}
		otherUOutId++
	}

	bs.engine.graph.ForInEdgesOf(uId, uExitPoint, func(eId, tail da.Index, exPoint, enPoint, turnTableId da.Index,
		turnType pkg.TurnType, hwType pkg.OsmHighwayType) {
		vId := tail

		vQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			bs.engine.graph.GetCellNumber(vId))

		edgeWeight := bs.engine.getWeight(eId, false)

		turnCost := bs.engine.metrics.GetTurnCost(turnTableId)

		newVCost := uOutIdCost + edgeWeight + turnCost

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}

		vOutId := bs.engine.graph.GetOutEdgeId(vId, exPoint)

		if vQueryLevel == 0 {

			vOutId = bs.engine.offsetBackward(vId, vOutId, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			// relax edge
			oldVOutIdCost := bs.backwardPq.GetCost(vOutId)
			vLabelled := util.Lt(oldVOutIdCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldVOutIdCost)) {

				if bvi := bs.stallingEx[vOutId]; util.Lt(bvi, util.Infinity[W]()) && util.Gt(newVCost, bvi) {
					// stalled
					return
				}

				if vLabelled {
					newPar := da.NewVertexEdgePair(uId, uOutId, true)
					bs.backwardPq.DecreaseKey(vOutId, newVCost, newVCost, newPar)
				} else {
					vData := da.NewVertexData(newVCost,
						da.NewVertexEdgePair(uId, uOutId, false))
					queryKey := da.NewCRPQueryKey(vId, vOutId, false)
					bs.backwardPq.Insert(vOutId, newVCost, vData, queryKey)
				}
			}

			inOffset := bs.engine.graph.GetEntryOffset(vId)

			inOffset = bs.engine.offsetForward(vId, inOffset, bs.engine.graph.GetCellNumber(vId), bs.sCellNumber)

			vInId := inOffset

			newVOutIdCost := bs.backwardPq.GetCost(vOutId)
			bs.engine.graph.ForInEdgesOf(vId, exPoint, func(_, _ da.Index, _, _, turnTableId2 da.Index,
				turnType2 pkg.TurnType, _ pkg.OsmHighwayType) {
				exploredByForwardSearch := bs.forwardPq.IsExplored(vInId)
				vInIdCost := bs.forwardPq.GetCost(vInId)

				midTurnCost := bs.engine.metrics.GetTurnCost(turnTableId2)

				newPathCost := vInIdCost + midTurnCost +
					newVOutIdCost
				if exploredByForwardSearch && util.Lt(newPathCost, bs.shortestCost) {

					bs.shortestCost = newPathCost

					bs.forwardMid = da.NewVertexEdgePair(vId, vInId, false)
					bs.backwardMid = da.NewVertexEdgePair(vId, vOutId, true)

				}
				vInId++
			})

		} else {
			// v is in another cell on higher level
			// Note that a level transition occurs when u and v have different query levels.
			// i.e. if v not in the same cell as s and t then v query level is different from u query level.
			v, _ := bs.engine.graph.GetOverlayVertex(vId, exPoint, true)
			ovVId := bs.engine.offsetOverlay(v)
			oldOvVIdCost := bs.backwardPq.GetCost(ovVId)
			vLabelled := util.Lt(oldOvVIdCost, util.Infinity[W]())
			if !vLabelled || (vLabelled && util.Lt(newVCost, oldOvVIdCost)) {

				newPar := da.NewVertexEdgePair(uId, uOutId, true)
				if !vLabelled {
					vData := da.NewVertexData(newVCost, newPar)
					queryKey := da.NewCRPQueryKey(v, da.Index(vQueryLevel), true)
					bs.backwardPq.Insert(ovVId, newVCost, vData, queryKey)
				} else {
					bs.backwardPq.DecreaseKey(ovVId, newVCost, newVCost, newPar)
				}
			}

			exploredByForwardSearch := bs.forwardPq.IsExplored(ovVId)
			newEstSpCost := bs.forwardPq.GetCost(ovVId) + bs.backwardPq.GetCost(ovVId)
			if exploredByForwardSearch && util.Lt(newEstSpCost, bs.shortestCost) {
				bs.shortestCost = newEstSpCost

				bs.forwardMid = da.NewVertexEdgePair(vId, ovVId, false)
				bs.backwardMid = da.NewVertexEdgePair(vId, ovVId, true)

			}
		}
	})
}

func (bs *CRPQueryTurnCost[W]) forwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search on overlay graph

	u := uItem.GetNode()              // overlay vertex id
	uId := bs.engine.offsetOverlay(u) // offset overlay vertex id
	uVertex := bs.engine.overlayGraph.GetVertex(u)
	uQueryLevel := int(uItem.GetEntryExitPoint())

	// outNeighbors of u = all overlay vertex v that has shortcut edge u->v in level l within the same cell as u.
	bs.engine.overlayGraph.ForOutNeighborsOf(u, uQueryLevel, func(v da.Index, wOffset da.Index) {
		shortcutWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newVCost := bs.forwardPq.GetCost(uId) + shortcutWeight

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}
		ovVId := bs.engine.offsetOverlay(v)

		// traverse edge to next cell
		vCutEId := vVertex.GetCutEdge()

		edgeWeight := bs.engine.getWeight(vCutEId, true)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		oriWId := wVertex.GetOrigVId()

		// relax edge
		oldOvVIdCost := bs.forwardPq.GetCost(ovVId)
		vLabelled := util.Lt(oldOvVIdCost, util.Infinity[W]())
		if !vLabelled || (vLabelled && util.Lt(newVCost, oldOvVIdCost)) {
			bs.forwardPq.Set(ovVId, da.NewVertexData(newVCost,
				da.NewVertexEdgePair(uVertex.GetOrigVId(), uId, false)), da.NewCRPQueryKey(da.INVALID_VERTEX_ID,
				da.INVALID_EDGE_ID, true))

			// karena kita langsung scan v & traverse to its neighbor (exit vertex dari suatu cell), kita harus tandain kalau v udah di scan
			bs.forwardPq.Explore(ovVId)

			newVCost = bs.forwardPq.GetCost(ovVId) + edgeWeight

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			if wQueryLevel == 0 {
				// w is in the same cell as s or t

				wEnPoint := bs.engine.graph.GetEntryPointOfOutEdge(vCutEId)
				wInId := bs.engine.graph.GetInEdgeId(oriWId, wEnPoint)
				wInId = bs.engine.offsetForward(oriWId, wInId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax entry Edge of w
				// update travelTime to reach entry point of w and insert enPoint of w to forwardPq
				oldWInIdCost := bs.forwardPq.GetCost(wInId)
				wLabelled := util.Lt(oldWInIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldWInIdCost)) {
					if wLabelled {

						newPar := da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, false)
						bs.forwardPq.DecreaseKey(wInId, newVCost, newVCost, newPar)
					} else {

						vData := da.NewVertexData(newVCost,
							da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, false))
						queryKey := da.NewCRPQueryKey(oriWId, wInId, false)
						bs.forwardPq.Insert(wInId, newVCost, vData, queryKey)
					}
				}

				outOffset := bs.engine.graph.GetExitOffset(oriWId)
				outOffset = bs.engine.offsetBackward(oriWId, outOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				newWInIdCost := bs.forwardPq.GetCost(wInId)
				wOutId := outOffset
				bs.engine.graph.ForOutEdgesOf(oriWId, wEnPoint, func(_, _ da.Index, _, _, turnTableId da.Index, turn pkg.TurnType,
					_ pkg.OsmHighwayType) {
					// check if forward and backward search already explored exit point of w. if so, check whether we can improve the shortest path
					exploredByBackwardSearch := bs.backwardPq.IsExplored(wOutId)
					wOutIdCost := bs.backwardPq.GetCost(wOutId)

					midTurnCost := bs.engine.metrics.GetTurnCost(turnTableId)

					newPathCost := newWInIdCost + midTurnCost +
						wOutIdCost
					if exploredByBackwardSearch && util.Lt(newPathCost, bs.shortestCost) {

						bs.shortestCost = newPathCost

						bs.forwardMid = da.NewVertexEdgePair(oriWId, wInId, false)
						bs.backwardMid = da.NewVertexEdgePair(oriWId, wOutId, true)
					}
					wOutId++
				})
			} else {
				// w is in another cell on higher level
				// update new travelTime to reach overlay vertex w
				// insert item overlay vertex w and its query level to forwardP
				overlayWId := bs.engine.offsetOverlay(w)
				oldOvWIdCost := bs.forwardPq.GetCost(overlayWId)
				wLabelled := util.Lt(oldOvWIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldOvWIdCost)) {

					if !wLabelled {
						vData := da.NewVertexData(newVCost,
							da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, false))
						queryKey := da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)
						bs.forwardPq.Insert(overlayWId, newVCost, vData, queryKey)
					} else {

						newPar := da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, false)
						bs.forwardPq.DecreaseKey(overlayWId, newVCost, newVCost, newPar)
					}
				}

				exploredByBackwardSearch := bs.backwardPq.IsExplored(overlayWId)
				newEstSpCost := bs.forwardPq.GetCost(overlayWId) + bs.backwardPq.GetCost(overlayWId)
				if exploredByBackwardSearch && util.Lt(newEstSpCost, bs.shortestCost) {
					// if overlay vertex w explored by backward search, check whether we can improve the shortestPath
					bs.shortestCost = newEstSpCost
					bs.forwardMid = da.NewVertexEdgePair(wVertex.GetOrigVId(), overlayWId, false)
					bs.backwardMid = da.NewVertexEdgePair(wVertex.GetOrigVId(), overlayWId, true)

				}
			}
		}

		exploredByBackwardSearch := bs.backwardPq.IsExplored(ovVId)
		newEstSpCost := bs.forwardPq.GetCost(ovVId) + bs.backwardPq.GetCost(ovVId)
		if exploredByBackwardSearch && util.Lt(newEstSpCost, bs.shortestCost) {
			bs.shortestCost = newEstSpCost
			bs.forwardMid = da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, false)
			bs.backwardMid = da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, true)

		}
	})
}

func (bs *CRPQueryTurnCost[W]) backwardOverlayGraphSearch(uItem da.CRPQueryKey, source, target da.Index) {
	// search backward on overlay graph
	//same as forward search on overlayGraph, but using inEdges and exPoint instead of outEdges and enPoint

	u := uItem.GetNode()

	uId := bs.engine.offsetOverlay(u) // offset overlay id
	uVertex := bs.engine.overlayGraph.GetVertex(u)

	uQueryLevel := uItem.GetEntryExitPoint()

	bs.engine.overlayGraph.ForInNeighborsOf(u, int(uQueryLevel), func(v da.Index,
		wOffset da.Index) {

		shotcutWeight := bs.engine.metrics.GetShortcutWeight(wOffset)

		vVertex := bs.engine.overlayGraph.GetVertex(v)

		newVCost := bs.backwardPq.GetCost(uId) + shotcutWeight

		if util.Ge(newVCost, util.Infinity[W]()) {
			return
		}

		ovVId := bs.engine.offsetOverlay(v)
		// traverse edge to next cell
		vCutEId := vVertex.GetCutEdge()

		cutEWeight := bs.engine.getWeight(vCutEId, false)

		w := vVertex.GetNeighborOverlayVertex()
		wVertex := bs.engine.overlayGraph.GetVertex(w)
		wQueryLevel := bs.engine.overlayGraph.GetQueryLevel(bs.sCellNumber, bs.tCellNumber,
			wVertex.GetCellNumber())
		oriWId := wVertex.GetOrigVId()

		// relax edge
		oldOvVIdCost := bs.backwardPq.GetCost(ovVId)
		vLabelled := util.Lt(oldOvVIdCost, util.Infinity[W]())
		if !vLabelled || (vLabelled && util.Lt(newVCost, oldOvVIdCost)) {

			bs.backwardPq.Set(ovVId, da.NewVertexData(newVCost,
				da.NewVertexEdgePair(uVertex.GetOrigVId(), uId, true)), da.NewCRPQueryKey(da.INVALID_VERTEX_ID,
				da.INVALID_EDGE_ID, true))

			bs.backwardPq.Explore(ovVId)

			newVCost = bs.backwardPq.GetCost(ovVId) + cutEWeight

			if util.Ge(newVCost, util.Infinity[W]()) {
				return
			}

			if wQueryLevel == 0 {

				inEdgeExPoint := bs.engine.graph.GetExitPointOfInEdge(vCutEId)
				wOutId := bs.engine.graph.GetExitOffset(oriWId) + inEdgeExPoint

				wOutId = bs.engine.offsetBackward(oriWId, wOutId, wVertex.GetCellNumber(), bs.sCellNumber)

				// relax edge
				oldWOutIdCost := bs.backwardPq.GetCost(wOutId)
				wLabelled := util.Lt(oldWOutIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldWOutIdCost)) {
					if wLabelled {
						newPar := da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, true)
						bs.backwardPq.DecreaseKey(wOutId, newVCost, newVCost, newPar)
					} else {
						queryKey := da.NewCRPQueryKey(oriWId, wOutId, false)

						vData := da.NewVertexData(newVCost,
							da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, true))
						bs.backwardPq.Insert(wOutId, newVCost, vData, queryKey)
					}
				}

				// check whether we already explored an entry point of w in forward search
				inOffset := bs.engine.graph.GetEntryOffset(oriWId)

				inOffset = bs.engine.offsetForward(oriWId, inOffset, wVertex.GetCellNumber(), bs.sCellNumber)

				wInId := inOffset

				newWOutIdCost := bs.backwardPq.GetCost(wOutId)
				bs.engine.graph.ForInEdgesOf(oriWId, inEdgeExPoint, func(_, _ da.Index, _, _, turnTableId da.Index,
					turn pkg.TurnType, _ pkg.OsmHighwayType) {
					exploredByForwardSearch := bs.forwardPq.IsExplored(wInId)
					wInIdCost := bs.forwardPq.GetCost(wInId)
					midTurnCost := bs.engine.metrics.GetTurnCost(turnTableId)

					newPathCost := wInIdCost + midTurnCost +
						newWOutIdCost
					if exploredByForwardSearch && util.Lt(newPathCost, bs.shortestCost) {

						bs.shortestCost = newPathCost
						bs.forwardMid = da.NewVertexEdgePair(oriWId, wInId, false)
						bs.backwardMid = da.NewVertexEdgePair(oriWId, wOutId, true)

					}
					wInId++
				})
			} else {
				overlayWId := bs.engine.offsetOverlay(w)
				oldOvWIdCost := bs.backwardPq.GetCost(overlayWId)
				wLabelled := util.Lt(oldOvWIdCost, util.Infinity[W]())
				if !wLabelled || (wLabelled && util.Lt(newVCost, oldOvWIdCost)) {

					if !wLabelled {
						queryKey := da.NewCRPQueryKey(w, da.Index(wQueryLevel), true)

						vData := da.NewVertexData(newVCost,
							da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, true))
						bs.backwardPq.Insert(overlayWId, newVCost, vData, queryKey)
					} else {
						newPar := da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, true)
						bs.backwardPq.DecreaseKey(overlayWId, newVCost, newVCost, newPar)
					}
				}

				exploredByForwardSearch := bs.forwardPq.IsExplored(overlayWId)
				newEstSpCost := bs.forwardPq.GetCost(overlayWId) + bs.backwardPq.GetCost(overlayWId)
				if exploredByForwardSearch && util.Lt(newEstSpCost, bs.shortestCost) {
					bs.shortestCost = newEstSpCost

					bs.forwardMid = da.NewVertexEdgePair(wVertex.GetOrigVId(), overlayWId, false)
					bs.backwardMid = da.NewVertexEdgePair(wVertex.GetOrigVId(), overlayWId, true)
				}
			}
		}

		exploredByForwardSearch := bs.forwardPq.IsExplored(ovVId)
		newEstSpCost := bs.backwardPq.GetCost(ovVId) + bs.forwardPq.GetCost(ovVId)
		if exploredByForwardSearch && util.Lt(newEstSpCost, bs.shortestCost) {

			bs.shortestCost = newEstSpCost
			bs.forwardMid = da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, false)
			bs.backwardMid = da.NewVertexEdgePair(vVertex.GetOrigVId(), ovVId, true)
		}
	})
}

func (bs *CRPQueryTurnCost[W]) Preallocate() {
	initInfWeight(bs.stallingEn)
	initInfWeight(bs.stallingEx)

	bs.forwardPq = bs.engine.fHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey, W])
	bs.backwardPq = bs.engine.bHeapPool.Get().(*da.QueryHeap[da.CRPQueryKey, W])

}

func (bs *CRPQueryTurnCost[W]) Done() {

	if bs.forAlternativeRoutes {
		// ingat: reslicing slice gak bakal bikin slice baru/resliced slices tetep refer ke original slice (https://go.dev/blog/slices-intro)
		// karena kita pake isi dari queryHeap.heap buat cek est cost di alternative_routes.go, kita gak boleh clear queryHeap
		// kalau queryHeap masih dipake buat algoritma buat find alternative routes....
		return
	}

	bs.forwardPq.Clear()
	bs.backwardPq.Clear()
	bs.engine.fHeapPool.Put(bs.forwardPq)
	bs.engine.bHeapPool.Put(bs.backwardPq)

	bs.engine.bidirSearchPool.Put(bs)
}

func (bs *CRPQueryTurnCost[W]) GetViaVertices() []da.ViaVertex {
	return bs.viaVertices
}

func (bs *CRPQueryTurnCost[W]) GetStats(n int) (float64, int, int64, int64) {
	// efficiency:
	//    https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf

	efficiency := float64(n) / float64(bs.numExploredVertices)
	return efficiency, bs.numExploredVertices, bs.runtime, bs.pathUnpackingRuntime
}

func (bs *CRPQueryTurnCost[W]) GetLastPQSum() float64 {
	return bs.lastpqSum
}

func (bs *CRPQueryTurnCost[W]) SetForAlternativeRoutes(yes bool) {
	bs.forAlternativeRoutes = yes
}

func (bs *CRPQueryTurnCost[W]) getShortcutPathSet() map[uint64]uint8 {
	return bs.shortcutPathSet
}

func (bs *CRPQueryTurnCost[W]) SetReroute() {
	bs.reroute = true

}

func (bs *CRPQueryTurnCost[W]) SetMaxSearchRadiusSecs(maxSearchRadiusSecs float64) {
	weight := bs.engine.GetCostFunction().WeightFromSeconds(maxSearchRadiusSecs)
	bs.maxSearchRadius = weight
}

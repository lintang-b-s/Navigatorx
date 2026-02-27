package routing

import (
	"maps"
	"math"
	"sort"
	"sync"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type AlternativeRoute struct {
	path              []datastructure.Coordinate
	edges             []datastructure.OutEdge
	objectiveValue    float64
	drivingDirections []datastructure.DrivingDirection
	polylinePath      string

	travelTime  float64
	dist        float64
	distSharing float64
	viaNode     datastructure.Index
	viaVertex   datastructure.ViaVertex
}

func (ar *AlternativeRoute) GetCoords() []datastructure.Coordinate {
	return ar.path
}

func (ar *AlternativeRoute) GetPolylinePath() string {
	return ar.polylinePath
}

func (ar *AlternativeRoute) SetPolylinePath(pp string) {
	ar.polylinePath = pp
}

func (ar *AlternativeRoute) GetPath() []datastructure.OutEdge {
	return ar.edges
}
func (ar *AlternativeRoute) GetObjectiveValue() float64 {
	return ar.objectiveValue
}

func (ar *AlternativeRoute) GetDrivingDirections() []datastructure.DrivingDirection {
	return ar.drivingDirections
}
func (ar *AlternativeRoute) SetDrivingDirections(dds []datastructure.DrivingDirection) {
	ar.drivingDirections = dds
}

func (ar *AlternativeRoute) GetDrivingTravelTime() float64 {
	return ar.travelTime
}

func (ar *AlternativeRoute) GetViaVertex() datastructure.ViaVertex {
	return ar.viaVertex
}

func (ar *AlternativeRoute) GetDist() float64 {
	return ar.dist
}

func (ar *AlternativeRoute) GetDistSharing() float64 {
	return ar.distSharing
}

func (ar *AlternativeRoute) GetViaNode() datastructure.Index {
	return ar.viaNode
}

func NewAlternativeRoute(objectiveValue, dist, travelTime, distSharing float64,
	viaNode datastructure.Index, path []datastructure.Coordinate, edges []datastructure.OutEdge,
	viaVertex datastructure.ViaVertex) *AlternativeRoute {
	return &AlternativeRoute{
		objectiveValue: objectiveValue,
		viaNode:        viaNode,
		path:           path,
		dist:           dist,
		edges:          edges,
		travelTime:     travelTime,
		viaVertex:      viaVertex,
		distSharing:    distSharing,
	}
}

type AlternativeRouteSearch struct {
	engine                *CRPRoutingEngine
	candidates            []*AlternativeRoute
	upperBound            float64
	gamma, alpha, epsilon float64
	lm                    *landmark.Landmark
	optTravelTime         float64
	runtime               int64

	lock *sync.RWMutex
}

func NewAlternativeRouteSearch(engine *CRPRoutingEngine, upperBound, gamma, alpha, epsilon float64, lm *landmark.Landmark,

) *AlternativeRouteSearch {
	return &AlternativeRouteSearch{
		engine:     engine,
		candidates: make([]*AlternativeRoute, 0),
		upperBound: upperBound,
		gamma:      gamma,
		alpha:      alpha,
		epsilon:    epsilon,
		lock:       &sync.RWMutex{},
		lm:         lm,
	}
}

/*
implementation of:
1. Abraham, I. et al. (2010) “Alternative Routes in Road Networks,” in P. Festa (ed.)
Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 23–34. Available at:
https://doi.org/10.1007/978-3-642-13193-6_3.
2. page 15: Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.


todo: invstigate kenapa success rate nya kecil banget
*/

func (ars *AlternativeRouteSearch) FindAlternativeRoutes(asId, atId datastructure.Index, k int) []*AlternativeRoute {

	/*
		let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any partition
		let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
		time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
		decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
		extract-min at most O(m_p+n_o) operations
	*/
	now := time.Now()
	crpQuery := NewCRPALTBidirectionalSearch(ars.engine, ars.upperBound, ars.lm)

	optTravelTime, _, _, optEdgePath, found := crpQuery.ShortestPathSearch(asId, atId)
	if !found {
		return []*AlternativeRoute{}
	}
	ars.optTravelTime = optTravelTime

	viaVertices := make([]datastructure.ViaVertex, len(crpQuery.GetViaVertices()))
	copy(viaVertices, crpQuery.GetViaVertices())

	ars.candidates = make([]*AlternativeRoute, 0, len(viaVertices))
	viaVertices = removeDuplicates(viaVertices)

	fInfo := crpQuery.GetForwardInfo()
	bInfo := crpQuery.GetBackwardInfo()

	sCellNumber := crpQuery.GetSCellNumber()

	for i := len(viaVertices) - 1; i >= 0; i-- {
		v := viaVertices[i]
		if !v.IsOverlay() {
			if !fInfo[v.GetEntryId()].IsScanned() && !bInfo[v.GetExitId()].IsScanned() {
				continue
			}
			if fInfo[v.GetEntryId()].GetTravelTime()+bInfo[v.GetExitId()].GetTravelTime() >= (1+ars.epsilon)*optTravelTime {
				viaVertices = append(viaVertices[:i], viaVertices[i+1:]...)
			}
		} else {
			if !fInfo[v.GetVId()].IsScanned() && !bInfo[v.GetVId()].IsScanned() {
				continue
			}
			if fInfo[v.GetVId()].GetTravelTime()+bInfo[v.GetVId()].GetTravelTime() >= (1+ars.epsilon)*optTravelTime {
				viaVertices = append(viaVertices[:i], viaVertices[i+1:]...)
			}
		}
	}

	computeAlternatives := func(v datastructure.ViaVertex) any {
		var (
			svTravelTime, vtTravelTime float64
			svDist, vtDist             float64
			svCoords, vtCoords         []datastructure.Coordinate
			svEdgePath, vtEdgePath     []datastructure.OutEdge
		)

		/*
			let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any partition
			let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
			lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (turn-based graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
			cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
			let q = number of shorcut edges in packedPath
			worst case  of unpackPath: O(\sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))

			let p = number of edges & shortcut edges in s-via-t path

			worst case of RetrieveForwardPackedPath+RetrieveForwardPackedPath: O(p)
			worst case  of calculatePlateau: O(p)
			worst case of  calculateDistanceShare: O(p)

			worst case of computeAlternatives: O( p + \sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))
		*/
		if !v.IsOverlay() {
			// forward
			svPackedPath := ars.engine.RetrieveForwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetEntryId(), false),
				fInfo, crpQuery.sForwardId, crpQuery.sCellNumber)
			unpacker := NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)
			svCoords, svEdgePath, svDist = unpacker.unpackPath(svPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)

			// backward
			vtPackedPath := ars.engine.RetrieveBackwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetExitId(), true),
				bInfo, crpQuery.tBackwardId, crpQuery.sCellNumber)
			unpacker = NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)
			vtCoords, vtEdgePath, vtDist = unpacker.unpackPath(vtPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)
		} else {
			// forward
			svPackedPath := ars.engine.RetrieveForwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				fInfo, crpQuery.sForwardId, crpQuery.sCellNumber)
			unpacker := NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)

			// backward
			vtPackedPath := ars.engine.RetrieveBackwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				bInfo, crpQuery.tBackwardId, crpQuery.sCellNumber)
			unpacker = NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)

			svPackedPath, vtPackedPath = ars.makePackedViaPathOverlayEven(svPackedPath, vtPackedPath)

			svCoords, svEdgePath, svDist = unpacker.unpackPath(svPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)
			vtCoords, vtEdgePath, vtDist = unpacker.unpackPath(vtPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)
		}

		for _, e := range svEdgePath {
			svTravelTime += ars.engine.metrics.GetWeight(&e)
		}
		for _, e := range vtEdgePath {
			vtTravelTime += ars.engine.metrics.GetWeight(&e)
		}

		pvEdgePath := append(svEdgePath, vtEdgePath...)
		sigmav := ars.calculateDistanceShare(optEdgePath, pvEdgePath)
		if sigmav >= ars.gamma*optTravelTime {
			return nil
		}

		lv := svTravelTime + vtTravelTime
		if lv >= (1+ars.epsilon)*optTravelTime {
			return nil
		}

		var plv float64
		if !v.IsOverlay() {
			plv = ars.calculatePlateau(v.GetVId(), v.GetOriginalVId(), v.GetEntryId(), v.GetExitId(), crpQuery.sForwardId, crpQuery.tBackwardId,
				fInfo, bInfo, sCellNumber, lv, false)

		} else {
			plv = ars.calculatePlateau(v.GetVId(), v.GetOriginalVId(), v.GetEntryId(), v.GetExitId(), crpQuery.sForwardId, crpQuery.tBackwardId,
				fInfo, bInfo, sCellNumber, lv, true)

		}

		T := ars.alpha * optTravelTime

		if plv <= T {
			// didnt pass t-test
			return nil
		}

		fv := 2*lv + sigmav - plv

		pvCoords := append(svCoords, vtCoords...)

		ars.lock.Lock()
		ars.candidates =
			append(ars.candidates, NewAlternativeRoute(fv, svDist+vtDist, lv, sigmav, v.GetOriginalVId(), pvCoords, pvEdgePath, v))
		ars.lock.Unlock()
		return nil
	}

	workers := concurrent.NewWorkerPool[datastructure.ViaVertex, any](1, len(viaVertices))

	for _, v := range viaVertices {
		workers.AddJob(v)
	}

	// let \alpha = number of via vertices
	// worst case computeAlternatives for all via vertices: O(\alpha * ( p + \sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p)))
	workers.Close()
	workers.Start(computeAlternatives)
	workers.WaitDirect()

	sort.Slice(ars.candidates, func(j, pivotIdx int) bool {
		return ars.candidates[j].objectiveValue < ars.candidates[pivotIdx].objectiveValue
	})

	ars.candidates = removeSimiliarAlternatives(ars.candidates)

	res := make([]*AlternativeRoute, 0, k)

	for i := 0; i < util.MinInt(k, len(ars.candidates)); i++ {
		res = append(res, ars.candidates[i])
	}

	// worst case of FindAlternativeRoutes: worst case crp query + worst case computeAlternatives for all via vertices
	// O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o) + \alpha * ( p + \sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p)))

	dur := time.Since(now).Milliseconds()
	ars.runtime = dur

	return res
}

func (ars *AlternativeRouteSearch) calculateDistanceShare(optPath, pvPath []datastructure.OutEdge) float64 {
	// O(N+M), N=len(optPath), M=len(pvPath)
	distanceShare := 0.0

	optPathSet := make(map[datastructure.Index]struct{}, len(optPath)*2)
	for _, e := range optPath {
		optPathSet[e.GetEdgeId()] = struct{}{}
	}

	for _, e := range pvPath {

		if _, ok := optPathSet[e.GetEdgeId()]; ok {
			distanceShare += e.GetWeight()
		}
	}
	return distanceShare
}

func (ars *AlternativeRouteSearch) calculatePlateau(vId, oriVId, viaEntryId, viaExitId, sForwardId, tBackwardId datastructure.Index,
	ps, pb []*VertexInfo[datastructure.CRPQueryKey], sCellNumber da.Pv, lv float64, overlay bool) float64 {

	var (
		u datastructure.Index
	)
	uVId := oriVId
	if overlay {
		u = vId
	} else {
		u = viaEntryId
	}

	/*
		1. Abraham, I. et al. (2010) “Alternative Routes in Road Networks,” in P. Festa (ed.)
		Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 23–34. Available at:
		https://doi.org/10.1007/978-3-642-13193-6_3.

		section 4.2 The Choice Routing Algorithm.
		It starts by building shortest path trees from s and to t. It
		then looks at plateaus, i.e., maximal paths that appear in both trees simultaneously.
		In general, a plateau u–w gives a natural s–t path: follow the out-tree from s to u, then
		the plateau, then the in-tree from w to t. The algorithm selects paths corresponding to
		long plateaus.
	*/

	// shortest path tree from s to v: all scanned (already extracted using extractMin from pq) vertices in forward search
	// shortest path tree from v to t: all scanned (already extracted using extractMin from pq) vertices in backward search
	/*
		Intuisi dari plateau (ref: https://dl.acm.org/doi/abs/10.1145/2444016.2444019):
		buat ngecek apakah alternative route P_v T-Localy Optimall (T-LO): every subpath P' of P_v with l(P') <= T adalah shortest path
		P_v is admissble alternative route iff P_v is T-locally optimal for T=\alpha* l(Opt)
		lemma 4.4 dari ref[1]:
		If P_v corresponds to a plateau u-w, P_v is dist(u,w)-LO
		proof:
		karena semua item antara u-v di scan forwad search dan v-w discan backward search, pakai lemma subpath of shortest path is shortest path (clrs): subpath u-v dan v-w adalah shortest path -> subpath u-w is shortest path
		pakai  lemma subpath of shortest path is shortest path (clrs) lagi: every subpath P' dari path u ke w, l(P') <= dist(u,w) is shortest path

	*/

	// u = vEntryId/vId  dari via
	// vId = overlayId dari via kalau via nya overlay vertex
	// s-> .... -> u -vInEdge-> via (bisa aja sebuah overlay vertex) <-vExitEdge- w <- ..... <-t

	// task kita disini adalah find total length dari plateau u-w dari definisi platau diatas
	// so kita harus backtrack dari vEntryId/vOverlayId dari via vertex ke vertex awal dari plateau (atau vertex u dari definisi diatas)
	// let n=number of edges in s-via-t path
	// worst case: O(n)
	for u != sForwardId {

		if ars.engine.isOverlay(u) {
			// plateau iff parent_backward_search(parent_forward_search(u)) == u

			oki := da.Lt(pb[ps[u].parent.edge].GetTravelTime(), pkg.INF_WEIGHT)
			if b := pb[ps[u].parent.edge]; !(oki && b.parent.edge == u) {
				// qParentOverlay -qShortcut-> qOverlay -vShortcut-> vOverlay
				// u == vOverlay, ps[u].parent.edge == qOverlay
				// kalau parent dari qOverlay di backward search equal to vOverlay && qOverlay udah di scan di backward search kita bisa lanjut backtrack
				// else: vOverlay (atau u) adalah overlayVertex pertama dari plateau path

				break
			}
		} else if !ars.engine.isOverlay(u) && !ars.engine.isOverlay(ps[u].parent.edge) {
			// u dan ps[u].parent.edge bukan overlay vertex
			// qParent -qInEdge-> q -vInEdge-> v
			// u == vInEdge,  ps[u].parent.edge == qInEdge

			// kalau u == entryId  dari edge, sedangkan di pb isinya exitId dari edge, shg u harus dijadiin exitId dari edgenya
			vEntryId := ars.engine.adjustForward(uVId, u)
			_, vOutEdge := ars.engine.graph.GetHeadOfInedgeWithOutEdge(vEntryId)
			vExitId := vOutEdge.GetEdgeId()

			q := ars.engine.graph.GetTailOfOutedge(vExitId)
			qInEdge := ps[u].parent.getEdge()
			qEntryId := ars.engine.adjustForward(q, qInEdge)
			_, qOutEdge := ars.engine.graph.GetHeadOfInedgeWithOutEdge(qEntryId)
			qExitId := qOutEdge.GetEdgeId()
			qParent := ars.engine.graph.GetTailOfOutedge(qExitId)

			offQExitId := ars.engine.offsetBackward(qParent, qExitId, ars.engine.graph.GetCellNumber(qParent), sCellNumber)
			oki := da.Lt(pb[offQExitId].GetTravelTime(), pkg.INF_WEIGHT)

			q = ars.engine.graph.GetTailOfOutedge(vExitId)

			offVExitId := ars.engine.offsetBackward(q, vExitId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			if b := pb[offQExitId]; !(oki && b.parent.edge == offVExitId) {
				// kalau parent dari qInEdge di backward search equal to vInEdge && qInEdge udah di scan di backward search kita bisa lanjut backtrack
				// else: vInEdge (atau u) adalah entryEdge pertama dari plateau path
				break
			}
		} else if !ars.engine.isOverlay(u) && ars.engine.isOverlay(ps[u].parent.getEdge()) {
			// q -vShortcut-> vOverlay -boundaryEdge/wInEdge-> w -> wOutEdge
			// u == wInEdge, ps[u].parent.getEdge() == vOverlay

			// cek apakah parent_backward_search(vOverlay) == wExitId
			// di backward search: parent dari vOverlay adalah wOutEdge

			vOverlay := ps[u].parent.getEdge()

			oki := da.Lt(pb[vOverlay].GetTravelTime(), pkg.INF_WEIGHT)

			if !oki {
				break
			}

		} else {
			// u overlay vertex tapi ps[u].parent.edge bukan overlay vertex
			// q -vInEdge-> vOverlay
			// u == vOverlay,  ps[u].parent.edge == vInEdge

			// cek parent_backward_searcH(vExitId) == vOverlay

			vOverlay := u

			vEntryId := ps[u].parent.getFirstOverlayEntryExitId()

			_, vOutEdge := ars.engine.graph.GetHeadOfInedgeWithOutEdge(vEntryId)
			vExitId := vOutEdge.GetEdgeId()
			q := ars.engine.graph.GetTailOfOutedge(vExitId)

			offVExitId := ars.engine.offsetBackward(q, vExitId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			oki := da.Lt(pb[offVExitId].GetTravelTime(), pkg.INF_WEIGHT)
			if b := pb[offVExitId]; !(oki && b.parent.edge == vOverlay) {
				// kalau parent dari vInEdge di backward search equal to vOverlay kita bisa lanjut backtrack
				// else: vOverlay (atau u) adalah overlayVertex pertama dari plateau path

				break
			}
		}

		uPar := ps[u].parent
		if !ps[uPar.edge].IsScanned() { // syarat parent(u) ada di shortest path tree forward search
			break
		}
		u = uPar.edge
		uVId = uPar.vertex
	}

	firstPlateauTT := ps[u].GetTravelTime()

	if overlay {
		u = vId
	} else {
		u = viaExitId
	}
	uVId = oriVId

	for u != tBackwardId {
		if ars.engine.isOverlay(u) {
			oki := da.Lt(ps[pb[u].parent.edge].GetTravelTime(), pkg.INF_WEIGHT)
			if f := ps[pb[u].parent.edge]; !(oki && f.parent.edge == u) {
				// vOverlay -vShortcut-> qOverlay -qShortcut-> qParentOverlay
				// u == vOverlay, pb[u].parent.edge == qOverlay
				break
			}
		} else if !ars.engine.isOverlay(u) && !ars.engine.isOverlay(pb[u].parent.edge) {
			// u dan pb[u].parent.edge bukan overlay vertex

			// v -vOutEdge/qInEdge-> q -qOutEdge/qParentInEdge-> qParent
			// u == vOutEdge,  pb[u].parent.edge == qOutEdge, pb[u].parent.vertex=q

			// cek apakaah parent_forward_search(qParentEntryId) == qEntryId
			v := uVId
			vExitId := ars.engine.adjustBackward(v, u)
			_, qInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(vExitId)
			qEntryId := qInEdge.GetEdgeId()

			q := ars.engine.graph.GetHeadOfInedge(qInEdge.GetEdgeId())
			offQExitId := pb[u].parent.getEdge()
			qExitId := ars.engine.adjustBackward(q, offQExitId)

			qParent := ars.engine.graph.GetOutEdge(qExitId).GetHead()
			_, qParentInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(qExitId)
			qParentEntryId := qParentInEdge.GetEdgeId()

			offQParentEntryId := ars.engine.offsetForward(qParent, qParentEntryId, ars.engine.graph.GetCellNumber(qParent), sCellNumber)
			oki := da.Lt(ps[offQParentEntryId].GetTravelTime(), pkg.INF_WEIGHT)

			offQEntryId := ars.engine.offsetForward(q, qEntryId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			if f := ps[offQParentEntryId]; !(oki && f.parent.edge == offQEntryId) {
				break
			}
		} else if !ars.engine.isOverlay(u) && ars.engine.isOverlay(pb[u].parent.edge) {
			// wInEdge -> w -boundaryEdge/vInEdge/wExitEdge-> vOverlay -qShortcut-> q
			// u == wExitEdge, pb[u].parent.getEdge() == vOverlay

			// cek apakah parent_forward_search(vOverlay) == vInEdge

			// di forward search: parent dari vOverlay adalah wInEdge

			vOverlay := pb[u].parent.getEdge()

			oki := da.Lt(ps[vOverlay].GetTravelTime(), pkg.INF_WEIGHT)

			if !oki {
				break
			}
		} else {
			// u overlay vertex tapi pb[u].parent.edge bukan overlay vertex
			// vOverlay -vExitEdge/qInEdge-> q -qExitEdge/qParentInEdge-> qParent
			// u == vOverlay,  pb[u].parent.edge == vExitId

			// cek apakah parent_forward_searach(qEntryId) == vOverlay

			vOverlay := u
			vExitId := pb[u].parent.getFirstOverlayEntryExitId()

			_, qInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(vExitId)
			qEntryId := qInEdge.GetEdgeId()
			q := ars.engine.graph.GetHeadOfInedge(qEntryId)

			offQEntryId := ars.engine.offsetForward(q, qEntryId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			oki := da.Lt(ps[offQEntryId].GetTravelTime(), pkg.INF_WEIGHT)
			if f := ps[offQEntryId]; !(oki && f.parent.edge == vOverlay) {
				break
			}
		}

		uPar := pb[u].parent
		if !pb[uPar.edge].IsScanned() {
			break
		}
		u = uPar.edge
		uVId = uPar.vertex
	}

	var lastPlateauTT float64
	if ars.engine.isOverlay(u) {
		// disini u == last overlay vertex Id dari plateau path
		lastPlateauTT = pb[u].GetTravelTime()
	} else {
		// disini u == last out edge dari plateau path
		// ......-vInEdge-> uVId -uInEdge/u-> head

		lastPlateauTT = pb[u].GetTravelTime()
	}

	// s-> ---- -> via -> ......-> u -> ..... -> t
	// pb[u] = dist(u,t)
	// lv - dist(u,t) = dist(s,u)
	lastPlateauTT = lv - lastPlateauTT

	plateau := math.Max(lastPlateauTT-firstPlateauTT, 0)

	return plateau
}

func removeSimiliarAlternatives(alts []*AlternativeRoute) []*AlternativeRoute {
	set := make([]map[datastructure.Index]struct{}, len(alts))
	for i := 0; i < len(alts); i++ {
		set[i] = make(map[datastructure.Index]struct{}, len(alts[i].GetPath())*2)
	}
	res := make([]*AlternativeRoute, 0, len(alts))
	for i, alt := range alts {
		// O(N^2 * M), N=len(alts), M=max{len(alts.edges[i])}, for each 0<=i<len(alts)

		addToRes := true
		for j := 0; j < i; j++ {
			// check similiarity with other previous alternative routes
			intersection := 0.0

			setJ := set[j]
			altPath := alt.GetPath()
			for _, e := range altPath {
				if _, exists := setJ[e.GetEdgeId()]; exists {
					intersection++
				}
			}

			union := make(map[da.Index]struct{}, len(altPath)+len(setJ))
			maps.Copy(union, setJ)
			for _, e := range altPath {
				union[e.GetEdgeId()] = struct{}{}
			}
			unionSize := float64(len(union))

			jaccardSimiliarity := (intersection / unionSize) * 100
			if jaccardSimiliarity >= pkg.ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD {
				// add alt to result if similiarity with other alternative route < pkg.ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD
				addToRes = false
				break
			}
		}

		if addToRes {
			res = append(res, alt)
		}
		for _, e := range alt.GetPath() {
			// make alternative route path set
			set[i][e.GetEdgeId()] = struct{}{}
		}
	}
	return res
}

func (ars *AlternativeRouteSearch) Reset() {
	ars.candidates = make([]*AlternativeRoute, 0)
}

func (ars *AlternativeRouteSearch) makePackedViaPathOverlayEven(svPackedPath, vtPackedPath []vertexEdgePair) ([]vertexEdgePair, []vertexEdgePair) {

	lSV := 0 // first overlayVertex
	// dari crp query, bisa aja via vertex nya di entryVertex sel sebelah
	// s-> u1 -> u2 -cutEdge-> v1 -shortcut-> v2 -cutEdge-> v3-> via <- ...... vertices scanned by backward search
	// karena sv overlayPath  nya [v1,v2,v3] ganjil dan syarat dari pathUnpacker: len(overlayPath) even, kita harus buat jadi even

	nSV := len(svPackedPath)
	for i := 0; i < nSV-1; i++ {
		if !isBitOn(svPackedPath[i].getEdge(), UNPACK_OVERLAY_OFFSET) && isBitOn(svPackedPath[i+1].getEdge(), UNPACK_OVERLAY_OFFSET) {
			lSV = i + 1
		}
	}

	if lSV != 0 && (nSV-lSV)%2 != 0 && isBitOn(svPackedPath[nSV-1].getEdge(), UNPACK_OVERLAY_OFFSET) {
		svPackedPath = append(svPackedPath, vtPackedPath[0])
		vtPackedPath = vtPackedPath[1:]
	}

	return svPackedPath, vtPackedPath
}

func (ars *AlternativeRouteSearch) GetStretch() float64 {
	/*
		https://dl.acm.org/doi/10.1145/3567421

		We quantify this by measuring the stretch of
		each path, which is the ratio of path’s cost to the shortest path cost.
	*/
	if len(ars.candidates) == 0 {
		return -1 // gak ke count karena gak ada alternative routes
	}

	stretch := 0.0

	for i := 0; i < len(ars.candidates); i++ {
		stretch += ars.candidates[i].GetDrivingTravelTime() / ars.optTravelTime
	}
	stretch /= float64(len(ars.candidates))

	return stretch
}

func (ars *AlternativeRouteSearch) GetDiversity() float64 {

	/*
		https://dl.acm.org/doi/10.1145/3567421
		Another desirable aspect from alternate paths is that each
		path should be sufficiently different from all the preceding ones. For
		this we use Jaccard distance, J(A, B)=\frac{|A \Delta B|}{A \cup B} , where A \Delta B is the
		symmetric set difference. For each path \pi_i , we record the minimum
		Jaccard distance to the preceding paths, min{j <i} J(\pi_i , \pi_j), where
		we view each path \pi_i as a set over edges.
	*/
	if len(ars.candidates) == 0 {
		return -1 // gak ke itung karena gak ada alternative routes
	}

	alts := ars.candidates
	set := make([]map[datastructure.Index]struct{}, len(alts))
	for i := 0; i < len(alts); i++ {
		set[i] = make(map[datastructure.Index]struct{}, len(alts[i].GetPath())*2)
	}

	diversity := 0.0
	for i, alt := range alts {
		// O(N^2 * M), N=len(alts), M=max{len(alts.edges[i])}, for each 0<=i<len(alts)

		minJaccardDist := math.MaxFloat64
		for j := 0; j < i; j++ {
			// check similiarity with other previous alternative routes
			intersection := 0.0

			setJ := set[j]
			altPath := alt.GetPath()
			for _, e := range altPath {
				if _, exists := setJ[e.GetEdgeId()]; exists {
					intersection++
				}
			}

			union := make(map[da.Index]struct{}, len(altPath)+len(setJ))
			maps.Copy(union, setJ)
			for _, e := range altPath {
				union[e.GetEdgeId()] = struct{}{}
			}
			unionSize := float64(len(union))
			jaccardSimiliarity := (intersection / unionSize)

			jaccardDistance := 1 - jaccardSimiliarity
			minJaccardDist = math.Min(minJaccardDist, jaccardDistance)
		}

		for _, e := range alt.GetPath() {
			// make alternative route path set
			set[i][e.GetEdgeId()] = struct{}{}
		}
		if i == 0 {
			continue
		}

		diversity += minJaccardDist

	}

	diversity /= float64(len(alts))

	return diversity
}

func (ars *AlternativeRouteSearch) GetRuntime() int64 {
	return ars.runtime
}

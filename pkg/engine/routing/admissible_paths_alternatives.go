package routing

import (
	"sort"
	"sync"

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

	travelTime float64
	dist       float64
	viaNode    datastructure.Index
	viaVertex  datastructure.ViaVertex
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

func (ar *AlternativeRoute) GetViaNode() datastructure.Index {
	return ar.viaNode
}

func NewAlternativeRoute(objectiveValue, dist, travelTime float64,
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
	}
}

type AlternativeRouteSearch struct {
	engine                *CRPRoutingEngine
	candidates            []*AlternativeRoute
	upperBound            float64
	gamma, alpha, epsilon float64
	lm                    *landmark.Landmark

	lock *sync.RWMutex
}

func NewAlternativeRouteSearch(engine *CRPRoutingEngine, upperBound, gamma, alpha, epsilon, delta float64, lm *landmark.Landmark,

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


todo: add alternative routes stats success rate, UBS, sharing, locally optimal (https://dl.acm.org/doi/abs/10.1145/2444016.2444019)
todo2: ada rute alternative yang unpacked path nya lompat (mungkin karena exclude 1 overlay vertex setelah makePackedViaPathOverlayEven? ), fix this bug
todo3: update tTest, hasil rute alternative buat jarak jauh masih jelek
todo4: mungkin ada yang salah di calculatePlateau?
*/

func (ars *AlternativeRouteSearch) FindAlternativeRoutes(asId, atId datastructure.Index, k int) []*AlternativeRoute {

	crpQuery := NewCRPBidirectionalSearch(ars.engine, ars.upperBound)

	optTravelTime, _, _, optEdgePath, found := crpQuery.ShortestPathSearch(asId, atId)
	if !found {
		return []*AlternativeRoute{}
	}

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
			if fInfo[v.GetEntryId()].GetTravelTime()+bInfo[v.GetExitId()].GetTravelTime() >= (1+ars.epsilon)*optTravelTime {
				viaVertices = append(viaVertices[:i], viaVertices[i+1:]...)
			}
		} else {
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

		if !v.IsOverlay() {
			// forward
			svPackedPath := ars.engine.RetrieveForwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetEntryId(), false),
				fInfo, crpQuery.sForwardId, crpQuery.sCellNumber)
			unpacker := NewPathUnpacker(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, false)
			svCoords, svEdgePath, svDist = unpacker.unpackPath(svPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)

			// backward
			vtPackedPath := ars.engine.RetrieveBackwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetExitId(), true),
				bInfo, crpQuery.tBackwardId, crpQuery.sCellNumber)
			unpacker = NewPathUnpacker(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, false)
			vtCoords, vtEdgePath, vtDist = unpacker.unpackPath(vtPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)
		} else {
			// forward
			svPackedPath := ars.engine.RetrieveForwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				fInfo, crpQuery.sForwardId, crpQuery.sCellNumber)
			unpacker := NewPathUnpacker(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, false)

			// backward
			vtPackedPath := ars.engine.RetrieveBackwardPackedPath(newVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				bInfo, crpQuery.tBackwardId, crpQuery.sCellNumber)
			unpacker = NewPathUnpacker(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, false)

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

		plv := ars.calculatePlateau(v.GetVId(), v.GetOriginalVId(), v.GetEntryId(), v.GetExitId(), crpQuery.sForwardId, crpQuery.tBackwardId,
			fInfo, bInfo, sCellNumber)

		T := ars.alpha * optTravelTime

		if plv <= T {
			// didnt pass t-test
			return nil
		}

		fv := 2*lv + sigmav - plv

		pvCoords := append(svCoords, vtCoords...)

		ars.lock.Lock()
		ars.candidates =
			append(ars.candidates, NewAlternativeRoute(fv, svDist+vtDist, lv, v.GetOriginalVId(), pvCoords, pvEdgePath, v))
		ars.lock.Unlock()
		return nil
	}

	workers := concurrent.NewWorkerPool[datastructure.ViaVertex, any](1, len(viaVertices))

	for _, v := range viaVertices {
		workers.AddJob(v)
	}

	workers.Close()
	workers.Start(computeAlternatives)
	workers.Wait()

	sort.Slice(ars.candidates, func(j, pivotIdx int) bool {
		return ars.candidates[j].objectiveValue < ars.candidates[pivotIdx].objectiveValue
	})

	ars.candidates = removeSimiliarAlternatives(ars.candidates)

	res := make([]*AlternativeRoute, 0, k)

	for i := 0; i < util.MinInt(k, len(ars.candidates)); i++ {
		res = append(res, ars.candidates[i])
	}

	return res
}

/*
Alternative Routes in Road Networks, Abraham et al., page 5:
For local optimality, there is a quick 2-approximation. Take a via path Pv and a
parameter T. Let P1 and P2 be the s–v and v–t subpaths of Pv, respectively. Among all
vertices in P1 that are at least T away from v, let x be the closest to v (and let x = s
if `(P1) < T). Let y be the analogous vertex in P2 (and let y = t if `(P2) < T). We say
that Pv passes the T-test if the portion of Pv between x and y is a shortest path.
Example for two T-tests. The T-test for v successful if shortest between x and y contains v.
*/
func (ars *AlternativeRouteSearch) tTest(T float64, v datastructure.Index, pvEdgePath []datastructure.OutEdge) bool {
	edgeIdContainsV := datastructure.Index(0)
	for i := 0; i < len(pvEdgePath); i++ {
		if pvEdgePath[i].GetHead() == v {
			edgeIdContainsV = datastructure.Index(i)
		}
	}

	x := datastructure.INVALID_VERTEX_ID
	vEdgeId := edgeIdContainsV
	distFromV := 0.0
	for distFromV < T {
		edge := pvEdgePath[vEdgeId]
		distFromV += edge.GetLength()
		x = edge.GetEdgeId()

		vEdgeId--
	}

	y := datastructure.INVALID_VERTEX_ID
	vEdgeId = edgeIdContainsV
	distFromV = 0.0

	for distFromV < T && vEdgeId < datastructure.Index(len(pvEdgePath)) {
		edge := pvEdgePath[vEdgeId]
		distFromV += edge.GetLength()
		y = edge.GetEdgeId()

		vEdgeId++
	}

	if x == datastructure.INVALID_VERTEX_ID || y == datastructure.INVALID_VERTEX_ID {
		return false
	}

	crpQuery := NewCRPBidirectionalSearch(ars.engine, UPPERBOUND_SHORTEST_PATH)

	_, _, _, xyOptPath, _ := crpQuery.ShortestPathSearch(x, y)
	for i := 0; i < len(xyOptPath); i++ {
		edge := xyOptPath[i]
		if edge.GetHead() == v {
			return true
		}
		tail := ars.engine.graph.GetTailOfOutedge(edge.GetEdgeId())
		if tail == v {
			return true
		}
	}
	return false
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
	ps, pb []*VertexInfo[datastructure.CRPQueryKey], sCellNumber da.Pv) float64 {

	u := viaEntryId
	uVId := oriVId
	ok := da.Lt(ps[u].GetTravelTime(), pkg.INF_WEIGHT)
	if !ok {
		u = vId
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
		long plateaus, orders them according to some “goodness” criteria (not explained in the
		original paper [Cambridge Vehicle Information Technology Ltd. 2005]), and outputs
		the best one (or more, if desired).
	*/

	// u = vEntryId/vId  dari via
	// vId = overlayId dari via kalau via nya overlay vertex
	// s-> .... -> u -vInEdge-> via (bisa aja sebuah overlay vertex) <-vExitEdge- w <- ..... <-t

	// task kita disini adalah find total length dari plateau u-w dari definisi platau diatas
	// so kita harus backtrack dari vEntryId/vOverlayId dari via vertex ke vertex awal dari plateau (atau vertex u dari definisi diatas)
	for u != sForwardId {
		if ars.engine.isOverlay(u) {
			// plateau iff parent_backward_search(parent_forward_search(u)) == u

			oki := da.Lt(pb[ps[u].parent.edge].GetTravelTime(), pkg.INF_WEIGHT)
			if b := pb[ps[u].parent.edge]; !(oki && b.parent.edge == u) {
				// qParentOverlay -qShortcut-> qOverlay -vShortcut-> vOverlay
				// u == vOverlay, ps[u].parent.edge == qOverlay
				// kalau parent dari qOverlay di backward search equal to vOverlay kita bisa lanjut backtrack
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
				// kalau parent dari qInEdge di backward search equal to vInEdge kita bisa lanjut backtrack
				// else: vInEdge (atau u) adalah entryEdge pertama dari plateau path
				break
			}
		} else if !ars.engine.isOverlay(u) && ars.engine.isOverlay(ps[u].parent.getEdge()) {
			// q -vShortcut-> vOverlay -boundaryEdge/wInEdge/wExitEdge-> w
			// u == wInEdge, ps[u].parent.getEdge() == vOverlay

			// cek apakah parent_backward_search(vOverlay) == wExitId
			offWEntryId := u
			w := uVId
			vOverlay := ps[u].parent.getEdge()
			wEntryId := ars.engine.adjustForward(w, offWEntryId)
			_, wOutEdge := ars.engine.graph.GetHeadOfInedgeWithOutEdge(wEntryId)
			wExitId := wOutEdge.GetEdgeId()
			v := ars.engine.graph.GetTailOfOutedge(wOutEdge.GetEdgeId())
			offWExitId := ars.engine.offsetBackward(v, wExitId, ars.engine.graph.GetCellNumber(v), sCellNumber)

			oki := da.Lt(pb[vOverlay].GetTravelTime(), pkg.INF_WEIGHT)

			if b := pb[vOverlay].parent.edge; !(oki && b == offWExitId) {
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

		u = ps[u].parent.edge
		uVId = ps[u].parent.vertex
	}

	firstPlateauTT := ps[u].GetTravelTime()

	u = viaExitId
	uVId = oriVId
	ok = da.Lt(pb[u].GetTravelTime(), pkg.INF_WEIGHT)
	if !ok {
		u = vId
	}

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
			// w -boundaryEdge/vInEdge/wExitEdge-> vOverlay -qShortcut-> q
			// u == wExitEdge, pb[u].parent.getEdge() == vOverlay

			// cek apakah parent_forward_search(vOverlay) == vInEdge
			offWExitId := u
			w := uVId
			vOverlay := pb[u].parent.getEdge()
			wExitId := ars.engine.adjustBackward(w, offWExitId)
			_, vInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(wExitId)
			vEntryId := vInEdge.GetEdgeId()

			oki := da.Lt(ps[vOverlay].GetTravelTime(), pkg.INF_WEIGHT)

			v := ars.engine.graph.GetHeadOfInedge(vEntryId)
			offVEntryId := ars.engine.offsetForward(v, vEntryId, ars.engine.graph.GetCellNumber(v), sCellNumber)
			if b := ps[vOverlay].parent.edge; !(oki && b == offVEntryId) {
				break
			}
		} else {
			// u overlay vertex tapi pb[u].parent.edge bukan overlay vertex
			// vOverlay -vExitEdge/qInEdge-> q -qExitEdge/qParentInEdge-> qParent
			// u == vOverlay,  pb[u].parent.edge == vExitId

			// cek apakah parent_forward_searach(qEntryId) == vOverlay

			vOverlay := u
			vExitId := ps[u].parent.getFirstOverlayEntryExitId()

			_, qInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(vExitId)
			qEntryId := qInEdge.GetEdgeId()
			q := ars.engine.graph.GetHeadOfInedge(qEntryId)

			offQEntryId := ars.engine.offsetForward(q, qEntryId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			oki := da.Lt(ps[offQEntryId].GetTravelTime(), pkg.INF_WEIGHT)
			if f := ps[offQEntryId]; !(oki && f.parent.edge == vOverlay) {
				break
			}
		}

		u = pb[u].parent.edge
		uVId = pb[u].parent.vertex
	}

	var lastPlateauTT float64
	if ars.engine.isOverlay(u) {
		// disini u == last overlay vertex Id dari plateau path
		lastPlateauTT = ps[u].GetTravelTime()
	} else {
		// disini u == last exit edge dari plateau path
		// ......-vInEdge-> v -uInEdge/u-> uVId
		adjU := ars.engine.adjustBackward(uVId, u)

		head := ars.engine.graph.GetOutEdge(adjU).GetHead()
		_, uInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(adjU)
		ueId := ars.engine.offsetForward(head, uInEdge.GetEdgeId(), ars.engine.graph.GetCellNumber(head), sCellNumber)
		lastPlateauTT = ps[ueId].GetTravelTime()
	}

	plateau := lastPlateauTT - firstPlateauTT

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
			similiarity := 0.0

			setJ := set[j]
			altPath := alt.GetPath()
			for _, e := range altPath {
				if _, exists := setJ[e.GetEdgeId()]; exists {
					similiarity++
				}
			}

			similiarity = (similiarity / float64(len(altPath))) * 100
			if similiarity > pkg.ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD {
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
	// s-> u1 -> u2 -cutEdge/boundaryEdge-> v1 -shortcut-> v2 -cutEdge/boundaryEdge-> v3-> via <- ...... vertices scanned by backward search
	// karena sv overlayPath  nya [v1,v2,v3] ganjil dan syarat dari pathUnpacker: len(overlayPath) even, kita harus buat jadi even

	nSV := len(svPackedPath)
	for i := 0; i < nSV-1; i++ {
		if !isBitOn(svPackedPath[i].getEdge(), UNPACK_OVERLAY_OFFSET) && isBitOn(svPackedPath[i+1].getEdge(), UNPACK_OVERLAY_OFFSET) {
			lSV = i + 1
		}
	}

	if (nSV-lSV)%2 != 0 {
		svPackedPath = append(svPackedPath, vtPackedPath[0])
		vtPackedPath = vtPackedPath[1:]
	}

	return svPackedPath, vtPackedPath
}

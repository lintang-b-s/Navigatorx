package routing

import (
	"cmp"
	"math"
	"slices"
	"sort"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
)

type AlternativeRouteParameters struct {
	gamma, alpha, epsilon, upperBound float64
	maxCandidatesToUnpack             int
}

func NewAlternativeRouteParameters(gamma, alpha, epsilon, upperBound float64,
	maxCandidatesToUnpack int) AlternativeRouteParameters {
	return AlternativeRouteParameters{
		gamma:                 gamma,
		alpha:                 alpha,
		epsilon:               epsilon,
		upperBound:            upperBound,
		maxCandidatesToUnpack: maxCandidatesToUnpack,
	}
}

type AlternativeRouteSearch[W util.RoutingNumber] struct {
	engine *CRPRoutingEngine[W]

	// parameter yang di init pakai read yml config
	maxCandidatesToUnpackMap       map[float64]int
	gammaMap, alphaMap, epsilonMap map[float64]float64
	upperBoundMap                  map[float64]float64

	defaultGamma, defaultAlpha, defaultEpsilon, defaultUpperbound float64
	defaultMaxCandidatesToUnpack                                  int
}

func NewAlternativeRouteSearch[W util.RoutingNumber](
	engine *CRPRoutingEngine[W],
) *AlternativeRouteSearch[W] {
	alt := &AlternativeRouteSearch[W]{
		engine: engine,
	}
	alt.initParameter()
	return alt
}

/*
implementation of:
1. Abraham, I. et al. (2010) “Alternative Routes in Road Networks,” in P. Festa (ed.)
Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 23–34. Available at:
https://doi.org/10.1007/978-3-642-13193-6_3.
2. page 15: Delling, D. et al. (2015) “Customizable Route Planning in Road
Networks,” Transportation Science [Preprint]. Available at:
https://doi.org/10.1287/trsc.2014.0579.

misalkan l(P) adalah sum of the lengths dari edge penyusun path P
ref[1] setiap st-path P dikatakan admissible jika memenuhi kondisi:
1. sharing amount (edge weights) dari alternative route P dan optimal/shortest route Opt <= \gamma * l(Opt)
2. P is T-Localy Optimall (T-LO): every subpath P' of P with l(P') <= T adalah shortest path
3. every subpath P' dari P dengan endpoints s', t', kita memiliki l(P') <= (1+eps)l(Opt(s,t)) (path P' dari s' ke t' tidak lebih panjang 1+eps relative to shortest path dari s' ke t')

inti dari FindAlternativeRoutes:
1. retrieve semua via vertices yang sudah diexplore (vertex v diexplore atau overlay vertex v sudah discan) oleh forward search dan backward search dari CRP query
2. susun kandidat alternative route s-v-t  untuk setiap via vertices v.
3. return semua kandidat alternative routes yang memenuhi 3 kriteria admissible diatas

*/

func (ars *AlternativeRouteSearch[W]) FindAlternativeRoutes(s, t da.Index, k int, reroute bool, startEdgeId da.Index) ([]AlternativeRoute, float64, int64) {

	/*
		let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any cell
		let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
		time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
		decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately explore v and add neighbor of v (vertex w) to priority queue
		extract-min at most O(m_p+n_o) operations
	*/
	now := time.Now()

	param := ars.parameterByRequest(s, t)

	crpQuery := NewCRPQuery(ars.engine)
	crpQuery.forAlternatives = true
	if reroute {
		crpQuery.reroute = true
	}

	defer func() {
		crpQuery.forAlternatives = false
		crpQuery.Done()
	}()
	crpQuery.SetUpperBound(param.upperBound)

	optWeight, optPath, found := crpQuery.ShortestPathSearch(s, t)
	if !found {
		return []AlternativeRoute{}, pkg.INF_WEIGHT, 0
	}
	optCost := util.WeightToSeconds(optWeight)

	fpq := crpQuery.GetForwardPQ()
	bpq := crpQuery.GetBackwardPQ()
	sCellNumber := crpQuery.GetSCellNumber()
	tCellNumber := crpQuery.GetTCellNumber()

	viaVertices := crpQuery.viaVertices
	viaVertices = ars.filterByUniqueId(viaVertices)

	optPathSet, motorwaySet := ars.buildPathMotorwaySet(optPath)

	scSet := crpQuery.scpSet
	unpacker := newPathUnpackerALT(ars.engine, false)
	arf := NewAlternativeRouteFilter(ars, fpq, bpq,
		sCellNumber, tCellNumber, s, t, optPathSet, motorwaySet, scSet, param, optCost, unpacker)

	filteredCandidates := viaVertices[:0]
	for _, v := range viaVertices {

		filteredCand := arf.filterCandidate(v)
		if isEmptyViaVertex(filteredCand) {
			continue
		}
		filteredCandidates = append(filteredCandidates, filteredCand)
	}

	slices.SortFunc(filteredCandidates, func(a, b ViaVertex) int {
		return cmp.Compare(a.GetApproxObjectiveValue(),
			b.GetApproxObjectiveValue())
	})

	c := util.MinInt(param.maxCandidatesToUnpack, len(filteredCandidates))
	filteredCandidates = filteredCandidates[:c]

	res := make([]AlternativeRoute, 0, c)
	resSet := make([]map[da.Index]struct{}, 0, c)

	for _, v := range filteredCandidates {
		alternativeRoute := arf.computeAlternative(v)
		if isEmptyAlternativeRoute(alternativeRoute) {
			continue
		}

		if len(res) > 0 && !ars.differToOtherAlternatives(resSet, alternativeRoute.segmentPath) {
			continue
		}

		res = append(res, alternativeRoute)
		resSet = append(resSet, ars.buildEdgesPathSet(alternativeRoute.segmentPath))
	}

	clear(arf.optPathSet)
	clear(arf.motorwaySet)

	slices.SortFunc(res, func(a, b AlternativeRoute) int {
		return cmp.Compare(a.objectiveValue, b.objectiveValue)
	})

	maxAltSize := util.MinInt(k, len(res))
	res = res[:maxAltSize]
	for i := 0; i < maxAltSize; i++ {
		finalPath, totalDistance := ars.engine.GetEdgePath(res[i].segmentPath)
		res[i].path = finalPath
		res[i].dist = totalDistance
	}

	// worst case of FindAlternativeRoutes: worst case crp query + worst case computeAlternative for all via vertices
	// O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o) + c * ( p + q * (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p)))
	runtime := time.Since(now).Milliseconds()

	return res, optCost, runtime
}

func intersection(otherAltSet map[da.Index]struct{}, alt []da.Index) int {
	n := 0
	for _, v := range alt {
		if _, ok := otherAltSet[v]; ok {
			n++
		}
	}
	return n
}

func jaccardDistance(otherAltSet map[da.Index]struct{}, alt []da.Index) float64 {
	inter := intersection(otherAltSet, alt)
	lena, lenb := len(alt), len(otherAltSet)
	union := lena + lenb - inter
	return 1.0 - float64(inter)/float64(union)
}

const (
	minAltsDiversity = 0.3
)

func (ars *AlternativeRouteSearch[W]) differToOtherAlternatives(otherAltSets []map[da.Index]struct{}, alt []da.Index) bool {
	minDist := math.MaxFloat64
	for i := 0; i < len(otherAltSets); i++ {
		jcdDist := jaccardDistance(otherAltSets[i], alt)
		if jcdDist < minDist {
			minDist = jcdDist
			if minDist < minAltsDiversity {
				return false
			}
		}
	}
	return minDist >= minAltsDiversity
}

type AlternativeRouteFilter[W util.RoutingNumber] struct {
	ars                      *AlternativeRouteSearch[W]
	fpq, bpq                 *da.QueryHeap[da.QueryKey, W]
	sCellNumber, tCellNumber da.Pv
	s, t                     da.Index
	optPathSet, motorwaySet  map[da.Index]struct{}
	scSet                    map[uint64]uint8
	param                    AlternativeRouteParameters
	optCost                  float64
	unpacker                 *PathUnpackerALT[W]
}

func NewAlternativeRouteFilter[W util.RoutingNumber](ars *AlternativeRouteSearch[W],
	fpq, bpq *da.QueryHeap[da.QueryKey, W],
	sCellNumber, tCellNumber da.Pv,
	s, t da.Index,
	optPathSet, motorwaySet map[da.Index]struct{},
	scSet map[uint64]uint8,
	param AlternativeRouteParameters,
	optCost float64,
	unpacker *PathUnpackerALT[W],
) *AlternativeRouteFilter[W] {
	return &AlternativeRouteFilter[W]{
		ars, fpq, bpq, sCellNumber, tCellNumber, s, t, optPathSet, motorwaySet, scSet,
		param, optCost, unpacker,
	}
}

func (arf *AlternativeRouteFilter[W]) filterCandidate(v ViaVertex) ViaVertex {
	var (
		svCost, vtCost float64

		svPackedPath, vtPackedPath []da.ParentVertex
	)

	/*
		let p = number of edges & shortcut edges in s-via-t path

		worst case of RetrieveForwardPackedPath+RetrieveForwardPackedPath: O(p)
		worst case  of calculatePlateau: O(p)
		worst case of  calculateApproxDistanceShare: O(p)

		worst case of filterCandidate: O(p)
	*/

	svCost = util.WeightToSeconds(arf.fpq.GetCost(v.GetVId()))
	vtCost = util.WeightToSeconds(arf.bpq.GetCost(v.GetVId()))

	// stretch
	lv := svCost + vtCost

	if util.Ge(lv, (1+arf.param.epsilon)*arf.optCost) {
		// dari lemma 4.3 ref[1], kita cukup cek stretch dari via path P_v dan cek sudah pass T-test atau tidak
		return NewEmptyViaVertex()
	}

	plv := arf.ars.calculatePlateau(v.GetVId(), arf.s, arf.t,
		arf.fpq, arf.bpq, arf.sCellNumber, lv)

	T := arf.param.alpha * arf.optCost

	if util.Le(plv, T) {
		// T-test dengan T=\alpha*l(Opt) , v-w path adalah plateau dari P_v
		// plateau must > arf.ars.alpha * arf.optCost
		// plateau = subpath dari Pv yang optimal (shortest path) dari first vertex ke last vertex dari subpath
		// atau every subpath P' of alternative route with l(P') <= T = \alpha* l(Opt) is optimal (shortest path). l(Opt) is the cost/travel time of the shortest path
		// didnt pass t-test
		return NewEmptyViaVertex()
	}

	vp := da.NewParentVertex(v.GetVId())
	if !v.IsOverlay() {
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(vp,
			arf.fpq, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(vp,
			arf.bpq, arf.sCellNumber, arf.t)

	} else {
		vp.SetIsOverlayVertex()
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(vp,
			arf.fpq, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(vp,
			arf.bpq, arf.sCellNumber, arf.t)
	}
	svPackedPath, vtPackedPath = arf.ars.makePackedViaPathOverlayEven(svPackedPath, vtPackedPath)

	approxDistanceShare := arf.ars.calculateApproxDistanceShare(svPackedPath, vtPackedPath, arf.optPathSet, arf.scSet,
		arf.sCellNumber, arf.tCellNumber, arf.motorwaySet)

	// cek approximate limited sharing
	if util.Ge(approxDistanceShare, arf.param.gamma*arf.optCost) {
		return NewEmptyViaVertex()
	}

	v.SetCost(lv)
	v.SetPlateau(plv)
	v.SetApproxSharedDist(approxDistanceShare)

	return v
}

func (arf *AlternativeRouteFilter[W]) unpackViaPath(v ViaVertex) ([]da.Index, []da.Index) {
	var (
		svPackedPath, vtPackedPath []da.ParentVertex
	)

	vp := da.NewParentVertex(v.GetVId())
	if !v.IsOverlay() {
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(vp,
			arf.fpq, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(vp,
			arf.bpq, arf.sCellNumber, arf.t)

	} else {
		vp.SetIsOverlayVertex()
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(vp,
			arf.fpq, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(vp,
			arf.bpq, arf.sCellNumber, arf.t)
	}

	var (
		svPath, vtPath []da.Index
	)
	if !v.IsOverlay() {
		// forward
		svPath = arf.unpacker.unpackPath(svPackedPath, arf.sCellNumber, arf.tCellNumber)
		// backward
		vtPath = arf.unpacker.unpackPath(vtPackedPath, arf.sCellNumber, arf.tCellNumber)
	} else {
		// forward
		svPackedPath, vtPackedPath = arf.ars.makePackedViaPathOverlayEven(svPackedPath, vtPackedPath)
		svPath = arf.unpacker.unpackPath(svPackedPath, arf.sCellNumber, arf.tCellNumber)
		// backward
		vtPath = arf.unpacker.unpackPath(vtPackedPath, arf.sCellNumber, arf.tCellNumber)
	}

	return svPath, vtPath
}

func (arf *AlternativeRouteFilter[W]) computeAlternative(v ViaVertex) AlternativeRoute {

	/*
		let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any cell
		let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
		lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (compact graph CRP graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
		cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
		let q = number of shorcut edges in packedPath
		worst case  of unpackPath: O(q * (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))

		worst case of computeAlternative: O( p + q * (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))
	*/

	svPath, vtPath := arf.unpackViaPath(v)

	sigmav := arf.ars.calculateDistanceShare(svPath, vtPath, arf.optPathSet)
	// cek limited sharing
	if util.Ge(sigmav, arf.param.gamma*arf.optCost) {
		return NewAEmptyAlternativeroute()
	}
	lv := v.GetCost()
	fv := 2*lv + sigmav - v.GetPlateau()

	altEdgeIdPath := removeConsecutiveDuplicates(append(svPath, vtPath...))

	return NewAlternativeRoute(fv, 0, lv, sigmav, v.GetVId(), EmptyCoords,
		altEdgeIdPath, v)
}

// filterByUniqueId removes duplicate via vertices by their VId in-place.
func (ars *AlternativeRouteSearch[W]) filterByUniqueId(vias []ViaVertex) []ViaVertex {
	uniqueViaSet := make(map[da.Index]struct{}, 25)
	j := 0
	for i := 0; i < len(vias); i++ {
		v := vias[i]
		if _, ok := uniqueViaSet[v.GetVId()]; !ok {
			uniqueViaSet[v.GetVId()] = struct{}{}
			vias[j] = v
			j++
		}
	}
	return vias[:j]
}

// calculateDistanceShare. calculate sharing amount (edge weights) dari unpacked path dari alternative route P_v dan optimal/shortest route Opt
func (ars *AlternativeRouteSearch[W]) calculateDistanceShare(svPath, vtPath []da.Index, optPathSet map[da.Index]struct{}) float64 {
	// O(M),  M=len(pvPath)
	distanceShare := 0.0

	for _, v := range svPath {
		if _, ok := optPathSet[v]; ok {
			// kualitas rute alternatif lebih bagus kalau length functionnya travel time
			distanceShare += ars.engine.GetDurationSeconds(v) // todo: kayake ini salah return function nya
		}
	}

	for _, v := range vtPath {
		if _, ok := optPathSet[v]; ok {
			// kualitas rute alternatif lebih bagus kalau length functionnya travel time
			distanceShare += ars.engine.GetDurationSeconds(v) // todo: kayake ini salah return function nya
		}
	}

	return distanceShare
}

func (ars *AlternativeRouteSearch[W]) buildEdgesPathSet(edgesPath []da.Index) map[da.Index]struct{} {
	edgesPathSet := make(map[da.Index]struct{}, 25)
	for _, v := range edgesPath {
		edgesPathSet[v] = struct{}{}
	}
	return edgesPathSet
}

func (ars *AlternativeRouteSearch[W]) buildPathSet(optPath []da.Index) map[da.Index]struct{} {
	optPathSet := make(map[da.Index]struct{}, 25)
	for _, v := range optPath {
		optPathSet[v] = struct{}{}
	}
	return optPathSet
}

func (ars *AlternativeRouteSearch[W]) buildPathMotorwaySet(optPath []da.Index) (map[da.Index]struct{}, map[da.Index]struct{}) {
	optPathSet := ars.buildPathSet(optPath)
	motorwaySet := make(map[da.Index]struct{}, 25)

	for _, v := range optPath {
		eRoadClass := ars.engine.rn.GetRoadClass(v)
		if eRoadClass == pkg.MOTORWAY {
			motorwaySet[v] = struct{}{}
		}
		optPathSet[v] = struct{}{}
	}
	return optPathSet, motorwaySet
}

// calculateApproxDistanceShare. calculate sharing amount (edge weights) dari packed path dari alternative route P_v dan optimal/shortest route Opt
// sekaligus filter rute alternatives yang di jalan tol, kalau shortest path untuk jarak jauh pakai jalan tol. motorwaySet akan ada isinya kalau sp lewat jalan tol
func (ars *AlternativeRouteSearch[W]) calculateApproxDistanceShare(svPackedPath, vtPackedPath []da.ParentVertex, optPathSet map[da.Index]struct{}, scSet map[uint64]uint8,
	sCellNumber, tCellNumber da.Pv, motorwaySet map[da.Index]struct{}) float64 {
	// O(M), M=len(svPackedPath) + len(vtPackedPath)
	distShare := 0.0

	process := func(i int, packedPath []da.ParentVertex) bool {
		u := packedPath[i].GetVertex()
		if isBitOn(u, UNPACK_OVERLAY_OFFSET) {
			// shortcut edge
			// shortcut edges di packed path berpola: (uVertex1, vVertex1), (uVertex1, vVertex2)...
			v := packedPath[i+1].GetVertex()
			uOvId := offBit(u, UNPACK_OVERLAY_OFFSET)
			vOvId := offBit(v, UNPACK_OVERLAY_OFFSET)
			uVertex := ars.engine.overlayGraph.GetVertex(uOvId)
			vVertex := ars.engine.overlayGraph.GetVertex(vOvId)

			uId := uVertex.GetOrigVId()
			vId := vVertex.GetOrigVId()
			_, ok1 := optPathSet[uId]
			_, ok2 := optPathSet[vId]

			uCellNum := uVertex.GetCellNumber()
			ql := ars.engine.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, uCellNum)

			scWeightOffset := ars.engine.overlayGraph.GetShortcutWeightId(uOvId, vOvId, int(ql))
			scWeight := ars.engine.metrics.GetShortcutWeight(scWeightOffset)
			bp := util.Bitpack(uint32(uId), uint32(vId))
			shortcutInOpt := ok1 && ok2 && scSet[bp] == uint8(ql)

			if shortcutInOpt {
				distShare += util.WeightToSeconds(scWeight)
			}
			_, okMtr1 := motorwaySet[uId]
			_, okMtr2 := motorwaySet[uId]

			shortcutInMotorway := okMtr1 && okMtr2 && scSet[bp] == uint8(ql)
			if shortcutInMotorway {
				distShare += util.WeightToSeconds(scWeight) * MOTORWAY_PENALTY
			}
			return true
		} else {
			// road segment
			eWeight := ars.engine.GetDurationSeconds(u) // todo: kayake ini salah return function nya
			if _, ok := optPathSet[u]; ok {
				// kualitas rute alternatif lebih bagus kalau length functionnya travel time
				distShare += eWeight
			}
			if _, ok := motorwaySet[u]; ok {
				distShare += eWeight * MOTORWAY_PENALTY
			}
			return false
		}
	}

	n := len(svPackedPath)
	for i := 0; i < n-1; i++ {
		ov := process(i, svPackedPath)
		if ov {
			i++
		}
	}

	n = len(vtPackedPath)
	for i := 0; i < n-1; i++ {
		ov := process(i, vtPackedPath)
		if ov {
			i++
		}
	}
	return distShare
}

// calculatePlateau. calculate plateau pl(v)
// pada CRP query, kita build shortest path trees dari s dan ke t
// forward search membuat shortest path tree dari s ke every explored vertices di forward search
// backward search membuat shortest path tree dari every vertices explored di backward search ke t (karena backward search pakai reversed edges)
// plateaus adalah maximal paths yang muncul  di kedua shortest path trees
// plateau u-w dari st-path: path dari s ke u + path dari u ke w + path dari w ke t
// semua vertices (vertex atau overlay vertex) path u-w dari u ke w tedapat pada kedua shortest path tree
// atau semua vertex dari path u-w sudah di explore oleh kedua search.
func (ars *AlternativeRouteSearch[W]) calculatePlateau(vId, s, t da.Index,
	ps, pb *da.QueryHeap[da.QueryKey, W], sCellNumber da.Pv, lv float64) float64 {

	// shortest path tree from s to v: all explored (already extracted using extractMin from pq) vertices in forward search
	// shortest path tree from v to t: all explored (already extracted using extractMin from pq) vertices in backward search
	// note that karena backward search pakai reversed edges (dengan bobot setiap rev edge (v,u) sama dengan bobot edge (u,v)), kalau v explored -> est sp cost dari t ke v di backward search equal to sp cost dari v ke t (kalau pakai original edges)
	/*
		Intuisi dari plateau (ref: https://dl.acm.org/doi/abs/10.1145/2444016.2444019):
		buat ngecek apakah alternative route P_v T-Localy Optimall (T-LO): every subpath P' of P_v with l(P') <= T adalah shortest path
		P_v is admissble alternative route iff P_v is T-locally optimal for T=\alpha* l(Opt)
		lemma 4.4 dari ref[1]:
		If P_v corresponds to a plateau u-w, P_v is dist(u,w)-LO
		proof:
		karena semua vertex antara u-w di explore forward search dan u-w diexplore backward search, pakai lemma every subpath of shortest path is shortest path (CLRS): subpath u-w is shortest path
		pakai lemma every subpath of shortest path is shortest path (CLRS) lagi: every subpath P' dari path u ke w, l(P') <= dist(u,w) is shortest path
		karena s-u explored di forward search dan w-t explored di backward search: subpath s-u is shortest path dan subpath w-t is shortest path
		pakai lemma every subpath of shortest path is shortest path lagi: every subpath dari shortest su-path dan wt-path adalah shortest path
		sehingga didapat every subpath P' of P_v with l(P') <= dist(u,w) adalah shortest path.
		perhatikan juga karena P_v bukan shortest path (rute alternative), terdapat vertex x yang belum di explore backward search (x-t is not shortest path) dan vertex y yang belum di explore forward search (s-y is not shortest path)
		x tepat berada sebelum u dan y tepat setelah w, x-y bukan plateau karena subpath x-y bukan shortest path, shg u-w adalah maximal paths that appear in both trees simultaneously
	*/

	// u = vertex id/overlay vertex id  dari via
	// s-> .... -> u -vInEdge-> via (bisa aja sebuah overlay vertex) <-vExitEdge- w <- ..... <-t

	// task kita disini adalah find total length dari plateau u-w dari definisi platau diatas
	// so kita harus backtrack dari vId/vOverlayId dari via vertex ke vertex awal dari plateau (atau vertex u dari definisi diatas)
	// bisa backtrack ke parent(u) kalau parent(u) explored in backward search, atau in shortest path tree dari backward search
	// let n=number of edges in s-via-t path
	// worst case: O(n)

	u := vId
	for u != s {
		p := ps.Get(u).GetParent()
		pvId := p.GetVertex()

		oki := util.Lt(pb.GetCost(pvId), util.Infinity[W]())
		if !oki {
			break
		}
		if explored := pb.IsExplored(pvId); !explored {
			break
		}

		if !ps.IsExplored(pvId) { // syarat parent(u) ada di shortest path tree forward search
			break
		}
		u = pvId
	}
	firstPlateauCost := util.WeightToSeconds(ps.GetCost(u))

	u = vId
	for u != t {
		p := pb.Get(u).GetParent()
		pvId := p.GetVertex()

		oki := util.Lt(ps.GetCost(pvId), util.Infinity[W]())
		if !oki {
			break
		}
		if explored := ps.IsExplored(pvId); !explored {
			break
		}

		if !pb.IsExplored(pvId) {
			break
		}
		u = pvId
	}

	lastPlateauCost := util.WeightToSeconds(pb.GetCost(u))

	// s-> ---- -> via -> ......-> u -> ..... -> t
	// pb[u] = dist(u,t)
	// lv - dist(u,t) = dist(s,u)
	lastPlateauCost = lv - lastPlateauCost

	plateau := max(
		lastPlateauCost-firstPlateauCost,
		0,
	)

	return plateau
}

func (ars *AlternativeRouteSearch[W]) parameterByRequest(s, t da.Index) AlternativeRouteParameters {

	sVertex := ars.engine.graph.GetVertex(s)
	tVertex := ars.engine.graph.GetVertex(t)
	sCoord := sVertex.GetCoordinate()
	tCoord := tVertex.GetCoordinate()
	gcDist := geo.CalculateGreatCircleDistance(sCoord.GetLat(), sCoord.GetLon(),
		tCoord.GetLat(), tCoord.GetLon()) // in km

	param := NewAlternativeRouteParameters(ars.defaultGamma, ars.defaultAlpha,
		ars.defaultEpsilon, ars.defaultUpperbound, ars.defaultMaxCandidatesToUnpack)

	param.gamma = pickParam(ars.gammaMap, gcDist, ars.defaultGamma)

	param.alpha = pickParam(ars.alphaMap, gcDist, ars.defaultAlpha)

	param.epsilon = pickParam(ars.epsilonMap, gcDist, ars.defaultEpsilon)
	param.upperBound = pickParam(ars.upperBoundMap, gcDist, ars.defaultUpperbound)

	param.maxCandidatesToUnpack = pickParam(ars.maxCandidatesToUnpackMap, gcDist, ars.defaultMaxCandidatesToUnpack)

	return param
}

func pickParam[T comparable](paramMap map[float64]T, dist float64, defaultVal T) T {
	keys := make([]float64, 0, len(paramMap))
	for k := range paramMap {
		keys = append(keys, k)
	}
	sort.Float64s(keys)

	for _, k := range keys {
		if util.Le(dist, k) {
			return paramMap[k]
		}
	}
	return defaultVal
}

func (ars *AlternativeRouteSearch[W]) initParameter() {
	altConfig := viper.GetStringMap("alternatives")
	ars.gammaMap, ars.defaultGamma = util.ToFloat64Map(altConfig["gamma"])
	ars.alphaMap, ars.defaultAlpha = util.ToFloat64Map(altConfig["alpha"])
	ars.epsilonMap, ars.defaultEpsilon = util.ToFloat64Map(altConfig["epsilon"])
	ars.upperBoundMap, ars.defaultUpperbound = util.ToFloat64Map(altConfig["upper_bound"])
	validateUpperbound(ars.upperBoundMap, ars.defaultUpperbound)
	ars.maxCandidatesToUnpackMap, ars.defaultMaxCandidatesToUnpack = util.ToFloat64IntMap(altConfig["max_candidates_to_unpack"])
}

func validateUpperbound(upperBoundMap map[float64]float64, defaultUpperbound float64) {
	if defaultUpperbound > 1.75 {
		panic("bidirectional search upperbound must less than or equal 1.75")
	}
	for _, val := range upperBoundMap {
		if val > 1.75 {
			panic("bidirectional search upperbound must less than or equal 1.75")
		}
	}
}

func (ars *AlternativeRouteSearch[W]) makePackedViaPathOverlayEven(svPackedPath, vtPackedPath []da.ParentVertex) ([]da.ParentVertex, []da.ParentVertex) {

	lSV := 0 // first overlayVertex
	// dari crp query, bisa aja via vertex nya di entryVertex sel sebelah
	// s-> u1 -> u2 -cutEdge-> v1 -shortcut-> v2 -cutEdge-> v3-> via <- ...... vertices explored by backward search
	// karena sv overlayPath  nya [v1,v2,v3] ganjil dan syarat dari pathUnpacker: len(overlayPath) even, kita harus buat jadi even

	nSV := len(svPackedPath)
	for i := 0; i < nSV-1; i++ {
		if !isBitOn(svPackedPath[i].GetVertex(), UNPACK_OVERLAY_OFFSET) && isBitOn(svPackedPath[i+1].GetVertex(), UNPACK_OVERLAY_OFFSET) {
			lSV = i + 1
		}
	}

	// todo: kok ada svPath yang semua nya overlay/boundary vertices ya pas didebug di latest changes??

	if (nSV-lSV)%2 != 0 && isBitOn(svPackedPath[nSV-1].GetVertex(), UNPACK_OVERLAY_OFFSET) {
		svPackedPath = append(svPackedPath, vtPackedPath[0])
		vtPackedPath = vtPackedPath[1:]
	}

	return svPackedPath, vtPackedPath
}

// GetStretch compute stretch metrics yang dijelasin di section 5.4 paper: https://dl.acm.org/doi/epdf/10.1145/3567421
// mengukur stretch, ratio dari alternative path cost / fastest path cost...
func (ars *AlternativeRouteSearch[W]) GetStretch(candidates []AlternativeRoute, optimalCost float64) float64 {

	if len(candidates) == 0 {
		return -1 // gak ke count karena gak ada alternative routes
	}

	stretch := 0.0

	for i := 0; i < len(candidates); i++ {
		stretch += candidates[i].travelTime / optimalCost
	}
	stretch /= float64(len(candidates))

	return stretch
}

// GetDiversity compute diversity metrics yang dijelasin di section 5.4 paper: https://dl.acm.org/doi/epdf/10.1145/3567421
// mengukur diversity dari rute alternative kedua,ketiga,.... dari rute alternatives sebelumnya
func (ars *AlternativeRouteSearch[W]) GetDiversity(candidates []AlternativeRoute) float64 {

	if len(candidates) == 0 {
		return -1 // gak ke itung karena gak ada alternative routes
	}

	alts := candidates
	set := make([]map[da.Index]struct{}, len(alts))
	for i := 0; i < len(alts); i++ {
		set[i] = make(map[da.Index]struct{}, len(alts[i].segmentPath)*2)
	}

	diversity := 0.0
	for i, alt := range alts {
		// O(N^2 * M), N=len(alts), M=max{len(alts.edges[i])}, for each 0<=i<len(alts)
		altPath := alt.segmentPath

		minJaccardDist := math.MaxFloat64
		for j := 0; j < i; j++ {
			// check similarity with other previous alternative routes
			intersection := 0.0

			setJ := set[j]
			for _, e := range altPath {
				if _, exists := setJ[e]; exists {
					intersection++
				}
			}

			unionSize := float64(len(setJ) + len(altPath) - int(intersection)) // |A \cup B| = |A|+|B|-|A \cap B|
			if unionSize == 0 {
				continue
			}
			jaccardSimilarity := (intersection / unionSize)

			jaccardDistance := 1 - jaccardSimilarity
			minJaccardDist = min(minJaccardDist, jaccardDistance)
		}

		for _, e := range altPath {
			// make alternative route path set
			set[i][e] = struct{}{}
		}
		if i == 0 {
			continue
		}

		diversity += minJaccardDist
	}

	if len(alts) <= 1 {
		return 0
	}

	diversity /= float64(len(alts) - 1)

	return diversity
}

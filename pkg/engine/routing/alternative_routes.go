package routing

import (
	"cmp"
	"math"
	"slices"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// AlternativeRouteParameters represents alternative route parameters.
type AlternativeRouteParameters struct {
	gamma, alpha, epsilon, upperBound float64
	maxCandidatesToUnpack             int
}

// NewAlternativeRouteParameters creates and returns alternative route parameters.
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

// AlternativeRouteSearch represents alternative route search.
type AlternativeRouteSearch[W util.Numeric] struct {
	re      *CRPRoutingEngine[W]
	metrics *met.Metric[W]
	lm      *landmark.Landmark[W]
}

// NewAlternativeRouteSearch creates and returns alternative route search.
func NewAlternativeRouteSearch[W util.Numeric](
	re *CRPRoutingEngine[W],
) *AlternativeRouteSearch[W] {
	alt := &AlternativeRouteSearch[W]{
		re: re,
	}
	alt.metrics = re.GetMetrics()
	alt.lm = re.GetLandmark()
	return alt
}

/*
implementation of:
1. Abraham, I., Delling, D., Goldberg, A. V., Werneck, R. F. (2010) “Alternative Routes in Road Networks,” in P. Festa (ed.)
Experimental Algorithms. Berlin, Heidelberg: Springer, pp. 23–34. Available at:
https://doi.org/10.1007/978-3-642-13193-6_3.
2. page 15:  Delling, D., Goldberg, A. V., Pajor, T., dan Werneck, R. F. (2015). Customizable
Route Planning in Road Networks. Transportation Science, No. 2, Volume 51,
pages 566-591. Available at: https://www.microsoft.com/en-us/research/wp-content/uploads/2013/01/crp_web_130724.pdf

misalkan l(P) adalah sum of the lengths dari edge penyusun path P
ref[1] setiap st-path P dikatakan admissible jika memenuhi kondisi:
1. sharing amount (edge weights) dari alternative route P dan optimal/shortest route Opt <= \gamma * l(Opt)
2. P is T-Localy Optimal (T-LO): every subpath P' of P with l(P') <= T adalah shortest path
3. P is (1 + eps)-UBS (uniformly bounded stretch). every subpath P' dari P dengan endpoints s', t', kita memiliki l(P') <= (1+eps)l(Opt(s,t)) (path P' dari s' ke t' tidak lebih panjang 1+eps relative to shortest path dari s' ke t')

inti dari ComputeAlternativeRoutes:
1. retrieve semua via vertices yang sudah diexplore (vertex v diexplore atau overlay vertex v sudah discan) oleh forward search dan backward search dari CRP query
2. susun kandidat alternative route s-v-t  untuk setiap via vertices v.
3. return semua kandidat alternative routes yang memenuhi 3 kriteria admissible diatas


todo: baca dan implement https://www.openstreetmap.org/user/daniel-j-h/diary/44020 . DONE mostly cuma jelasin ref[1].. implementasi saat ini udah bagus sih rute alternative nya wkkwk
maybe bisa pakai strategi alternative routes mld nya osrm yg filterPackedPathsByCellSharing() tapi bisa aja yang cell nya sama/sharing kriteria admissible alternative routes nya masih PASS,, jadi gak usah diimplement wkwk
buat bedain dari alternative routes 1 dengan lainnya juga udah pakai jaccardDistance() disini dan udah beda banget alternative routes 1 dengan lainnya....
todo2: baca dan implement https://dl.acm.org/doi/abs/10.1145/2674395   (related to article diatas). DONE, runtime di table VII bisa sampe 1 jam lebih.. gak usah diimplement..
todo3: kadang di rute alternative (implementasi saat ini) di dekat via-vertices agak detour... debug dan cari tahu kenapa & fix it... DONE
todo3 ternyata kemarin ada cycle & detour karena turn costs by angle (di CalcTurningCost() pkg/constant.go) nya bisa negative wkwkwk
masih ada cycle di rute alternative di sekitar Jalan Penggung-Jatinom: https://navigatorx-crp-fe.vercel.app/?source=-7.571379974595928%2C+110.82773784403025+&destination=-7.778774854321185%2C+110.41389941753334+
kayake bisa tinggal ilangin cycle di sekitar via-vertices
DONE :) (removeViaVertexCycle()), detour rute alternative yang aneh juga bisa diilangin degan filter all rute alternative candidates P_v dengan  l(Pv \ Opt) >= (1 + epsilon)*l(Opt \ Pv ) (read section 6.1 ref1)

todo4: optimize runtime & space allocation dari ComputeAlternativeRoutes() (mungkin bisa kurangin pneggunaan hashmap berlebihan wkwkwk, allocation ComputeAlternativeRoutes() nya gede banget, see tests/benchmark/alternative_routes_test.go)
DONE  :), kayake udah mentok optimize disini, tinggal optimize package guidance/
todo5: improve success rate dari ComputeAlternativeRoutes sampai ~90% lagi
kalo kita ilangin loc buat cek UBS:
"if util.Ge((lv - sigmav), (1+arf.param.epsilon)*(arf.optCost-sigmav)) {
		// section 6.1 X-BDV:  https://dl.acm.org/doi/epdf/10.1145/2444016.2444019
		// l(Pv \ Opt) < (1 + epsilon) * l(Opt \ Pv)
		// (the detour is not much longer than the optimal subpath it skips)
		return NewAEmptyAlternativeroute()
	}"
success rate ~90% (naik 10% dari success rate cara heuristik saat ini).
tapi returned rute alternatives nya detournya parah...
mungkin kita bisa cek kalau len(res)==0 setelah loop computeAlternative()
kita bisa iterate filteredCandidates lagi dan only filter alternative route candidates by relaxed UBS criteria???
DONE :), sekarang sucess ratenya >= 88% untuk solo.osm.pbf dan yogyakarta.osm.pbf

*/

// ComputeAlternativeRoutes computes alternative routes.
func (ars *AlternativeRouteSearch[W]) ComputeAlternativeRoutes(s, t da.Index, k int, _ bool, optCost float64, optPath []da.Index,
	crpQuery *CRPQuery[W], param AlternativeRouteParameters) ([]AlternativeRoute, int64) {

	now := time.Now()

	fpq := crpQuery.GetForwardPQ()
	bpq := crpQuery.GetBackwardPQ()
	sCellNumber := crpQuery.GetSCellNumber()
	tCellNumber := crpQuery.GetTCellNumber()
	viaVertices := crpQuery.viaVertices

	viaVertices = filterByUniqueId(viaVertices)
	sortedOptPath := buildSortedOptPath(optPath)
	unpacker := newPathUnpackerALT(ars.re, ars.metrics, ars.lm)
	arf := NewAlternativeRouteFilter(ars, fpq, bpq,
		sCellNumber, tCellNumber, s, t, sortedOptPath, param, optCost, unpacker)

	filteredCandidates := viaVertices[:0]
	for _, v := range viaVertices {
		if arf.filterCandidate(v) {
			filteredCandidates = append(filteredCandidates, v)
		}
	}

	slices.SortFunc(filteredCandidates, func(a, b *ViaVertex) int {
		return cmp.Compare(a.GetApproxObjectiveValue(),
			b.GetApproxObjectiveValue())
	})

	c := min(param.maxCandidatesToUnpack, len(filteredCandidates))
	filteredCandidates = filteredCandidates[:c]

	res := make([]AlternativeRoute, 0, c)
	oAlts := make([][]da.Index, 0, c)

	crpQuery.forAlternatives = false
	crpQuery.Done() // in here, the priority queues of crpQuery not used anymore...

	for _, v := range filteredCandidates {
		alternativeRoute := arf.computeAlternative(v)
		if isEmptyAlternativeRoute(alternativeRoute) {
			continue
		}

		alt := alternativeRoute.segmentPath
		sortedAlt := make([]da.Index, len(alt))
		copy(sortedAlt, alt)
		slices.Sort(sortedAlt)
		if len(res) > 0 && !ars.differToOtherAlternatives(oAlts, sortedAlt) {
			continue
		}

		res = append(res, alternativeRoute)
		oAlts = append(oAlts, sortedAlt)
	}

	reps := RelaxedUBSEpsilonStart
	for len(res) == 0 && util.Le(reps, RelaxedUBSEpsilonMax) {
		// biar success rate alternative routes nya masih >= 88 % wkwkwkw.
		for _, v := range filteredCandidates {
			lv := v.lv
			sigmav := v.sigmav

			if !v.passLimitedSharing && !v.passTTest {
				continue
			}

			if util.Ge((lv - sigmav), (1+arf.param.epsilon*reps)*(arf.optCost-sigmav)) {
				// section 6.1 X-BDV:  https://dl.acm.org/doi/epdf/10.1145/2444016.2444019
				// l(Pv \ Opt) < (1 + epsilon) * l(Opt \ Pv) (relaxed constraint... (1+epsilon*reps)*l(Opt \ Pv) )
				// (the detour is not much longer than the optimal subpath it skips)
				continue
			}

			plv := v.plv
			distSharing := sigmav / arf.optCost
			fv := 2*lv + sigmav - plv

			alternativeRoute := NewAlternativeRoute(fv, 0, lv, distSharing, v.GetVId(), EmptyCoords,
				v.path, v)
			alt := alternativeRoute.segmentPath
			sortedAlt := make([]da.Index, len(alt))
			copy(sortedAlt, alt)
			slices.Sort(sortedAlt)
			if len(res) > 0 && !ars.differToOtherAlternatives(oAlts, sortedAlt) {
				continue
			}

			res = append(res, alternativeRoute)
			oAlts = append(oAlts, sortedAlt)
		}
		reps += 0.05
	}

	slices.SortFunc(res, func(a, b AlternativeRoute) int {
		return cmp.Compare(a.objectiveValue, b.objectiveValue)
	})

	maxAltSize := min(k, len(res))
	res = res[:maxAltSize]
	for i := 0; i < maxAltSize; i++ {
		segmentPath := ars.removeViaVertexCycle(res[i].segmentPath, res[i].viaVertex)
		finalPath, totalDistance := ars.re.GetSegmentPath(segmentPath)
		res[i].path = finalPath
		res[i].segmentPath = segmentPath
		res[i].dist = totalDistance
	}

	// worst case of ComputeAlternativeRoutes: worst case crp query + worst case computeAlternative for all via vertices
	// let c = max number of alternative routes candidates to unpackPath()
	// let pu=time complexity of path unpacking (path_unpacker.go)
	// let p=number of road segments in (s,via,t) alternative route candidate path
	// m=len(optPath)
	// debugging fastest path anyer banten -> sumenep madura cuma m~2400 vertices
	// O(c*(p*log(m) + pu))
	// dominated by path unpacking runtime
	// we only unpack small numbers of alternative route candidates, determined by parameter in car.yaml
	runtime := time.Since(now).Milliseconds()

	return res, runtime
}
func intersection(oAlt, alt []da.Index) int {
	n := 0
	i, j := 0, 0
	na, no := len(alt), len(oAlt)
	for i < na && j < no {
		switch {
		case alt[i] == oAlt[j]:
			n++
			i++
			j++
		case alt[i] < oAlt[j]:
			i++
		default:
			j++
		}
	}
	return n
}

func jaccardDistance(oAlt, alt []da.Index) float64 {
	inter := intersection(oAlt, alt)
	lena, lenb := len(alt), len(oAlt)
	union := lena + lenb - inter
	return 1.0 - float64(inter)/float64(union)
}

const (
	minAltsDiversity = 0.3
)

func (ars *AlternativeRouteSearch[W]) differToOtherAlternatives(oAlts [][]da.Index, alt []da.Index) bool {
	minDist := math.MaxFloat64
	for i := 0; i < len(oAlts); i++ {
		jcdDist := jaccardDistance(oAlts[i], alt)
		if jcdDist < minDist {
			minDist = jcdDist
			if minDist < minAltsDiversity {
				return false
			}
		}
	}
	return minDist >= minAltsDiversity
}

// AlternativeRouteFilter represents alternative route filter.
type AlternativeRouteFilter[W util.Numeric] struct {
	ars                      *AlternativeRouteSearch[W]
	fpq, bpq                 *da.QueryHeap[da.QueryKey, W]
	sCellNumber, tCellNumber da.Pv
	s, t                     da.Index
	sortedOptPath            []da.Index
	param                    AlternativeRouteParameters
	optCost                  float64
	unpacker                 *PathUnpackerALT[W]
}

// NewAlternativeRouteFilter creates and returns alternative route filter.
func NewAlternativeRouteFilter[W util.Numeric](ars *AlternativeRouteSearch[W],
	fpq, bpq *da.QueryHeap[da.QueryKey, W],
	sCellNumber, tCellNumber da.Pv,
	s, t da.Index,
	sortedOptPath []da.Index,
	param AlternativeRouteParameters,
	optCost float64,
	unpacker *PathUnpackerALT[W],
) AlternativeRouteFilter[W] {
	return AlternativeRouteFilter[W]{
		ars, fpq, bpq, sCellNumber, tCellNumber, s, t, sortedOptPath,
		param, optCost, unpacker,
	}
}

func (arf AlternativeRouteFilter[W]) filterCandidate(v *ViaVertex) bool {
	var (
		svCost, vtCost float64
	)

	/*
		let p = number of edges & shortcut edges in s-via-t path

		worst case of RetrieveForwardPackedPath+RetrieveForwardPackedPath: O(p)
		worst case  of calculatePlateau: O(p)
		worst case of  calculateApproxDistanceShare: O(p)

		worst case of filterCandidate: O(p)
	*/

	svCost = util.WeightToSeconds(arf.fpq.Cost(v.GetVId()))
	vtCost = util.WeightToSeconds(arf.bpq.Cost(v.GetVId()))

	// stretch
	lv := svCost + vtCost

	if util.Ge(lv, (1+arf.param.epsilon)*arf.optCost) {
		// dari lemma 4.3 ref[1], kita cukup cek stretch dari via path P_v dan cek sudah pass T-test atau tidak, untuk cek P_v PASS UBS criteria
		return false
	}

	var mid da.ParentVertex
	if v.IsOverlay() {
		mid = da.NewParentVertex(v.GetVId(), true)
	} else {
		mid = da.NewParentVertex(v.GetVId(), false)
	}

	packedPath := arf.ars.re.RetrievePackedPath(mid, arf.fpq, arf.bpq)

	approxDistanceShare := arf.ars.calculateApproxDistanceShare(packedPath, arf.sortedOptPath,
		arf.sCellNumber, arf.tCellNumber) // O(p*log(m)), m=len(optPath)

	// cek approximate limited sharing
	if util.Ge(approxDistanceShare, arf.param.gamma*arf.optCost) {
		return false
	}

	plv := arf.ars.calculatePlateau(v.GetVId(), arf.s, arf.t,
		arf.fpq, arf.bpq, lv) // O(p)
	v.plv = plv // harus dicalculate disini karena setelah ini kita sort alternative route candidates by GetApproxObjectiveValue()

	v.lv = lv
	v.approxSharedDist = approxDistanceShare
	v.packedPath = packedPath
	return true
}

func (arf AlternativeRouteFilter[W]) unpackViaPath(v *ViaVertex) []da.Index {
	packedPath := v.packedPath
	path := arf.unpacker.unpackPath(packedPath, arf.sCellNumber, arf.tCellNumber)
	return path
}

func (arf AlternativeRouteFilter[W]) computeAlternative(v *ViaVertex) AlternativeRoute {

	/*
		let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcut edges (and cut edges) within any cell
		let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
		lowest level cell path unpacking: O((n_p + m_p)*log(n_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most n_p, decrease-key and insert at most O(m_p) operations, extract-min at-most O(n_p) operations
		cell level > 1 path unpacking: O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
		let pu=time complexity of path unpacking (path_unpacker.go)
		let p = number of road segments in (s,via,t) alternative route candidate path
		m=len(sortedOptPath)
		debugging fastest path anyer banten -> sumenep madura cuma m~2400 vertices
		worst case of computeAlternative: O(p*log(m)+pu)
	*/

	path := arf.unpackViaPath(v)

	sigmav := arf.ars.calculateDistanceShare(path, arf.sortedOptPath) // O(p*log(m))
	// cek limited sharing
	if util.Ge(sigmav, arf.param.gamma*arf.optCost) {
		return NewAEmptyAlternativeroute()
	}
	v.sigmav = sigmav
	v.path = path

	lv := v.Cost()
	// cek T-test
	T := arf.param.alpha * (lv - sigmav)
	plv := v.plv
	if util.Le(plv, T) {
		// section 6.1 X-BDV:  https://dl.acm.org/doi/epdf/10.1145/2444016.2444019
		// T-test dengan T=\alpha*l(P_v \ Opt) , if alternative route candidate P_v have s-u-w-t, and all vertices between u-w have appears on shortest path trees of forward search and backward search, path u-w adalah plateau dari P_v
		// length of the plateau must > arf.param.alpha * l(P_v \ Opt)
		// plateau = subpath dari Pv yang optimal (shortest path) dari first vertex ke last vertex dari subpath
		// atau every subpath P' of alternative route with l(P') <= T = \alpha* l(P_v \ Opt) is optimal (shortest path). l(P_v \ Opt) is the cost/travel time of the alternative route candidate excluding its subpath that also contained in the optimal fastest path.
		// didnt pass t-test
		return NewAEmptyAlternativeroute()
	}
	v.passLimitedSharing = true
	v.passTTest = true

	if util.Ge((lv - sigmav), (1+arf.param.epsilon)*(arf.optCost-sigmav)) {
		// section 6.1 X-BDV:  https://dl.acm.org/doi/epdf/10.1145/2444016.2444019
		// l(Pv \ Opt) < (1 + epsilon) * l(Opt \ Pv)
		// (the detour is not much longer than the optimal subpath it skips)
		return NewAEmptyAlternativeroute()
	}

	fv := 2*lv + sigmav - plv
	altSegmentPath := v.path
	distSharing := sigmav / arf.optCost
	return NewAlternativeRoute(fv, 0, lv, distSharing, v.GetVId(), EmptyCoords,
		altSegmentPath, v)
}

func buildSortedOptPath(optPath []da.Index) []da.Index {
	// O(mlog(m))
	// debugging fastest path anyer banten -> sumenep madura cuma m~2400 vertices
	// mungin kalau fastest path ujung ke ujung benua eropa masih < 50k vertices??
	sortedOptPath := make([]da.Index, len(optPath))
	copy(sortedOptPath, optPath)
	slices.Sort(sortedOptPath)
	return sortedOptPath
}

// filterByUniqueId removes duplicate via vertices by their vertex id in-place.
func filterByUniqueId(vias []*ViaVertex) []*ViaVertex {
	slices.SortFunc(vias, func(a, b *ViaVertex) int {
		return cmp.Compare(a.GetVId(), b.GetVId())
	})
	return slices.CompactFunc(vias, func(a, b *ViaVertex) bool {
		return a.GetVId() == b.GetVId()
	})
}

// calculateDistanceShare. calculate sharing amount (edge weights) dari unpacked path dari alternative route P_v dan optimal/shortest route Opt
func (ars *AlternativeRouteSearch[W]) calculateDistanceShare(path, sortedOptPath []da.Index) float64 {
	// O(p* log(m)),  p=len(pvPath), m=len(sortedOptPath)
	// debugging fastest path anyer banten -> sumenep madura cuma m~2400 vertices
	distanceShare := 0.0

	for _, v := range path {
		weight := ars.metrics.GetDurationSeconds(v)
		inOpt := isInOpt(v, sortedOptPath)
		if inOpt {
			// kualitas rute alternatif lebih bagus kalau length functionnya travel time
			distanceShare += weight
		}

		if ok := ars.isMotorway(v); inOpt && ok {
			distanceShare += weight * ars.re.motorwayPenalty
		}
	}

	return distanceShare
}

func (ars *AlternativeRouteSearch[W]) isMotorway(v da.Index) bool {
	return ars.re.rn.GetRoadClass(v) == pkg.MOTORWAY
}

// calculateApproxDistanceShare. calculate sharing amount (edge weights) dari packed path dari alternative route P_v dan optimal/shortest route Opt
// sekaligus filter rute alternatives yang di jalan tol, kalau shortest path untuk jarak jauh pakai jalan tol. motorwaySet akan ada isinya kalau sp lewat jalan tol
func (ars *AlternativeRouteSearch[W]) calculateApproxDistanceShare(packedPath, sortedOptPath []da.Index,
	sCellNumber, tCellNumber da.Pv) float64 {
	// O(p). p=len(packedPath)
	distShare := 0.0

	n := len(packedPath)
	for i := 0; i < n-1; i++ {
		u := packedPath[i]
		if isBitOn(u, UnpackOverlayOffset) {
			// shortcut edge
			// shortcut edges di packed path berpola: (uVertex1, vVertex1), (uVertex1, vVertex2)...
			v := packedPath[i+1]
			uOvId := offBit(u, UnpackOverlayOffset)
			vOvId := offBit(v, UnpackOverlayOffset)
			uVertex := ars.re.overlayGraph.GetVertex(uOvId)
			vVertex := ars.re.overlayGraph.GetVertex(vOvId)

			uId := uVertex.GetOrigVId()
			vId := vVertex.GetOrigVId()
			oku := isInOpt(uId, sortedOptPath)
			okv := isInOpt(vId, sortedOptPath)

			uCellNum := uVertex.GetCellNumber()
			ql := ars.re.overlayGraph.QueryLevel(sCellNumber, tCellNumber, uCellNum)

			shortcutInOpt := oku && okv

			if shortcutInOpt {
				scWeightOffset := ars.re.overlayGraph.GetShortcutWeightId(uOvId, vOvId, int(ql))
				scWeight := ars.re.metrics.GetShortcutWeight(scWeightOffset)
				distShare += util.WeightToSeconds(scWeight)
				shortcutInMotorway := ars.isMotorway(uId) && ars.isMotorway(vId)
				if shortcutInMotorway {
					distShare += util.WeightToSeconds(scWeight) * ars.re.motorwayPenalty
				}
			}
			i++
		} else {
			// road segment
			eWeight := ars.metrics.GetDurationSeconds(u)
			inOpt := isInOpt(u, sortedOptPath)
			if inOpt {
				// kualitas rute alternatif lebih bagus kalau length functionnya travel time
				distShare += eWeight
			}
			if ok := ars.isMotorway(u); inOpt && ok {
				distShare += eWeight * ars.re.motorwayPenalty
			}
		}
	}

	return distShare
}

func isInOpt(v da.Index, sortedOptPath []da.Index) bool {
	// O(log(m)). m=len(sortedOptPath)
	// debugging fastest path anyer banten -> sumenep madura cuma m~2400 vertices
	// mungin kalau fastest path ujung ke ujung benua eropa masih < 50k vertices??
	_, ok := slices.BinarySearch(sortedOptPath, v)
	return ok
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
	ps, pb *da.QueryHeap[da.QueryKey, W], lv float64) float64 {

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
	// s-> .... -> u -e1-> via (bisa aja sebuah overlay vertex) <-e2- w <- ..... <-t

	// task kita disini adalah find total length dari plateau u-w dari definisi platau diatas
	// so kita harus backtrack dari vId/vOverlayId dari via vertex ke vertex awal dari plateau (atau vertex u dari definisi diatas)
	// bisa backtrack ke parent(u) kalau parent(u) explored in backward search, atau in shortest path tree dari backward search
	// let n=number of edges in s-via-t path
	// worst case: O(n)

	u := vId
	for u != s {
		p := ps.Get(u).GetParent()
		pvId := p.GetVertex()

		oki := util.Lt(pb.Cost(pvId), util.Infinity[W]())
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
	firstPlateauCost := util.WeightToSeconds(ps.Cost(u))

	u = vId
	for u != t {
		p := pb.Get(u).GetParent()
		pvId := p.GetVertex()

		oki := util.Lt(ps.Cost(pvId), util.Infinity[W]())
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

	lastPlateauCost := util.WeightToSeconds(pb.Cost(u))

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

// ParameterByRequest handles parameter by request.
func (ars *AlternativeRouteSearch[W]) ParameterByRequest(s, t da.Index) AlternativeRouteParameters {

	sVertex := ars.re.graph.GetVertex(s)
	tVertex := ars.re.graph.GetVertex(t)
	sCoord := sVertex.GetCoordinate()
	tCoord := tVertex.GetCoordinate()
	gcDist := geo.CalculateGreatCircleDistance(sCoord.GetLat(), sCoord.GetLon(),
		tCoord.GetLat(), tCoord.GetLon()) // in km

	param := NewAlternativeRouteParameters(ars.re.defaultGamma, ars.re.defaultAlpha,
		ars.re.defaultEpsilon, ars.re.defaultUpperbound, ars.re.defaultMaxCandidatesToUnpack)

	param.gamma = ars.re.gammaLookup.Pick(gcDist)
	param.alpha = ars.re.alphaLookup.Pick(gcDist)
	param.epsilon = ars.re.epsilonLookup.Pick(gcDist)
	param.upperBound = ars.re.upperBoundLookup.Pick(gcDist)
	param.maxCandidatesToUnpack = ars.re.maxCandidatesLookup.Pick(gcDist)
	return param
}

/*
removeViaVertexCycle removing cycles around via-vertices from an alternative route

example: 1     1
		v7<--v6<--
        |	     /\
        |        |
		\/       |
	1  v8	    v5  1
		|	    /\
		|	    |
		\/	    |
v1 --> v2 ----> v3 --> v4 ->
1 	    2       2      1
			    rs     re

via-vertex=v7
input path with cycle: (v1,v2,v3,v5,v6,v7,v8,v2,v3,v4)
the expected path without cycle: (v1,v2,v3,v4,...)
O(n) algorithm to remove all vertices cycle path (v5,v6,v7,v8,v2,v3):
1. iterate all vertices backward from via-vertex v7 (or until a threshold number of traversed vertices=200), count the encounter of every vertex it traversed. example path: (v7,v6,v5,v3,v2,v1)
2. iterate all vertices forward from via-vertex v7 (or until a threshold number of traversed vertices=200), count the encounter of every vertex it traversed. if we found a edge (u,v) with count(v)==1 && count(u)==2, set index re=index of v in the alternative route path.
in this example v4 is the index of re.
3. iterate all vertices backward from via-vertex v7 (or until a threshold number of traversed vertices=200), if we found a first vertex v with count(v)==2 stop and set index rs= index of v in the alternative route path.
int this example v3 is the index of rs.
4. remove all vertices in index [rs+1, re). in this example vertices to be removed is (v5,v6,v7,v8,v2,v3) of the cycle.
worst case is O(n) where n=len(segmentPath).
*/ // nolint: gofmt
func (ars *AlternativeRouteSearch[W]) removeViaVertexCycle(segmentPath []da.Index, via *ViaVertex) []da.Index {
	n := len(segmentPath)

	viaV := via.v
	if via.IsOverlay() {
		viaV = ars.re.adjustoffsetOverlay(viaV)
		viaOvVertex := ars.re.overlayGraph.GetVertex(viaV)
		viaV = viaOvVertex.GetOrigVId()
	}

	mid := 0
	for i := 0; i < n; i++ {
		if segmentPath[i] == viaV {
			mid = i
			break
		}
	}

	segmentCount := make(map[da.Index]uint8, TraverseFromViaVertexCount)
	s := max(mid-TraverseFromViaVertexCount+1, 0)
	for i := mid; i >= s; i-- {
		v := segmentPath[i]
		segmentCount[v]++
	}

	enterCycle := false
	re := da.Index(math.MaxUint32)
	e := min(mid+TraverseFromViaVertexCount, n)
	for i := mid + 1; i < e; i++ {
		v := segmentPath[i]
		segmentCount[v]++
		if segmentCount[v] == 2 {
			enterCycle = true
		}
		if enterCycle && segmentCount[v] == 1 {
			re = da.Index(i)
			break
		}
	}

	rs := da.Index(math.MaxUint32)
	for i := mid; i >= s; i-- {
		v := segmentPath[i]
		if segmentCount[v] == 2 {

			rs = da.Index(i)
			break
		}
	}

	if rs != math.MaxUint32 && re != math.MaxUint32 {
		segmentPath = append(segmentPath[:rs+1], segmentPath[re:]...)
	}

	return segmentPath
}

// GetStretch compute stretch metrics yang dijelasin di section 5.4 paper: https://dl.acm.org/doi/epdf/10.1145/3567421
// mengukur stretch, ratio dari alternative path cost / fastest path cost...
func (ars *AlternativeRouteSearch[W]) GetStretch(candidates []AlternativeRoute, optimalCost float64) float64 {

	if len(candidates) == 0 {
		return -1 // gak ke count karena gak ada alternative routes
	}

	stretch := 0.0

	for i := 0; i < len(candidates); i++ {
		stretch += ((candidates[i].travelTime / optimalCost) - 1.0) * 100
	}
	stretch /= float64(len(candidates))

	return stretch
}

// GetDiversity compute diversity metrics yang dijelasin di section 5.4 paper: https://dl.acm.org/doi/epdf/10.1145/3567421
// mengukur diversity dari rute alternative kedua,ketiga,.... dari rute alternatives sebelumnya
// cuma di call di evaluation code
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

// GetLocalOptimality compute local optimality in percentage. read table I in https://dl.acm.org/doi/epdf/10.1145/2444016.2444019
func (ars *AlternativeRouteSearch[W]) GetLocalOptimality(candidates []AlternativeRoute, optimalCost float64) float64 {

	if len(candidates) == 0 {
		return -1 // gak ke count karena gak ada alternative routes
	}

	locOpt := 0.0

	for i := 0; i < len(candidates); i++ {
		locOpt += (candidates[i].viaVertex.GetPlateau() / optimalCost) * 100
	}
	locOpt /= float64(len(candidates))

	return locOpt
}

// GetDistanceSharing compute distance sharing in percentage. read table I in https://dl.acm.org/doi/epdf/10.1145/2444016.2444019
func (ars *AlternativeRouteSearch[W]) GetDistanceSharing(candidates []AlternativeRoute) float64 {

	if len(candidates) == 0 {
		return -1 // gak ke count karena gak ada alternative routes
	}

	distSharing := 0.0

	for i := 0; i < len(candidates); i++ {
		distSharing += candidates[i].distSharing * 100
	}

	distSharing /= float64(len(candidates))
	return distSharing
}

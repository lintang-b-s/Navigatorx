package routing

import (
	"cmp"
	"math"
	"slices"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/spf13/viper"
)

type AlternativeRoute struct {
	path              *da.Coordinates
	edgeIdPath        []da.Index
	objectiveValue    float64
	drivingDirections []da.DrivingDirection
	polylinePath      string

	travelTime  float64
	dist        float64
	distSharing float64
	viaNode     da.Index
	viaVertex   da.ViaVertex
}

func (ar *AlternativeRoute) GetCoords() *da.Coordinates {
	return ar.path
}

func (ar *AlternativeRoute) GetPolylinePath() string {
	return ar.polylinePath
}

func (ar *AlternativeRoute) SetPolylinePath(pp string) {
	ar.polylinePath = pp
}

func (ar *AlternativeRoute) GetObjectiveValue() float64 {
	return ar.objectiveValue
}

func (ar *AlternativeRoute) GetDrivingDirections() []da.DrivingDirection {
	return ar.drivingDirections
}
func (ar *AlternativeRoute) SetDrivingDirections(dds []da.DrivingDirection) {
	ar.drivingDirections = dds
}

func (ar *AlternativeRoute) GetDrivingTravelTime() float64 {
	return ar.travelTime
}

func (ar *AlternativeRoute) SetDrivingTravelTime(travelTime float64) {
	ar.travelTime = travelTime
}

func (ar *AlternativeRoute) GetViaVertex() da.ViaVertex {
	return ar.viaVertex
}

func (ar *AlternativeRoute) GetDist() float64 {
	return ar.dist
}

func (ar *AlternativeRoute) GetDistSharing() float64 {
	return ar.distSharing
}

func (ar *AlternativeRoute) GetViaNode() da.Index {
	return ar.viaNode
}

func (ar *AlternativeRoute) GetEdgeIdPath() []da.Index {
	return ar.edgeIdPath
}

func (ar *AlternativeRoute) SetEdgeIdPath(edgeIdPath []da.Index) {
	ar.edgeIdPath = edgeIdPath
}

func (ar *AlternativeRoute) SetCoordPath(path *da.Coordinates) {
	ar.path = path
}

func (ar *AlternativeRoute) SetDist(dist float64) {
	ar.dist = dist
}

func NewAlternativeRoute(objectiveValue, dist, travelTime, distSharing float64,
	viaNode da.Index, path *da.Coordinates, edgeIdPath []da.Index,
	viaVertex da.ViaVertex) AlternativeRoute {
	return AlternativeRoute{
		objectiveValue: objectiveValue,
		viaNode:        viaNode,
		path:           path,
		dist:           dist,
		travelTime:     travelTime,
		viaVertex:      viaVertex,
		distSharing:    distSharing,
		edgeIdPath:     edgeIdPath,
	}
}

func NewAEmptyAlternativeroute() AlternativeRoute {
	return AlternativeRoute{viaNode: da.INVALID_VERTEX_ID}
}

func isEmptyAlternativeRoute(ar AlternativeRoute) bool {
	return ar.viaNode == da.INVALID_VERTEX_ID
}

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

func (al *AlternativeRouteParameters) setGamma(gm float64) {
	al.gamma = gm
}

func (al *AlternativeRouteParameters) setAlpha(alp float64) {
	al.alpha = alp
}

func (al *AlternativeRouteParameters) setEpsilon(eps float64) {
	al.epsilon = eps
}

func (al *AlternativeRouteParameters) setUpperbound(upb float64) {
	al.upperBound = upb
}

func (al *AlternativeRouteParameters) setMaxCandidatesToUnpack(maxCandidatesToUnpack int) {
	al.maxCandidatesToUnpack = maxCandidatesToUnpack
}

func (al *AlternativeRouteParameters) getGamma() float64 {
	return al.gamma
}

func (al *AlternativeRouteParameters) getAlpha() float64 {
	return al.alpha
}

func (al *AlternativeRouteParameters) getEpsilon() float64 {
	return al.epsilon
}

func (al *AlternativeRouteParameters) getUpperbound() float64 {
	return al.upperBound
}

func (al *AlternativeRouteParameters) getMaxCandidatesToUnpack() int {
	return al.maxCandidatesToUnpack
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

var (
	indexMapPool sync.Pool = sync.Pool{
		New: func() any {
			return make(map[da.Index]struct{})
		},
	}
)

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
1. retrieve semua via vertices yang sudah discan (beberapa entry & exit points dari vertex v discan atau overlay vertex v sudah discan) oleh forward search dan backward search dari CRP query
2. susun kandidat alternative route s-v-t  untuk setiap via vertices v.
3. return semua kandidat alternative routes yang memenuhi 3 kriteria admissible diatas

*/

func (ars *AlternativeRouteSearch[W]) FindAlternativeRoutes(sp, tp da.PhantomNode, k int, reroute bool, startEdgeId da.Index) ([]AlternativeRoute, float64, int64) {

	/*
		let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any cell
		let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
		time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
		decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
		extract-min at most O(m_p+n_o) operations
	*/
	now := time.Now()

	asId := sp.GetOutEdgeId()
	atId := tp.GetInEdgeId()
	asEdge := ars.engine.graph.GetOutEdge(asId)
	s := asEdge.GetHead()
	atEdge := ars.engine.graph.GetInEdge(atId)
	t := atEdge.GetTail()

	param := ars.parameterByRequest(s, t)

	crpQuery := NewCRPBidirectionalSearch(ars.engine, param.getUpperbound())
	crpQuery.SetForAlternativeRoutes(true)
	if reroute {
		crpQuery.SetReroute()
	}

	defer func() {
		crpQuery.SetForAlternativeRoutes(false)
		crpQuery.Done()
	}()

	optWeight, optEdgeIdPath, found := crpQuery.ShortestPathSearch(sp, tp)
	if !found {
		return []AlternativeRoute{}, pkg.INF_WEIGHT, 0
	}
	optTravelTime := ars.engine.GetCostFunction().WeightToSeconds(optWeight)

	fpq := crpQuery.GetForwardPQ()
	bpq := crpQuery.GetBackwardPQ()

	sCellNumber := crpQuery.GetSCellNumber()
	tCellNumber := crpQuery.GetTCellNumber()

	viaVertices := crpQuery.GetViaVertices()
	viaVertices = ars.filterByUniqueId(viaVertices)

	optPathSet, motorwaySet := ars.buildPathMotorwaySet(optEdgeIdPath)

	shortcutPathSet := crpQuery.getShortcutPathSet()
	unpacker := NewPathUnpackerALT(ars.engine)
	defer unpacker.DonePooled()
	arf := NewAlternativeRouteFilter(ars, fpq, bpq, crpQuery.sForwardId, crpQuery.tBackwardId,
		sCellNumber, tCellNumber, s, t, optPathSet, motorwaySet, shortcutPathSet, param, optTravelTime, unpacker)

	filteredCandidates := viaVertices[:0]
	for _, v := range viaVertices {

		filteredCand := arf.filterCandidate(v)
		if da.IsEmptyViaVertex(filteredCand) {
			continue
		}
		filteredCandidates = append(filteredCandidates, filteredCand)
	}

	slices.SortFunc(filteredCandidates, func(a, b da.ViaVertex) int {
		return cmp.Compare(a.GetApproxObjectiveValue(),
			b.GetApproxObjectiveValue())
	})

	c := util.MinInt(param.getMaxCandidatesToUnpack(), len(filteredCandidates))
	filteredCandidates = filteredCandidates[:c]

	res := make([]AlternativeRoute, 0, c)
	resSet := make([]map[da.Index]struct{}, 0, c)

	for _, v := range filteredCandidates {
		alternativeRoute := arf.computeAlternative(v)
		if isEmptyAlternativeRoute(alternativeRoute) {
			continue
		}

		if len(res) > 0 && !ars.differToOtherAlternatives(resSet, alternativeRoute.GetEdgeIdPath()) {
			continue
		}

		res = append(res, alternativeRoute)
		resSet = append(resSet, ars.buildEdgesPathSet(alternativeRoute.GetEdgeIdPath()))
	}

	clear(arf.optPathSet)
	clear(arf.motorwaySet)
	indexMapPool.Put(arf.optPathSet)
	indexMapPool.Put(arf.motorwaySet)
	putSetsToPool(resSet)

	slices.SortFunc(res, func(a, b AlternativeRoute) int {
		return cmp.Compare(a.GetObjectiveValue(), b.GetObjectiveValue())
	})

	maxAltSize := util.MinInt(k, len(res))
	res = res[:maxAltSize]
	for i := 0; i < maxAltSize; i++ {
		finalPath, totalDistance := ars.engine.GetEdgePath(res[i].GetEdgeIdPath(), 0)
		res[i].SetCoordPath(finalPath)
		res[i].SetDist(totalDistance)
	}

	// worst case of FindAlternativeRoutes: worst case crp query + worst case computeAlternative for all via vertices
	// O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o) + c * ( p + q * (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p)))
	runtime := time.Since(now).Milliseconds()

	return res, optTravelTime, runtime
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
	fpq, bpq                 *da.QueryHeap[da.CRPQueryKey, W]
	sForwardId, tBackwardId  da.Index
	sCellNumber, tCellNumber da.Pv
	s, t                     da.Index
	optPathSet, motorwaySet  map[da.Index]struct{}
	shortcutPathSet          map[uint64]uint8
	param                    AlternativeRouteParameters
	optTravelTime            float64
	unpacker                 *PathUnpackerALT[W]
}

func NewAlternativeRouteFilter[W util.RoutingNumber](ars *AlternativeRouteSearch[W],
	fpq, bpq *da.QueryHeap[da.CRPQueryKey, W],
	sForwardId, tBackwardId da.Index,
	sCellNumber, tCellNumber da.Pv,
	s, t da.Index,
	optPathSet, motorwaySet map[da.Index]struct{},
	shortcutPathSet map[uint64]uint8,
	param AlternativeRouteParameters,
	optTravelTime float64,
	unpacker *PathUnpackerALT[W],
) *AlternativeRouteFilter[W] {
	return &AlternativeRouteFilter[W]{
		ars, fpq, bpq, sForwardId, tBackwardId, sCellNumber, tCellNumber, s, t, optPathSet, motorwaySet, shortcutPathSet,
		param, optTravelTime, unpacker,
	}
}

func (arf *AlternativeRouteFilter[W]) filterCandidate(v da.ViaVertex) da.ViaVertex {
	var (
		svTravelTime, vtTravelTime float64

		svPackedPath, vtPackedPath []da.VertexEdgePair
	)

	/*
		let p = number of edges & shortcut edges in s-via-t path

		worst case of RetrieveForwardPackedPath+RetrieveForwardPackedPath: O(p)
		worst case  of calculatePlateau: O(p)
		worst case of  calculateApproxDistanceShare: O(p)

		worst case of filterCandidate: O(p)
	*/

	if !v.IsOverlay() {
		// via vertex is an overlay vertex
		svTravelTime = arf.ars.engine.GetCostFunction().WeightToSeconds(arf.fpq.GetPriority(v.GetEntryId()))
		vtTravelTime = arf.ars.engine.GetCostFunction().WeightToSeconds(arf.bpq.GetPriority(v.GetExitId()))
	} else {
		// via vertex is not an overlay vertex
		svTravelTime = arf.ars.engine.GetCostFunction().WeightToSeconds(arf.fpq.GetPriority(v.GetVId()))
		vtTravelTime = arf.ars.engine.GetCostFunction().WeightToSeconds(arf.bpq.GetPriority(v.GetVId()))
	}

	// stretch
	lv := svTravelTime + vtTravelTime

	if util.Ge(lv, (1+arf.param.getEpsilon())*arf.optTravelTime) {
		// dari lemma 4.3 ref[1], kita cukup cek stretch dari via path P_v dan cek sudah pass T-test atau tidak
		return da.NewEmptyViaVertex()
	}

	plv := arf.ars.calculatePlateau(v.GetVId(), v.GetOriginalVId(), v.GetEntryId(), v.GetExitId(), arf.sForwardId, arf.tBackwardId,
		arf.fpq, arf.bpq, arf.sCellNumber, lv, v.IsOverlay())

	T := arf.param.getAlpha() * arf.optTravelTime

	if util.Le(plv, T) {
		// T-test dengan T=\alpha*l(Opt) , v-w path adalah plateau dari P_v
		// plateau must > arf.ars.alpha * arf.optTravelTime
		// plateau = subpath dari Pv yang optimal (shortest path) dari first vertex ke last vertex dari subpath
		// atau every subpath P' of alternative route with l(P') <= T = \alpha* l(Opt) is optimal (shortest path). l(Opt) is the cost/travel time of the shortest path
		// didnt pass t-test
		return da.NewEmptyViaVertex()
	}

	if !v.IsOverlay() {
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetEntryId(), false),
			arf.fpq, arf.sForwardId, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetExitId(), true),
			arf.bpq, arf.tBackwardId, arf.sCellNumber, arf.t)

	} else {
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
			arf.fpq, arf.sForwardId, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
			arf.bpq, arf.tBackwardId, arf.sCellNumber, arf.t)
	}

	approxDistanceShare := arf.ars.calculateApproxDistanceShare(svPackedPath, vtPackedPath, arf.optPathSet, arf.shortcutPathSet,
		arf.sCellNumber, arf.tCellNumber, arf.motorwaySet)

	// cek approximate limited sharing
	if util.Ge(approxDistanceShare, arf.param.getGamma()*arf.optTravelTime) {
		return da.NewEmptyViaVertex()
	}

	v.SetCost(lv)
	v.SetPlateau(plv)
	v.SetApproxSharedDist(approxDistanceShare)

	return v
}

func (arf *AlternativeRouteFilter[W]) unpackViaPath(v da.ViaVertex) ([]da.Index, []da.Index) {
	var (
		svPackedPath, vtPackedPath []da.VertexEdgePair
	)

	if !v.IsOverlay() {
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetEntryId(), false),
			arf.fpq, arf.sForwardId, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetExitId(), true),
			arf.bpq, arf.tBackwardId, arf.sCellNumber, arf.t)

	} else {
		// forward
		svPackedPath = arf.ars.engine.RetrieveForwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
			arf.fpq, arf.sForwardId, arf.sCellNumber, arf.s)

		// backward
		vtPackedPath = arf.ars.engine.RetrieveBackwardPackedPath(da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
			arf.bpq, arf.tBackwardId, arf.sCellNumber, arf.t)
	}

	var (
		svEdgeIdPath, vtEdgeIdPath []da.Index
	)
	if !v.IsOverlay() {
		// forward
		svEdgeIdPath = arf.unpacker.unpackPathEdgesOnly(svPackedPath, arf.sCellNumber, arf.tCellNumber)
		// backward
		vtEdgeIdPath = arf.unpacker.unpackPathEdgesOnly(vtPackedPath, arf.sCellNumber, arf.tCellNumber)
	} else {
		// forward
		svPackedPath, vtPackedPath = arf.ars.makePackedViaPathOverlayEven(svPackedPath, vtPackedPath)
		svEdgeIdPath = arf.unpacker.unpackPathEdgesOnly(svPackedPath, arf.sCellNumber, arf.tCellNumber)
		// backward
		vtEdgeIdPath = arf.unpacker.unpackPathEdgesOnly(vtPackedPath, arf.sCellNumber, arf.tCellNumber)
	}

	return svEdgeIdPath, vtEdgeIdPath
}

func (arf *AlternativeRouteFilter[W]) computeAlternative(v da.ViaVertex) AlternativeRoute {

	/*
		let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any cell
		let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, number of cells in level 1 (excluded cell dari s dan cell dari t di level 1), and number of overlay vertices respectively.
		lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (compact graph CRP graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
		cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
		let q = number of shorcut edges in packedPath
		worst case  of unpackPath: O(q * (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))

		worst case of computeAlternative: O( p + q * (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))
	*/

	svEdgeIdPath, vtEdgeIdPath := arf.unpackViaPath(v)

	sigmav := arf.ars.calculateDistanceShare(svEdgeIdPath, vtEdgeIdPath, arf.optPathSet)
	// cek limited sharing
	if util.Ge(sigmav, arf.param.gamma*arf.optTravelTime) {

		return NewAEmptyAlternativeroute()
	}
	lv := v.GetCost()
	fv := 2*lv + sigmav - v.GetPlateau()

	altEdgeIdPath := removeConsecutiveDuplicates(append(svEdgeIdPath, vtEdgeIdPath...))

	return NewAlternativeRoute(fv, 0, lv, sigmav, v.GetOriginalVId(), EmptyCoords,
		altEdgeIdPath, v)
}

// filterByUniqueId removes duplicate via vertices by their VId in-place.
func (ars *AlternativeRouteSearch[W]) filterByUniqueId(vias []da.ViaVertex) []da.ViaVertex {
	uniqueViaSet := indexMapPool.Get().(map[da.Index]struct{})
	j := 0

	for i := 0; i < len(vias); i++ {
		v := vias[i]
		if _, ok := uniqueViaSet[v.GetVId()]; !ok {
			uniqueViaSet[v.GetVId()] = struct{}{}
			vias[j] = v
			j++
		}
	}

	clear(uniqueViaSet)
	indexMapPool.Put(uniqueViaSet)
	return vias[:j]
}

// calculateDistanceShare. calculate sharing amount (edge weights) dari unpacked path dari alternative route P_v dan optimal/shortest route Opt
func (ars *AlternativeRouteSearch[W]) calculateDistanceShare(svPath, vtPath []da.Index, optPathSet map[da.Index]struct{}) float64 {
	// O(M),  M=len(pvPath)
	distanceShare := 0.0

	for _, eId := range svPath {

		eHead := ars.engine.graph.GetHeadOfOutEdge(eId)
		if _, ok := optPathSet[eHead]; ok {
			// kualitas rute alternatif lebih bagus kalau length functionnya travel time
			distanceShare += ars.engine.GetWeightSeconds(eId, true)
		}
	}

	for _, eId := range vtPath {
		eHead := ars.engine.graph.GetHeadOfOutEdge(eId)
		if _, ok := optPathSet[eHead]; ok {
			// kualitas rute alternatif lebih bagus kalau length functionnya travel time
			distanceShare += ars.engine.GetWeightSeconds(eId, true)
		}
	}

	return distanceShare
}

func (ars *AlternativeRouteSearch[W]) buildEdgesPathSet(edgesPath []da.Index) map[da.Index]struct{} {
	edgesPathSet := indexMapPool.Get().(map[da.Index]struct{})
	for _, eId := range edgesPath {
		edgesPathSet[eId] = struct{}{}
	}
	return edgesPathSet
}

func (ars *AlternativeRouteSearch[W]) buildPathSet(optPath []da.Index) map[da.Index]struct{} {
	optPathSet := indexMapPool.Get().(map[da.Index]struct{})

	for _, eId := range optPath {
		eHead := ars.engine.graph.GetHeadOfOutEdge(eId)
		v := eHead
		optPathSet[v] = struct{}{}
	}
	return optPathSet
}

func (ars *AlternativeRouteSearch[W]) buildPathMotorwaySet(optPath []da.Index) (map[da.Index]struct{}, map[da.Index]struct{}) {
	optPathSet := ars.buildPathSet(optPath)
	motorwaySet := indexMapPool.Get().(map[da.Index]struct{})

	for _, eId := range optPath {
		eHead := ars.engine.graph.GetHeadOfOutEdge(eId)
		eRoadClass := ars.engine.graph.GetRoadClass(eId)

		if strings.Contains(eRoadClass, "motorway") {
			motorwaySet[eHead] = struct{}{}
		}

		v := eHead
		optPathSet[v] = struct{}{}
	}
	return optPathSet, motorwaySet
}

func putSetsToPool(sets []map[da.Index]struct{}) {
	for i, v := range sets {
		clear(sets[i])
		indexMapPool.Put(v)
	}
}

// calculateApproxDistanceShare. calculate sharing amount (edge weights) dari packed path dari alternative route P_v dan optimal/shortest route Opt
// sekaligus filter rute alternatives yang di jalan tol, kalau shortest path untuk jarak jauh pakai jalan tol. motorwaySet akan ada isinya kalau sp lewat jalan tol
func (ars *AlternativeRouteSearch[W]) calculateApproxDistanceShare(svPackedPath, vtPackedPath []da.VertexEdgePair, optPathSet map[da.Index]struct{}, shortcutPathSet map[uint64]uint8,
	sCellNumber, tCellNumber da.Pv, motorwaySet map[da.Index]struct{}) float64 {
	// O(M), M=len(svPackedPath) + len(vtPackedPath)
	distanceShare := 0.0

	vt := false
	packedPvPath := svPackedPath
	n := len(svPackedPath) + len(vtPackedPath)
	for i := 0; i < n; i++ {
		if i >= len(packedPvPath) && !vt {
			vt = true
			packedPvPath = vtPackedPath
		}

		j := i
		if vt {
			j -= len(svPackedPath)
		}

		pi := packedPvPath[j]
		if isBitOn(pi.GetEdge(), UNPACK_OVERLAY_OFFSET) {
			// shortcuts di packed path berpola: (entryVertex1, exitVertex1), (entryVertex2, exitVertex2)...
			// index last exitVertex di packed path < n-1
			if i+1 >= len(packedPvPath) && !vt {
				vt = true
				packedPvPath = vtPackedPath
			}

			k := i
			if vt {
				k -= len(svPackedPath)
			}

			nextPi := packedPvPath[k+1]
			entryVertexId := offBit(pi.GetEdge(), UNPACK_OVERLAY_OFFSET)
			exitVertexId := offBit(nextPi.GetEdge(), UNPACK_OVERLAY_OFFSET)
			entryVertex := ars.engine.overlayGraph.GetVertex(entryVertexId)
			exitVertex := ars.engine.overlayGraph.GetVertex(exitVertexId)

			enoriVid := entryVertex.GetOriginalVertex()
			exitoriVId := exitVertex.GetOriginalVertex()
			_, ok1 := optPathSet[enoriVid]
			_, ok2 := optPathSet[exitoriVId]

			entryCellNumber := entryVertex.GetCellNumber()
			queryLevel := ars.engine.overlayGraph.GetQueryLevel(sCellNumber, tCellNumber, entryCellNumber)

			shortcutWeightOffset := ars.engine.overlayGraph.GetShortcutWeightId(entryVertexId, exitVertexId, int(queryLevel))

			shortcutWeight := ars.engine.metrics.GetShortcutWeight(shortcutWeightOffset)

			bp := util.Bitpack(uint32(enoriVid), uint32(exitoriVId))
			shortcutInOpt := ok1 && ok2 && shortcutPathSet[bp] == uint8(queryLevel)

			if shortcutInOpt {
				distanceShare += ars.engine.GetCostFunction().WeightToSeconds(shortcutWeight)
			}
			_, okMtr1 := motorwaySet[enoriVid]
			_, okMtr2 := motorwaySet[enoriVid]

			shortcutInMotorway := okMtr1 && okMtr2 && shortcutPathSet[bp] == uint8(queryLevel)
			if shortcutInMotorway {
				distanceShare += ars.engine.GetCostFunction().WeightToSeconds(shortcutWeight) * MOTORWAY_PENALTY
			}
			i++
		} else {
			eHead := ars.engine.graph.GetHeadOfOutEdge(pi.GetEdge())
			eWeight := ars.engine.GetWeightSeconds(pi.GetEdge(), true)
			if _, ok := optPathSet[eHead]; ok {
				// kualitas rute alternatif lebih bagus kalau length functionnya travel time
				distanceShare += eWeight
			}
			if _, ok := motorwaySet[eHead]; ok {
				distanceShare += eWeight * MOTORWAY_PENALTY
			}
		}
	}

	return distanceShare
}

// calculatePlateau. calculate plateau pl(v)
// pada CRP query, kita build shortest path trees dari s dan ke t
// forward search membuat shortest path tree dari s ke every scanned vertices di forward search
// backward search membuat shortest path tree dari every vertices scanned di backward search ke t (karena backward search pakai reversed edges)
// plateaus adalah maximal paths yang muncul  di kedua shortest path trees
// plateau u-w dari st-path: path dari s ke u + path dari u ke w + path dari w ke t
// semua item (pasangan (entry/exit point, vertex) atau overlay vertex) path u-w dari u ke w tedapat pada kedua shortest path tree
// atau semua item dari path u-w sudah di scan oleh kedua search.
func (ars *AlternativeRouteSearch[W]) calculatePlateau(vId, oriVId, viaEntryId, viaExitId, sForwardId, tBackwardId da.Index,
	ps, pb *da.QueryHeap[da.CRPQueryKey, W], sCellNumber da.Pv, lv float64, overlay bool) float64 {

	var (
		u da.Index
	)
	uVId := oriVId
	if overlay {
		u = vId
	} else {
		u = viaEntryId
	}

	// shortest path tree from s to v: all scanned (already extracted using extractMin from pq) vertices in forward search
	// shortest path tree from v to t: all scanned (already extracted using extractMin from pq) vertices in backward search
	// note that karena backward search pakai reversed edges (dengan bobot setiap rev edge (v,u) sama dengan bobot edge (u,v)), kalau v scanned -> est sp cost dari t ke v di backward search equal to sp cost dari v ke t (kalau pakai original edges)
	/*
		Intuisi dari plateau (ref: https://dl.acm.org/doi/abs/10.1145/2444016.2444019):
		buat ngecek apakah alternative route P_v T-Localy Optimall (T-LO): every subpath P' of P_v with l(P') <= T adalah shortest path
		P_v is admissble alternative route iff P_v is T-locally optimal for T=\alpha* l(Opt)
		lemma 4.4 dari ref[1]:
		If P_v corresponds to a plateau u-w, P_v is dist(u,w)-LO
		proof:
		karena semua item antara u-w di scan forward search dan u-w discan backward search, pakai lemma every subpath of shortest path is shortest path (CLRS): subpath u-w is shortest path
		pakai lemma every subpath of shortest path is shortest path (CLRS) lagi: every subpath P' dari path u ke w, l(P') <= dist(u,w) is shortest path
		karena s-u scanned di forward search dan w-t scanned di backward search: subpath s-u is shortest path dan subpath w-t is shortest path
		pakai lemma every subpath of shortest path is shortest path lagi: every subpath dari shortest su-path dan wt-path adalah shortest path
		sehingga didapat every subpath P' of P_v with l(P') <= dist(u,w) adalah shortest path.
		perhatikan juga karena P_v bukan shortest path (rute alternative), terdapat item x yang belum di scan backward search (x-t is not shortest path) dan item y yang belum di scan forward search (s-y is not shortest path)
		x tepat berada sebelum u dan y tepat setelah w, x-y bukan plateau karena subpath x-y bukan shortest path, shg u-w adalah maximal paths that appear in both trees simultaneously
	*/

	// u = vEntryId/vId  dari via
	// vId = overlayId dari via kalau via nya overlay vertex
	// s-> .... -> u -vInEdge-> via (bisa aja sebuah overlay vertex) <-vExitEdge- w <- ..... <-t

	// task kita disini adalah find total length dari plateau u-w dari definisi platau diatas
	// so kita harus backtrack dari vEntryId/vOverlayId dari via vertex ke vertex awal dari plateau (atau vertex u dari definisi diatas)
	// bisa backtrack ke parent(u) kalau parent(u) scanned in backward search, atau in shortest path tree dari backward search
	// let n=number of edges in s-via-t path
	// worst case: O(n)
	for u != sForwardId {

		if ars.engine.isOverlay(u) {
			// parent_forward_search(u) is in plateau iff parent_forward_search(u) scanned in backward search

			oki := util.Lt(pb.GetPriority(ps.Get(u).GetParent().GetEdge()), util.Infinity[W]())
			if !oki {
				break
			}
			if scanned := pb.IsScanned(ps.Get(u).GetParent().GetEdge()); !scanned {
				// qParentOverlay -qShortcut-> qOverlay -vShortcut-> vOverlay
				// u == vOverlay, ps.Get(u).GetParent().GetEdge() == qOverlay
				// kalau qOverlay udah di scan di backward search kita bisa lanjut backtrack
				// else: vOverlay (atau u) adalah overlayVertex pertama dari plateau path

				break
			}
		} else if !ars.engine.isOverlay(u) && !ars.engine.isOverlay(ps.Get(u).GetParent().GetEdge()) {
			// u dan ps.Get(u).GetParent().GetEdge() bukan overlay vertex
			// qParent -qInEdge-> q -vInEdge-> v
			// u == vInEdge,  ps.Get(u).GetParent().GetEdge() == qInEdge

			// kalau u == entryId  dari edge, sedangkan di pb isinya exitId dari edge, shg u harus dijadiin exitId dari edgenya
			vEntryId := ars.engine.adjustForward(uVId, u)
			_, vExitId := ars.engine.graph.GetHeadOfInedgeWithOutEdge(vEntryId)

			q := ars.engine.graph.GetTailOfOutedge(vExitId)
			qInEdge := ps.Get(u).GetParent().GetEdge()
			qEntryId := ars.engine.adjustForward(q, qInEdge)
			_, qExitId := ars.engine.graph.GetHeadOfInedgeWithOutEdge(qEntryId)
			qParent := ars.engine.graph.GetTailOfOutedge(qExitId)

			offQExitId := ars.engine.offsetBackward(qParent, qExitId, ars.engine.graph.GetCellNumber(qParent), sCellNumber)
			oki := util.Lt(pb.GetPriority(offQExitId), util.Infinity[W]())

			if !oki {
				break
			}

			if scanned := pb.IsScanned(offQExitId); !scanned {
				// kalau qInEdge udah di scan di backward search kita bisa lanjut backtrack
				// else: vInEdge (atau u) adalah entryEdge pertama dari plateau path
				break
			}
		} else if !ars.engine.isOverlay(u) && ars.engine.isOverlay(ps.Get(u).GetParent().GetEdge()) {
			// q -vShortcut-> vOverlay -boundaryEdge/wInEdge-> w -> wOutEdge
			// u == wInEdge, ps.Get(u).GetParent().GetEdge() == vOverlay

			// cek apakah vOverlay already scanned di backward search, kalau iya bisa lanjut backtrack ke parent_forward_search(u)
			// di backward search: parent dari vOverlay adalah wOutEdge

			vOverlay := ps.Get(u).GetParent().GetEdge()

			notOki := !util.Lt(pb.GetPriority(vOverlay), util.Infinity[W]()) && !pb.IsScanned(vOverlay)

			if notOki {
				break
			}

		} else {
			// u overlay vertex tapi ps.Get(u).GetParent().GetEdge() bukan overlay vertex
			// q -vInEdge/qOutEdge-> vOverlay
			// u == vOverlay,  ps.Get(u).GetParent().GetEdge() == vInEdge

			// cek apakah vExitId scanned di backward search

			// vOverlay := u

			vEntryId := ps.Get(u).GetParent().GetFirstOverlayEntryExitId()

			_, qExitId := ars.engine.graph.GetHeadOfInedgeWithOutEdge(vEntryId)

			q := ars.engine.graph.GetTailOfOutedge(qExitId)

			offQExitId := ars.engine.offsetBackward(q, qExitId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			oki := util.Lt(pb.GetPriority(offQExitId), util.Infinity[W]())
			if !oki {
				break
			}
			if scanned := pb.IsScanned(offQExitId); !scanned {
				// kalau qOutEdge scanned di backward search, kita bisa lanjut backtrack
				// else: vOverlay (atau u) adalah overlayVertex pertama dari plateau path

				break
			}
		}

		uPar := ps.Get(u).GetParent()
		if !ps.IsScanned(uPar.GetEdge()) { // syarat parent(u) ada di shortest path tree forward search
			break
		}
		u = uPar.GetEdge()
		uVId = uPar.GetVertex()
	}

	firstPlateauTT := ps.GetPriority(u)

	if overlay {
		u = vId
	} else {
		u = viaExitId
	}
	uVId = oriVId

	for u != tBackwardId {
		if ars.engine.isOverlay(u) {

			oki := util.Lt(ps.GetPriority(pb.Get(u).GetParent().GetEdge()), util.Infinity[W]())
			if !oki {
				break
			}
			if scanned := ps.IsScanned(pb.Get(u).GetParent().GetEdge()); !scanned {
				// vOverlay -vShortcut-> qOverlay -qShortcut-> qParentOverlay
				// u == vOverlay, pb.Get(u).GetParent().GetEdge() == qOverlay
				// cek kalau qOverlay scanned in forward search, kalau yes, backtrack ke parent_backward_search(u) atau qOverlay
				break
			}
		} else if !ars.engine.isOverlay(u) && !ars.engine.isOverlay(pb.Get(u).GetParent().GetEdge()) {
			// u dan pb.Get(u).GetParent().GetEdge() bukan overlay vertex

			// v -vOutEdge/qInEdge-> q -qOutEdge/qParentInEdge-> qParent
			// u == vOutEdge,  pb.Get(u).GetParent().GetEdge() == qOutEdge, pb.Get(u).GetParent().vertex=q

			// cek qParentInEdge udah di scan di forward search, kalau yes, backtrack ke parent_backward_search(u) atau qOutEdge
			v := uVId
			vExitId := ars.engine.adjustBackward(v, u)
			_, qInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(vExitId)

			q := ars.engine.graph.GetHeadOfInedge(qInEdge)
			offQExitId := pb.Get(u).GetParent().GetEdge()
			qExitId := ars.engine.adjustBackward(q, offQExitId)

			qExitEdge := ars.engine.graph.GetOutEdge(qExitId)
			qParent := qExitEdge.GetHead()
			_, qParentInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(qExitId)
			qParentEntryId := qParentInEdge

			offQParentEntryId := ars.engine.offsetForward(qParent, qParentEntryId, ars.engine.graph.GetCellNumber(qParent), sCellNumber)
			oki := util.Lt(ps.GetPriority(offQParentEntryId), util.Infinity[W]())

			if !oki {
				break
			}
			if scanned := ps.IsScanned(offQParentEntryId); !scanned {
				break
			}
		} else if !ars.engine.isOverlay(u) && ars.engine.isOverlay(pb.Get(u).GetParent().GetEdge()) {
			// wInEdge -> w -boundaryEdge/vInEdge/wExitEdge-> vOverlay -qShortcut-> q
			// u == wExitEdge, pb.Get(u).GetParent().GetEdge() == vOverlay

			// cek apakah vOverlay scanned in forward search, kalau yes, backtrack ke parent_backward_search(u) atau vOverlay

			vOverlay := pb.Get(u).GetParent().GetEdge()

			notOki := !util.Lt(ps.GetPriority(vOverlay), util.Infinity[W]()) && !ps.IsScanned(vOverlay)

			if notOki {
				break
			}
		} else {
			// u overlay vertex tapi pb.Get(u).GetParent().GetEdge() bukan overlay vertex
			// vOverlay -vExitEdge/qInEdge-> q -qExitEdge/qParentInEdge-> qParent
			// u == vOverlay,  pb.Get(u).GetParent().GetEdge() == vExitId

			// cek qParentInEdge scanned in forward search, kalau yes, backtrack ke parent_backward_search(u) atau qExitEdge

			// vOverlay := u
			vExitId := pb.Get(u).GetParent().GetFirstOverlayEntryExitId()

			_, qEntryId := ars.engine.graph.GetTailOfOutedgeWithInEdge(vExitId)
			q := ars.engine.graph.GetHeadOfInedge(qEntryId)

			offQEntryId := ars.engine.offsetForward(q, qEntryId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			oki := util.Lt(ps.GetPriority(offQEntryId), util.Infinity[W]())
			if !oki {
				break
			}
			if scanned := ps.IsScanned(offQEntryId); !scanned {
				break
			}
		}

		uPar := pb.Get(u).GetParent()
		if !pb.IsScanned(uPar.GetEdge()) {
			break
		}
		u = uPar.GetEdge()
		uVId = uPar.GetVertex()
	}

	var lastPlateauTT float64
	if ars.engine.isOverlay(u) {
		// disini u == last overlay vertex Id dari plateau path
		lastPlateauTT = ars.engine.GetCostFunction().WeightToSeconds(pb.GetPriority(u))
	} else {
		// disini u == last out edge dari plateau path
		// ......-vInEdge-> uVId -uInEdge/u-> head

		lastPlateauTT = ars.engine.GetCostFunction().WeightToSeconds(pb.GetPriority(u))
	}

	// s-> ---- -> via -> ......-> u -> ..... -> t
	// pb[u] = dist(u,t)
	// lv - dist(u,t) = dist(s,u)
	lastPlateauTT = lv - lastPlateauTT

	firstPlateauTTSeconds := ars.engine.GetCostFunction().WeightToSeconds(firstPlateauTT)
	plateau := util.MaxFloat(
		lastPlateauTT-firstPlateauTTSeconds,
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

	param.setGamma(pickParam(ars.gammaMap, gcDist, ars.defaultGamma))

	param.setAlpha(pickParam(ars.alphaMap, gcDist, ars.defaultAlpha))

	param.setEpsilon(pickParam(ars.epsilonMap, gcDist, ars.defaultEpsilon))
	param.setUpperbound(pickParam(ars.upperBoundMap, gcDist, ars.defaultUpperbound))

	param.setMaxCandidatesToUnpack(pickParam(ars.maxCandidatesToUnpackMap, gcDist, ars.defaultMaxCandidatesToUnpack))

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

func (ars *AlternativeRouteSearch[W]) makePackedViaPathOverlayEven(svPackedPath, vtPackedPath []da.VertexEdgePair) ([]da.VertexEdgePair, []da.VertexEdgePair) {

	lSV := 0 // first overlayVertex
	// dari crp query, bisa aja via vertex nya di entryVertex sel sebelah
	// s-> u1 -> u2 -cutEdge-> v1 -shortcut-> v2 -cutEdge-> v3-> via <- ...... vertices scanned by backward search
	// karena sv overlayPath  nya [v1,v2,v3] ganjil dan syarat dari pathUnpacker: len(overlayPath) even, kita harus buat jadi even

	nSV := len(svPackedPath)
	for i := 0; i < nSV-1; i++ {
		if !isBitOn(svPackedPath[i].GetEdge(), UNPACK_OVERLAY_OFFSET) && isBitOn(svPackedPath[i+1].GetEdge(), UNPACK_OVERLAY_OFFSET) {
			lSV = i + 1
		}
	}

	if lSV != 0 && (nSV-lSV)%2 != 0 && isBitOn(svPackedPath[nSV-1].GetEdge(), UNPACK_OVERLAY_OFFSET) {
		svPackedPath = append(svPackedPath, vtPackedPath[0])
		vtPackedPath = vtPackedPath[1:]
	}

	return svPackedPath, vtPackedPath
}

// GetStretch compute stretch metrics yang dijelasin di section 5.4 paper: https://dl.acm.org/doi/epdf/10.1145/3567421
// mengukur stretch, ratio dari alternative path cost / fastest path cost...
func (ars *AlternativeRouteSearch[W]) GetStretch(candidates []AlternativeRoute, optimalTravelTime float64) float64 {

	if len(candidates) == 0 {
		return -1 // gak ke count karena gak ada alternative routes
	}

	stretch := 0.0

	for i := 0; i < len(candidates); i++ {
		stretch += candidates[i].GetDrivingTravelTime() / optimalTravelTime
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
		set[i] = make(map[da.Index]struct{}, len(alts[i].GetEdgeIdPath())*2)
	}

	diversity := 0.0
	for i, alt := range alts {
		// O(N^2 * M), N=len(alts), M=max{len(alts.edges[i])}, for each 0<=i<len(alts)
		altPath := alt.GetEdgeIdPath()

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
			minJaccardDist = util.MinFloat(minJaccardDist, jaccardDistance)
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

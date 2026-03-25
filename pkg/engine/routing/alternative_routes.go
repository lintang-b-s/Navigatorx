package routing

import (
	"math"
	"sort"
	"time"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type AlternativeRoute struct {
	path              []da.Coordinate
	edges             []da.OutEdge
	edgeIdPath        []da.Index
	objectiveValue    float64
	drivingDirections []da.DrivingDirection
	polylinePath      string

	travelTime  float64
	dist        float64
	distSharing float64
	viaNode     da.Index
	viaVertex   *da.ViaVertex
}

func (ar *AlternativeRoute) GetCoords() []da.Coordinate {
	return ar.path
}

func (ar *AlternativeRoute) GetPolylinePath() string {
	return ar.polylinePath
}

func (ar *AlternativeRoute) SetPolylinePath(pp string) {
	ar.polylinePath = pp
}

func (ar *AlternativeRoute) GetPath() []da.OutEdge {
	return ar.edges
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

func (ar *AlternativeRoute) GetViaVertex() *da.ViaVertex {
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

func (ar *AlternativeRoute) SetEdgePath(edges []da.OutEdge) {
	ar.edges = edges
}

func (ar *AlternativeRoute) SetCoordPath(path []da.Coordinate) {
	ar.path = path
}

func (ar *AlternativeRoute) SetDist(dist float64) {
	ar.dist = dist
}

func NewAlternativeRoute(objectiveValue, dist, travelTime, distSharing float64,
	viaNode da.Index, path []da.Coordinate, edges []da.OutEdge, edgeIdPath []da.Index,
	viaVertex *da.ViaVertex) *AlternativeRoute {
	return &AlternativeRoute{
		objectiveValue: objectiveValue,
		viaNode:        viaNode,
		path:           path,
		dist:           dist,
		edges:          edges,
		travelTime:     travelTime,
		viaVertex:      viaVertex,
		distSharing:    distSharing,
		edgeIdPath:     edgeIdPath,
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
	numOfInitialCands     int
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

func (ars *AlternativeRouteSearch) FindAlternativeRoutes(asId, atId da.Index, k int) []*AlternativeRoute {

	/*
		let n_p,m_p,and \hat{m_p} denote the maximum number of nodes, edges, and shortcuts within any partition
		let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
		time complexity of CRP query is: O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o)), in this implementation, priority queue (4-ary heap) contains at most all edges in lowest level cell that containing s or t and all overlay vertices in all cell other than cell that containing s or t
		decrease-key and insert at most O(k * \hat{m_p} + m_p) operations, for each shortcut (u,v) we immediately scan v and add neighbor of v (vertex w) to priority queue
		extract-min at most O(m_p+n_o) operations
	*/
	now := time.Now()

	crpQuery := NewCRPBidirectionalSearch(ars.engine, ars.upperBound)
	crpQuery.SetForAlternativeRoutes(true)

	defer func() {
		crpQuery.SetForAlternativeRoutes(false)
		crpQuery.Done()
	}()

	optTravelTime, optEdgeIdPath, found := crpQuery.ShortestPathSearch(asId, atId)
	if !found {
		return []*AlternativeRoute{}
	}
	ars.optTravelTime = optTravelTime

	fpq := crpQuery.GetForwardPQ()
	bpq := crpQuery.GetBackwardPQ()

	sCellNumber := crpQuery.GetSCellNumber()
	tCellNumber := crpQuery.GetTCellNumber()

	viaVertices := ars.retrieveViaVertices(fpq, bpq, sCellNumber, optTravelTime)
	ars.candidates = make([]*AlternativeRoute, 0, MAX_FILTERED_ALTERNATIVE_ROUTE_CANDIDATES)

	optPathSet := ars.buildPathSet(optEdgeIdPath)

	shortcutPathSet := crpQuery.getShortcutPathSet()

	ars.numOfInitialCands = len(viaVertices)

	filterCandidates := func(v *da.ViaVertex) *da.ViaVertex {
		var (
			svTravelTime, vtTravelTime float64

			svPackedPath, vtPackedPath []da.VertexEdgePair
		)

		/*
			let p = number of edges & shortcut edges in s-via-t path

			worst case of RetrieveForwardPackedPath+RetrieveForwardPackedPath: O(p)
			worst case  of calculatePlateau: O(p)
			worst case of  calculateApproxDistanceShare: O(p)

			worst case of filterCandidates: O(p)
		*/

		if !v.IsOverlay() {
			// via vertex is an overlay vertex
			svTravelTime = fpq.GetPriority(v.GetEntryId())
			vtTravelTime = bpq.GetPriority(v.GetExitId())
		} else {
			// via vertex is not an overlay vertex
			svTravelTime = fpq.GetPriority(v.GetVId())
			vtTravelTime = bpq.GetPriority(v.GetVId())
		}

		// stretch
		// lebih cepet eliminasi via vertices yang gak lolos local optimallity dan uniformly bounded stretch dulu, sebelum limited sharing
		// karena buat cek limited sharing kita harus unpack packed path dari via path nya dulu....
		lv := svTravelTime + vtTravelTime

		if lv >= (1+ars.epsilon)*optTravelTime {
			// dari lemma 4.3 ref[1], kita cukup cek stretch dari via path P_v dan cek sudah pass T-test atau tidak
			return nil
		}

		var plv float64
		plv = ars.calculatePlateau(v.GetVId(), v.GetOriginalVId(), v.GetEntryId(), v.GetExitId(), crpQuery.sForwardId, crpQuery.tBackwardId,
			fpq, bpq, sCellNumber, lv, v.IsOverlay())

		T := ars.alpha * optTravelTime

		if plv <= T {
			// T-test dengan T=\alpha*l(Opt) , v-w path adalah plateau dari P_v
			// plateau must > ars.alpha * optTravelTime
			// plateau = subpath dari Pv yang optimal (shortest path) dari first vertex ke last vertex dari subpath
			// atau every subpath P' of alternative route with l(P') <= T = \alpha* l(Opt) is optimal (shortest path). l(Opt) is the cost/travel time of the shortest path
			// didnt pass t-test
			return nil
		}

		// space allocations paling gede ada di retrievePackedPath & unpackPath, di commit 1b893621de80fa055e9def85e7d8224c2410b8d1: benchmark space allocation 2mb/op
		// ngaruh ke load tests

		// sebelum cek limited sharing pakai unpacked path, kita cek approximate limited sharing pakai packed path dulu
		// karena path unpacking lemot

		svPackedPath = ars.engine.packedPathPool.Get().([]da.VertexEdgePair)
		vtPackedPath = ars.engine.packedPathPool.Get().([]da.VertexEdgePair)

		svPackedPath = svPackedPath[:0] // reset length , tapi capacity tetep sama, ngaruh ke latency load test
		vtPackedPath = vtPackedPath[:0]

		if !v.IsOverlay() {
			// forward
			svPackedPath = ars.engine.RetrieveForwardPackedPath(svPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetEntryId(), false),
				fpq, crpQuery.sForwardId, crpQuery.sCellNumber)

			// backward
			vtPackedPath = ars.engine.RetrieveBackwardPackedPath(vtPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetExitId(), true),
				bpq, crpQuery.tBackwardId, crpQuery.sCellNumber)

		} else {
			// forward
			svPackedPath = ars.engine.RetrieveForwardPackedPath(svPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				fpq, crpQuery.sForwardId, crpQuery.sCellNumber)

			// backward
			vtPackedPath = ars.engine.RetrieveBackwardPackedPath(vtPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				bpq, crpQuery.tBackwardId, crpQuery.sCellNumber)
		}

		defer func() {
			ars.engine.packedPathPool.Put(svPackedPath)
			ars.engine.packedPathPool.Put(vtPackedPath)
		}()

		approxDistanceShare := ars.calculateApproxDistanceShare(svPackedPath, vtPackedPath, optPathSet, shortcutPathSet,
			sCellNumber, tCellNumber)

		// cek approximate limited sharing
		if approxDistanceShare >= ars.gamma*lv {
			return nil
		}

		v.SetCost(lv)
		v.SetPlateau(plv)
		v.SetApproxSharedDist(approxDistanceShare)

		return v
	}

	workers := concurrent.NewWorkerPool[*da.ViaVertex, *da.ViaVertex](ALTERNATIVE_ROUTES_WORKERS, len(viaVertices))

	for _, v := range viaVertices {
		workers.AddJob(v)
	}

	workers.Close()
	workers.Start(filterCandidates)
	workers.Wait()

	filteredCandidates := make([]*da.ViaVertex, 0, len(viaVertices))

	for filteredCand := range workers.CollectResults() {
		if filteredCand == nil {
			continue
		}
		filteredCandidates = append(filteredCandidates, filteredCand)
	}

	sort.Slice(filteredCandidates, func(i, j int) bool {
		return filteredCandidates[i].GetApproxObjectiveValue() <
			filteredCandidates[j].GetApproxObjectiveValue()
	})

	maxFilteredCandSize := util.MinInt(MAX_FILTERED_ALTERNATIVE_ROUTE_CANDIDATES, len(filteredCandidates))
	filteredCandidates = filteredCandidates[:maxFilteredCandSize]

	computeAlternatives := func(v *da.ViaVertex) *AlternativeRoute {

		/*
			let n_p,m_p,n_op,and \hat{m_p} denote the maximum number of nodes, edges, overlay vertices (include overlay vertices in its all direct subcells/subcells in level-1), and shortcuts within any partition
			let n,m,k,n_o denote the number vertices of the original graph,edges of the original graph, partitioning depth, and number of overlay vertices respectively.
			lowest level cell: O(m_p*log(m_p)), in unpackInLowestLevelCell(), priority queue (4-ary heap) contains at most m_p (turn-based graph), decrease-key and insert at most O(m_p) operations, extract-min at-most O(m_p) operations
			cell level > 1 : O((n_op + \hat{m_p})*log(n_op)), decrease-key and insert at most O(\hat{m_p}) operations, extract-min is at most O(n_op) operations
			let q = number of shorcut edges in packedPath
			worst case  of unpackPath: O(\sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))

			worst case of computeAlternatives: O( p + \sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p))
		*/

		var (
			svPackedPath, vtPackedPath []da.VertexEdgePair
			svEdgeIdPath, vtEdgeIdPath []da.Index
		)

		svPackedPath = ars.engine.packedPathPool.Get().([]da.VertexEdgePair)
		vtPackedPath = ars.engine.packedPathPool.Get().([]da.VertexEdgePair)

		svPackedPath = svPackedPath[:0]
		vtPackedPath = vtPackedPath[:0]

		if !v.IsOverlay() {
			// forward
			svPackedPath = ars.engine.RetrieveForwardPackedPath(svPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetEntryId(), false),
				fpq, crpQuery.sForwardId, crpQuery.sCellNumber)

			// backward
			vtPackedPath = ars.engine.RetrieveBackwardPackedPath(vtPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetExitId(), true),
				bpq, crpQuery.tBackwardId, crpQuery.sCellNumber)

		} else {
			// forward
			svPackedPath = ars.engine.RetrieveForwardPackedPath(svPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				fpq, crpQuery.sForwardId, crpQuery.sCellNumber)

			// backward
			vtPackedPath = ars.engine.RetrieveBackwardPackedPath(vtPackedPath, da.NewVertexEdgePair(v.GetOriginalVId(), v.GetVId(), false),
				bpq, crpQuery.tBackwardId, crpQuery.sCellNumber)
		}

		defer func() {
			ars.engine.packedPathPool.Put(svPackedPath)
			ars.engine.packedPathPool.Put(vtPackedPath)
		}()

		// // unpack packed path
		if !v.IsOverlay() {
			// forward

			unpacker := NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)
			svEdgeIdPath, _ = unpacker.unpackPath(svPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)

			// backward

			unpacker = NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)
			vtEdgeIdPath, _ = unpacker.unpackPath(vtPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)
		} else {
			// forward

			unpacker := NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)

			// backward

			unpacker = NewPathUnpackerALT(crpQuery.engine, crpQuery.engine.metrics, crpQuery.engine.puCache, true, ars.lm)

			svPackedPath, vtPackedPath = ars.makePackedViaPathOverlayEven(svPackedPath, vtPackedPath)

			svEdgeIdPath, _ = unpacker.unpackPath(svPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)
			vtEdgeIdPath, _ = unpacker.unpackPath(vtPackedPath, crpQuery.sCellNumber, crpQuery.tCellNumber)
		}

		sigmav := ars.calculateDistanceShare(svEdgeIdPath, vtEdgeIdPath, optPathSet)
		// cek limited sharing
		if sigmav >= ars.gamma*optTravelTime {
			return nil
		}
		lv := v.GetCost()
		fv := 2*lv + sigmav - v.GetPlateau()

		return NewAlternativeRoute(fv, 0, lv, sigmav, v.GetOriginalVId(), []da.Coordinate{}, []da.OutEdge{},
			append(svEdgeIdPath, vtEdgeIdPath...), v)
	}

	// worst case computeAlternatives for all via vertices: O(c * ( p + \sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p)))
	// c = min(MAX_FILTERED_ALTERNATIVE_ROUTE_CANDIDATES, len(filteredCandidates))

	workersAlt := concurrent.NewWorkerPool[*da.ViaVertex, *AlternativeRoute](ALTERNATIVE_ROUTES_WORKERS, len(filteredCandidates))

	for _, v := range filteredCandidates {
		workersAlt.AddJob(v)
	}

	workersAlt.Close()
	workersAlt.Start(computeAlternatives)
	workersAlt.Wait()

	for alternativeRoute := range workersAlt.CollectResults() {
		if alternativeRoute == nil {
			continue
		}
		ars.candidates = append(ars.candidates, alternativeRoute)
	}

	sort.Slice(ars.candidates, func(i, j int) bool {
		return ars.candidates[i].GetObjectiveValue() < ars.candidates[j].GetObjectiveValue()
	})

	maxAltSize := util.MinInt(k, len(ars.candidates))
	ars.candidates = ars.candidates[:maxAltSize]
	for i := 0; i < maxAltSize; i++ {

		finalEdgePath, finalPath, totalDistance := ars.engine.GetEdgePath(ars.candidates[i].GetEdgeIdPath())
		ars.candidates[i].SetCoordPath(finalPath)
		ars.candidates[i].SetEdgePath(finalEdgePath)
		ars.candidates[i].SetDist(totalDistance)

	}

	res := removeSimiliarAlternatives(ars.candidates)

	// worst case of FindAlternativeRoutes: worst case crp query + worst case computeAlternatives for all via vertices
	// O((n_o + m_p + k * \hat{m_p}) * log (m_p+n_o) + c * ( p + \sum_{i=1}^{q} (n_op + \hat{m_p})*log (n_op) + m_p*log(m_p)))

	dur := time.Since(now).Milliseconds()
	ars.runtime = dur

	return res
}

/*
retrieveViaVertices. retrieve via vertices. vertices that scanned by forward and backward search.
*/
func (ars *AlternativeRouteSearch) retrieveViaVertices(fpq, bpq *da.QueryHeap[da.CRPQueryKey],
	sCellNumber da.Pv, optTravelTime float64) []*da.ViaVertex {
	viaVertices := make([]*da.ViaVertex, 0, 16)
	set := make(map[da.Index]struct{}, 100)

	fpq.ForLabelledItems(func(itemId da.Index, vInfo da.VertexInfo) {
		if ars.engine.isOverlay(itemId) {
			// via vertex is an overlay vertex
			overlayVId := itemId

			scannedByBackwardSearch := bpq.IsScanned(overlayVId)
			scannedByForwardSearch := fpq.IsScanned(overlayVId)
			scannedByBothSearch := scannedByBackwardSearch && scannedByForwardSearch
			admissibleCost := fpq.GetPriority(overlayVId)+bpq.GetPriority(overlayVId) < (1+ars.epsilon)*optTravelTime
			if scannedByBothSearch && admissibleCost {

				v := ars.engine.offOverlay(overlayVId)
				vVertex := ars.engine.overlayGraph.GetVertex(v)
				originalVId := vVertex.GetOriginalVertex()

				_, added := set[overlayVId]
				if added {
					return
				}
				set[overlayVId] = struct{}{}

				viaVertices = append(viaVertices, da.NewViaVertex(overlayVId, 0, 0, originalVId, true))
			}

		} else {
			// via vertex is not an overlay vertex

			vEntryId := itemId
			vId, outArc := ars.engine.graph.GetHeadOfInedgeWithOutEdge(vEntryId)
			// check wether we already Labelled an exit point of vId
			exitOffset := ars.engine.graph.GetExitOffset(vId)

			exitOffset = ars.engine.offsetBackward(vId, exitOffset, ars.engine.graph.GetCellNumber(vId), sCellNumber)

			vExitId := exitOffset

			// traverse outEdges of v
			ars.engine.graph.ForOutEdgesOf(vId, da.Index(outArc.GetEntryPoint()), func(e2 *da.OutEdge,
				exitPoint da.Index, turnType2 pkg.TurnType) {

				_, added := set[vId]
				if added {
					return
				}

				set[vId] = struct{}{}

				scannedByBackwardSearch := bpq.IsScanned(vExitId)
				scannedByForwardSearch := fpq.IsScanned(vEntryId)
				scannedByBothSearch := scannedByBackwardSearch && scannedByForwardSearch
				admissibleCost := fpq.GetPriority(vEntryId)+bpq.GetPriority(vExitId) < (1+ars.epsilon)*optTravelTime
				if scannedByBothSearch && admissibleCost {
					viaVertices = append(viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
				}
				vExitId++
			})
		}

	})

	bpq.ForLabelledItems(func(itemId da.Index, vInfo da.VertexInfo) {
		if ars.engine.isOverlay(itemId) {
			// via vertex is an overlay vertex
			overlayVId := itemId

			scannedByBackwardSearch := bpq.IsScanned(overlayVId)
			scannedByForwardSearch := fpq.IsScanned(overlayVId)
			scannedByBothSearch := scannedByBackwardSearch && scannedByForwardSearch
			admissibleCost := fpq.GetPriority(overlayVId)+bpq.GetPriority(overlayVId) < (1+ars.epsilon)*optTravelTime
			if scannedByBothSearch && admissibleCost {

				v := ars.engine.offOverlay(overlayVId)
				vVertex := ars.engine.overlayGraph.GetVertex(v)
				originalVId := vVertex.GetOriginalVertex()

				_, added := set[overlayVId]
				if added {
					return
				}

				set[overlayVId] = struct{}{}

				viaVertices = append(viaVertices, da.NewViaVertex(overlayVId, 0, 0, originalVId, true))
			}
		} else {
			// via vertex is not an overlay vertex
			vExitId := itemId
			vId, inArc := ars.engine.graph.GetTailOfOutedgeWithInEdge(vExitId)

			// check wether we already Labelled an entry point
			entryOffset := ars.engine.graph.GetEntryOffset(vId)

			entryOffset = ars.engine.offsetForward(vId, entryOffset, ars.engine.graph.GetCellNumber(vId), sCellNumber)

			vEntryId := entryOffset

			ars.engine.graph.ForInEdgesOf(vId, da.Index(inArc.GetExitPoint()), func(inArc2 *da.InEdge,
				entryPoint2 da.Index, turnType2 pkg.TurnType) {
				_, added := set[vId]
				if added {
					return
				}

				set[vId] = struct{}{}

				scannedByForwardSearch := fpq.IsScanned(vEntryId)
				scannedByBackwardSearch := bpq.IsScanned(vExitId)
				scannedByBothSearch := scannedByBackwardSearch && scannedByForwardSearch
				admissibleCost := fpq.GetPriority(vEntryId)+bpq.GetPriority(vExitId) < (1+ars.epsilon)*optTravelTime
				if scannedByBothSearch && admissibleCost {

					viaVertices = append(viaVertices, da.NewViaVertex(vId, vEntryId, vExitId, vId, false))
				}
				vEntryId++
			})
		}
	})

	return viaVertices
}

// calculateDistanceShare. calculate sharing amount (edge weights) dari unpacked path dari alternative route P_v dan optimal/shortest route Opt
func (ars *AlternativeRouteSearch) calculateDistanceShare(svPath, vtPath []da.Index, optPathSet map[da.Index]struct{}) float64 {
	// O(M),  M=len(pvPath)
	distanceShare := 0.0

	for _, eId := range svPath {
		e := ars.engine.graph.GetOutEdge(eId)
		if _, ok := optPathSet[e.GetHead()]; ok {
			// kualitas rute alternatif lebih bagus kalau length functionnya travel time
			distanceShare += ars.engine.metrics.GetWeight(e)
		}
	}

	for _, eId := range vtPath {
		e := ars.engine.graph.GetOutEdge(eId)
		if _, ok := optPathSet[e.GetHead()]; ok {
			// kualitas rute alternatif lebih bagus kalau length functionnya travel time
			distanceShare += ars.engine.metrics.GetWeight(e)
		}
	}

	return distanceShare
}

func (ars *AlternativeRouteSearch) buildPathSet(optPath []da.Index) map[da.Index]struct{} {

	optPathSet := make(map[da.Index]struct{}, len(optPath))
	for _, eId := range optPath {
		e := ars.engine.graph.GetOutEdge(eId)
		v := e.GetHead()
		optPathSet[v] = struct{}{}

	}

	return optPathSet
}

// calculateApproxDistanceShare. calculate sharing amount (edge weights) dari packed path dari alternative route P_v dan optimal/shortest route Opt
func (ars *AlternativeRouteSearch) calculateApproxDistanceShare(svPackedPath, vtPackedPath []da.VertexEdgePair, optPathSet map[da.Index]struct{}, shortcutPathSet map[uint64]uint8,
	sCellNumber, tCellNumber da.Pv) float64 {
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

			bp := bitpack(enoriVid, exitoriVId)
			shortcutInOpt := ok1 && ok2 && shortcutPathSet[bp] == uint8(queryLevel)

			if shortcutInOpt {
				distanceShare += shortcutWeight
			}
			i++
		} else {
			e := ars.engine.graph.GetOutEdge(pi.GetEdge())
			if _, ok := optPathSet[e.GetHead()]; ok {
				// kualitas rute alternatif lebih bagus kalau length functionnya travel time
				distanceShare += ars.engine.metrics.GetWeight(e)
			}
		}
	}

	return distanceShare
}

func (ars *AlternativeRouteSearch) viaVertexInOptimalPath(optPathSet map[da.Index]struct{}, via *da.ViaVertex) bool {
	if via.IsOverlay() {

		if _, ok := optPathSet[via.GetOriginalVId()]; ok {
			return true
		}
		return false
	}

	if _, ok := optPathSet[via.GetVId()]; ok {
		return true
	}
	return false
}

// calculatePlateau. calculate plateau pl(v)
// pada CRP query, kita build shortest path trees dari s dan ke t
// forward search membuat shortest path tree dari s ke every scanned vertices di forward search
// backward search membuat shortest path tree dari every vertices scanned di backward search ke t (karena backward search pakai reversed edges)
// plateaus adalah maximal paths yang muncul  di kedua shortest path trees
// plateau u-w dari st-path: path dari s ke u + path dari u ke w + path dari w ke t
// semua item (pasangan (entry/exit point, vertex) atau overlay vertex) path u-w dari u ke w tedapat pada kedua shortest path tree
// atau semua item dari path u-w sudah di scan oleh kedua search.
func (ars *AlternativeRouteSearch) calculatePlateau(vId, oriVId, viaEntryId, viaExitId, sForwardId, tBackwardId da.Index,
	ps, pb *da.QueryHeap[da.CRPQueryKey], sCellNumber da.Pv, lv float64, overlay bool) float64 {

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

			oki := util.Lt(pb.GetPriority(ps.Get(u).GetParent().GetEdge()), pkg.INF_WEIGHT)
			if !oki {
				break
			}
			if b := pb.Get(ps.Get(u).GetParent().GetEdge()); !b.IsScanned() {
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
			_, vOutEdge := ars.engine.graph.GetHeadOfInedgeWithOutEdge(vEntryId)
			vExitId := vOutEdge.GetEdgeId()

			q := ars.engine.graph.GetTailOfOutedge(vExitId)
			qInEdge := ps.Get(u).GetParent().GetEdge()
			qEntryId := ars.engine.adjustForward(q, qInEdge)
			_, qOutEdge := ars.engine.graph.GetHeadOfInedgeWithOutEdge(qEntryId)
			qExitId := qOutEdge.GetEdgeId()
			qParent := ars.engine.graph.GetTailOfOutedge(qExitId)

			offQExitId := ars.engine.offsetBackward(qParent, qExitId, ars.engine.graph.GetCellNumber(qParent), sCellNumber)
			oki := util.Lt(pb.GetPriority(offQExitId), pkg.INF_WEIGHT)

			q = ars.engine.graph.GetTailOfOutedge(vExitId)

			if !oki {
				break
			}

			if b := pb.Get(offQExitId); !b.IsScanned() {
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

			notOki := !util.Lt(pb.GetPriority(vOverlay), pkg.INF_WEIGHT) && !pb.IsScanned(vOverlay)

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

			_, vOutEdge := ars.engine.graph.GetHeadOfInedgeWithOutEdge(vEntryId)
			qExitId := vOutEdge.GetEdgeId()
			q := ars.engine.graph.GetTailOfOutedge(qExitId)

			offQExitId := ars.engine.offsetBackward(q, qExitId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			oki := util.Lt(pb.GetPriority(offQExitId), pkg.INF_WEIGHT)
			if !oki {
				break
			}
			if b := pb.Get(offQExitId); !b.IsScanned() {
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

			oki := util.Lt(ps.GetPriority(pb.Get(u).GetParent().GetEdge()), pkg.INF_WEIGHT)
			if !oki {
				break
			}
			if f := ps.Get(pb.Get(u).GetParent().GetEdge()); !f.IsScanned() {
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

			q := ars.engine.graph.GetHeadOfInedge(qInEdge.GetEdgeId())
			offQExitId := pb.Get(u).GetParent().GetEdge()
			qExitId := ars.engine.adjustBackward(q, offQExitId)

			qParent := ars.engine.graph.GetOutEdge(qExitId).GetHead()
			_, qParentInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(qExitId)
			qParentEntryId := qParentInEdge.GetEdgeId()

			offQParentEntryId := ars.engine.offsetForward(qParent, qParentEntryId, ars.engine.graph.GetCellNumber(qParent), sCellNumber)
			oki := util.Lt(ps.GetPriority(offQParentEntryId), pkg.INF_WEIGHT)

			if !oki {
				break
			}
			if f := ps.Get(offQParentEntryId); !f.IsScanned() {
				break
			}
		} else if !ars.engine.isOverlay(u) && ars.engine.isOverlay(pb.Get(u).GetParent().GetEdge()) {
			// wInEdge -> w -boundaryEdge/vInEdge/wExitEdge-> vOverlay -qShortcut-> q
			// u == wExitEdge, pb.Get(u).GetParent().GetEdge() == vOverlay

			// cek apakah vOverlay scanned in forward search, kalau yes, backtrack ke parent_backward_search(u) atau vOverlay

			vOverlay := pb.Get(u).GetParent().GetEdge()

			notOki := !util.Lt(ps.GetPriority(vOverlay), pkg.INF_WEIGHT) && !ps.IsScanned(vOverlay)

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

			_, qInEdge := ars.engine.graph.GetTailOfOutedgeWithInEdge(vExitId)
			qEntryId := qInEdge.GetEdgeId()
			q := ars.engine.graph.GetHeadOfInedge(qEntryId)

			offQEntryId := ars.engine.offsetForward(q, qEntryId, ars.engine.graph.GetCellNumber(q), sCellNumber)
			oki := util.Lt(ps.GetPriority(offQEntryId), pkg.INF_WEIGHT)
			if !oki {
				break
			}
			if f := ps.Get(offQEntryId); !f.IsScanned() {
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
		lastPlateauTT = pb.GetPriority(u)
	} else {
		// disini u == last out edge dari plateau path
		// ......-vInEdge-> uVId -uInEdge/u-> head

		lastPlateauTT = pb.GetPriority(u)
	}

	// s-> ---- -> via -> ......-> u -> ..... -> t
	// pb[u] = dist(u,t)
	// lv - dist(u,t) = dist(s,u)
	lastPlateauTT = lv - lastPlateauTT

	plateau := util.MaxFloat(lastPlateauTT-firstPlateauTT, 0)

	return plateau
}

func removeSimiliarAlternatives(alts []*AlternativeRoute) []*AlternativeRoute {
	set := make([]map[da.Index]struct{}, 0, len(alts))

	res := make([]*AlternativeRoute, 0, len(alts))
	for _, alt := range alts {
		// O(N^2 * M), N=len(alts), M=max{len(alts.edges[i])}, for each 0<=i<len(alts)
		altPath := alt.GetPath()

		addToRes := true
		for j := 0; j < len(set); j++ {
			// check similiarity with other previous alternative routes
			intersection := 0.0

			setJ := set[j]
			for _, e := range altPath {
				if _, exists := setJ[e.GetEdgeId()]; exists {
					intersection++
				}
			}

			unionSize := float64(len(setJ) + len(altPath) - int(intersection)) // |A \cup B| = |A|+|B|-|A \cap B|

			if unionSize == 0 {
				continue
			}

			jaccardSimiliarity := (intersection / unionSize) * 100
			if jaccardSimiliarity >= pkg.ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD {
				// add alt to result if similiarity with other alternative route < pkg.ALTERNATIVE_ROUTE_SIMILIARITY_THRESHOLD
				addToRes = false
				break
			}
		}

		if addToRes {
			res = append(res, alt)

			altSet := make(map[da.Index]struct{}, len(altPath))
			for _, e := range altPath {
				altSet[e.GetEdgeId()] = struct{}{}
			}
			set = append(set, altSet)
		}
	}
	return res
}

func (ars *AlternativeRouteSearch) Reset() {
	ars.candidates = make([]*AlternativeRoute, 0)
}

func (ars *AlternativeRouteSearch) makePackedViaPathOverlayEven(svPackedPath, vtPackedPath []da.VertexEdgePair) ([]da.VertexEdgePair, []da.VertexEdgePair) {

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

func (ars *AlternativeRouteSearch) GetStretch() float64 {

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

	if len(ars.candidates) == 0 {
		return -1 // gak ke itung karena gak ada alternative routes
	}

	alts := ars.candidates
	set := make([]map[da.Index]struct{}, len(alts))
	for i := 0; i < len(alts); i++ {
		set[i] = make(map[da.Index]struct{}, len(alts[i].GetPath())*2)
	}

	diversity := 0.0
	for i, alt := range alts {
		// O(N^2 * M), N=len(alts), M=max{len(alts.edges[i])}, for each 0<=i<len(alts)
		altPath := alt.GetPath()

		minJaccardDist := math.MaxFloat64
		for j := 0; j < i; j++ {
			// check similiarity with other previous alternative routes
			intersection := 0.0

			setJ := set[j]
			for _, e := range altPath {
				if _, exists := setJ[e.GetEdgeId()]; exists {
					intersection++
				}
			}

			unionSize := float64(len(setJ) + len(altPath) - int(intersection)) // |A \cup B| = |A|+|B|-|A \cap B|
			if unionSize == 0 {
				continue
			}
			jaccardSimiliarity := (intersection / unionSize)

			jaccardDistance := 1 - jaccardSimiliarity
			minJaccardDist = math.Min(minJaccardDist, jaccardDistance)
		}

		for _, e := range altPath {
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

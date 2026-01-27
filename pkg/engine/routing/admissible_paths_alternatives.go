package routing

import (
	"sort"
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
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
	lock                  *sync.RWMutex
	delta                 float64 //  Evolution and Evaluation of the Penalty Method for Alternative Graphs, by Kobitzsch et al. (2013)
	td                    bool    // time dependent routing
}

func NewAlternativeRouteSearch(engine *CRPRoutingEngine, upperBound, gamma, alpha, epsilon, delta float64,
	td bool,
) *AlternativeRouteSearch {
	return &AlternativeRouteSearch{
		engine:     engine,
		candidates: make([]*AlternativeRoute, 0),
		upperBound: upperBound,
		gamma:      gamma,
		alpha:      alpha,
		epsilon:    epsilon,
		lock:       &sync.RWMutex{},
		td:         td,
		delta:      delta,
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


*/

func (ars *AlternativeRouteSearch) FindAlternativeRoutes(asId, atId datastructure.Index, k int) []*AlternativeRoute {

	unicrpQuery := NewCRPUniDijkstra(ars.engine)
	optTravelTime, _, _, optEdgePath, found := unicrpQuery.ShortestPathSearch(asId, atId)

	if !found {
		return []*AlternativeRoute{}
	}

	crpQuery := NewCRPBidirectionalSearch(ars.engine, ars.upperBound)

	optTravelTime, _, _, _, _ = crpQuery.ShortestPathSearch(asId, atId)

	viaVertices := make([]datastructure.ViaVertex, len(crpQuery.GetViaVertices()))
	copy(viaVertices, crpQuery.GetViaVertices())

	ars.candidates = make([]*AlternativeRoute, 0, len(viaVertices))
	viaVertices = removeDuplicates(viaVertices)

	fInfo := crpQuery.GetForwardInfo()
	bInfo := crpQuery.GetBackwardInfo()

	for i := len(viaVertices) - 1; i >= 0; i-- {
		v := viaVertices[i]
		if fInfo[v.GetEntryId()].GetTravelTime()+bInfo[v.GetExitId()].GetTravelTime() >= (1+ars.epsilon)*optTravelTime {
			viaVertices = append(viaVertices[:i], viaVertices[i+1:]...)
		}
	}

	computeAlternatives := func(v datastructure.ViaVertex) any {
		var (
			svTravelTime, vtTravelTime float64
			svDist, vtDist             float64
			svCoords, vtCoords         []datastructure.Coordinate
			svEdgePath, vtEdgePath     []datastructure.OutEdge
			svFound, vtFound           bool
			crpQuerysv, crpQueryvt     Router
		)
		if !ars.td {
			crpQuerysv = NewCRPBidirectionalSearch(ars.engine, UPPERBOUND_SHORTEST_PATH)
			crpQueryvt = NewCRPBidirectionalSearch(ars.engine, UPPERBOUND_SHORTEST_PATH)
		} else {
			crpQuerysv = NewTDCRPUnidirectionalSearch(ars.engine)
			crpQueryvt = NewTDCRPUnidirectionalSearch(ars.engine)
		}

		wg := sync.WaitGroup{}
		wg.Add(2)
		go func() {
			viaEntryId := adjustForwardOffBit(v.GetEntryId())
			svTravelTime, svDist, svCoords, svEdgePath, svFound = crpQuerysv.ShortestPathSearch(asId, viaEntryId)
			wg.Done()
		}()

		go func() {
			viaExitId := adjustBackwardOffbit(v.GetExitId())
			vtTravelTime, vtDist, vtCoords, vtEdgePath, vtFound = crpQueryvt.ShortestPathSearch(viaExitId, atId)
			wg.Done()
		}()

		wg.Wait()
		if !svFound || !vtFound {
			return nil
		}

		pvEdgePath := append(svEdgePath, vtEdgePath...)
		sigmav := ars.calculateDistanceShare(optEdgePath, pvEdgePath)
		if sigmav >= ars.gamma*optTravelTime {
			return nil
		}

		lv := svTravelTime + vtTravelTime
		lvExcludeOpt := lv - sigmav
		lOptExcludePv := optTravelTime - sigmav
		if lvExcludeOpt >= (1+ars.epsilon)*lOptExcludePv {
			return nil
		}

		plv := ars.calculatePlateau(v.GetVId(), v.GetEntryId(), v.GetExitId(), asId, atId,
			fInfo, bInfo)

		T := ars.alpha * lvExcludeOpt

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

	workers := concurrent.NewWorkerPool[datastructure.ViaVertex, any](4, len(viaVertices))

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
	// O(N), N=max{len(pvPath), len(optPath)}
	distanceShare := 0.0

	optPathSet := make(map[datastructure.Index]struct{})
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

func (ars *AlternativeRouteSearch) calculatePlateau(vId, vEntryId, vExitId, asId, atId datastructure.Index,
	ps, pb map[datastructure.Index]VertexInfo) float64 {
	plateau := 0.0
	u := vEntryId
	_, ok := ps[u]
	if !ok {
		u = vId
	}
	for u != datastructure.INVALID_EDGE_ID {
		if _, oki := pb[u]; oki {
			costf := ps[u].GetTravelTime() - ps[ps[u].parent.edge].GetTravelTime()
			plateau += costf
		}
		u = ps[u].parent.edge
	}

	u = vExitId
	_, ok = pb[u]
	if !ok {
		u = vId
	}
	for u != datastructure.INVALID_EDGE_ID {
		if _, oki := ps[u]; oki {
			costf := pb[u].GetTravelTime() - pb[pb[u].parent.edge].GetTravelTime()
			plateau += costf
		}
		u = pb[u].parent.edge
	}
	return plateau
}

func (ars *AlternativeRouteSearch) PathAdmissible(asId, atId datastructure.Index, v datastructure.ViaVertex) bool {
	crpQuery := NewCRPBidirectionalSearch(ars.engine, ars.upperBound)

	optTravelTime, _, _, optEdgePath, found := crpQuery.ShortestPathSearch(asId, atId)
	if !found {
		return false
	}

	containV := false
	for _, via := range crpQuery.GetViaVertices() {
		if via.GetOriginalVId() == v.GetOriginalVId() && via.GetEntryId() == v.GetEntryId() &&
			via.GetExitId() == v.GetExitId() {
			containV = true
			break
		}
	}

	if !containV {
		return false
	}

	crpQuerysv := NewCRPBidirectionalSearch(ars.engine, UPPERBOUND_SHORTEST_PATH)
	svTravelTime, _, _, svEdgePath, svFound := crpQuerysv.ShortestPathSearch(asId, v.GetEntryId())
	crpQueryvt := NewCRPBidirectionalSearch(ars.engine, UPPERBOUND_SHORTEST_PATH)
	vtTravelTime, _, _, vtEdgePath, vtFound := crpQueryvt.ShortestPathSearch(v.GetExitId(), atId)
	if !svFound || !vtFound {
		return false
	}

	pvEdgePath := append(svEdgePath, vtEdgePath...)
	sigmav := ars.calculateDistanceShare(optEdgePath, pvEdgePath)
	if sigmav >= ars.gamma*optTravelTime {
		return false
	}

	lv := svTravelTime + vtTravelTime
	lvExcludeOpt := lv - sigmav
	lOptExcludePv := optTravelTime - sigmav
	if lvExcludeOpt >= (1+ars.epsilon)*lOptExcludePv {
		return false
	}

	plv := ars.calculatePlateau(v.GetVId(), v.GetEntryId(), v.GetExitId(), asId, atId,
		crpQuery.GetForwardInfo(), crpQuery.GetBackwardInfo())

	T := ars.alpha * lvExcludeOpt

	if plv <= T {
		// didnt pass t-test
		return false
	}

	return true
}

func removeSimiliarAlternatives(alts []*AlternativeRoute) []*AlternativeRoute {
	set := make([]map[datastructure.Index]struct{}, len(alts))
	for i := 0; i < len(alts); i++ {
		set[i] = make(map[datastructure.Index]struct{})
	}
	res := make([]*AlternativeRoute, 0, len(alts))
	for i, alt := range alts {
		// O(N^2 * M), N=len(alts), M=max{len(alts[i])}, for each 0<=i<len(alts)

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

			for _, e := range alt.GetPath() {
				// make alternative route path set
				set[i][e.GetEdgeId()] = struct{}{}
			}
		}

	}
	return res
}

func removeDuplicates(viaVertices []datastructure.ViaVertex) []datastructure.ViaVertex {
	set := make(map[datastructure.Index]struct{})
	res := make([]datastructure.ViaVertex, 0, len(viaVertices))

	for _, v := range viaVertices {
		if _, ok := set[v.GetVId()]; !ok {
			set[v.GetVId()] = struct{}{}
			res = append(res, v)
		}
	}
	return res
}

func (ars *AlternativeRouteSearch) Reset() {
	ars.candidates = make([]*AlternativeRoute, 0)
}

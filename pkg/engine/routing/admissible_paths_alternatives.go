package routing

import (
	"sync"

	"github.com/lintang-b-s/Navigatorx/pkg"
	"github.com/lintang-b-s/Navigatorx/pkg/concurrent"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

type viaVertex struct {
	v           datastructure.Index
	originalVId datastructure.Index
	entryId     datastructure.Index
	exitId      datastructure.Index
}

func newViaVertex(v, entryId, exitId, originalVId datastructure.Index) viaVertex {
	return viaVertex{v: v, entryId: entryId, exitId: exitId, originalVId: originalVId}
}

func (v viaVertex) getEntryId() datastructure.Index {
	return v.entryId
}

func (v viaVertex) getExitId() datastructure.Index {
	return v.exitId
}

func (v viaVertex) getVId() datastructure.Index {
	return v.v
}

func (v viaVertex) getOriginalVId() datastructure.Index {
	return v.originalVId
}

type AlternativeRoute struct {
	path              []datastructure.Coordinate
	edges             []datastructure.OutEdge
	objectiveValue    float64
	drivingDirections []datastructure.DrivingDirection
	polylinePath      string

	travelTime float64
	dist       float64
	viaNode    datastructure.Index
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

func (ar *AlternativeRoute) GetDist() float64 {
	return ar.dist
}

func (ar *AlternativeRoute) GetViaNode() datastructure.Index {
	return ar.viaNode
}

func NewAlternativeRoute(objectiveValue, dist, travelTime float64,
	viaNode datastructure.Index, path []datastructure.Coordinate, edges []datastructure.OutEdge) *AlternativeRoute {
	return &AlternativeRoute{
		objectiveValue: objectiveValue,
		viaNode:        viaNode,
		path:           path,
		dist:           dist,
		edges:          edges,
		travelTime:     travelTime,
	}
}

type AlternativeRouteSearch struct {
	engine                *CRPRoutingEngine
	candidates            []*AlternativeRoute
	upperBound            float64
	gamma, alpha, epsilon float64
	lock                  *sync.RWMutex
}

func NewAlternativeRouteSearch(engine *CRPRoutingEngine, upperBound, gamma, alpha, epsilon float64,
) *AlternativeRouteSearch {
	return &AlternativeRouteSearch{
		engine:     engine,
		candidates: make([]*AlternativeRoute, 0),
		upperBound: upperBound,
		gamma:      gamma,
		alpha:      alpha,
		epsilon:    epsilon,
		lock:       &sync.RWMutex{},
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

	crpQuery := NewCRPBidirectionalSearch(ars.engine, ars.upperBound)

	optTravelTime, _, _, optEdgePath, found := crpQuery.ShortestPathSearch(asId, atId)

	if !found {
		return []*AlternativeRoute{}
	}

	viaVertices := make([]viaVertex, len(crpQuery.GetViaVertices()))
	copy(viaVertices, crpQuery.GetViaVertices())

	ars.candidates = make([]*AlternativeRoute, 0, len(viaVertices))
	viaVertices = removeDuplicates(viaVertices)

	computeAlternatives := func(v viaVertex) any {
		crpQuerysv := NewCRPBidirectionalSearch(ars.engine, UPPERBOUND_SHORTEST_PATH)
		svTravelTime, svDist, svCoords, svEdgePath, svFound := crpQuerysv.ShortestPathSearch(asId, v.getEntryId())
		crpQueryvt := NewCRPBidirectionalSearch(ars.engine, UPPERBOUND_SHORTEST_PATH)
		vtTravelTime, vtDist, vtCoords, vtEdgePath, vtFound := crpQueryvt.ShortestPathSearch(v.getExitId(), atId)
		if !svFound || !vtFound {
			return nil
		}
		lv := svTravelTime + vtTravelTime
		if lv >= (1+ars.epsilon)*optTravelTime {
			return nil
		}
		plv := ars.calculatePlateau(v.getVId(), v.getEntryId(), v.getExitId(), asId, atId,
			crpQuery.GetForwardInfo(), crpQuery.GetBackwardInfo())

		if plv <= ars.alpha*optTravelTime {
			return nil
		}

		pvEdgePath := append(svEdgePath, vtEdgePath...)
		sigmav := ars.calculateDistanceShare(optEdgePath, pvEdgePath)
		if sigmav >= ars.gamma*optTravelTime {
			return nil
		}

		fv := 2*lv + sigmav - plv

		pvCoords := append(svCoords, vtCoords...)

		ars.lock.Lock()
		ars.candidates =
			append(ars.candidates, NewAlternativeRoute(fv, svDist+vtDist, lv, v.getOriginalVId(), pvCoords, pvEdgePath))
		ars.lock.Unlock()
		return nil
	}

	workers := concurrent.NewWorkerPool[viaVertex, any](3, len(viaVertices))

	for _, v := range viaVertices {
		workers.AddJob(v)
	}

	workers.Close()
	workers.Start(computeAlternatives)
	workers.Wait()

	util.QuickSortGIdx(ars.candidates, func(j, pivotIdx int) bool {
		return ars.candidates[j].objectiveValue < ars.candidates[pivotIdx].objectiveValue
	})

	ars.candidates = removeSimiliarAlternatives(ars.candidates)

	res := make([]*AlternativeRoute, 0, k)

	for i := 0; i < util.MinInt(k, len(ars.candidates)); i++ {
		res = append(res, ars.candidates[i])
	}

	return res
}

func (ars *AlternativeRouteSearch) calculateDistanceShare(optPath, pvPath []datastructure.OutEdge) float64 {
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
	for u != asId {
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
	for u != atId {
		if _, oki := ps[u]; oki {
			costf := pb[u].GetTravelTime() - pb[pb[u].parent.edge].GetTravelTime()
			plateau += costf
		}
		u = pb[u].parent.edge
	}
	return plateau
}

func removeSimiliarAlternatives(alts []*AlternativeRoute) []*AlternativeRoute {
	set := make([]map[datastructure.Index]struct{}, len(alts))
	for i := 0; i < len(alts); i++ {
		set[i] = make(map[datastructure.Index]struct{})
	}
	res := make([]*AlternativeRoute, 0, len(alts))
	for i, alt := range alts {

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

func removeDuplicates(viaVertices []viaVertex) []viaVertex {
	set := make(map[datastructure.Index]struct{})
	res := make([]viaVertex, 0, len(viaVertices))
	for _, v := range viaVertices {
		if _, ok := set[v.getVId()]; !ok {
			set[v.getVId()] = struct{}{}
			res = append(res, v)
		}
	}
	return res
}

package usecases

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/guidance"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type RoutingService struct {
	log                          *zap.Logger
	engine                       RoutingEngine
	spatialIndex                 SpatialIndex
	searchRadius                 float64
	clockwise, lefthandDriving   bool
	gamma, alpha, epsilon, delta float64
	upperBoundAlternativeSearch  float64
	lm                           *landmark.Landmark
}

func NewRoutingService(log *zap.Logger, engine RoutingEngine, spatialindex SpatialIndex,
	searchRadius float64, clockwise, lefthandDriving bool,
	gamma, alpha, epsilon, upperBoundAlternativeSearch, delta float64, lm *landmark.Landmark,
) *RoutingService {
	return &RoutingService{
		log:                         log,
		engine:                      engine,
		spatialIndex:                spatialindex,
		searchRadius:                searchRadius,
		clockwise:                   clockwise,
		lefthandDriving:             lefthandDriving,
		gamma:                       gamma,
		alpha:                       alpha,
		epsilon:                     epsilon,
		upperBoundAlternativeSearch: upperBoundAlternativeSearch,
		delta:                       delta,
		lm:                          lm,
	}
}

func (rs *RoutingService) ShortestPath(origLat, origLon, dstLat, dstLon float64) (float64, float64, string, []datastructure.DrivingDirection, bool, error) {
	as, at, err := rs.SnapOrigDestToNearbyEdges(origLat, origLon, dstLat, dstLon)
	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if err != nil {
		return 0, 0, "", []datastructure.DrivingDirection{}, false, err
	}
	var (
		travelTime, dist float64
		pathCoords       []datastructure.Coordinate
		edgePath         []datastructure.OutEdge
		found            bool
	)

	crpQuery := routing.NewCRPALTBidirectionalSearch(rs.engine.(*routing.CRPRoutingEngine), 1.0, rs.lm)
	travelTime, dist, pathCoords, edgePath, found = crpQuery.ShortestPathSearch(as, at)

	if !found {
		errmsg := fmt.Sprintf("no route found from %f,%f to %f,%f", origLat, origLon, dstLat, dstLon)
		return 0, 0, "", []datastructure.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	pathPolyline := geo.PoylineFromCoords(datastructure.NewGeoCoordinates(pathCoords))
	directionBuilder := guidance.NewDirectionBuilder(rs.engine.GetGraph(), rs.clockwise, rs.lefthandDriving)
	drivingDirection := directionBuilder.GetDrivingDirections(edgePath)
	return travelTime, dist, pathPolyline, drivingDirection, true, nil
}

func (rs *RoutingService) AlternativeRouteSearch(origLat, origLon, dstLat, dstLon float64, k int) ([]*routing.AlternativeRoute, bool, error) {
	as, at, err := rs.SnapOrigDestToNearbyEdges(origLat, origLon, dstLat, dstLon)
	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if err != nil {
		return []*routing.AlternativeRoute{}, false, err
	}

	altSearch := routing.NewAlternativeRouteSearch(rs.engine.(*routing.CRPRoutingEngine), rs.upperBoundAlternativeSearch, rs.gamma, rs.alpha,
		rs.epsilon, rs.delta)
	alternatives := altSearch.FindAlternativeRoutes(as, at, k)
	if len(alternatives) == 0 {
		return []*routing.AlternativeRoute{}, false, nil
	}

	for _, alt := range alternatives {
		pathPolyline := geo.PoylineFromCoords(datastructure.NewGeoCoordinates(alt.GetCoords()))
		alt.SetPolylinePath(pathPolyline)
		directionBuilder := guidance.NewDirectionBuilder(rs.engine.GetGraph(), rs.clockwise, rs.lefthandDriving)
		drivingDirection := directionBuilder.GetDrivingDirections(alt.GetPath())
		alt.SetDrivingDirections(drivingDirection)
	}
	return alternatives, true, nil
}

func (rs *RoutingService) GetEngine() RoutingEngine {
	return rs.engine
}

package usecases

import (
	"fmt"

	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/guidance"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type RoutingService struct {
	log                        *zap.Logger
	engine                     RoutingEngine
	spatialIndex               SpatialIndex
	searchRadius               float64
	clockwise, lefthandDriving bool
}

func NewRoutingService(log *zap.Logger, engine RoutingEngine, spatialindex SpatialIndex,
	searchRadius float64, clockwise, lefthandDriving bool) *RoutingService {
	return &RoutingService{
		log:             log,
		engine:          engine,
		spatialIndex:    spatialindex,
		searchRadius:    searchRadius,
		clockwise:       clockwise,
		lefthandDriving: lefthandDriving,
	}
}

func (rs *RoutingService) ShortestPath(origLat, origLon, dstLat, dstLon float64) (float64, float64, string, []datastructure.DrivingDirection, bool, error) {
	as, at, err := rs.snapOrigDestToNearbyEdges(origLat, origLon, dstLat, dstLon)
	// as = exit/outEdge offset of origin
	// at = entry/inEdge offset of destination
	if err != nil {
		return 0, 0, "", []datastructure.DrivingDirection{}, false, err
	}

	crpQuery := routing.NewCRPBidirectionalSearch(rs.engine.(*routing.CRPRoutingEngine))
	travelTime, dist, pathCoords, edgePath, found := crpQuery.ShortestPathSearch(as, at)
	if !found {
		return 0, 0, "", []datastructure.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput, fmt.Sprintf("no path found from %f,%f to %f,%f", origLat, origLon, dstLat, dstLon))
	}

	pathPolyline := geo.PoylineFromCoords(datastructure.NewGeoCoordinates(pathCoords))
	directionBuilder := guidance.NewDirectionBuilder(rs.engine.GetGraph(), rs.clockwise, rs.lefthandDriving)
	drivingDirection := directionBuilder.GetDrivingDirections(edgePath)
	return travelTime, dist, pathPolyline, drivingDirection, true, nil
}

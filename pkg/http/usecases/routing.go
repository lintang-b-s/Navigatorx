package usecases

import (
	"fmt"
	"sync"

	lru "github.com/hashicorp/golang-lru/v2"
	"github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/geo"
	"github.com/lintang-b-s/Navigatorx/pkg/guidance"
	"github.com/lintang-b-s/Navigatorx/pkg/landmark"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type RoutingService struct {
	log                        *zap.Logger
	engine                     RoutingEngine
	spatialIndex               SpatialIndex
	altRouting                 AlternativeRouteAlgorithm
	searchRadius               float64
	clockwise, lefthandDriving bool
	lm                         *landmark.Landmark

	// sync pools
	drivingInstructionPool *sync.Pool
	coordinatesPool        *sync.Pool
	drivingEdgeIdsPool     *sync.Pool
	drivingDirectionPool   *sync.Pool
	directionBuilderPool   *sync.Pool
	turnSignCache          *lru.Cache[uint64, int]
}

func NewRoutingService(log *zap.Logger, engine RoutingEngine, spatialindex SpatialIndex, altRouting AlternativeRouteAlgorithm,
	searchRadius float64, clockwise, lefthandDriving bool,
	lm *landmark.Landmark,
) *RoutingService {
	rs := &RoutingService{
		log:             log,
		engine:          engine,
		spatialIndex:    spatialindex,
		searchRadius:    searchRadius,
		clockwise:       clockwise,
		lefthandDriving: lefthandDriving,
		altRouting:      altRouting,

		lm: lm,
	}

	rs.drivingInstructionPool = &sync.Pool{
		New: func() any {
			drinvingInstructions := make([]*datastructure.Instruction, 0, 128)
			return drinvingInstructions
		},
	}

	rs.coordinatesPool = &sync.Pool{
		New: func() any {
			legCoordinates := make([]datastructure.Coordinate, 0, 128)
			return legCoordinates
		},
	}

	rs.drivingEdgeIdsPool = &sync.Pool{
		New: func() any {
			legEdgeIds := make([]datastructure.Index, 0, 128)
			return legEdgeIds
		},
	}

	rs.drivingDirectionPool = &sync.Pool{
		New: func() any {
			drivingDirections := make([]datastructure.DrivingDirection, 0, 128)
			return drivingDirections
		},
	}

	rs.turnSignCache, _ = lru.New[uint64, int](1 << 21)

	rs.directionBuilderPool = &sync.Pool{
		New: func() any {
			return guidance.NewDirectionBuilder(
				rs.engine.GetGraph(), rs.clockwise, rs.lefthandDriving,
				rs.drivingInstructionPool,
				rs.coordinatesPool,
				rs.drivingEdgeIdsPool,
				rs.turnSignCache,
			)
		},
	}

	return rs
}

func (rs *RoutingService) ShortestPath(origLat, origLon, dstLat, dstLon float64) (float64, float64, string, []datastructure.DrivingDirection, bool, error) {
	as, at, err := rs.SnapOrigDestToNearbyEdges(origLat, origLon, dstLat, dstLon)
	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if err != nil {
		errmsg := fmt.Sprintf("no nearby intersections found from %f,%f to %f,%f", origLat, origLon, dstLat, dstLon)
		return 0, 0, "", []datastructure.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
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
	directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)
	// todo: update kode driving direction buat improve performance

	drivingDirection := rs.drivingDirectionPool.Get().([]datastructure.DrivingDirection)
	drivingDirection = drivingDirection[:0]
	directionBuilder.Reset()

	drivingDirection = directionBuilder.GetDrivingDirections(edgePath, drivingDirection)

	rs.directionBuilderPool.Put(directionBuilder)
	rs.engine.DoneQuery(edgePath, pathCoords)
	return travelTime, dist, pathPolyline, drivingDirection, true, nil
}

func (rs *RoutingService) AlternativeRouteSearch(origLat, origLon, dstLat, dstLon float64, k int) ([]*routing.AlternativeRoute, bool, error) {
	as, at, err := rs.SnapOrigDestToNearbyEdges(origLat, origLon, dstLat, dstLon)
	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if err != nil {
		errmsg := fmt.Sprintf("no nearby intersections found from %f,%f to %f,%f", origLat, origLon, dstLat, dstLon)
		return []*routing.AlternativeRoute{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	alternatives, _, _ := rs.altRouting.FindAlternativeRoutes(as, at, k)
	if len(alternatives) == 0 {
		return []*routing.AlternativeRoute{}, false, nil
	}

	for _, alt := range alternatives {
		pathPolyline := geo.PoylineFromCoords(datastructure.NewGeoCoordinates(alt.GetCoords()))
		alt.SetPolylinePath(pathPolyline)
		directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)
		directionBuilder.Reset()
		// todo: update kode driving direction buat improve performance

		drivingDirection := rs.drivingDirectionPool.Get().([]datastructure.DrivingDirection)
		drivingDirection = drivingDirection[:0]
		drivingDirection = directionBuilder.GetDrivingDirections(alt.GetPath(), drivingDirection)
		alt.SetDrivingDirections(drivingDirection)

		rs.directionBuilderPool.Put(directionBuilder)
		rs.engine.DoneQuery(alt.GetPath(), alt.GetCoords())
	}
	return alternatives, true, nil
}

func (rs *RoutingService) GetEngine() RoutingEngine {
	return rs.engine
}

func (rs *RoutingService) DoneDrivingDirection(drivingDirection []datastructure.DrivingDirection) {
	rs.drivingDirectionPool.Put(drivingDirection)
}

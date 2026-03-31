package usecases

import (
	"fmt"
	"sync"

	"github.com/cockroachdb/errors"
	"github.com/dgraph-io/ristretto/v2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/guidance"
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

	// sync pools
	drivingInstructionPool *sync.Pool
	coordinatesPool        *sync.Pool
	drivingEdgeIdsPool     *sync.Pool
	drivingDirectionPool   *sync.Pool
	directionBuilderPool   *sync.Pool
	turnSignCache          *ristretto.Cache[uint64, int]
}

func NewRoutingService(log *zap.Logger, engine RoutingEngine, spatialindex SpatialIndex, altRouting AlternativeRouteAlgorithm,
	searchRadius float64, clockwise, lefthandDriving bool,
) (*RoutingService, error) {
	rs := &RoutingService{
		log:             log,
		engine:          engine,
		spatialIndex:    spatialindex,
		searchRadius:    searchRadius,
		clockwise:       clockwise,
		lefthandDriving: lefthandDriving,
		altRouting:      altRouting,
	}

	rs.drivingInstructionPool = &sync.Pool{
		New: func() any {
			drinvingInstructions := make([]*da.Instruction, 0, 128)
			return drinvingInstructions
		},
	}

	rs.coordinatesPool = &sync.Pool{
		New: func() any {
			legCoordinates := make([]da.Coordinate, 0, 128)
			return legCoordinates
		},
	}

	rs.drivingEdgeIdsPool = &sync.Pool{
		New: func() any {
			legEdgeIds := make([]da.Index, 0, 128)
			return legEdgeIds
		},
	}

	rs.drivingDirectionPool = &sync.Pool{
		New: func() any {
			drivingDirections := make([]da.DrivingDirection, 0, 128)
			return drivingDirections
		},
	}

	// rs.turnSignCache, _ = lru.New[uint64, int](1 << 21)

	var err error
	const keyValByteSize = 12
	maxCost := int64(1) << 26
	rs.turnSignCache, err = ristretto.NewCache(&ristretto.Config[uint64, int]{
		NumCounters: (maxCost / keyValByteSize) * 10, // number of keys to track frequency of .
		MaxCost:     maxCost,                         // maximum cost of cache .
		BufferItems: 64,                              // number of keys per Get buffer.
	})
	if err != nil {
		return nil, errors.Wrapf(err, "initializeRoutingEngine: failed to create new ristretto cache with capacity: %v", maxCost)
	}

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

	return rs, nil
}

func (rs *RoutingService) ShortestPath(qOrigLat, qOrigLon, qDstLat, qDstLon float64) (float64, float64, string, []da.DrivingDirection, bool, error) {

	as, at, snappedOrig, snappedDst := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon)

	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if rs.notFoundOriginDestinationWithinRadius(as, at) {
		errmsg := fmt.Sprintf("no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	var (
		travelTime, dist float64
		pathCoords       []da.Coordinate
		edgePath         []da.Index
		found            bool
	)

	crpQuery := routing.NewCRPALTBidirectionalSearch(rs.engine.(*routing.CRPRoutingEngine), 1.0)
	travelTime, dist, pathCoords, edgePath, found = crpQuery.ShortestPathSearch(as, at)

	if !found {
		errmsg := fmt.Sprintf("no route found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	pathCoords = append([]da.Coordinate{snappedOrig}, pathCoords...)
	pathCoords = append(pathCoords, snappedDst)

	pathPolyline := da.PoylineFromCoords(pathCoords)
	rs.engine.DoneQuery(pathCoords)
	directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)
	// todo: update kode driving direction buat improve performance

	drivingDirection := rs.drivingDirectionPool.Get().([]da.DrivingDirection)

	drivingDirection = directionBuilder.GetDrivingDirections(edgePath, drivingDirection)

	directionBuilder.Reset()
	rs.directionBuilderPool.Put(directionBuilder)
	return travelTime, dist, pathPolyline, drivingDirection, true, nil
}

func (rs *RoutingService) AlternativeRouteSearch(qOrigLat, qOrigLon, qDstLat, qDstLon float64, k int) ([]*routing.AlternativeRoute, bool, error) {
	as, at, snappedOrig, snappedDst := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon)

	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if rs.notFoundOriginDestinationWithinRadius(as, at) {
		errmsg := fmt.Sprintf("no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return []*routing.AlternativeRoute{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	alternatives, _, _ := rs.altRouting.FindAlternativeRoutes(as, at, k)
	if len(alternatives) == 0 {
		return []*routing.AlternativeRoute{}, false, nil
	}

	for _, alt := range alternatives {
		altPathCoords := alt.GetCoords()
		altPathCoords = append([]da.Coordinate{snappedOrig}, altPathCoords...)
		altPathCoords = append(altPathCoords, snappedDst)

		pathPolyline := da.PoylineFromCoords(altPathCoords)
		rs.engine.DoneQuery(altPathCoords)
		alt.SetPolylinePath(pathPolyline)
		directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)
		// todo: update kode driving direction buat improve performance

		drivingDirection := rs.drivingDirectionPool.Get().([]da.DrivingDirection)
		drivingDirection = directionBuilder.GetDrivingDirections(alt.GetPath(), drivingDirection)
		alt.SetDrivingDirections(drivingDirection)
		directionBuilder.Reset()
		rs.directionBuilderPool.Put(directionBuilder)
	}
	return alternatives, true, nil
}

func (rs *RoutingService) GetEngine() RoutingEngine {
	return rs.engine
}

func (rs *RoutingService) DoneDrivingDirection(drivingDirection []da.DrivingDirection) {
	drivingDirection = drivingDirection[:0] // reset length, tapi capacity tetep sama
	rs.drivingDirectionPool.Put(drivingDirection)
}

func (rs *RoutingService) Close() {
	rs.turnSignCache.Close()
}

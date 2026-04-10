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

	directionBuilderPool *sync.Pool
	turnSignCache        *ristretto.Cache[uint64, []byte]
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

	var err error
	const keyValByteSize = 12
	const maxCost = int64(1) << 25
	const numCounters = (maxCost / keyValByteSize) * 3
	rs.turnSignCache, err = ristretto.NewCache(&ristretto.Config[uint64, []byte]{
		NumCounters: numCounters, // number of keys to track frequency of .
		MaxCost:     maxCost,     // maximum cost of cache .
		BufferItems: 64,          // number of keys per Get buffer.
	})
	if err != nil {
		return nil, errors.Wrapf(err, "initializeRoutingEngine: failed to create new ristretto cache with capacity: %v", maxCost)
	}

	rs.directionBuilderPool = &sync.Pool{
		New: func() any {
			return guidance.NewDirectionBuilder(
				rs.engine.GetGraph(), rs.clockwise, rs.lefthandDriving,
				rs.turnSignCache,
			)
		},
	}

	return rs, nil
}

func (rs *RoutingService) ShortestPath(qOrigLat, qOrigLon, qDstLat, qDstLon float64) (float64, float64, string, []da.DrivingDirection, bool, error) {

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon)

	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		errmsg := fmt.Sprintf("no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	var (
		travelTime, dist float64
		pathCoords       *da.Coordinates
		edgePath         []da.Index
		found            bool
	)

	crpQuery := routing.NewCRPALTBidirectionalSearch(rs.engine.(*routing.CRPRoutingEngine), 1.0)
	travelTime, dist, pathCoords, edgePath, found = crpQuery.ShortestPathSearch(sp, tp)

	if !found {
		errmsg := fmt.Sprintf("no route found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	travelTime = rs.AppendPhantomNodesToPath(pathCoords, sp, tp, travelTime)

	pathPolyline := da.GooglePoylineFromCoords(*pathCoords)
	directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)

	drivingDirection := directionBuilder.GetDrivingDirections(edgePath)

	directionBuilder.Reset()
	rs.directionBuilderPool.Put(directionBuilder)
	return travelTime, dist, pathPolyline, drivingDirection, true, nil
}

func (rs *RoutingService) AlternativeRouteSearch(qOrigLat, qOrigLon, qDstLat, qDstLon float64, k int) ([]routing.AlternativeRoute, bool, error) {
	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon)

	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		errmsg := fmt.Sprintf("no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return []routing.AlternativeRoute{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	alternatives, _, _ := rs.altRouting.FindAlternativeRoutes(sp, tp, k)
	if len(alternatives) == 0 {
		return []routing.AlternativeRoute{}, false, nil
	}

	for i, alt := range alternatives {
		altPathCoords := alt.GetCoords()

		newTravelTime := rs.AppendPhantomNodesToPath(altPathCoords, sp, tp, alternatives[i].GetDrivingTravelTime())
		alternatives[i].SetDrivingTravelTime(newTravelTime)

		pathPolyline := da.GooglePoylineFromCoords(*altPathCoords)
		alternatives[i].SetPolylinePath(pathPolyline)
		directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)

		drivingDirection := directionBuilder.GetDrivingDirections(alt.GetEdgeIdPath())
		alternatives[i].SetDrivingDirections(drivingDirection)

		directionBuilder.Reset()
		rs.directionBuilderPool.Put(directionBuilder)
	}
	return alternatives, true, nil
}

func (rs *RoutingService) GetEngine() RoutingEngine {
	return rs.engine
}

func (rs *RoutingService) Close() {
	rs.turnSignCache.Close()
}

func (rs *RoutingService) AppendPhantomNodesToPath(path *da.Coordinates, sp, tp da.PhantomNode, travelTime float64) float64 {
	if !rs.engine.IsDummyOutEdge(sp.GetOutEdgeId()) {
		path.Prepend(append([]da.Coordinate{sp.GetSnappedCoord()}, sp.GetForwardGeometry()...))
		travelTime += sp.GetForwardTravelTime()
	}
	if !rs.engine.IsDummyInEdge(tp.GetInEdgeId()) {
		path.Append(append(tp.GetReverseGeometry(), tp.GetSnappedCoord()))
		travelTime += tp.GetReverseTravelTime()
	}
	return travelTime
}

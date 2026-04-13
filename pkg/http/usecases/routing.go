package usecases

import (
	"context"
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
	graph                      *da.Graph
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
		graph:           engine.GetGraph(),
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
			return guidance.NewDirectionBuilder(rs.engine,
				rs.engine.GetGraph(), rs.clockwise, rs.lefthandDriving,
				rs.turnSignCache,
			)
		},
	}

	return rs, nil
}

func (rs *RoutingService) ShortestPath(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64) (float64, float64, string, []da.DrivingDirection, bool, error) {

	if util.IsTimeout(ctx) { // https://engineering.grab.com/context-deadlines-and-how-to-set-them
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ctx.Err(), util.ErrContextDeadline, fmt.Sprintf("request timeout"))
	}

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon)

	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		errmsg := fmt.Sprintf("no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	if util.IsTimeout(ctx) {
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ctx.Err(), util.ErrContextDeadline, fmt.Sprintf("request timeout"))
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

	if util.IsTimeout(ctx) {
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ctx.Err(), util.ErrContextDeadline, fmt.Sprintf("request timeout"))
	}

	travelTime, dist = rs.AppendPhantomNodesToPath(pathCoords, sp, tp, travelTime, dist)

	pathPolyline := da.GooglePoylineFromCoords(*pathCoords)
	directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)

	drivingDirection := directionBuilder.GetDrivingDirections(edgePath, sp, tp)

	directionBuilder.Reset()
	rs.directionBuilderPool.Put(directionBuilder)
	return travelTime, dist, pathPolyline, drivingDirection, true, nil
}

func (rs *RoutingService) AlternativeRouteSearch(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64, k int) ([]routing.AlternativeRoute, bool, error) {
	if util.IsTimeout(ctx) {
		return []routing.AlternativeRoute{}, false, util.WrapErrorf(ctx.Err(), util.ErrContextDeadline, fmt.Sprintf("request timeout"))
	}

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon)

	// as = exit/outEdge index of origin
	// at = entry/inEdge index of destination
	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		errmsg := fmt.Sprintf("no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
		return []routing.AlternativeRoute{}, false, util.WrapErrorf(ERRPATHNOTFOND, util.ErrBadParamInput,
			errmsg)
	}

	if util.IsTimeout(ctx) {
		return []routing.AlternativeRoute{}, false, util.WrapErrorf(ctx.Err(), util.ErrContextDeadline, fmt.Sprintf("request timeout"))
	}

	alternatives, _, _ := rs.altRouting.FindAlternativeRoutes(sp, tp, k)
	if len(alternatives) == 0 {
		return []routing.AlternativeRoute{}, false, nil
	}

	if util.IsTimeout(ctx) {
		return []routing.AlternativeRoute{}, false, util.WrapErrorf(ctx.Err(), util.ErrContextDeadline, fmt.Sprintf("request timeout"))
	}

	for i, alt := range alternatives {
		altPathCoords := alt.GetCoords()

		newTravelTime, dist := rs.AppendPhantomNodesToPath(altPathCoords, sp, tp, alt.GetDrivingTravelTime(), alt.GetDist())
		alternatives[i].SetDrivingTravelTime(newTravelTime)
		alternatives[i].SetDist(dist)

		pathPolyline := da.GooglePoylineFromCoords(*altPathCoords)
		alternatives[i].SetPolylinePath(pathPolyline)
		directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)

		drivingDirection := directionBuilder.GetDrivingDirections(alt.GetEdgeIdPath(), sp, tp)
		alternatives[i].SetDrivingDirections(drivingDirection)

		directionBuilder.Reset()
		rs.directionBuilderPool.Put(directionBuilder)
	}
	return alternatives, true, nil
}

func (rs *RoutingService) Close() {
	rs.turnSignCache.Close()
}

func (rs *RoutingService) AppendPhantomNodesToPath(path *da.Coordinates, sp, tp da.PhantomNode, travelTime float64, dist float64) (float64, float64) {
	if !rs.engine.IsDummyOutEdge(sp.GetOutEdgeId()) {
		path.Prepend(append([]da.Coordinate{sp.GetSnappedCoord()}, sp.GetForwardGeometry()...))
		travelTime += sp.GetForwardTravelTime()
		dist += sp.GetForwardDistance()
	}
	if !rs.engine.IsDummyInEdge(tp.GetInEdgeId()) {
		path.Append(append(tp.GetReverseGeometry(), tp.GetSnappedCoord()))
		travelTime += tp.GetReverseTravelTime()
		dist += sp.GetReverseDistance()
	}
	return travelTime, dist
}

func (rs *RoutingService) GetRoutingEngine() *routing.CRPRoutingEngine {
	return rs.engine.(*routing.CRPRoutingEngine)
}

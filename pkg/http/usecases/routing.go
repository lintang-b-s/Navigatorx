package usecases

import (
	"context"
	"fmt"
	"sync"

	"github.com/dgraph-io/ristretto/v2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/offline"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/guidance"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"go.uber.org/zap"
)

type RoutingService struct {
	log             *zap.Logger
	engine          controllers.RoutingEngine
	graph           *da.Graph
	spatialIndex    SpatialIndex
	altRouting      AlternativeRouteAlgorithm
	searchRadius    float64
	lefthandDriving bool

	// sync pools

	directionBuilderPool *sync.Pool
	turnSignCache        *ristretto.Cache[uint64, []byte]
}

func NewRoutingService(log *zap.Logger, engine controllers.RoutingEngine, spatialindex SpatialIndex, altRouting AlternativeRouteAlgorithm,
	searchRadius float64, lefthandDriving bool,
) (*RoutingService, error) {
	rs := &RoutingService{
		log:             log,
		engine:          engine,
		spatialIndex:    spatialindex,
		searchRadius:    searchRadius,
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
		return nil, fmt.Errorf("initializeRoutingEngine: failed to create new ristretto cache with capacity: %v: %w", maxCost, err)
	}

	rs.directionBuilderPool = &sync.Pool{
		New: func() any {
			return guidance.NewDirectionBuilder(rs.engine,
				rs.engine.GetGraph(), rs.lefthandDriving,
				rs.turnSignCache,
			)
		},
	}

	return rs, nil
}

func (rs *RoutingService) ShortestPath(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64, reroute bool, startEdgeId da.Index) (float64, float64, string, []da.DrivingDirection, bool, error) {
	var (
		travelTime, dist float64
		pathCoords       *da.Coordinates
		edgePath         []da.Index
		found            bool
	)

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, reroute, startEdgeId)

	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}

	if !rs.isSameSourceDestinationSegment(sp, tp) {
		travelTime, dist, pathCoords, edgePath, found = rs.engine.ShortestPathSearch(sp, tp, reroute)
	}

	if !found {
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no route found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}

	travelTime, dist = rs.AppendPhantomNodesToPath(pathCoords, sp, tp, travelTime, dist)

	pathPolyline := da.GooglePoylineFromCoords(*pathCoords)
	directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)
	if reroute {
		directionBuilder.SetReroute(startEdgeId)
	}
	drivingDirection := directionBuilder.GetDrivingDirections(edgePath, sp, tp)

	directionBuilder.Reset()
	rs.directionBuilderPool.Put(directionBuilder)
	return travelTime, dist, pathPolyline, drivingDirection, true, nil
}

func (rs *RoutingService) AlternativeRouteSearch(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64, k int, reroute bool, startEdgeId da.Index) ([]routing.AlternativeRoute, error) {

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, reroute, startEdgeId)

	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		return make([]routing.AlternativeRoute, 0), util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}

	if rs.isSameSourceDestinationSegment(sp, tp) {
		return make([]routing.AlternativeRoute, 0), nil
	}

	alternatives, _, _ := rs.altRouting.FindAlternativeRoutes(sp, tp, k, reroute, startEdgeId)
	if len(alternatives) == 0 {
		return make([]routing.AlternativeRoute, 0), nil
	}

	for i, alt := range alternatives {
		altPathCoords := alt.GetCoords()
		newTravelTime, dist := rs.AppendPhantomNodesToPath(altPathCoords, sp, tp, alt.GetDrivingTravelTime(), alt.GetDist())
		alternatives[i].SetDrivingTravelTime(newTravelTime) // in seconds
		alternatives[i].SetDist(dist)

		pathPolyline := da.GooglePoylineFromCoords(*altPathCoords)
		alternatives[i].SetPolylinePath(pathPolyline)
		directionBuilder := rs.directionBuilderPool.Get().(*guidance.DirectionBuilder)
		if reroute {
			directionBuilder.SetReroute(startEdgeId)
		}
		drivingDirection := directionBuilder.GetDrivingDirections(alt.GetEdgeIdPath(), sp, tp)
		alternatives[i].SetDrivingDirections(drivingDirection)

		directionBuilder.Reset()
		rs.directionBuilderPool.Put(directionBuilder)
	}
	return alternatives, nil
}

func (rs *RoutingService) Close() {
	rs.turnSignCache.Close()
}

func (rs *RoutingService) AppendPhantomNodesToPath(path *da.Coordinates, sp, tp da.PhantomNode, travelTime float64, dist float64) (float64, float64) {
	if !rs.engine.IsDummyOutEdge(sp.GetOutEdgeId()) {
		if !rs.isSameSourceDestinationSegment(sp, tp) {
			path.Prepend(append([]da.Coordinate{sp.GetSnappedCoord()}, sp.GetForwardGeometry()...))
		} else {
			path.Prepend([]da.Coordinate{sp.GetSnappedCoord()})
		}
		travelTime += sp.GetForwardTravelTime()
		dist += sp.GetForwardDistance()
	}

	if !rs.engine.IsDummyInEdge(tp.GetInEdgeId()) {
		if !rs.isSameSourceDestinationSegment(sp, tp) {
			path.Append(append(tp.GetReverseGeometry(), tp.GetSnappedCoord()))
		} else {
			path.Append([]da.Coordinate{tp.GetSnappedCoord()})
		}

		travelTime += tp.GetReverseTravelTime()
		dist += tp.GetReverseDistance()
	}

	return travelTime, dist
}

func (rs *RoutingService) GetRoutingEngine() controllers.RoutingEngine {
	return rs.engine
}

func (rs *RoutingService) Snap(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64) (da.PhantomNode, da.PhantomNode) {
	if util.IsTimeout(ctx) {
		return da.NewInvalidPhantomNode(), da.NewInvalidPhantomNode()
	}

	return rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, false, da.INVALID_EDGE_ID)
}

func (rs *RoutingService) GetBoundingBox(ctx context.Context) da.BoundingBox {
	return *rs.graph.GetBoundingBox()
}

func (rs *RoutingService) InitBackgroundWorker(ctx context.Context) {
	rs.engine.InitBackgroundWorker(ctx)
}

func (rs *RoutingService) OfflineMapMatch(ctx context.Context, gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.Coordinate, error) {
	if util.IsTimeout(ctx) {
		return nil, nil, ctx.Err()
	}

	rt, ok := rs.spatialIndex.(*spatialindex.Rtree)
	if !ok {
		return nil, nil, fmt.Errorf("spatial index is not of type *spatialindex.Rtree")
	}

	re, ok := rs.engine.(*routing.CRPRoutingEngine)
	if !ok {
		return nil, nil, fmt.Errorf("routing engine is not of type *routing.CRPRoutingEngine")
	}

	hmm := offline.NewHiddenMarkovModelMapMatching(rs.graph, re, rt)
	var matchedPoints []*da.MatchedGPSPoint
	var routePath []da.Coordinate

	matchedPoints, routePath = hmm.MapMatchWithGPSRadiuses(gpsTraj, gpsRadiusesM)

	return matchedPoints, routePath, nil
}

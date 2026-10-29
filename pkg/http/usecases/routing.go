package usecases

import (
	"context"
	"fmt"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher/offline"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/guidance"

	cont "github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	"github.com/lintang-b-s/Navigatorx/pkg/spatialindex"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/maypok86/otter/v2"
	"go.uber.org/zap"
)

// RoutingService represents routing service.
type RoutingService struct {
	log          *zap.Logger
	engine       cont.RoutingEngine
	metrics      cont.Metrics
	graph        *da.Graph
	rn           *da.RoadNetworkDataContainer
	spatialIndex SpatialIndex

	searchRadius    float64
	lefthandDriving bool

	turnSignCache *otter.Cache[uint64, uint64]
}

// NewRoutingService creates and returns routing service.
func NewRoutingService(log *zap.Logger, engine cont.RoutingEngine, rn *da.RoadNetworkDataContainer, spatialIndex SpatialIndex,
	searchRadius float64, lefthandDriving bool,
) (*RoutingService, error) {
	rs := &RoutingService{
		log:             log,
		engine:          engine,
		spatialIndex:    spatialIndex,
		searchRadius:    searchRadius,
		lefthandDriving: lefthandDriving,
		graph:           engine.GetGraph(),
		rn:              rn,
	}
	rs.turnSignCache = da.NewTurnSignCache()
	rs.metrics = engine.GetMetrics()
	return rs, nil
}

// ShortestPath computes fastest path.
func (rs *RoutingService) ShortestPath(
	_ context.Context,
	qOrigLat, qOrigLon, qDstLat, qDstLon float64,
	reroute bool,
	startSegId da.Index,
	useAnnotation bool,
	useSteps bool,
) (float64, float64, string, []da.DrivingDirection, bool, error) {
	var (
		travelTime, dist  float64
		pathCoords        *da.Coordinates
		segmentPath       []da.Index
		found             bool
		drivingDirections []da.DrivingDirection
	)

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, reroute, startSegId)

	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}

	if !rs.isSameSourceDestinationSegment(sp, tp) {
		travelTime, dist, pathCoords, segmentPath, found = rs.engine.ShortestPathSearch(sp, tp, reroute)
	}

	if !found {
		return 0, 0, "", []da.DrivingDirection{}, false, util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no route found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}

	travelTime, dist = rs.AppendPhantomNodesToPath(pathCoords, sp, tp, travelTime, dist)

	pathPolyline := da.GooglePoylineFromCoords(*pathCoords)

	if useSteps {
		directionBuilder := guidance.NewDirectionBuilder(
			rs.engine.GetGraph(), rs.rn, rs.engine.GetMetrics(), rs.lefthandDriving,
			rs.turnSignCache,
		)
		if reroute {
			directionBuilder.SetReroute(startSegId)
		}
		drivingDirections = directionBuilder.GetDrivingDirections(segmentPath, sp, tp, useAnnotation)
	}

	return travelTime, dist, pathPolyline, drivingDirections, true, nil
}

// AlternativeRouteSearch handles alternative route search.
func (rs *RoutingService) AlternativeRouteSearch(
	_ context.Context,
	qOrigLat, qOrigLon, qDstLat, qDstLon float64,
	k int,
	reroute bool,
	startSegId da.Index,
	useAnnotation bool,
	useSteps bool,
) ([]float64, []float64, []string, [][]da.DrivingDirection, error) {

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, reroute, startSegId)

	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		return routing.EmptyCost, routing.EmptyCost, routing.EmptyPolylines, routing.EmptyDrivingDirections, util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}

	routeCosts, routeDists, routeSegmentPathCoords, routePaths, found := rs.engine.ShortestPathWithAlternativesSearch(sp, tp, k, reroute, startSegId)
	if !found {
		return routing.EmptyCost, routing.EmptyCost, routing.EmptyPolylines, routing.EmptyDrivingDirections, util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no route found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}
	n := len(routeCosts)
	routePolylines := make([]string, n)
	routeDrivingDirections := make([][]da.DrivingDirection, n)
	for i := 0; i < n; i++ {
		var drivingDirections []da.DrivingDirection

		rPathCoords := routeSegmentPathCoords[i]
		rCost := routeCosts[i]
		rDist := routeDists[i]
		rPath := routePaths[i]
		newCost, dist := rs.AppendPhantomNodesToPath(rPathCoords, sp, tp, rCost, rDist)
		routeCosts[i] = newCost // in seconds
		routeDists[i] = dist

		pathPolyline := da.GooglePoylineFromCoords(*rPathCoords)
		routePolylines[i] = pathPolyline
		if useSteps {
			directionBuilder := guidance.NewDirectionBuilder(
				rs.engine.GetGraph(), rs.rn, rs.engine.GetMetrics(), rs.lefthandDriving,
				rs.turnSignCache,
			)
			if reroute {
				directionBuilder.SetReroute(startSegId)
			}
			drivingDirections = directionBuilder.GetDrivingDirections(rPath, sp, tp, useAnnotation)
		}

		routeDrivingDirections[i] = drivingDirections
	}
	return routeCosts, routeDists, routePolylines, routeDrivingDirections, nil
}

// Close handles close caches.
func (rs *RoutingService) Close() {
	rs.turnSignCache.InvalidateAll()
	rs.turnSignCache.StopAllGoroutines()
}

// AppendPhantomNodesToPath appends phantom nodes to path.
func (rs *RoutingService) AppendPhantomNodesToPath(path *da.Coordinates, sp, tp da.PhantomNode, travelTime float64, dist float64) (float64, float64) {

	if !rs.isSameSourceDestinationSegment(sp, tp) {
		spgeom := sp.ForwardGeometry()
		path.Prepend(append([]da.Coordinate{sp.SnappedCoord()}, spgeom...))
	} else {
		path.Prepend([]da.Coordinate{sp.SnappedCoord()})
	}
	travelTime -= sp.ForwardCost() // by default pakai edge-based graph, hasil router added duration/travelTime dari road segment s
	dist += sp.ForwardDistance()   // kita gak tambahin distance dari projected s ke head dari road segment s di GetSegmentPath()

	if !rs.isSameSourceDestinationSegment(sp, tp) {
		path.Append(tp.ReverseGeometry())
		path.AppendCoordinate(tp.SnappedCoord())
	} else {
		path.AppendCoordinate(tp.SnappedCoord())
	}

	travelTime += tp.ReverseCost() // by default pakai edge-based graph, hasil router gak add duration/travelTime dari road segment t
	dist += tp.ReverseDistance()   // kita gak tambahin distance dari projected t ke head dari road segment t di GetSegmentPath()

	return travelTime, dist
}

// GetRoutingEngine returns routing engine.
func (rs *RoutingService) GetRoutingEngine() cont.RoutingEngine {
	return rs.engine
}

// Snap handles snap query coordinate to road segments.
func (rs *RoutingService) Snap(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64) (da.PhantomNode, da.PhantomNode) {
	if util.IsTimeout(ctx) {
		return da.NewInvalidPhantomNode(), da.NewInvalidPhantomNode()
	}

	return rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, false, da.InvalidSegmentId)
}

// GetBoundingBox returns bounding box.
func (rs *RoutingService) GetBoundingBox(_ context.Context) da.BoundingBox {
	return *rs.rn.GetBoundingBox()
}

// InitBackgroundWorker initializes background worker.
func (rs *RoutingService) InitBackgroundWorker(ctx context.Context) {
	rs.engine.InitBackgroundWorker(ctx)
}

// OfflineMapMatch handles offline map matching.
func (rs *RoutingService) OfflineMapMatch(ctx context.Context, gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.FloatCoordinate, error) {
	if util.IsTimeout(ctx) {
		return nil, nil, ctx.Err()
	}

	rt, ok := rs.spatialIndex.(*spatialindex.Rtree)
	if !ok {
		return nil, nil, fmt.Errorf("spatial index is not of type *spatialindex.Rtree")
	}

	re, ok := rs.engine.(*routing.CRPRoutingEngine[int32])
	if !ok {
		return nil, nil, fmt.Errorf("routing engine is not of type *routing.CRPRoutingEngine")
	}

	hmm := offline.NewHiddenMarkovModelMapMatching(rs.graph, re, rt)
	var matchedPoints []*da.MatchedGPSPoint
	var routePath []da.FloatCoordinate

	matchedPoints, routePath = hmm.MapMatchWithGPSRadiuses(gpsTraj, gpsRadiusesM)

	return matchedPoints, routePath, nil
}

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

type RoutingService struct {
	log             *zap.Logger
	engine          cont.RoutingEngine
	graph           *da.Graph
	rn              *da.RoadNetworkDataContainer
	spatialIndex    SpatialIndex
	altRouting      AlternativeRouteAlgorithm
	searchRadius    float64
	lefthandDriving bool

	turnSignCache *otter.Cache[uint64, uint64]
}

func NewRoutingService(log *zap.Logger, engine cont.RoutingEngine, rn *da.RoadNetworkDataContainer, spatialIndex SpatialIndex, altRouting AlternativeRouteAlgorithm,
	searchRadius float64, lefthandDriving bool,
) (*RoutingService, error) {
	rs := &RoutingService{
		log:             log,
		engine:          engine,
		spatialIndex:    spatialIndex,
		searchRadius:    searchRadius,
		lefthandDriving: lefthandDriving,
		graph:           engine.GetGraph(),
		altRouting:      altRouting,
		rn:              rn,
	}

	rs.turnSignCache = da.NewTurnSignCache()

	return rs, nil
}

func (rs *RoutingService) ShortestPath(
	ctx context.Context,
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

	rs.engine.PutCoordsToPool(pathCoords)
	return travelTime, dist, pathPolyline, drivingDirections, true, nil
}

func (rs *RoutingService) AlternativeRouteSearch(
	ctx context.Context,
	qOrigLat, qOrigLon, qDstLat, qDstLon float64,
	k int,
	reroute bool,
	startSegId da.Index,
	useAnnotation bool,
	useSteps bool,
) ([]routing.AlternativeRoute, error) {

	sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, reroute, startSegId)

	if rs.notFoundOriginDestinationWithinRadius(sp, tp) {
		return make([]routing.AlternativeRoute, 0), util.WrapErrorf(ErrPathNotFound, util.ErrBadParamInput,
			"no nearby road segments found from %f,%f to %f,%f", qOrigLat, qOrigLon, qDstLat, qDstLon)
	}

	if rs.isSameSourceDestinationSegment(sp, tp) {
		return make([]routing.AlternativeRoute, 0), nil
	}

	alternatives, _, _ := rs.altRouting.FindAlternativeRoutes(sp.GetVId(), tp.GetVId(), k, reroute, startSegId)
	if len(alternatives) == 0 {
		return make([]routing.AlternativeRoute, 0), nil
	}

	for i, alt := range alternatives {
		var drivingDirections []da.DrivingDirection

		altPathCoords := alt.GetCoords()
		newCost, dist := rs.AppendPhantomNodesToPath(altPathCoords, sp, tp, alt.GetTravelTime(), alt.GetDist())
		alternatives[i].SetTravelTime(newCost) // in seconds
		alternatives[i].SetDist(dist)

		pathPolyline := da.GooglePoylineFromCoords(*altPathCoords)
		alternatives[i].SetPolylinePath(pathPolyline)
		if useSteps {
			directionBuilder := guidance.NewDirectionBuilder(
				rs.engine.GetGraph(), rs.rn, rs.engine.GetMetrics(), rs.lefthandDriving,
				rs.turnSignCache,
			)
			if reroute {
				directionBuilder.SetReroute(startSegId)
			}
			drivingDirections = directionBuilder.GetDrivingDirections(alt.GetSegmentPath(), sp, tp, useAnnotation)
		}

		alternatives[i].SetDrivingDirections(drivingDirections)
		rs.engine.PutCoordsToPool(altPathCoords)
	}
	return alternatives, nil
}

func (rs *RoutingService) Close() {
	rs.turnSignCache.InvalidateAll()
	rs.turnSignCache.StopAllGoroutines()
}

func (rs *RoutingService) AppendPhantomNodesToPath(path *da.Coordinates, sp, tp da.PhantomNode, travelTime float64, dist float64) (float64, float64) {

	if !rs.isSameSourceDestinationSegment(sp, tp) {
		spgeom := sp.GetForwardGeometry()
		path.Prepend(append([]da.Coordinate{sp.GetSnappedCoord()}, spgeom...))
	} else {
		path.Prepend([]da.Coordinate{sp.GetSnappedCoord()})
	}
	travelTime -= sp.GetForwardCost() // by default pakai edge-based graph, hasil router added duration/travelTime dari road segment s
	dist += sp.GetForwardDistance()   // kita gak tambahin distance dari projected s ke head dari road segment s di GetSegmentPath()

	if !rs.isSameSourceDestinationSegment(sp, tp) {
		path.Append(tp.GetReverseGeometry())
		path.AppendCoordinate(tp.GetSnappedCoord())
	} else {
		path.AppendCoordinate(tp.GetSnappedCoord())
	}

	travelTime += tp.GetReverseCost() // by default pakai edge-based graph, hasil router gak add duration/travelTime dari road segment t
	dist += tp.GetReverseDistance()   // kita gak tambahin distance dari projected t ke head dari road segment t di GetSegmentPath()

	return travelTime, dist
}

func (rs *RoutingService) GetRoutingEngine() cont.RoutingEngine {
	return rs.engine
}

func (rs *RoutingService) Snap(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64) (da.PhantomNode, da.PhantomNode) {
	if util.IsTimeout(ctx) {
		return da.NewInvalidPhantomNode(), da.NewInvalidPhantomNode()
	}

	return rs.SnapOrigDestQueryToNearbyRoadSegments(qOrigLat, qOrigLon, qDstLat, qDstLon, false, da.INVALID_SEGMENT_ID)
}

func (rs *RoutingService) GetBoundingBox(ctx context.Context) da.BoundingBox {
	return *rs.rn.GetBoundingBox()
}

func (rs *RoutingService) InitBackgroundWorker(ctx context.Context) {
	rs.engine.InitBackgroundWorker(ctx)
}

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

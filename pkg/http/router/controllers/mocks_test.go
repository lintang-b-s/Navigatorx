package controllers

import (
	"context"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/stretchr/testify/mock"
)

type MockRoutingService struct {
	mock.Mock
}

func (m *MockRoutingService) ShortestPath(ctx context.Context, origLat, origLon, dstLat, dstLon float64, reroute bool, startEdgeId da.Index, useAnnotation bool) (float64, float64, string, []da.DrivingDirection, bool, error) {
	args := m.Called(ctx, origLat, origLon, dstLat, dstLon, reroute, startEdgeId, useAnnotation)
	var r3 []da.DrivingDirection
	if args.Get(3) != nil {
		r3 = args.Get(3).([]da.DrivingDirection)
	}
	return args.Get(0).(float64), args.Get(1).(float64), args.String(2), r3, args.Bool(4), args.Error(5)
}

func (m *MockRoutingService) AlternativeRouteSearch(ctx context.Context, origLat, origLon, dstLat, dstLon float64, k int, reroute bool, startEdgeId da.Index, useAnnotation bool) ([]routing.AlternativeRoute, error) {
	args := m.Called(ctx, origLat, origLon, dstLat, dstLon, k, reroute, startEdgeId, useAnnotation)
	var r0 []routing.AlternativeRoute
	if args.Get(0) != nil {
		r0 = args.Get(0).([]routing.AlternativeRoute)
	}
	return r0, args.Error(1)
}

func (m *MockRoutingService) GetRoutingEngine() RoutingEngine {
	args := m.Called()
	return args.Get(0).(RoutingEngine)
}

func (m *MockRoutingService) Close() {
	m.Called()
}

func (m *MockRoutingService) InitBackgroundWorker(ctx context.Context) {
	m.Called(ctx)
}

func (m *MockRoutingService) GetBoundingBox(ctx context.Context) da.BoundingBox {
	args := m.Called(ctx)
	return args.Get(0).(da.BoundingBox)
}

func (m *MockRoutingService) OfflineMapMatch(ctx context.Context, gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.Coordinate, error) {
	args := m.Called(ctx, gpsTraj, gpsRadiusesM)
	var r0 []*da.MatchedGPSPoint
	if args.Get(0) != nil {
		r0 = args.Get(0).([]*da.MatchedGPSPoint)
	}
	var r1 []da.Coordinate
	if args.Get(1) != nil {
		r1 = args.Get(1).([]da.Coordinate)
	}
	return r0, r1, args.Error(2)
}

type MockMapMatcherService struct {
	mock.Mock
}

func (m *MockMapMatcherService) OnlineMapMatch(ctx context.Context, gps *da.GPSPoint, k int,
	candidates []*ma.Candidate, speedMeanK, speedStdK, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64, error) {
	args := m.Called(ctx, gps, k, candidates, speedMeanK, speedStdK, lastBearing)
	var r0 *da.MatchedGPSPoint
	if args.Get(0) != nil {
		r0 = args.Get(0).(*da.MatchedGPSPoint)
	}
	var r1 []*ma.Candidate
	if args.Get(1) != nil {
		r1 = args.Get(1).([]*ma.Candidate)
	}
	return r0, r1, args.Get(2).(float64), args.Get(3).(float64), args.Error(4)
}

type MockRoutingEngine struct {
	mock.Mock
}

func (m *MockRoutingEngine) GetGraph() *da.Graph {
	args := m.Called()
	return args.Get(0).(*da.Graph)
}

func (m *MockRoutingEngine) PathExists(u, v da.Index) bool {
	args := m.Called(u, v)
	return args.Bool(0)
}

func (m *MockRoutingEngine) GetWeight(eId da.Index, outEdge bool) float64 {
	args := m.Called(eId, outEdge)
	return args.Get(0).(float64)
}

func (m *MockRoutingEngine) GetWeightFromLength(eId da.Index, outEdge bool, eLength float64) float64 {
	args := m.Called(eId, eLength, outEdge)
	return args.Get(0).(float64)
}

func (m *MockRoutingEngine) GetSegmentSpeed(eId da.Index, outEdge bool) float64 {
	args := m.Called(eId, outEdge)
	return args.Get(0).(float64)
}

func (m *MockRoutingEngine) GetSegmentLength(eId da.Index, outEdge bool) float64 {
	args := m.Called(eId, outEdge)
	return args.Get(0).(float64)
}

func (m *MockRoutingEngine) IsDummyOutEdge(eId da.Index) bool {
	args := m.Called(eId)
	return args.Bool(0)
}

func (m *MockRoutingEngine) IsDummyInEdge(eId da.Index) bool {
	args := m.Called(eId)
	return args.Bool(0)
}

func (m *MockRoutingEngine) InitBackgroundWorker(ctx context.Context) {
	m.Called(ctx)
}

func (m *MockRoutingEngine) ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, *da.Coordinates, []da.Index, bool) {
	args := m.Called(sp, tp, reroute)
	var r2 *da.Coordinates
	if args.Get(2) != nil {
		r2 = args.Get(2).(*da.Coordinates)
	}
	var r3 []da.Index
	if args.Get(3) != nil {
		r3 = args.Get(3).([]da.Index)
	}
	return args.Get(0).(float64), args.Get(1).(float64), r2, r3, args.Bool(4)
}

func (m *MockRoutingEngine) PutPathToPool([]da.Index) {}

func (m *MockRoutingEngine) PutCoordsToPool(*da.Coordinates) {}

func (m *MockRoutingEngine) Close() {
	m.Called()
}

type MockTilingService struct {
	mock.Mock
}

// GetNumberOfVertices implements [TilingService].
func (m *MockTilingService) GetNumberOfVertices(ctx context.Context) int {
	args := m.Called(ctx)
	return args.Int(0)
}

func (m *MockTilingService) GetTileFilePath(ctx context.Context, userGeohash string) string {
	args := m.Called(ctx, userGeohash)
	return args.String(0)
}

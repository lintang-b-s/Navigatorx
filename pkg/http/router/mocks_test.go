package router

import (
	"context"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/http/router/controllers"
	"github.com/stretchr/testify/mock"
)

type MockRoutingService struct {
	mock.Mock
}

func (m *MockRoutingService) ShortestPath(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64, reroute bool, startEdgeId da.Index) (float64, float64, string, []da.DrivingDirection, bool, error) {
	args := m.Called(ctx, qOrigLat, qOrigLon, qDstLat, qDstLon, reroute, startEdgeId)
	return args.Get(0).(float64), args.Get(1).(float64), args.String(2), args.Get(3).([]da.DrivingDirection), args.Bool(4), args.Error(5)
}

func (m *MockRoutingService) AlternativeRouteSearch(ctx context.Context, qOrigLat, qOrigLon, qDstLat, qDstLon float64, k int, reroute bool, startEdgeId da.Index) ([]routing.AlternativeRoute, error) {
	args := m.Called(ctx, qOrigLat, qOrigLon, qDstLat, qDstLon, k, reroute, startEdgeId)
	return args.Get(0).([]routing.AlternativeRoute), args.Error(1)
}

func (m *MockRoutingService) GetBoundingBox(ctx context.Context) da.BoundingBox {
	args := m.Called(ctx)
	return args.Get(0).(da.BoundingBox)
}

func (m *MockRoutingService) Close() {
	m.Called()
}

func (m *MockRoutingService) GetRoutingEngine() controllers.RoutingEngine {
	args := m.Called()
	return args.Get(0).(controllers.RoutingEngine)
}

func (m *MockRoutingService) InitBackgroundWorker(ctx context.Context) {
	m.Called(ctx)
}

type MockMapMatcherService struct {
	mock.Mock
}

func (m *MockMapMatcherService) OnlineMapMatch(ctx context.Context, gpsPoint *da.GPSPoint, k int, candidates []*ma.Candidate, speedMeanK float64, speedStdK float64, lastBearing float64) (*da.MatchedGPSPoint, []*ma.Candidate, float64, float64, error) {
	args := m.Called(ctx, gpsPoint, k, candidates, speedMeanK, speedStdK, lastBearing)
	if args.Get(0) == nil {
		return nil, args.Get(1).([]*ma.Candidate), args.Get(2).(float64), args.Get(3).(float64), args.Error(4)
	}
	return args.Get(0).(*da.MatchedGPSPoint), args.Get(1).([]*ma.Candidate), args.Get(2).(float64), args.Get(3).(float64), args.Error(4)
}

type MockRoutingEngine struct {
	mock.Mock
}

func (m *MockRoutingEngine) Close() {
	m.Called()
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
func (m *MockRoutingEngine) GetWeightFromLength(eId da.Index, eLength float64, outEdge bool) float64 {
	args := m.Called(eId, eLength, outEdge)
	return args.Get(0).(float64)
}
func (m *MockRoutingEngine) IsDummyOutEdge(edgeId da.Index) bool {
	args := m.Called(edgeId)
	return args.Bool(0)
}
func (m *MockRoutingEngine) IsDummyInEdge(edgeId da.Index) bool {
	args := m.Called(edgeId)
	return args.Bool(0)
}
func (m *MockRoutingEngine) InitBackgroundWorker(ctx context.Context) {
	m.Called(ctx)
}
func (m *MockRoutingEngine) ShortestPathSearch(sp, tp da.PhantomNode, reroute bool) (float64, float64, *da.Coordinates, []da.Index, bool) {
	args := m.Called(sp, tp, reroute)
	return args.Get(0).(float64), args.Get(1).(float64), args.Get(2).(*da.Coordinates), args.Get(3).([]da.Index), args.Bool(4)
}

type MockTilingService struct {
	mock.Mock
}

func (m *MockTilingService) GetTileFilePath(ctx context.Context, userGeohash string) string {
	args := m.Called(ctx, userGeohash)
	return args.String(0)
}

func (m *MockTilingService) GetNumberOfVertices(ctx context.Context) int {
	args := m.Called(ctx)
	return args.Int(0)
}

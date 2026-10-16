package controllers

import (
	"context"

	"github.com/golang/geo/s2"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	met "github.com/lintang-b-s/Navigatorx/pkg/metrics"
	"github.com/stretchr/testify/mock"
)

type MockRoutingService struct {
	mock.Mock
}

func (m *MockRoutingService) ShortestPath(ctx context.Context, origLat, origLon, dstLat, dstLon float64, reroute bool, startEdgeId da.Index, useAnnotation, useSteps bool) (float64, float64, string, []da.DrivingDirection, bool, error) {
	args := m.Called(ctx, origLat, origLon, dstLat, dstLon, reroute, startEdgeId, useAnnotation, useSteps)
	var r3 []da.DrivingDirection
	if args.Get(3) != nil {
		r3 = args.Get(3).([]da.DrivingDirection)
	}
	return args.Get(0).(float64), args.Get(1).(float64), args.String(2), r3, args.Bool(4), args.Error(5)
}

func (m *MockRoutingService) AlternativeRouteSearch(ctx context.Context, origLat, origLon, dstLat, dstLon float64, k int, reroute bool, startEdgeId da.Index, useAnnotation, useSteps bool) ([]float64, []float64, []string, [][]da.DrivingDirection, error) {
	args := m.Called(ctx, origLat, origLon, dstLat, dstLon, k, reroute, startEdgeId, useAnnotation, useSteps)
	var r0 []float64
	if args.Get(0) != nil {
		r0 = args.Get(0).([]float64)
	}
	var r1 []float64
	if args.Get(1) != nil {
		r1 = args.Get(1).([]float64)
	}
	var r2 []string
	if args.Get(2) != nil {
		r2 = args.Get(2).([]string)
	}
	var r3 [][]da.DrivingDirection
	if args.Get(3) != nil {
		r3 = args.Get(3).([][]da.DrivingDirection)
	}
	return r0, r1, r2, r3, args.Error(4)
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

func (m *MockRoutingService) OfflineMapMatch(ctx context.Context, gpsTraj []*da.GPSPoint, gpsRadiusesM []float64) ([]*da.MatchedGPSPoint, []da.FloatCoordinate, error) {
	args := m.Called(ctx, gpsTraj, gpsRadiusesM)
	var r0 []*da.MatchedGPSPoint
	if args.Get(0) != nil {
		r0 = args.Get(0).([]*da.MatchedGPSPoint)
	}
	var r1 []da.FloatCoordinate
	if args.Get(1) != nil {
		r1 = args.Get(1).([]da.FloatCoordinate)
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

func (m *MockRoutingEngine) GetLength(eId da.Index, outEdge bool, eLength float64) float64 {
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

func (m *MockRoutingEngine) ShortestPathWithAlternativesSearch(sp, tp da.PhantomNode, k int, reroute bool, startSegId da.Index) ([]float64, []float64, []*da.Coordinates, [][]da.Index, bool) {
	args := m.Called(sp, tp, k, reroute, startSegId)
	var r0 []float64
	if args.Get(0) != nil {
		r0 = args.Get(0).([]float64)
	}
	var r1 []float64
	if args.Get(1) != nil {
		r1 = args.Get(1).([]float64)
	}
	var r2 []*da.Coordinates
	if args.Get(2) != nil {
		r2 = args.Get(2).([]*da.Coordinates)
	}
	var r3 [][]da.Index
	if args.Get(3) != nil {
		r3 = args.Get(3).([][]da.Index)
	}
	return r0, r1, r2, r3, args.Bool(4)
}

func (m *MockRoutingEngine) GetMetrics() *met.Metric[int32] {
	args := m.Called()
	if args.Get(0) == nil {
		return nil
	}
	return args.Get(0).(*met.Metric[int32])
}

func (m *MockRoutingEngine) PutPathToPool([]da.Index) {}

func (m *MockRoutingEngine) PutCoordsToPool(*da.Coordinates) {}

func (m *MockRoutingEngine) Close() {
	m.Called()
}

type MockMapAttributesService struct {
	mock.Mock
}

// GetNumberOfVertices implements [MapAttributesService].
func (m *MockMapAttributesService) GetNumberOfVertices(ctx context.Context) int {
	args := m.Called(ctx)
	return args.Int(0)
}

func (m *MockMapAttributesService) GetMapAttributes(ctx context.Context, s2CellId s2.CellID, radius float64) ([]byte, error) {
	args := m.Called(ctx, s2CellId, radius)
	return []byte(args.String(0)), args.Error(1)
}

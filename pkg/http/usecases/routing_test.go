package usecases

import (
	"context"
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/mock"
	"go.uber.org/zap"
)

const (
	yogyakartaOriginLat = -7.7956
	yogyakartaOriginLon = 110.3695
	yogyakartaDestLat   = -7.7828
	yogyakartaDestLon   = 110.4145
)

func setupRoutingService() (*RoutingService, *MockRoutingEngine, *MockSpatialIndex, *MockAlternativeRouteAlgorithm) {
	mockEngine := new(MockRoutingEngine)
	mockSI := new(MockSpatialIndex)
	mockARA := new(MockAlternativeRouteAlgorithm)
	log := zap.NewNop()

	// Create a minimal working graph for Snap
	gs := da.NewGraphStorage(41)
	gs.AppendOsmNodePoints([]da.Coordinate{{Lat: yogyakartaOriginLat, Lon: yogyakartaOriginLon}, {Lat: yogyakartaDestLat, Lon: yogyakartaDestLon}})
	gs.AppendEdgeMetadata(100, 0, 2, 0, 0, 0, 1) // Edge 0
	gs.AppendEdgeMetadata(101, 0, 2, 0, 0, 0, 1) // Edge 1

	g := da.NewGraph(
		[]da.Vertex{
			da.NewVertex(yogyakartaOriginLat, yogyakartaOriginLon, 0),
			da.NewVertex(yogyakartaDestLat, yogyakartaDestLon, 1),
			da.NewVertex(-7.7660, 110.4310, 2),
		},
		[]da.OutEdge{da.NewOutEdge(1, 1, 0, 1, 0, 0), da.NewOutEdge(1, 1, 1, 2, 0, 0)},
		[]da.InEdge{da.NewInEdge(1, 1, 0, 0, 0, 0), da.NewInEdge(1, 1, 1, 1, 0, 0)},
		nil,
		true,
		nil,
	)
	g.SetGraphStorage(gs)
	g.SetBoundingBox(&da.BoundingBox{})

	mockEngine.On("GetGraph").Return(g)

	rs, _ := NewRoutingService(log, mockEngine, mockSI, mockARA, 1000.0, false)
	return rs, mockEngine, mockSI, mockARA
}

func TestRoutingService_ShortestPath(t *testing.T) {
	rs, _, _, _ := setupRoutingService()

	t.Run("Timeout", func(t *testing.T) {
		ctx, cancel := context.WithCancel(context.Background())
		cancel()

		_, _, _, _, ok, err := rs.ShortestPath(ctx, yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, false, da.INVALID_EDGE_ID)

		assert.Error(t, err)
		assert.False(t, ok)
	})

}

func setupSnapReadyRoutingService(t *testing.T) (*RoutingService, *MockRoutingEngine, *MockSpatialIndex, *MockAlternativeRouteAlgorithm, da.PhantomNode, da.PhantomNode) {
	t.Helper()

	log := zap.NewNop()
	mockEngine := new(MockRoutingEngine)
	mockSpatial := new(MockSpatialIndex)
	mockAlt := new(MockAlternativeRouteAlgorithm)

	const (
		searchRadius  = 1.0
		originEdgeLen = 120.0
		destEdgeLen   = 150.0
	)

	originCoord := da.NewCoordinate(yogyakartaOriginLat, yogyakartaOriginLon)
	destCoord := da.NewCoordinate(yogyakartaDestLat, yogyakartaDestLon)
	originHeadCoord := da.NewCoordinate(-7.7900, 110.3860)

	v0 := da.NewVertex(-7.7970, 110.3660, 0)
	v0.SetFirstOut(0)
	v0.SetFirstIn(0)
	v1 := da.NewVertex(originHeadCoord.Lat, originHeadCoord.Lon, 1)
	v1.SetFirstOut(1)
	v1.SetFirstIn(0)
	v2 := da.NewVertex(-7.7780, 110.4240, 2)
	v2.SetFirstOut(2)
	v2.SetFirstIn(1)
	vDummy := da.NewVertex(0, 0, 3)
	vDummy.SetFirstOut(2)
	vDummy.SetFirstIn(2)

	g := da.NewGraph(
		[]da.Vertex{v0, v1, v2, vDummy},
		[]da.OutEdge{da.NewOutEdge(0, 1, 12.0, originEdgeLen, 0, 0), da.NewOutEdge(1, 2, 15.0, destEdgeLen, 0, 0)},
		[]da.InEdge{da.NewInEdge(0, 0, 12.0, originEdgeLen, 0, 0), da.NewInEdge(1, 1, 15.0, destEdgeLen, 0, 0)},
		nil,
		false,
		nil,
	)
	gs := da.NewGraphStorage(64)
	gs.AppendOsmNodePoints([]da.Coordinate{
		{Lat: -7.7970, Lon: 110.3660}, originCoord, originHeadCoord,
		{Lat: -7.7860, Lon: 110.4080}, destCoord, {Lat: -7.7780, Lon: 110.4240},
	})
	gs.AppendEdgeMetadata(100, 0, 3, 0, 0, 0, 1)
	gs.AppendEdgeMetadata(200, 3, 6, 0, 0, 0, 1)
	g.SetGraphStorage(gs)
	g.SetBoundingBox(da.NewBoundingBox(-7.8290, 110.3196, -7.7447, 110.4569))

	mockEngine.On("GetGraph").Return(g)
	rs, err := NewRoutingService(log, mockEngine, mockSpatial, mockAlt, searchRadius, false)
	assert.NoError(t, err)

	mockSpatial.On("SearchWithinRadius", yogyakartaOriginLat, yogyakartaOriginLon, searchRadius, uint8(0)).Return([]da.Index{0})
	mockSpatial.On("SearchWithinRadius", yogyakartaDestLat, yogyakartaDestLon, searchRadius, uint8(1)).Return([]da.Index{1})
	mockEngine.On("PathExists", da.Index(1), da.Index(1)).Return(true)
	mockEngine.On("GetWeightFromLength", da.Index(0), originEdgeLen, true).Return(12.0)
	mockEngine.On("GetWeightFromLength", da.Index(1), destEdgeLen, false).Return(15.0)

	sp := da.NewPhantomNode(da.NewCoordinate(yogyakartaOriginLat, yogyakartaOriginLon), 12.0, 0, 0, da.INVALID_EDGE_ID, originEdgeLen, 0, []da.Coordinate{originCoord, originHeadCoord}, []da.Coordinate{})
	tp := da.NewPhantomNode(da.NewCoordinate(yogyakartaDestLat, yogyakartaDestLon), 0, 15.0, 1, 1, 0, destEdgeLen, []da.Coordinate{}, []da.Coordinate{})

	return rs, mockEngine, mockSpatial, mockAlt, sp, tp
}

// coordApproxEqual compares two Coordinates using epsilon-based float comparison.
func coordApproxEqual(a, b da.Coordinate) bool {
	return util.Eq(a.Lat, b.Lat) && util.Eq(a.Lon, b.Lon)
}

// coordSliceApproxEqual compares two Coordinate slices using epsilon-based float comparison.
func coordSliceApproxEqual(a, b []da.Coordinate) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !coordApproxEqual(a[i], b[i]) {
			return false
		}
	}
	return true
}

// phantomNodeApproxEqual returns a matcher function for use with mock.MatchedBy
// that compares PhantomNode fields using epsilon-based float comparison (util.Eq).
func phantomNodeApproxEqual(expected da.PhantomNode) func(da.PhantomNode) bool {
	return func(actual da.PhantomNode) bool {
		return coordApproxEqual(actual.GetSnappedCoord(), expected.GetSnappedCoord()) &&
			util.Eq(actual.GetForwardTravelTime(), expected.GetForwardTravelTime()) &&
			util.Eq(actual.GetReverseTravelTime(), expected.GetReverseTravelTime()) &&
			util.Eq(actual.GetForwardDistance(), expected.GetForwardDistance()) &&
			util.Eq(actual.GetReverseDistance(), expected.GetReverseDistance()) &&
			actual.GetOutEdgeId() == expected.GetOutEdgeId() &&
			actual.GetInEdgeId() == expected.GetInEdgeId() &&
			coordSliceApproxEqual(actual.GetForwardGeometry(), expected.GetForwardGeometry()) &&
			coordSliceApproxEqual(actual.GetReverseGeometry(), expected.GetReverseGeometry())
	}
}

func TestRoutingService_ShortestPathBranches(t *testing.T) {
	t.Run("No Nearby Road Segments", func(t *testing.T) {
		rs, _, mockSpatial, _ := setupRoutingService()
		mockSpatial.On("SearchWithinRadius", yogyakartaOriginLat, yogyakartaOriginLon, 1000.0, uint8(0)).Return([]da.Index{})
		mockSpatial.On("SearchWithinRadius", yogyakartaDestLat, yogyakartaDestLon, 1000.0, uint8(1)).Return([]da.Index{})

		_, _, _, _, ok, err := rs.ShortestPath(context.Background(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, false, da.INVALID_EDGE_ID)

		assert.Error(t, err)
		assert.False(t, ok)
	})

	t.Run("Engine Route Not Found", func(t *testing.T) {
		rs, mockEngine, mockSpatial, _, sp, tp := setupSnapReadyRoutingService(t)
		path := da.NewCoordinatesWithInitialValues([]da.Coordinate{{Lat: -7.7900, Lon: 110.3860}})
		mockEngine.On("ShortestPathSearch", mock.MatchedBy(phantomNodeApproxEqual(sp)), mock.MatchedBy(phantomNodeApproxEqual(tp)), false, da.INVALID_EDGE_ID).
			Return(0.0, 0.0, path, []da.Index{}, false)

		_, _, _, _, ok, err := rs.ShortestPath(context.Background(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, false, da.INVALID_EDGE_ID)

		assert.Error(t, err)
		assert.False(t, ok)
		mockSpatial.AssertExpectations(t)
		mockEngine.AssertExpectations(t)
	})

	t.Run("Success Without Directions", func(t *testing.T) {
		rs, mockEngine, mockSpatial, _, sp, tp := setupSnapReadyRoutingService(t)
		path := da.NewCoordinatesWithInitialValues([]da.Coordinate{{Lat: -7.7900, Lon: 110.3860}})
		mockEngine.On("ShortestPathSearch", mock.MatchedBy(phantomNodeApproxEqual(sp)), mock.MatchedBy(phantomNodeApproxEqual(tp)), true, da.Index(1)).
			Return(20.0, 200.0, path, []da.Index{}, true)
		mockEngine.On("IsDummyOutEdge", da.Index(0)).Return(false)
		mockEngine.On("IsDummyInEdge", da.Index(1)).Return(false)

		travelTime, dist, polyline, dirs, ok, err := rs.ShortestPath(context.Background(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, true, da.Index(1))

		assert.NoError(t, err)
		assert.True(t, ok)
		assert.Equal(t, 47.0, travelTime)
		assert.Equal(t, 320.0, dist)
		assert.NotEmpty(t, polyline)
		assert.Empty(t, dirs)
		mockSpatial.AssertExpectations(t)
		mockEngine.AssertExpectations(t)
	})
}

func TestRoutingService_AlternativeRouteSearch(t *testing.T) {
	rs, _, _, _ := setupRoutingService()

	t.Run("Timeout", func(t *testing.T) {
		ctx, cancel := context.WithCancel(context.Background())
		cancel()

		alts, err := rs.AlternativeRouteSearch(ctx, yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, false, da.INVALID_EDGE_ID)

		assert.Error(t, err)
		assert.Empty(t, alts)
	})
}

func TestRoutingService_AlternativeRouteSearchBranches(t *testing.T) {
	t.Run("No Nearby Road Segments", func(t *testing.T) {
		rs, _, mockSpatial, _ := setupRoutingService()
		mockSpatial.On("SearchWithinRadius", yogyakartaOriginLat, yogyakartaOriginLon, 1000.0, uint8(0)).Return([]da.Index{})
		mockSpatial.On("SearchWithinRadius", yogyakartaDestLat, yogyakartaDestLon, 1000.0, uint8(1)).Return([]da.Index{})

		alts, err := rs.AlternativeRouteSearch(context.Background(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, false, da.INVALID_EDGE_ID)

		assert.Error(t, err)
		assert.Empty(t, alts)
	})

	t.Run("No Alternatives", func(t *testing.T) {
		rs, mockEngine, mockSpatial, mockAlt, sp, tp := setupSnapReadyRoutingService(t)
		mockAlt.On("FindAlternativeRoutes", mock.MatchedBy(phantomNodeApproxEqual(sp)), mock.MatchedBy(phantomNodeApproxEqual(tp)), 3, false, da.INVALID_EDGE_ID).
			Return([]routing.AlternativeRoute{}, 0.0, int64(0))

		alts, err := rs.AlternativeRouteSearch(context.Background(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, false, da.INVALID_EDGE_ID)

		assert.NoError(t, err)
		assert.Empty(t, alts)
		mockSpatial.AssertExpectations(t)
		mockEngine.AssertExpectations(t)
		mockAlt.AssertExpectations(t)
	})

	t.Run("Success Without Directions", func(t *testing.T) {
		rs, mockEngine, mockSpatial, mockAlt, sp, tp := setupSnapReadyRoutingService(t)
		path := da.NewCoordinatesWithInitialValues([]da.Coordinate{{Lat: -7.7900, Lon: 110.3860}})
		alt := routing.NewAlternativeRoute(5.0, 200.0, 20.0, 0.0, 1, path, []da.Index{}, da.ViaVertex{})
		mockAlt.On("FindAlternativeRoutes", mock.MatchedBy(phantomNodeApproxEqual(sp)), mock.MatchedBy(phantomNodeApproxEqual(tp)), 3, true, da.Index(1)).
			Return([]routing.AlternativeRoute{alt}, 0.0, int64(0))
		mockEngine.On("IsDummyOutEdge", da.Index(0)).Return(false)
		mockEngine.On("IsDummyInEdge", da.Index(1)).Return(false)

		alts, err := rs.AlternativeRouteSearch(context.Background(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, true, da.Index(1))

		assert.NoError(t, err)
		assert.Len(t, alts, 1)
		assert.Equal(t, 47.0, alts[0].GetDrivingTravelTime())
		assert.Equal(t, 320.0, alts[0].GetDist())
		assert.NotEmpty(t, alts[0].GetPolylinePath())
		assert.Empty(t, alts[0].GetDrivingDirections())
		mockSpatial.AssertExpectations(t)
		mockEngine.AssertExpectations(t)
		mockAlt.AssertExpectations(t)
	})
}

func TestRoutingService_Snap(t *testing.T) {
	rs, _, _, _ := setupRoutingService()

	t.Run("Timeout", func(t *testing.T) {
		ctx, cancel := context.WithCancel(context.Background())
		cancel()

		sp, tp := rs.Snap(ctx, yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon)
		assert.True(t, da.IsPhantomNodeInvalid(sp))
		assert.True(t, da.IsPhantomNodeInvalid(tp))
	})

	t.Run("Success", func(t *testing.T) {
		rs, mockEngine, mockSpatial, _, _, _ := setupSnapReadyRoutingService(t)

		sp, tp := rs.Snap(context.Background(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon)

		assert.False(t, da.IsPhantomNodeInvalid(sp))
		assert.False(t, da.IsPhantomNodeInvalid(tp))
		mockSpatial.AssertExpectations(t)
		mockEngine.AssertExpectations(t)
	})

}

func TestRoutingService_AppendPhantomNodesToPath(t *testing.T) {
	rs, mockEngine, _, _, sp, tp := setupSnapReadyRoutingService(t)
	path := da.NewCoordinatesWithInitialValues([]da.Coordinate{{Lat: -7.7900, Lon: 110.3860}})
	mockEngine.On("IsDummyOutEdge", da.Index(0)).Return(false)
	mockEngine.On("IsDummyInEdge", da.Index(1)).Return(false)

	travelTime, dist := rs.AppendPhantomNodesToPath(path, sp, tp, 20.0, 200.0)

	assert.Equal(t, 47.0, travelTime)
	assert.Equal(t, 320.0, dist)
	assert.InDelta(t, yogyakartaOriginLat, (*path)[0].Lat, 0.0000000001)
	assert.InDelta(t, yogyakartaDestLat, (*path)[len(*path)-1].Lat, 0.0000000001)
}

func TestRoutingService_Misc(t *testing.T) {
	rs, mockEngine, _, _ := setupRoutingService()

	t.Run("GetRoutingEngine", func(t *testing.T) {
		assert.Equal(t, mockEngine, rs.GetRoutingEngine())
	})

	t.Run("Close", func(t *testing.T) {
		rs.Close()
	})

	t.Run("GetBoundingBox", func(t *testing.T) {
		rs.GetBoundingBox(context.Background())
	})

	t.Run("InitBackgroundWorker", func(t *testing.T) {
		ctx := context.Background()
		mockEngine.On("InitBackgroundWorker", ctx).Return()
		rs.InitBackgroundWorker(ctx)
		mockEngine.AssertExpectations(t)
	})
}

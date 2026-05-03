package usecases

import (
	"testing"

	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

func TestRoutingService_SnapOrigDestQueryToNearbyRoadSegments(t *testing.T) {
	log := zap.NewNop()
	mockEngine := new(MockRoutingEngine)
	mockSpatial := new(MockSpatialIndex)
	mockAlt := new(MockAlternativeRouteAlgorithm)

	const (
		originQueryLat = -7.7956
		originQueryLon = 110.3695
		destQueryLat   = -7.7828
		destQueryLon   = 110.4145
		searchRadius   = 1.0
		originEdgeLen  = 120.0
		destEdgeLen    = 150.0
	)

	v0 := da.NewVertex(-7.7970, 110.3660, 0)
	v0.SetFirstOut(0)
	v0.SetFirstIn(0)

	v1 := da.NewVertex(-7.7900, 110.3860, 1)
	v1.SetFirstOut(1)
	v1.SetFirstIn(0)

	v2 := da.NewVertex(-7.7780, 110.4240, 2)
	v2.SetFirstOut(2)
	v2.SetFirstIn(1)

	vDummy := da.NewVertex(0, 0, 3)
	vDummy.SetFirstOut(2)
	vDummy.SetFirstIn(2)

	mockGraph := da.NewGraph(
		[]da.Vertex{v0, v1, v2, vDummy},
		[]da.OutEdge{da.NewOutEdge(0, 1, 12.0, originEdgeLen, 0, 0), da.NewOutEdge(1, 2, 15.0, destEdgeLen, 0, 0)},
		[]da.InEdge{da.NewInEdge(0, 0, 12.0, originEdgeLen, 0, 0), da.NewInEdge(1, 1, 15.0, destEdgeLen, 0, 0)},
		nil, false, nil,
	)

	gs := da.NewGraphStorage(64)
	gs.AppendOsmNodePoints([]da.Coordinate{
		{Lat: -7.7970, Lon: 110.3660}, {Lat: -7.7952, Lon: 110.3700}, {Lat: -7.7900, Lon: 110.3860},
		{Lat: -7.7860, Lon: 110.4080}, {Lat: -7.7825, Lon: 110.4150}, {Lat: -7.7780, Lon: 110.4240},
	})
	gs.AppendEdgeMetadata(100, 0, 3, 0, 0, 0, 1)
	gs.AppendEdgeMetadata(200, 3, 6, 0, 0, 0, 1)

	mockGraph.SetGraphStorage(gs)
	mockGraph.SetBoundingBox(&da.BoundingBox{})
	mockEngine.On("GetGraph").Return(mockGraph)

	rs, _ := NewRoutingService(log, mockEngine, mockSpatial, mockAlt, searchRadius, false)

	t.Run("Snap Success", func(t *testing.T) {
		mockSpatial.On("SearchWithinRadius", originQueryLat, originQueryLon, searchRadius, uint8(0)).Return([]da.Index{0})
		mockSpatial.On("SearchWithinRadius", destQueryLat, destQueryLon, searchRadius, uint8(1)).Return([]da.Index{1})

		mockEngine.On("PathExists", da.Index(1), da.Index(1)).Return(true)
		mockEngine.On("GetWeightFromLength", da.Index(0), originEdgeLen, true).Return(12.0)
		mockEngine.On("GetWeightFromLength", da.Index(1), destEdgeLen, false).Return(15.0)

		sp, tp := rs.SnapOrigDestQueryToNearbyRoadSegments(originQueryLat, originQueryLon, destQueryLat, destQueryLon, false, da.INVALID_EDGE_ID)

		assert.False(t, da.IsPhantomNodeInvalid(sp))
		assert.False(t, da.IsPhantomNodeInvalid(tp))
		mockSpatial.AssertExpectations(t)
		mockEngine.AssertExpectations(t)
	})
}

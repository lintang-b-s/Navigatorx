package controllers

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/julienschmidt/httprouter"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	ma "github.com/lintang-b-s/Navigatorx/pkg/engine/mapmatcher"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

type closeErrorBody struct {
	*bytes.Reader
}

func (b closeErrorBody) Close() error {
	return errors.New("close error")
}

const (
	yogyakartaOriginLat = -7.7956
	yogyakartaOriginLon = 110.3695
	yogyakartaDestLat   = -7.7828
	yogyakartaDestLon   = 110.4145
)

func TestRoutingAPI_ShortestPath(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockMMS := new(MockMapMatcherService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockMMS, mockTS)

	t.Run("Missing Params", func(t *testing.T) {
		req, _ := http.NewRequest("GET", "/computeRoutes", nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "origin_lat is required")
	})

	t.Run("Invalid Float Param", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=abc&origin_lon=%f&destination_lat=%f&destination_lon=%f", yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "origin_lat is required and must be a valid float")
	})

	t.Run("Invalid Origin Lon Param", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=abc&destination_lat=%f&destination_lon=%f", yogyakartaOriginLat, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "origin_lon is required and must be a valid float")
	})

	t.Run("Invalid Destination Lat Param", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=abc&destination_lon=%f", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "destination_lat is required and must be a valid float")
	})

	t.Run("Invalid Destination Lon Param", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=abc", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat), nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "destination_lon is required and must be a valid float")
	})

	t.Run("Validation Error", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=100&origin_lon=%f&destination_lat=%f&destination_lon=%f", yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "validation error")
	})

	t.Run("Invalid Reroute Bool", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&reroute=maybe&start_edge_id=1", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "reroute must be boolean")
	})

	t.Run("Invalid Start Edge ID", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&reroute=true&start_edge_id=bad", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "edgeId must be unsigned integer 32 bit")
	})

	t.Run("Success", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.On("ShortestPath", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, false, da.INVALID_EDGE_ID).
			Return(100.0, 1000.0, "polyline", []da.DrivingDirection{}, true, nil)

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Contains(t, rr.Body.String(), "polyline")
		mockRS.AssertExpectations(t)
	})

	t.Run("Full Success with reroute", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&reroute=true&start_edge_id=1", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.ExpectedCalls = nil
		mockRS.On("ShortestPath", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, true, da.Index(1)).
			Return(100.0, 1000.0, "polyline", []da.DrivingDirection{}, true, nil)

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		mockRS.AssertExpectations(t)
	})

	t.Run("Service Error", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.ExpectedCalls = nil
		mockRS.On("ShortestPath", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, false, da.INVALID_EDGE_ID).
			Return(0.0, 0.0, "", []da.DrivingDirection{}, false, errors.New("internal error"))

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
		mockRS.AssertExpectations(t)
	})

	t.Run("Not Found", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.ExpectedCalls = nil
		mockRS.On("ShortestPath", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, false, da.INVALID_EDGE_ID).
			Return(0.0, 0.0, "", []da.DrivingDirection{}, false, nil)

		api.shortestPath(rr, req, nil)

		assert.Equal(t, http.StatusNotFound, rr.Code)
		mockRS.AssertExpectations(t)
	})
}

func TestRoutingAPI_AlternativeRoutes(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockMMS := new(MockMapMatcherService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockMMS, mockTS)

	t.Run("Success", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&k=3", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.On("AlternativeRouteSearch", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, false, da.INVALID_EDGE_ID).
			Return([]routing.AlternativeRoute{{}}, nil)

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		mockRS.AssertExpectations(t)
	})

	t.Run("Full Success with reroute", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&k=3&reroute=true&start_edge_id=1", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.ExpectedCalls = nil
		mockRS.On("AlternativeRouteSearch", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, true, da.Index(1)).
			Return([]routing.AlternativeRoute{{}}, nil)

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		mockRS.AssertExpectations(t)
	})

	t.Run("Missing K", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "number of alternatives k is required")
	})

	t.Run("Invalid Origin Lon Param", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=abc&destination_lat=%f&destination_lon=%f&k=3", yogyakartaOriginLat, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "origin_lon is required and must be a valid float")
	})

	t.Run("Invalid Destination Lat Param", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=abc&destination_lon=%f&k=3", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "destination_lat is required and must be a valid float")
	})

	t.Run("Invalid Destination Lon Param", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=abc&k=3", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat), nil)
		rr := httptest.NewRecorder()

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "destination_lon is required and must be a valid float")
	})

	t.Run("Validation Error", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=100&origin_lon=%f&destination_lat=%f&destination_lon=%f&k=3", yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "validation error")
	})

	t.Run("Invalid Reroute Bool", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&k=3&reroute=maybe&start_edge_id=1", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "reroute must be boolean")
	})

	t.Run("Invalid Start Edge ID", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&k=3&reroute=true&start_edge_id=bad", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "edgeId must be unsigned integer 32 bit")
	})

	t.Run("Service Error", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&k=3", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.ExpectedCalls = nil
		mockRS.On("AlternativeRouteSearch", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, false, da.INVALID_EDGE_ID).
			Return([]routing.AlternativeRoute{}, errors.New("internal error"))

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
	})

	t.Run("Not Found", func(t *testing.T) {
		req, _ := http.NewRequest("GET", fmt.Sprintf("/computeAlternativeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f&k=3", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)
		rr := httptest.NewRecorder()

		mockRS.ExpectedCalls = nil
		mockRS.On("AlternativeRouteSearch", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, 3, false, da.INVALID_EDGE_ID).
			Return([]routing.AlternativeRoute{}, nil)

		api.AlternativeRoutes(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
	})
}

func TestRoutingAPI_GetBoundingBox(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockMMS := new(MockMapMatcherService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockMMS, mockTS)

	t.Run("Success", func(t *testing.T) {
		req, _ := http.NewRequest("GET", "/boundingBox", nil)
		rr := httptest.NewRecorder()

		mockRS.On("GetBoundingBox", req.Context()).Return(da.BoundingBox{})

		api.GetBoundingBox(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		mockRS.AssertExpectations(t)
	})

	t.Run("WriteJSON Error", func(t *testing.T) {
		req, _ := http.NewRequest("GET", "/boundingBox", nil)
		rr := httptest.NewRecorder()
		erw := &errorResponseWriter{ResponseWriter: rr}

		mockRS.On("GetBoundingBox", req.Context()).Return(da.BoundingBox{})

		api.GetBoundingBox(erw, req, nil)
		// Should not panic, but trigger the error path
	})
}

func TestRoutingAPI_OnlineMapMatch(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockMMS := new(MockMapMatcherService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockMMS, mockTS)

	t.Run("Invalid JSON", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/onlineMapMatch", bytes.NewBufferString("invalid"))
		rr := httptest.NewRecorder()

		api.onlineMapMatch(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
	})

	t.Run("Success", func(t *testing.T) {
		requestBody := map[string]interface{}{
			"gps_point": map[string]interface{}{
				"lat":  yogyakartaOriginLat,
				"lon":  yogyakartaOriginLon,
				"time": "2026-04-29T10:00:00Z",
			},
			"k":            1,
			"candidates":   []interface{}{},
			"speed_mean_k": 0.0,
			"speed_std_k":  0.0,
			"last_bearing": 0.0,
		}
		requestTime := time.Date(2026, 4, 29, 10, 0, 0, 0, time.UTC)
		body, _ := json.Marshal(requestBody)
		req, _ := http.NewRequest("POST", "/onlineMapMatch", bytes.NewBuffer(body))
		rr := httptest.NewRecorder()

		dummyGPS := da.NewGPSPoint(yogyakartaOriginLat, yogyakartaOriginLon, requestTime, 0, 0, false)
		dummyMatched := da.NewMatchedGPSPoint(dummyGPS, 1, da.Coordinate{Lat: yogyakartaOriginLat, Lon: yogyakartaOriginLon}, 0)

		mockMMS.On("OnlineMapMatch", req.Context(), dummyGPS, 1, []*ma.Candidate{}, 0.0, 0.0, 0.0).
			Return(dummyMatched, []*ma.Candidate{}, 0.0, 0.0, nil)

		api.onlineMapMatch(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		mockMMS.AssertExpectations(t)
	})

	t.Run("Validation Error", func(t *testing.T) {
		requestBody := map[string]interface{}{
			"gps_point": map[string]interface{}{
				"lat": 100.0, // Invalid lat
				"lon": yogyakartaOriginLon,
			},
		}
		body, _ := json.Marshal(requestBody)
		req, _ := http.NewRequest("POST", "/onlineMapMatch", bytes.NewBuffer(body))
		rr := httptest.NewRecorder()

		api.onlineMapMatch(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "validation error")
	})

	t.Run("Service Error", func(t *testing.T) {
		requestBody := map[string]interface{}{
			"gps_point": map[string]interface{}{
				"lat":  yogyakartaOriginLat,
				"lon":  yogyakartaOriginLon,
				"time": "2026-04-29T10:00:00Z",
			},
			"k": 1,
		}
		body, _ := json.Marshal(requestBody)
		req, _ := http.NewRequest("POST", "/onlineMapMatch", bytes.NewBuffer(body))
		rr := httptest.NewRecorder()

		requestTime := time.Date(2026, 4, 29, 10, 0, 0, 0, time.UTC)
		gpsPoint := da.NewGPSPoint(yogyakartaOriginLat, yogyakartaOriginLon, requestTime, 0, 0, false)
		mockMMS.ExpectedCalls = nil
		mockMMS.On("OnlineMapMatch", req.Context(), gpsPoint, 1, []*ma.Candidate{}, 0.0, 0.0, 0.0).
			Return(nil, []*ma.Candidate{}, 0.0, 0.0, errors.New("internal error"))

		api.onlineMapMatch(rr, req, nil)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
	})

	t.Run("Body Close Error", func(t *testing.T) {
		requestBody := []byte(fmt.Sprintf(`{"gps_point":{"lat":%f,"lon":%f,"time":"2026-04-29T10:00:00Z"},"k":1}`, yogyakartaOriginLat, yogyakartaOriginLon))
		req, _ := http.NewRequest("POST", "/onlineMapMatch", nil)
		req.Body = closeErrorBody{Reader: bytes.NewReader(requestBody)}
		rr := httptest.NewRecorder()

		api.onlineMapMatch(rr, req, nil)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
		assert.Contains(t, rr.Body.String(), "the server encountered a problem")
	})

	t.Run("WriteJSON Error", func(t *testing.T) {
		requestBody := map[string]interface{}{
			"gps_point": map[string]interface{}{
				"lat":  yogyakartaOriginLat,
				"lon":  yogyakartaOriginLon,
				"time": "2026-04-29T10:00:00Z",
			},
			"k": 1,
		}
		body, _ := json.Marshal(requestBody)
		req, _ := http.NewRequest("POST", "/onlineMapMatch", bytes.NewBuffer(body))
		rr := httptest.NewRecorder()
		erw := &errorResponseWriter{ResponseWriter: rr}

		requestTime := time.Date(2026, 4, 29, 10, 0, 0, 0, time.UTC)
		gpsPoint := da.NewGPSPoint(yogyakartaOriginLat, yogyakartaOriginLon, requestTime, 0, 0, false)
		matched := da.NewMatchedGPSPoint(gpsPoint, 1, da.Coordinate{Lat: yogyakartaOriginLat, Lon: yogyakartaOriginLon}, 0)
		mockMMS.ExpectedCalls = nil
		mockMMS.On("OnlineMapMatch", req.Context(), gpsPoint, 1, []*ma.Candidate{}, 0.0, 0.0, 0.0).
			Return(matched, []*ma.Candidate{}, 0.0, 0.0, nil)

		api.onlineMapMatch(erw, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
	})
}

func TestRoutingAPI_Routes(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockMMS := new(MockMapMatcherService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockMMS, mockTS)

	router := httprouter.New()
	group := helper.NewRouteGroup(router, "/api")
	api.Routes(group)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", fmt.Sprintf("/api/computeRoutes?origin_lat=%f&origin_lon=%f&destination_lat=%f&destination_lon=%f", yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon), nil)

	mockRS.On("ShortestPath", req.Context(), yogyakartaOriginLat, yogyakartaOriginLon, yogyakartaDestLat, yogyakartaDestLon, false, da.INVALID_EDGE_ID).
		Return(100.0, 1000.0, "polyline", []da.DrivingDirection{}, true, nil)

	router.ServeHTTP(w, req)
	assert.Equal(t, http.StatusOK, w.Code)
}

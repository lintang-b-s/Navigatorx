package controllers

import (
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"reflect"
	"strings"
	"testing"
	"time"

	"github.com/julienschmidt/httprouter"
	"github.com/lintang-b-s/Navigatorx/pkg"
	da "github.com/lintang-b-s/Navigatorx/pkg/datastructure"
	"github.com/lintang-b-s/Navigatorx/pkg/engine/routing"
	helper "github.com/lintang-b-s/Navigatorx/pkg/http/router/routerhelper"
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

func TestRoutingAPI_ShortestPath(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockTS)

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
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockTS)

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
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockTS)

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

func TestRoutingAPI_Routes(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockTS)

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

func TestRoutingAPI_TransitionMatrixTrailingSlashDoesNotRedirect(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockTS)

	router := httprouter.New()
	group := helper.NewRouteGroup(router, "/api")
	api.Routes(group)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/tile-init-transition-matrix/", nil)

	router.ServeHTTP(w, req)

	assert.NotEqual(t, http.StatusMovedPermanently, w.Code)
}

func TestRoutingAPI_TransitionMatrixIgnoresConditionalCacheHeaders(t *testing.T) {
	previousWorkingDir, err := os.Getwd()
	assert.NoError(t, err)
	tempDir := t.TempDir()
	assert.NoError(t, os.Chdir(tempDir))
	t.Cleanup(func() {
		assert.NoError(t, os.Chdir(previousWorkingDir))
	})

	previousProfileName := pkg.ProfileName
	previousRegionName := pkg.RegionName
	pkg.ProfileName = "car"
	pkg.RegionName = "test"
	t.Cleanup(func() {
		pkg.ProfileName = previousProfileName
		pkg.RegionName = previousRegionName
	})

	matrixPath := "data/profiles/car/test_transition_matrix.ntm"
	assert.NoError(t, os.MkdirAll("data/profiles/car", 0o755))
	assert.NoError(t, os.WriteFile(matrixPath, []byte("matrix"), 0o644))
	assert.NoError(t, os.Chtimes(matrixPath, time.Now().Add(-time.Hour), time.Now().Add(-time.Hour)))

	api := New(new(MockRoutingService), zap.NewNop(), new(MockTilingService))
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/tile-init-transition-matrix/", nil)
	req.Header.Set("If-Modified-Since", time.Now().UTC().Format(http.TimeFormat))

	api.initClientSideRealTimeMapMatchingTransitionMatrix(w, req, nil)

	assert.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, "matrix", w.Body.String())
	assert.Contains(t, w.Header().Get("Cache-Control"), "no-store")
}

func TestRoutingAPI_OfflineMapMatching(t *testing.T) {
	log := zap.NewNop()
	mockRS := new(MockRoutingService)
	mockTS := new(MockTilingService)
	api := New(mockRS, log, mockTS)

	validGPX := `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="GraphHopper">
  <trk>
    <trkseg>
      <trkpt lat="-7.7956" lon="110.3695">
        <time>2026-05-21T15:00:00Z</time>
      </trkpt>
      <trkpt lat="-7.7828" lon="110.4145">
        <time>2026-05-21T15:00:05Z</time>
      </trkpt>
    </trkseg>
  </trk>
</gpx>`

	t.Run("Invalid GPS Radiuses", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching?gps_radiuses=50;-10", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "gps_radiuses must contain non-negative float values")
	})

	t.Run("GPS Radiuses Count Mismatch", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching?gps_radiuses=50", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "gps_radiuses count must match GPX trackpoint count")
	})

	t.Run("Invalid Single GPS Radius", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching?gps_radius=-10", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "gps_radius must be a non-negative float value")
	})

	t.Run("Invalid XML GPX Body", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching", strings.NewReader("invalid xml"))
		rr := httptest.NewRecorder()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "failed to parse GPX XML")
	})

	t.Run("Empty GPX Trackpoints", func(t *testing.T) {
		gpxStr := `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="GraphHopper">
  <trk>
    <trkseg>
    </trkseg>
  </trk>
</gpx>`
		req, _ := http.NewRequest("POST", "/mapmatching", strings.NewReader(gpxStr))
		rr := httptest.NewRecorder()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "GPX contains no track points")
	})

	t.Run("Invalid Coordinates", func(t *testing.T) {
		gpxStr := `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="GraphHopper">
  <trk>
    <trkseg>
      <trkpt lat="100.0" lon="11.0">
        <time>2026-05-21T15:00:00Z</time>
      </trkpt>
    </trkseg>
  </trk>
</gpx>`
		req, _ := http.NewRequest("POST", "/mapmatching", strings.NewReader(gpxStr))
		rr := httptest.NewRecorder()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "invalid trackpoint coordinates")
	})

	t.Run("Success with custom GPS radiuses", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching?gps_radiuses=50;60", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		mockRS.On("OfflineMapMatch",
			mock.Anything,
			mock.Anything,
			mock.MatchedBy(func(gpsRadiusesM []float64) bool {
				return reflect.DeepEqual(gpsRadiusesM, []float64{50, 60})
			}),
		).Return([]*da.MatchedGPSPoint{}, []da.Coordinate{}, nil).Once()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Contains(t, rr.Body.String(), "matched_points")
		assert.Contains(t, rr.Body.String(), "route_path")
		mockRS.AssertExpectations(t)
	})

	t.Run("Success with single GPS radius", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching?gps_radius=70", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		mockRS.On("OfflineMapMatch",
			mock.Anything,
			mock.Anything,
			mock.MatchedBy(func(gpsRadiusesM []float64) bool {
				return reflect.DeepEqual(gpsRadiusesM, []float64{70, 70})
			}),
		).Return([]*da.MatchedGPSPoint{}, []da.Coordinate{}, nil).Once()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Contains(t, rr.Body.String(), "matched_points")
		assert.Contains(t, rr.Body.String(), "route_path")
		mockRS.AssertExpectations(t)
	})

	t.Run("Success with default GPS radiuses", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		mockRS.On("OfflineMapMatch",
			mock.Anything,
			mock.Anything,
			mock.MatchedBy(func(gpsRadiusesM []float64) bool {
				return reflect.DeepEqual(gpsRadiusesM, []float64{40, 40})
			}),
		).Return([]*da.MatchedGPSPoint{}, []da.Coordinate{}, nil).Once()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Contains(t, rr.Body.String(), "matched_points")
		assert.Contains(t, rr.Body.String(), "route_path")
		mockRS.AssertExpectations(t)
	})

	t.Run("Success without route path", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching?gps_radius=70&include_route_path=false", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		mockRS.On("OfflineMapMatch",
			mock.Anything,
			mock.Anything,
			mock.MatchedBy(func(gpsRadiusesM []float64) bool {
				return reflect.DeepEqual(gpsRadiusesM, []float64{70, 70})
			}),
		).Return([]*da.MatchedGPSPoint{}, []da.Coordinate{}, nil).Once()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Contains(t, rr.Body.String(), "matched_points")
		assert.Contains(t, rr.Body.String(), "route_path")
		mockRS.AssertExpectations(t)
	})

	t.Run("Invalid include route path", func(t *testing.T) {
		req, _ := http.NewRequest("POST", "/mapmatching?include_route_path=wat", strings.NewReader(validGPX))
		rr := httptest.NewRecorder()

		api.offlineMapMatching(rr, req, nil)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "include_route_path must be a boolean value")
	})
}

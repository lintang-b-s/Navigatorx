package controllers

import (
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/lintang-b-s/Navigatorx/pkg/util"
	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

type errorResponseWriter struct {
	http.ResponseWriter
}

func (e *errorResponseWriter) Header() http.Header {
	return e.ResponseWriter.Header()
}

func (e *errorResponseWriter) Write(b []byte) (int, error) {
	return 0, errors.New("write error")
}

func (e *errorResponseWriter) WriteHeader(statusCode int) {
	e.ResponseWriter.WriteHeader(statusCode)
}

func TestRoutingAPI_ErrorResponse(t *testing.T) {
	log := zap.NewNop()
	api := &routingAPI{log: log}

	t.Run("Internal Server Error", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		api.ServerErrorResponse(rr, req, errors.New("test error"))
		assert.Equal(t, http.StatusInternalServerError, rr.Code)
		assert.Contains(t, rr.Body.String(), "the server encountered a problem")
	})

	t.Run("Not Found", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		api.NotFoundResponse(rr, req)
		assert.Equal(t, http.StatusNotFound, rr.Code)
		assert.Contains(t, rr.Body.String(), "the requested resource could not be found")
	})

	t.Run("Method Not Allowed", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		api.MethodNotAllowedResponse(rr, req)
		assert.Equal(t, http.StatusMethodNotAllowed, rr.Code)
		assert.Contains(t, rr.Body.String(), "method is not supported this resource")
	})

	t.Run("Bad Request", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		api.BadRequestResponse(rr, req, errors.New("bad request"))
		assert.Equal(t, http.StatusBadRequest, rr.Code)
		assert.Contains(t, rr.Body.String(), "bad request")
	})

	t.Run("Invalid Credentials", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		api.InvalidCredentialsResponse(rr, req)
		assert.Equal(t, http.StatusUnauthorized, rr.Code)
	})

	t.Run("Edit Conflict", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		api.EditConflictResponse(rr, req)
		assert.Equal(t, http.StatusConflict, rr.Code)
	})

	t.Run("Invalid Authentication Token", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		api.InvalidAuthenticationTokenResponse(rr, req)
		assert.Equal(t, http.StatusUnauthorized, rr.Code)
		assert.Equal(t, "Bearer", rr.Header().Get("WWWW-Authenticate"))
	})

	t.Run("WriteJSON Error", func(t *testing.T) {
		rr := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/", nil)
		erw := &errorResponseWriter{ResponseWriter: rr}
		api.errorResponse(erw, req, http.StatusInternalServerError, "test")
		// Should trigger log.Error and WriteHeader(500)
	})
}

func TestRoutingAPI_GetStatusCode(t *testing.T) {
	api := &routingAPI{log: zap.NewNop()}
	req, _ := http.NewRequest("GET", "/", nil)

	t.Run("Nil Error", func(t *testing.T) {
		rr := httptest.NewRecorder()
		api.getStatusCode(rr, req, nil)
		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Contains(t, rr.Body.String(), "success")
	})

	tests := []struct {
		name string
		err  error
		code int
	}{
		{"Internal", util.WrapErrorf(errors.New("internal"), util.ErrInternalServerError, "internal"), http.StatusInternalServerError},
		{"Not Found", util.WrapErrorf(errors.New("not found"), util.ErrNotFound, "not found"), http.StatusNotFound},
		{"Conflict", util.WrapErrorf(errors.New("conflict"), util.ErrConflict, "conflict"), http.StatusConflict},
		{"Bad Param", util.WrapErrorf(errors.New("bad"), util.ErrBadParamInput, "bad"), http.StatusBadRequest},
		{"Plain Error", errors.New("plain"), http.StatusInternalServerError},
		{"Unknown Code", util.WrapErrorf(errors.New("unknown"), errors.New("unknown code"), "unknown"), http.StatusInternalServerError},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			rr := httptest.NewRecorder()
			api.getStatusCode(rr, req, tt.err)
			assert.Equal(t, tt.code, rr.Code)
		})
	}
}

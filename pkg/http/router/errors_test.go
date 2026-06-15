package router

import (
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

type routerErrorResponseWriter struct {
	http.ResponseWriter
}

func (w routerErrorResponseWriter) Write(_ []byte) (int, error) {
	return 0, errors.New("write error")
}

func TestAPI_ErrorResponses(t *testing.T) {
	api := NewAPI(zap.NewNop())
	r := httptest.NewRequest("GET", "/test", nil)

	t.Run("ServerErrorResponse", func(t *testing.T) {
		w := httptest.NewRecorder()
		api.ServerErrorResponse(w, r, errors.New("test error"))
		assert.Equal(t, http.StatusInternalServerError, w.Code)
		assert.Contains(t, w.Body.String(), "the server encountered a problem")
	})

	t.Run("NotFoundResponse", func(t *testing.T) {
		w := httptest.NewRecorder()
		api.NotFoundResponse(w, r)
		assert.Equal(t, http.StatusNotFound, w.Code)
		assert.Contains(t, w.Body.String(), "the requested resource could not be found")
	})

	t.Run("MethodNotAllowedResponse", func(t *testing.T) {
		w := httptest.NewRecorder()
		api.MethodNotAllowedResponse(w, r)
		assert.Equal(t, http.StatusMethodNotAllowed, w.Code)
		assert.Contains(t, w.Body.String(), "the GET method is not supported")
	})

	t.Run("BadRequestResponse", func(t *testing.T) {
		w := httptest.NewRecorder()
		api.BadRequestResponse(w, r, errors.New("bad request test"))
		assert.Equal(t, http.StatusBadRequest, w.Code)
		assert.Contains(t, w.Body.String(), "bad request test")
	})

	t.Run("EditConflictResponse", func(t *testing.T) {
		w := httptest.NewRecorder()
		api.EditConflictResponse(w, r)
		assert.Equal(t, http.StatusConflict, w.Code)
		assert.Contains(t, w.Body.String(), "edit conflict")
	})

	t.Run("InvalidCredentialsResponse", func(t *testing.T) {
		w := httptest.NewRecorder()
		api.InvalidCredentialsResponse(w, r)
		assert.Equal(t, http.StatusUnauthorized, w.Code)
		assert.Contains(t, w.Body.String(), "invalid authentication credentials")
	})

	t.Run("InvalidAuthenticationTokenResponse", func(t *testing.T) {
		w := httptest.NewRecorder()
		api.InvalidAuthenticationTokenResponse(w, r)
		assert.Equal(t, http.StatusUnauthorized, w.Code)
		assert.Equal(t, "Bearer", w.Header().Get("WWWW-Authenticate"))
		assert.Contains(t, w.Body.String(), "invalid or missing authentication token")
	})
}

func TestAPI_WriteJSON_Error(t *testing.T) {
	api := NewAPI(zap.NewNop())
	w := httptest.NewRecorder()

	data := errorEnvelope{Error: errorBody{Code: "error", Message: "test"}}
	err := api.writeJSON(w, http.StatusOK, data)
	assert.NoError(t, err)
}

func TestAPI_WriteJSON_UnserializableContent(t *testing.T) {
	api := NewAPI(zap.NewNop())
	w := httptest.NewRecorder()

	type unserializable struct {
		Ch chan int `json:"ch"`
	}
	data := unserializable{Ch: make(chan int)}
	err := api.writeJSON(w, http.StatusOK, data)
	assert.Error(t, err)
}

func TestAPI_WriteJSON_SuccessWithHeaders(t *testing.T) {
	api := NewAPI(zap.NewNop())
	w := httptest.NewRecorder()

	type okResponse struct {
		Data string `json:"data"`
	}
	err := api.writeJSON(w, http.StatusAccepted, okResponse{Data: "ok"})

	assert.NoError(t, err)
	assert.Equal(t, http.StatusAccepted, w.Code)
	assert.Contains(t, w.Body.String(), "ok")
}

func TestAPI_ErrorResponse_WriteError(t *testing.T) {
	api := NewAPI(zap.NewNop())
	req := httptest.NewRequest("GET", "/test", nil)
	rr := httptest.NewRecorder()

	api.errorResponse(routerErrorResponseWriter{ResponseWriter: rr}, req, http.StatusBadRequest, "bad")

	assert.Equal(t, http.StatusBadRequest, rr.Code)
}

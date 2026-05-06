package router

import (
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

func TestMiddleware_RealIP(t *testing.T) {
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusOK)
	})

	mw := RealIP(handler)

	t.Run("X-Forwarded-For", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.Header.Set("X-Forwarded-For", "1.2.3.4, 5.6.7.8")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, "1.2.3.4", req.RemoteAddr)
	})

	t.Run("X-Forwarded-For Single", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.Header.Set("X-Forwarded-For", "1.2.3.4")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, "1.2.3.4", req.RemoteAddr)
	})

	t.Run("X-Real-IP", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.Header.Set("X-Real-IP", "1.2.3.4")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, "1.2.3.4", req.RemoteAddr)
	})

	t.Run("True-Client-IP", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.Header.Set("True-Client-IP", "1.2.3.4")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, "1.2.3.4", req.RemoteAddr)
	})

	t.Run("RemoteAddr", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.RemoteAddr = "1.2.3.4:1234"
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, "1.2.3.4:1234", req.RemoteAddr)
	})

	t.Run("Invalid IP Header", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.Header.Set("X-Real-IP", "invalid")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.NotEqual(t, "invalid", req.RemoteAddr)
	})
}

func TestMiddleware_Heartbeat(t *testing.T) {
	log := zap.NewNop()
	api := NewAPI(log)
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusOK)
		_, _ = w.Write([]byte("next"))
	})

	mw := api.Heartbeat("/healthz")(handler)

	t.Run("Match", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/healthz", nil)
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)
		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Equal(t, ".", rr.Body.String())
	})

	t.Run("No Match", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/other", nil)
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)
		assert.Equal(t, http.StatusOK, rr.Code)
		assert.Equal(t, "next", rr.Body.String())
	})

	t.Run("Shutting Down", func(t *testing.T) {
		isShuttingDown.Store(true)
		defer isShuttingDown.Store(false)
		req := httptest.NewRequest("GET", "/healthz", nil)
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusServiceUnavailable, rr.Code)
		assert.Contains(t, rr.Body.String(), "Shutting down")
	})
}

func TestMiddleware_Logger(t *testing.T) {
	log := zap.NewNop()
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusOK)
	})

	mw := Logger(log)(handler)

	req := httptest.NewRequest("GET", "/", nil)
	rr := httptest.NewRecorder()

	mw.ServeHTTP(rr, req)

	assert.Equal(t, http.StatusOK, rr.Code)
}

func TestMiddleware_EnforceJSONHandler(t *testing.T) {
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusOK)
	})

	mw := EnforceJSONHandler(handler)

	t.Run("Valid Content-Type", func(t *testing.T) {
		req := httptest.NewRequest("POST", "/", nil)
		req.Header.Set("Content-Type", "application/json")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusOK, rr.Code)
	})

	t.Run("Missing Content-Type", func(t *testing.T) {
		req := httptest.NewRequest("POST", "/", nil)
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusOK, rr.Code)
	})

	t.Run("Invalid Content-Type", func(t *testing.T) {
		req := httptest.NewRequest("POST", "/", nil)
		req.Header.Set("Content-Type", "text/plain")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusUnsupportedMediaType, rr.Code)
	})

	t.Run("Malformed Content-Type", func(t *testing.T) {
		req := httptest.NewRequest("POST", "/", nil)
		req.Header.Set("Content-Type", "application/json; charset=")
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusBadRequest, rr.Code)
	})

	t.Run("GET Request", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusOK, rr.Code)
	})
}

func TestMiddleware_Labels(t *testing.T) {
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusOK)
	})

	mw := Labels(handler)

	req := httptest.NewRequest("GET", "/", nil)
	rr := httptest.NewRecorder()

	mw.ServeHTTP(rr, req)

	assert.Equal(t, http.StatusOK, rr.Code)
}

func TestMiddleware_Limit(t *testing.T) {
	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusOK)
	})
	api := NewAPI(zap.NewNop())

	mw := api.Limit(handler)

	t.Run("Allow", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.RemoteAddr = "127.0.0.1:1234"
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusOK, rr.Code)
	})

	t.Run("Missing Port", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.RemoteAddr = "127.0.0.1" // Should be handled by net.SplitHostPort check
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusOK, rr.Code)
	})

	t.Run("SplitHostPort Error", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.RemoteAddr = "[::1:8080" // Invalid address (missing ])
		rr := httptest.NewRecorder()

		mw.ServeHTTP(rr, req)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
	})

	t.Run("Rate Limit Exceeded", func(t *testing.T) {
		req := httptest.NewRequest("GET", "/", nil)
		req.RemoteAddr = "9.9.9.9:1234"

		// Exhaust the limiter (starts with 'burst' tokens)
		// We loop slightly more than the burst to trigger the 429 error
		for i := 0; i < burst+5; i++ {
			rr := httptest.NewRecorder()
			mw.ServeHTTP(rr, req)

			if i < burst {
				// First 'burst' requests should succeed
				assert.Equal(t, http.StatusOK, rr.Code, "Request %d should be OK", i)
			} else {
				// Requests beyond the burst should fail
				assert.Equal(t, http.StatusTooManyRequests, rr.Code, "Request %d should be rate limited", i)
			}
		}
	})

}

func TestResponseWriter_Status(t *testing.T) {
	rr := httptest.NewRecorder()
	rw := &responseWriter{ResponseWriter: rr}

	assert.Equal(t, 0, rw.Status())
	rw.WriteHeader(http.StatusCreated)
	rw.WriteHeader(http.StatusAccepted)
	assert.Equal(t, http.StatusCreated, rw.status)
	assert.Equal(t, http.StatusCreated, rr.Code)

	_, _ = rw.Write([]byte("test"))
	assert.Equal(t, http.StatusCreated, rw.status)
}

func TestAPI_RecoverPanic(t *testing.T) {
	log := zap.NewNop()
	api := NewAPI(log)

	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		panic("test panic")
	})

	mw := api.recoverPanic(handler)

	req := httptest.NewRequest("GET", "/", nil)
	rr := httptest.NewRecorder()

	mw.ServeHTTP(rr, req)

	assert.Equal(t, http.StatusInternalServerError, rr.Code)
	assert.Contains(t, rr.Body.String(), "the server encountered a problem")
}

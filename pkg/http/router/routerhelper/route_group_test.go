package routerhelper

import (
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/julienschmidt/httprouter"
	"github.com/stretchr/testify/assert"
)

func TestNewRouteGroup(t *testing.T) {
	r := httprouter.New()

	t.Run("Valid Path", func(t *testing.T) {
		group := NewRouteGroup(r, "/api")
		assert.Equal(t, "/api", group.p)
	})

	t.Run("Path with Trailing Slash", func(t *testing.T) {
		group := NewRouteGroup(r, "/api/")
		assert.Equal(t, "/api", group.p)
	})

	t.Run("Invalid Path Panic", func(t *testing.T) {
		assert.Panics(t, func() {
			NewRouteGroup(r, "api")
		})
	})
}

func TestRouteGroup_NewGroup(t *testing.T) {
	r := httprouter.New()
	group := NewRouteGroup(r, "/api")
	subGroup := group.NewGroup("/v1")
	assert.Equal(t, "/api/v1", subGroup.p)
}

func TestRouteGroup_Methods(t *testing.T) {
	r := httprouter.New()
	group := NewRouteGroup(r, "/api")

	handle := func(w http.ResponseWriter, r *http.Request, p httprouter.Params) {
		w.WriteHeader(http.StatusOK)
	}
	group.GET("/test_get", handle)
	group.POST("/test_post", handle)
	group.PUT("/test_put", handle)
	group.DELETE("/test_delete", handle)
	group.PATCH("/test_patch", handle)
	group.HEAD("/test_head", handle)
	group.OPTIONS("/test_options", handle)
	group.HandlerFunc("GET", "/test_handlerfunc", func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusOK)
	})

	methods := []string{"GET", "POST", "PUT", "DELETE", "PATCH", "HEAD", "OPTIONS"}
	for _, m := range methods {
		w := httptest.NewRecorder()
		req, _ := http.NewRequest(m, "/api/test_"+strings.ToLower(m), nil)
		r.ServeHTTP(w, req)
		assert.Equal(t, http.StatusOK, w.Code)
	}

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/test_handlerfunc", nil)
	r.ServeHTTP(w, req)
	assert.Equal(t, http.StatusOK, w.Code)
}

func TestRouteGroup_Handler(t *testing.T) {
	r := httprouter.New()
	group := NewRouteGroup(r, "/api")

	handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusCreated)
	})
	group.Handler("GET", "/handler", handler)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/handler", nil)
	r.ServeHTTP(w, req)
	assert.Equal(t, http.StatusCreated, w.Code)
}

func TestRouteGroup_SubPathPanic(t *testing.T) {
	r := httprouter.New()
	group := NewRouteGroup(r, "/api")

	assert.Panics(t, func() {
		group.GET("test", nil)
	})
}

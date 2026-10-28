package routerhelper

import (
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/julienschmidt/httprouter"
	"github.com/stretchr/testify/assert"
)

func TestNewRouteGroup(t *testing.T) {
	r := httprouter.New()

	t.Run("Valid Path", func(t *testing.T) {
		group, err := NewRouteGroup(r, "/api")
		assert.NoError(t, err)
		assert.Equal(t, "/api", group.p)
	})

	t.Run("Path with Trailing Slash", func(t *testing.T) {
		group, err := NewRouteGroup(r, "/api/")
		assert.NoError(t, err)
		assert.Equal(t, "/api", group.p)
	})

	t.Run("Invalid Path error", func(t *testing.T) {
		_, err := NewRouteGroup(r, "api")

		assert.Error(t, err)
	})
}

func TestRouteGroup_Handler(t *testing.T) {
	r := httprouter.New()
	group, err := NewRouteGroup(r, "/api")
	assert.NoError(t, err)
	handler := http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusCreated)
	})
	group.Handler("GET", "/handler", handler)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/handler", nil)
	r.ServeHTTP(w, req)
	assert.Equal(t, http.StatusCreated, w.Code)
}

func TestRouteGroup_NewGroupError(t *testing.T) {
	r := httprouter.New()
	_, err := NewRouteGroup(r, "api")
	assert.Error(t, err)
}

func TestRouteGroup_SubPathError(t *testing.T) {
	r := httprouter.New()
	g, err := NewRouteGroup(r, "/api")
	assert.NoError(t, err)
	assert.Panics(t, func() {
		g.GET("si", func(w http.ResponseWriter, r *http.Request, p httprouter.Params) {})
	})
}

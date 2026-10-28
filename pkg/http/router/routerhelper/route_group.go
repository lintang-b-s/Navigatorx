// Package routerhelper provides helper functions for HTTP routing.
package routerhelper

import (
	"fmt"
	"net/http"

	"github.com/julienschmidt/httprouter"
	"github.com/lintang-b-s/Navigatorx/pkg/util"
)

// RouteGroup wraps an httprouter.Router with a path prefix.
type RouteGroup struct {
	r *httprouter.Router
	p string
}

// NewRouteGroup creates a new RouteGroup with the specified base path prefix.
func NewRouteGroup(r *httprouter.Router, path string) (*RouteGroup, error) {

	if path[0] != '/' {
		return nil, fmt.Errorf(`path must begin with '/' in path '"%s"'`, path)
	}

	if path[len(path)-1] == '/' {
		path = path[:len(path)-1]
	}
	return &RouteGroup{r: r, p: path}, nil
}

// Handle registers a new request handle with the given method and path.
func (r *RouteGroup) Handle(method, path string, handle httprouter.Handle) {
	r.r.Handle(method, r.subPath(path), handle)
}

// Handler registers a new request http.Handler with the given method and path.
func (r *RouteGroup) Handler(method, path string, handler http.Handler) {
	r.r.Handler(method, r.subPath(path), handler)
}

// HandlerFunc registers a new request handler func with the given method and path.
func (r *RouteGroup) HandlerFunc(method, path string, handler http.HandlerFunc) {
	r.r.HandlerFunc(method, r.subPath(path), handler)
}

// GET is a shortcut for Handle("GET", path, handle).
func (r *RouteGroup) GET(path string, handle httprouter.Handle) {
	r.Handle("GET", path, handle)
}

// HEAD is a shortcut for Handle("HEAD", path, handle).
func (r *RouteGroup) HEAD(path string, handle httprouter.Handle) {
	r.Handle("HEAD", path, handle)
}

// OPTIONS is a shortcut for Handle("OPTIONS", path, handle).
func (r *RouteGroup) OPTIONS(path string, handle httprouter.Handle) {
	r.Handle("OPTIONS", path, handle)
}

// POST is a shortcut for Handle("POST", path, handle).
func (r *RouteGroup) POST(path string, handle httprouter.Handle) {
	r.Handle("POST", path, handle)
}

// PUT is a shortcut for Handle("PUT", path, handle).
func (r *RouteGroup) PUT(path string, handle httprouter.Handle) {
	r.Handle("PUT", path, handle)
}

// PATCH is a shortcut for Handle("PATCH", path, handle).
func (r *RouteGroup) PATCH(path string, handle httprouter.Handle) {
	r.Handle("PATCH", path, handle)
}

// DELETE is a shortcut for Handle("DELETE", path, handle).
func (r *RouteGroup) DELETE(path string, handle httprouter.Handle) {
	r.Handle("DELETE", path, handle)
}

const (
	subPathErrMsg = "path must start with a '/'"
)

func (r *RouteGroup) subPath(path string) string {

	util.AssertPanic(path[0] == '/', subPathErrMsg)
	return r.p + path
}

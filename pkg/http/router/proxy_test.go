package router

import (
	"bufio"
	"errors"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/stretchr/testify/assert"
	"go.uber.org/zap"
)

type proxyErrorReader struct{}

func (proxyErrorReader) Read(_ []byte) (int, error) {
	return 0, errors.New("read error")
}

type proxyHijackErrorWriter struct {
	*httptest.ResponseRecorder
}

func (w proxyHijackErrorWriter) Hijack() (net.Conn, *bufio.ReadWriter, error) {
	return nil, nil, errors.New("hijack error")
}

func withDialUpstream(t *testing.T, fn func(network, addr string) (net.Conn, error)) {
	t.Helper()
	orig := dialUpstream
	dialUpstream = fn
	t.Cleanup(func() { dialUpstream = orig })
}

func TestAPI_Upstream(t *testing.T) {
	log := zap.NewNop()
	api := NewAPI(log)

	t.Run("Dial Error", func(t *testing.T) {
		handler := api.upstream("test", "tcp", "localhost:12345") // Unlikely to be open
		req := httptest.NewRequest("GET", "/", nil)
		rr := httptest.NewRecorder()

		handler(rr, req)

		assert.Equal(t, http.StatusBadGateway, rr.Code)
	})

	t.Run("Success", func(t *testing.T) {
		// Create a mock upstream server
		ln, err := net.Listen("tcp", "127.0.0.1:0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		go func() {
			conn, err := ln.Accept()
			if err != nil {
				return
			}
			defer conn.Close()
			// Read request and write response
			buf := make([]byte, 1024)
			_, _ = conn.Read(buf)
			_, _ = conn.Write([]byte("HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nOK"))
		}()

		handler := api.upstream("test", "tcp", ln.Addr().String())

		ts := httptest.NewServer(http.HandlerFunc(handler))
		defer ts.Close()

		resp, err := http.Get(ts.URL)
		assert.NoError(t, err)
		defer resp.Body.Close()
		assert.Equal(t, http.StatusOK, resp.StatusCode)
		body, _ := io.ReadAll(resp.Body)
		assert.Equal(t, "OK", string(body))
	})

	t.Run("Write Request Error", func(t *testing.T) {
		ln, err := net.Listen("tcp", "127.0.0.1:0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		go func() {
			conn, err := ln.Accept()
			if err != nil {
				return
			}
			defer conn.Close()
			_, _ = io.Copy(io.Discard, conn)
		}()

		handler := api.upstream("test", "tcp", ln.Addr().String())
		req := httptest.NewRequest("POST", "/", io.NopCloser(proxyErrorReader{}))
		rr := httptest.NewRecorder()

		handler(rr, req)

		assert.Equal(t, http.StatusBadGateway, rr.Code)
	})

	t.Run("ResponseWriter Is Not Hijacker", func(t *testing.T) {
		ln, err := net.Listen("tcp", "127.0.0.1:0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		go func() {
			conn, err := ln.Accept()
			if err != nil {
				return
			}
			defer conn.Close()
			buf := make([]byte, 1024)
			_, _ = conn.Read(buf)
		}()

		handler := api.upstream("test", "tcp", ln.Addr().String())
		req := httptest.NewRequest("GET", "/", nil)
		rr := httptest.NewRecorder()

		handler(rr, req)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
	})

	t.Run("Hijack Error", func(t *testing.T) {
		ln, err := net.Listen("tcp", "127.0.0.1:0")
		skipSocketPermission(t, err)
		if err != nil {
			t.Fatal(err)
		}
		defer ln.Close()

		go func() {
			conn, err := ln.Accept()
			if err != nil {
				return
			}
			defer conn.Close()
			buf := make([]byte, 1024)
			_, _ = conn.Read(buf)
		}()

		handler := api.upstream("test", "tcp", ln.Addr().String())
		req := httptest.NewRequest("GET", "/", nil)
		rr := httptest.NewRecorder()

		handler(proxyHijackErrorWriter{ResponseRecorder: rr}, req)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
	})
}

func TestAPI_Upstream_WithoutTCPListen(t *testing.T) {
	api := NewAPI(zap.NewNop())

	t.Run("Dial Error", func(t *testing.T) {
		withDialUpstream(t, func(network, addr string) (net.Conn, error) {
			return nil, errors.New("dial error")
		})
		handler := api.upstream("test", "tcp", "example")
		req := httptest.NewRequest("GET", "/", nil)
		rr := httptest.NewRecorder()

		handler(rr, req)

		assert.Equal(t, http.StatusBadGateway, rr.Code)
	})

	t.Run("Write Request Error", func(t *testing.T) {
		upstreamConn, upstreamPeer := net.Pipe()
		defer upstreamPeer.Close()
		withDialUpstream(t, func(network, addr string) (net.Conn, error) {
			return upstreamConn, nil
		})
		go func() {
			_, _ = io.Copy(io.Discard, upstreamPeer)
		}()
		handler := api.upstream("test", "tcp", "example")
		req := httptest.NewRequest("POST", "/", io.NopCloser(proxyErrorReader{}))
		rr := httptest.NewRecorder()

		handler(rr, req)

		assert.Equal(t, http.StatusBadGateway, rr.Code)
	})

	t.Run("ResponseWriter Is Not Hijacker", func(t *testing.T) {
		upstreamConn, upstreamPeer := net.Pipe()
		defer upstreamPeer.Close()
		withDialUpstream(t, func(network, addr string) (net.Conn, error) {
			return upstreamConn, nil
		})
		go func() {
			buf := make([]byte, 1024)
			_, _ = upstreamPeer.Read(buf)
		}()
		handler := api.upstream("test", "tcp", "example")
		req := httptest.NewRequest("GET", "/", nil)
		rr := httptest.NewRecorder()

		handler(rr, req)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
	})

	t.Run("Hijack Error", func(t *testing.T) {
		upstreamConn, upstreamPeer := net.Pipe()
		defer upstreamPeer.Close()
		withDialUpstream(t, func(network, addr string) (net.Conn, error) {
			return upstreamConn, nil
		})
		go func() {
			buf := make([]byte, 1024)
			_, _ = upstreamPeer.Read(buf)
		}()
		handler := api.upstream("test", "tcp", "example")
		req := httptest.NewRequest("GET", "/", nil)
		rr := httptest.NewRecorder()

		handler(proxyHijackErrorWriter{ResponseRecorder: rr}, req)

		assert.Equal(t, http.StatusInternalServerError, rr.Code)
	})
}

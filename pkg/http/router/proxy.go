package router

import (
	"errors"
	"io"
	"net"
	"net/http"
	"strings"

	"go.uber.org/zap"
)

var dialUpstream = net.Dial

// isExpectedCloseError returns true if the error is a normal result of one side
// of the bidirectional io.Copy bridge closing (e.g. user stopped navigation).
func isExpectedCloseError(err error) bool {
	if errors.Is(err, net.ErrClosed) {
		return true
	}
	if errors.Is(err, io.EOF) {
		return true
	}
	// Fallback string check for wrapped errors
	return strings.Contains(err.Error(), "use of closed network connection")
}

// buat get raw tcp connection dari client (browser) -> upgrade to websocket -> create bidirectional bridge between client and wsServer

func (api *API) upstream(name, network, addr string) func(w http.ResponseWriter, r *http.Request) {

	return func(w http.ResponseWriter, r *http.Request) {

		wsServer, err := dialUpstream(network, addr)
		if err != nil {
			api.log.Error("dial upstream error:", zap.Error(err))
			w.WriteHeader(502)
			return
		}
		if err := r.Write(wsServer); err != nil {
			api.log.Error("write request to upstream error: %v", zap.Error(err))
			w.WriteHeader(502)
			return
		}
		hj, ok := w.(http.Hijacker)
		if !ok {
			w.WriteHeader(500)
			return
		}
		client, _, err := hj.Hijack() // get tcp socket
		// client (ws client/browser)
		if err != nil {
			w.WriteHeader(500)
			return
		}

		go func() {
			defer wsServer.Close()
			defer client.Close()
			if _, err := io.Copy(wsServer, client); err != nil {
				if !isExpectedCloseError(err) {
					api.log.Error("copy response to upstream error:", zap.Error(err))
				}
				return
			}
		}()
		go func() {
			defer wsServer.Close()
			defer client.Close()
			if _, err := io.Copy(client, wsServer); err != nil {
				if !isExpectedCloseError(err) {
					api.log.Error("copy request to client error:", zap.Error(err))
				}
				return
			}
		}()
	}
}

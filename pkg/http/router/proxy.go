package router

import (
	"io"
	"net"
	"net/http"

	"go.uber.org/zap"
)

var dialUpstream = net.Dial

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
				api.log.Error("copy response to upstream error:", zap.Error(err))
				return
			}
		}()
		go func() {
			defer wsServer.Close()
			defer client.Close()
			if _, err := io.Copy(client, wsServer); err != nil {
				api.log.Error("copy request to client error:", zap.Error(err))
				return
			}
		}()
	}
}
